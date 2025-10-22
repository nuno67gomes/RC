// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h> 
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <stddef.h>

#include <fcntl.h>   // open(), O_RDONLY, etc.
#include <unistd.h> 

#define T_FILE_SIZE 0x00
#define T_FILE_NAME 0x01

#define CTRL_DATA 0x01
#define CTRL_START 0x02
#define CTRL_END 0x03

#define APP_MAX_INFO_LEN 1024
#define APP_DATA_HDR_SIZE 4
#define APP_CHUNK (APP_MAX_INFO_LEN - APP_DATA_HDR_SIZE)

static void hexdump(const char *tag, const unsigned char *b, size_t n){
    if (tag) printf("%s (%zu): ", tag, n);
    for (size_t i = 0; i < n; ++i) printf("%02X ", b[i]);
    putchar('\n');
}


// START / END

static int buildStartEnd(unsigned char *out, uint8_t ctrl, const char *fname, off_t fsize) {
    if (!out || !fname) return -1;

    char sizeStr[32];
    snprintf(sizeStr, sizeof(sizeStr), "%lld", (long long)fsize);

    size_t nameLen = strlen(fname);
    size_t sizeLen = strlen(sizeStr);
    if (nameLen > 255) nameLen = 255;
    if (sizeLen > 255) sizeLen = 255;

    size_t i = 0;
    out[i++] = ctrl;

    out[i++] = (uint8_t)T_FILE_SIZE; out[i++] = (uint8_t)sizeLen;
    memcpy(&out[i], sizeStr, sizeLen); i += sizeLen;

    out[i++] = (uint8_t)T_FILE_NAME; out[i++] = (uint8_t)nameLen;
    memcpy(&out[i], fname, nameLen);  i += nameLen;

    return (int)i;
}

static int sendStartEnd(uint8_t ctrl, const char *fname, off_t fsize) {
    if (ctrl != CTRL_START && ctrl != CTRL_END) return -1;
    unsigned char buf[APP_MAX_INFO_LEN];
    int n = buildStartEnd(buf, ctrl, fname, fsize);
    if (n < 0) return -1;
    hexdump(ctrl == CTRL_START ? "TX START" : "TX END", buf, (size_t)n);
    return (llwrite(buf, n) < 0) ? -1 : 0;
}

static int parseStartEnd(const unsigned char *in, size_t inLen, off_t *outSize, char *outName, size_t outNameCap) {
    if (!in || inLen < 1 || !outSize || !outName || outNameCap <= 0) return -1;
    if (in[0] != CTRL_START && in[0] != CTRL_END) return -1;

    *outSize = 0; outName[0] = '\0';

    size_t i = 1;
    int haveSize = 0, haveName = 0;
    while (i + 2 <= inLen) {
        uint8_t T = in[i++], L = in[i++];
        if (i + L > inLen) break;

        if (T == T_FILE_SIZE) {
            char tmp[64];
            size_t c = (L < sizeof(tmp) - 1) ? L : sizeof(tmp) - 1;
            memcpy(tmp, &in[i], c); tmp[c] = '\0';
            long long v = strtoll(tmp, NULL, 10);
            if (v < 0) v = 0;
            *outSize = (off_t)v; haveSize = 1;
        } else if (T == T_FILE_NAME) {
            size_t c = (L < outNameCap - 1) ? L : (outNameCap - 1);
            memcpy(outName, &in[i], c); outName[c] = '\0'; haveName = 1;
        }
        i += L;
    }
    return (haveSize && haveName) ? 0 : -1; // ensure both were present
}


// PACKET

static int buildDataPacket(unsigned char *dst, uint8_t seq, const unsigned char *payload, uint16_t len) {
    if (!dst || !payload) return -1;
    if (len > APP_CHUNK) return -1;
    dst[0] = CTRL_DATA;
    dst[1] = seq;
    dst[2] = (uint8_t)((len >> 8) & 0xFF);
    dst[3] = (uint8_t)( len       & 0xFF);
    memcpy(&dst[4], payload, len);
    return APP_DATA_HDR_SIZE + (int)len;
}

static int sendDataPacket(uint8_t seq, const unsigned char *payload, uint16_t len) {
    if (!payload || len > APP_CHUNK) return -1;
    unsigned char frame[APP_DATA_HDR_SIZE + APP_CHUNK];
    int n = buildDataPacket(frame, seq, payload, len);
    if (n < 0) return -1;
    //hexdump("TX DATA", frame, (size_t)n);
    int r = llwrite(frame, n);
    return (r < 0) ? -1 : (int)len;
}

static int parseDataPacket(const unsigned char *src, size_t srcLen, uint8_t *seq, const unsigned char **payload, uint16_t *len) {
    if (!src || srcLen < APP_DATA_HDR_SIZE || !seq || !payload || !len) return -1;
    if (src[0] != CTRL_DATA) return -1;
    *seq = src[1];
    *len = ((uint16_t)src[2] << 8) | (uint16_t)src[3];
    if ((size_t)APP_DATA_HDR_SIZE + *len > srcLen) return -1;
    *payload = &src[4];
    return 0;
}


// APLICATION


void applicationLayer(const char *serialPort, const char *role, int baudRate, int nTries, int timeout, const char *filename){
    LinkLayer ll = {0};
    strncpy(ll.serialPort, serialPort, sizeof(ll.serialPort) - 1);
    ll.role = (strcmp(role, "tx") == 0) ? LlTx : LlRx;
    ll.baudRate = baudRate;
    ll.nRetransmissions = nTries;
    ll.timeout = timeout;

    if (llopen(ll) < 0) {printf("[APP] llopen failed\n"); return;}

    // TRANSMITTER
    if (ll.role == LlTx) {

        // FILE SIZE
        struct stat st;
        if (stat(filename, &st) != 0) { perror("[APP][Tx] stat"); llclose(); return; }
        off_t fsize = st.st_size;

        // OPEN FILE
        int fd = open(filename, O_RDONLY);
        if (fd < 0) { perror("[APP][Tx] open"); llclose(); return; }


        // SEND START
        if (sendStartEnd(CTRL_START, filename, fsize) != 0) { 
            printf("[APP][Tx] failed to send START\n");
            close(fd);
            llclose();
            return;
        }

        // READ FILE
        unsigned char chunk[APP_CHUNK];
        uint8_t seq = 0;
        ssize_t rd;
        off_t sent = 0;

        while ((rd = read(fd, chunk, sizeof(chunk))) > 0) {
            if (sendDataPacket(seq, chunk, (uint16_t)rd) < 0) {
                printf("[APP][Tx] failed to send DATA (seq=%u)\n", seq);
                close(fd);
                llclose();
                return;
            }
            sent += rd;
            seq = (uint8_t)((seq + 1) & 0xFF);
        }

        if (rd < 0) { perror("[APP][Tx] read"); close(fd); llclose(); return; }

        // SEND END
        if (sendStartEnd(CTRL_END, filename, fsize) != 0) {
            printf("[APP][Tx] failed to send END\n");
            close(fd);
            llclose();
            return;
        }

        printf("[APP][Tx] Done. Sent %lld bytes.\n", (long long)sent);
        close(fd);
        (void)llclose();
    }

    // RECEIVER
    if (ll.role == LlRx) {
        unsigned char packet[APP_MAX_INFO_LEN];

        off_t expected_size = 0;
        off_t received = 0;
        char name_from_start[512] = {0};
        int have_start = 0;
        int out_fd = -1;
        uint8_t expected_seq = 0;

        while(1){

            // READ
            int n = llread(packet);
            if (n < 0) {
                printf("[APP][Rx] llread error (%d)\n", n);
                if (out_fd >= 0) close(out_fd);
                llclose();
                return;
            }
            if (n == 0) continue;

            unsigned char ctrl = packet[0];
            
            // START
            if (ctrl == CTRL_START) {
                off_t size_tmp = 0;
                char  name_tmp[sizeof name_from_start] = {0};

                // PARSE START
                if (parseStartEnd(packet, (size_t)n, &size_tmp, name_tmp, sizeof(name_tmp)) != 0) {
                    printf("[APP][Rx] malformed START\n");
                    if (out_fd >= 0) close(out_fd);
                    llclose();
                    return;
                }

                if (have_start) { printf("[APP][Rx] duplicate START ignored\n"); continue;}

                expected_size = size_tmp;
                const char *target = (filename && filename[0]) ? filename : (name_tmp[0] ? name_tmp : "received.bin"); //WAR

                out_fd = open(target, O_CREAT | O_TRUNC | O_WRONLY, 0644);
                if (out_fd < 0) {
                    perror("[APP][Rx] open(out)");
                    llclose();
                    return;
                }

                strncpy(name_from_start, target, sizeof(name_from_start) - 1);
                received = 0;
                expected_seq = 0;
                have_start = 1;

                printf("[APP][Rx] START ok. name=\"%s\" size=%lld\n", name_from_start, (long long)expected_size);
            }

            // DATA
            if (ctrl == CTRL_DATA) {
                if (!have_start) { printf("[APP][Rx] DATA before START dropping\n"); continue; }

                const unsigned char *payload = NULL;
                uint16_t len = 0;
                uint8_t seq = 0;

                // PARSE DATA
                if (parseDataPacket(packet, (size_t)n, &seq, &payload, &len) != 0) {
                    printf("[APP][Rx] malformed DATA\n");
                    if (out_fd >= 0) close(out_fd);
                    llclose();
                    return;
                }

                // CHECK SEQ
                if (seq != expected_seq) {
                    printf("[APP][Rx] WARN: expected seq=%u got=%u — resyncing\n", expected_seq, seq);
                    expected_seq = (uint8_t)(seq + 1);
                } else {
                    expected_seq = (uint8_t)(expected_seq + 1);
                }

                // WRITE TO FILE
                ssize_t wr = write(out_fd, payload, len);
                if (wr != (ssize_t)len) {
                    perror("[APP][Rx] write");
                    if (out_fd >= 0) close(out_fd);
                    llclose();
                    return;
                }
                received += len;
            }

            // END
            if (ctrl == CTRL_END) {
                if (!have_start) { printf("[APP][Rx] END before START aborting\n"); llclose(); return;}

                off_t size_chk = 0;
                char  name_chk[sizeof name_from_start] = {0};

                // PARSE END
                if (parseStartEnd(packet, (size_t)n, &size_chk, name_chk, sizeof(name_chk)) != 0) {
                    printf("[APP][Rx] malformed END\n");
                    if (out_fd >= 0) close(out_fd);
                    llclose();
                    return;
                }

                if (out_fd >= 0) {
                    if (fsync(out_fd) != 0) perror("[APP][Rx] fsync");
                    close(out_fd);
                    out_fd = -1;
                }

                printf("[APP][Rx] END ok. Received %lld / %lld bytes.\n",(long long)received, (long long)expected_size);
                llclose();
                return;
            }
        }

    }

}
