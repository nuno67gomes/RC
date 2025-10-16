// Link layer protocol implementation


#define _POSIX_SOURCE 1 // POSIX compliant source

#include <stdio.h>
#include <string.h>
#include <signal.h> 
#include <unistd.h>  

#include "link_layer.h"
#include "serial_port.h"
#include "timer.h"
#include "state_machine.h"

#define MAX_INFO_LEN 1024
#define MAX_FRAME_SIZE (2*MAX_INFO_LEN + 7)

//state after llopen()
static int g_timeout_s = 0;
static int g_maxRetries = 0;
static unsigned char g_txNs = 0;
static LinkLayerRole g_role = LlTx;



static void buildCtrlFrame(unsigned char *ctrlFrame, unsigned char A, unsigned char C) {
    ctrlFrame[0] = FLAG;
    ctrlFrame[1] = A;
    ctrlFrame[2] = C;
    ctrlFrame[3] = (unsigned char)(A ^ C);
    ctrlFrame[4] = FLAG;
}


static int sendFrame(const unsigned char *frame, int frameSize, unsigned char sentControl, FrameFSM *fsm, int timeout_s, int maxRetries){
    if(!frame || frameSize <=0) return -2;
    int retry = (sentControl == C_SET) || (sentControl == C_DISC) || (sentControl == C_I0)  || (sentControl == C_I1);
    if(!retry) return (writeBytesSerialPort(frame, frameSize) < 0) ? -2 : 0;
    if (!fsm) return -2;

    //Alarm
    static int handler_installed = 0;
    if (!handler_installed) {
        struct sigaction act;
        memset(&act, 0, sizeof(act));
        act.sa_handler = alarmHandler;
        if (sigaction(SIGALRM, &act, NULL) == -1) return -2;
        handler_installed = 1;
    }

    const int isI = (sentControl == C_I0 || sentControl == C_I1);
    unsigned char expected_rr = 0x00;
    if (isI) { expected_rr = (sentControl == C_I0) ? C_RR1 : C_RR0;}

    unsigned char byte;
    alarmCount = 0;


    for (int tries = 0; tries <= maxRetries; ++tries) {
        printf("[DEBUG] Sending C=0x%02X, try #%d\n", sentControl, tries + 1);
        if (writeBytesSerialPort(frame, frameSize) < 0) return -2;
        alarmEnabled = 1;
        alarm(timeout_s);
        fsm_reset(fsm);

        while (alarmEnabled && !fsm_is_done(fsm)){
            int r = readByteSerialPort(&byte);
            if (r <0){ alarm(0); alarmEnabled = 0; return -2;}
            if(r == 0){ continue;}

            if (fsm_feed(fsm, byte)) {
                unsigned char aReceived = fsm->address;
                unsigned char cReceived = fsm->control;

                //Handshake
                if(!isI){
                    if(sentControl== C_SET){
                        if(cReceived== C_UA  && aReceived == A_RX ){
                            alarm(0); alarmEnabled = 0;
                            return cReceived;
                        }
                        fsm_reset(fsm);
                    }else if(sentControl== C_DISC){
                        if((cReceived== C_DISC && aReceived == A_RX) || (cReceived== C_UA && aReceived == A_RX)){
                            alarm(0); alarmEnabled = 0;
                            return cReceived;
                        }
                        fsm_reset(fsm);
                    }else {
                        if(cReceived== C_UA  && aReceived == A_RX){
                            alarm(0); alarmEnabled = 0;
                            return cReceived;
                        }
                        fsm_reset(fsm);
                    }
                }

                //I-Frame
                if(isI){
                    if (cReceived == expected_rr  && aReceived == A_RX) {
                        alarm(0); alarmEnabled = 0;
                        return cReceived;
                    }
                    else if (cReceived == C_REJ0 || cReceived == C_REJ1) {
                        alarm(0); alarmEnabled = 0;
                        break;
                    }
                    if (cReceived == C_DISC) {
                        alarm(0); alarmEnabled = 0;
                        return -1;
                    }
                    fsm_reset(fsm);
                }
            }
        }
    }
    return -1;
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters){
    if (openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate) < 0) return -2;

    //Transmitter
    if (connectionParameters.role == LlTx) {
        unsigned char F[5]; buildCtrlFrame(F, A_TX, C_SET);
        FrameFSM fsm; fsm_init(&fsm);
        int r = sendFrame(F, 5, C_SET, &fsm, connectionParameters.timeout, connectionParameters.nRetransmissions);

        if (r == C_UA) {
            printf("[llopen][Tx] UA received. Link established.\n");
            g_timeout_s = connectionParameters.timeout; g_maxRetries = connectionParameters.nRetransmissions; g_role = connectionParameters.role; g_txNs = 0;
            return 0;

        } else if (r == -1) { printf("[llopen][Tx] Timeout waiting for UA after %d attempt(s). Aborting.\n", connectionParameters.nRetransmissions);
        } else if (r == -2) { printf("[llopen][Tx] I/O error while sending SET or waiting for reply.\n");
        } else if (r == C_DISC) { printf("[llopen][Tx] Received DISC during open (peer is closing). Aborting.\n");
        } else if (r >= 0) { printf("[llopen][Tx] Unexpected control 0x%02X received during open. Aborting.\n", r);
        } else { printf("[llopen][Tx] Unknown error (%d). Aborting.\n", r); }

        closeSerialPort();
        return -1;
    }
    
    //Receiver
    if (connectionParameters.role == LlRx) {
        unsigned char F[5]; buildCtrlFrame(F, A_RX, C_UA);
        FrameFSM fsm; fsm_init(&fsm);
        unsigned char byte;

        while (1) {
            int rB = readByteSerialPort(&byte);
            if (rB < 0) { printf("[llopen][Rx] Read error while waiting for SET.\n"); closeSerialPort(); return -1; }
            if (rB == 1 && fsm_feed(&fsm, byte)) {
                if (fsm.control == C_SET && fsm.address == A_TX) {
                    if (sendFrame(F, 5, C_UA, NULL, 0, 1) < 0) { closeSerialPort(); return -1;}
                    printf("[llopen][Rx] UA sent. Link established.\n");
                    g_timeout_s = connectionParameters.timeout; g_maxRetries = connectionParameters.nRetransmissions; g_role = connectionParameters.role; g_txNs = 0;
                    return 0;
                }
                fsm_reset(&fsm);
            }
        }
    }
    
    closeSerialPort();
    return -1;
}



static inline unsigned char compute_bcc2(const unsigned char *f, int n) {
    unsigned char bcc = 0x00;
    for (int i = 0; i < n; ++i) bcc ^= f[i];
    return bcc;
}

static int stuffBytes(const unsigned char *f, int fLen, unsigned char *out) {
    int j = 0;
    for (int i = 0; i < fLen; ++i) {
        unsigned char b = f[i];
        if (b == FLAG || b == ESC) {
            out[j++] = ESC;
            out[j++] = (unsigned char)(b ^ 0x20);
        } else {
            out[j++] = b;
        }
    }
    return j;
}



////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize){
    if (!buf || bufSize < 0) return -2;
    if (g_role != LlTx) return -2;
    if (bufSize > MAX_INFO_LEN) return -2;

    const unsigned char C = (g_txNs == 0) ? C_I0 : C_I1;
    const unsigned char A = A_TX;
    const unsigned char BCC1 = (unsigned char)(A ^ C);

    //add bcc2
    unsigned char tmp[MAX_INFO_LEN + 1];
    memcpy(tmp, buf, bufSize);
    tmp[bufSize] = compute_bcc2(buf, bufSize);

    //stuff
    unsigned char stuffed[(MAX_INFO_LEN + 1) * 2];
    int stuffed_len = stuffBytes(tmp, bufSize + 1, stuffed);


    //add FLAG A C BCC1 DATA FLAG together
    unsigned char frame[MAX_FRAME_SIZE];
    int k = 0;
    frame[k++] = FLAG;
    frame[k++] = A;
    frame[k++] = C;
    frame[k++] = BCC1;
    memcpy(&frame[k], stuffed, stuffed_len); k += stuffed_len;
    frame[k++] = FLAG;

    //send frame
    FrameFSM fsm; fsm_init(&fsm);
    int r = sendFrame(frame, k, C, &fsm, g_timeout_s, g_maxRetries);

    if ((g_txNs == 0 && r == C_RR1) || (g_txNs == 1 && r == C_RR0)) {
        g_txNs ^= 1;
        return bufSize;
    }

    if (r == -2) return -2; 
    return -1;
}


////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet){
    
    
    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose()
{
    // TODO: Implement this function

    return 0;
}
