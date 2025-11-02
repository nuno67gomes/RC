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
#include "stats.h"

#define INFO_MAX 1024
#define FRAME_OVERHEAD 5 // FLAG,A,C,BCC1,FLAG
#define STUFF_GROWTH_MAX  2 // worst-case expansion factor
#define STUFFED_INFO_MAX (STUFF_GROWTH_MAX * (INFO_MAX+1)) // +1=BCC2
#define FRAME_MAX (FRAME_OVERHEAD + STUFFED_INFO_MAX)

//state after llopen()
static LinkLayer g_cfg;
static unsigned char g_txNs = 0;
static unsigned char g_rxExpectedNs = 0;
static int g_disc_already_seen = 0;


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
    int want_retry = retry && (maxRetries > 0) && (fsm != NULL);
    if(!want_retry){
        if (writeBytesSerialPort(frame, frameSize) < 0) return -2;
        stats_frame_sent();
        stats_tx_ctrl_sent();
        return 0;
    }

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

    for (int tries = 0; tries < maxRetries; ++tries) {
        printf("[DEBUG] Sending C=0x%02X, try #%d\n", sentControl, tries + 1);
        if (writeBytesSerialPort(frame, frameSize) < 0) return -2;
        
        stats_frame_sent();
        if (isI) { stats_tx_i_sent(); if (tries > 0) stats_tx_i_retr();} 
        else { stats_tx_ctrl_sent(); if (tries > 0) stats_tx_ctrl_retr();}

        alarmEnabled = 1;
        alarm(timeout_s);
        fsm_reset(fsm);

        while (alarmEnabled && !fsm_is_done(fsm)){
            int r = readByteSerialPort(&byte);
            if (r!=1){ continue;}

            if (fsm_feed(fsm, byte)) {
                unsigned char aReceived = fsm->address;
                unsigned char cReceived = fsm->control;

                //Handshake
                if(!isI){
                    if(sentControl == C_SET){
                        if(cReceived== C_UA  && aReceived == A_TX ){
                            stats_tx_ctrl_ua_rx();
                            alarm(0); alarmEnabled = 0;
                            return cReceived;
                        }
                        fsm_reset(fsm);
                    }else if(sentControl == C_DISC){
                        if((cReceived == C_DISC && aReceived == A_RX) || (cReceived == C_UA && aReceived == A_RX)){
                            if (cReceived == C_DISC && aReceived == A_RX) stats_tx_ctrl_ua_rx(); 
                            if (cReceived == C_UA) stats_tx_ctrl_ua_rx();
                            alarm(0); alarmEnabled = 0;
                            return cReceived;
                        }
                        fsm_reset(fsm);
                    }else {
                        if(cReceived == C_UA  && aReceived == A_TX){
                            stats_tx_ctrl_ua_rx();
                            alarm(0); alarmEnabled = 0;
                            return cReceived;
                        }
                        fsm_reset(fsm);
                    }
                }

                //I-Frame
                if(isI){
                    if (cReceived == expected_rr  && aReceived == A_TX) {
                        stats_tx_i_acked();
                        alarm(0); alarmEnabled = 0;
                        return cReceived;
                    }
                    else if ((cReceived == C_REJ0 || cReceived == C_REJ1) && aReceived == A_TX) {
                        stats_tx_i_rej_rx();
                        alarm(0); alarmEnabled = 0;
                        break;
                    }
                    if (cReceived == C_DISC && aReceived == A_RX) {
                        alarm(0); alarmEnabled = 0;
                        return -1;
                    }
                    fsm_reset(fsm);
                }
            }
        }
        if (!fsm_is_done(fsm) && alarmCount > 0) {stats_alarm();}
    }

    alarm(0);
    alarmEnabled = 0;
    return -1;
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters){
    if (openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate) < 0) return -2;
    stats_start(connectionParameters.role);
    //Transmitter
    if (connectionParameters.role == LlTx) {
        unsigned char F[5]; buildCtrlFrame(F, A_TX, C_SET);
        FrameFSM fsm; fsm_init(&fsm);
        int r = sendFrame(F, 5, C_SET, &fsm, connectionParameters.timeout, connectionParameters.nRetransmissions);

        if (r == C_UA) {
            printf("[llopen][Tx] UA received. Link established.\n");
            g_cfg = connectionParameters;
            g_txNs = 0; g_rxExpectedNs = 0; g_disc_already_seen = 0;
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
        unsigned char F[5]; buildCtrlFrame(F, A_TX, C_UA);
        FrameFSM fsm; fsm_init(&fsm);
        unsigned char byte;

        while (1) {
            int rB = readByteSerialPort(&byte);
            if (rB < 0) { printf("[llopen][Rx] Read error while waiting for SET.\n"); closeSerialPort(); return -2; }
            if (rB == 1 && fsm_feed(&fsm, byte)) {
                if (fsm.control == C_SET && fsm.address == A_TX) {
                    stats_frame_recv();
                    stats_rx_ctrl_seen();
                    stats_rx_ctrl_ok();
                    if (sendFrame(F, 5, C_UA, NULL, 0, 0) < 0) { closeSerialPort(); return -2;}
                    printf("[llopen][Rx] UA sent. Link established.\n");
                    g_cfg = connectionParameters; 
                    g_txNs = 0; g_rxExpectedNs = 0; g_disc_already_seen = 0;
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
    if (g_cfg.role != LlTx) return -2;
    if (bufSize > INFO_MAX) return -2;

    const unsigned char C = (g_txNs == 0) ? C_I0 : C_I1;
    const unsigned char A = A_TX;
    const unsigned char BCC1 = (unsigned char)(A ^ C);

    //add bcc2
    unsigned char tmp[INFO_MAX + 1];
    memcpy(tmp, buf, bufSize);
    tmp[bufSize] = compute_bcc2(buf, bufSize);

    //stuff
    unsigned char stuffed[STUFFED_INFO_MAX];
    int stuffed_len = stuffBytes(tmp, bufSize + 1, stuffed);


    //add FLAG A C BCC1 DATA FLAG together
    unsigned char frame[FRAME_MAX];
    int k = 0;
    frame[k++] = FLAG;
    frame[k++] = A;
    frame[k++] = C;
    frame[k++] = BCC1;
    memcpy(&frame[k], stuffed, stuffed_len); k += stuffed_len;
    frame[k++] = FLAG;

    //send frame
    FrameFSM fsm; fsm_init(&fsm);
    int r = sendFrame(frame, k, C, &fsm, g_cfg.timeout, g_cfg.nRetransmissions);

    if ((g_txNs == 0 && r == C_RR1) || (g_txNs == 1 && r == C_RR0)) {
        g_txNs ^= 1;
        return bufSize;
    }

    if (r == -2) return -2; 
    return -1;
}



static int destuffBytes(const unsigned char *in, int n, unsigned char *out, int out_cap) {
    int k = 0;
    for (int i = 0; i < n; ++i) {
        unsigned char b = in[i];
        if (b == ESC) {
            if (i + 1 >= n) return -1;
            unsigned char d = (unsigned char)(in[++i] ^ 0x20);
            if (k >= out_cap) return -2;
            out[k++] = d;
        } else {
            if (k >= out_cap) return -2;
            out[k++] = b;
        }
    }
    return k;
}

static int sendRR(unsigned char expectNextNr) {
    unsigned char rrC = (expectNextNr == 0) ? C_RR0 : C_RR1;
    unsigned char f[5]; buildCtrlFrame(f, A_TX, rrC);
    return sendFrame(f, 5, rrC, NULL, 0, 0);
}

static int sendREJ(unsigned char expectNextNr) {
    unsigned char rejC = (expectNextNr == 0) ? C_REJ0 : C_REJ1;
    unsigned char f[5]; buildCtrlFrame(f, A_TX, rejC);
    printf("[DEBUG] Reject sent \n");
    return sendFrame(f, 5, rejC, NULL, 0, 0);
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet){  
    if (!packet || g_cfg.role != LlRx) return -2;
    FrameFSM fsm; fsm_init(&fsm);

    while(1){
        unsigned char byte;
        int r = readByteSerialPort(&byte);
        if (r < 0) return -2;
        if (r == 0) continue;

        if (!fsm_feed(&fsm, byte)) continue;
        stats_frame_recv();

        const unsigned char A = fsm.address;
        const unsigned char C = fsm.control;

        if (A == A_TX && C == C_DISC) { g_disc_already_seen = 1; return -1; } // peer wants to close
        if (C!=C_I0 && C!=C_I1) { fsm_reset(&fsm); continue;}
        if (A != A_TX) { fsm_reset(&fsm); continue; }
        stats_rx_i_seen();
        if (fsm.data_size < 1) { stats_rx_i_rej_tx(); (void)sendREJ(g_rxExpectedNs); fsm_reset(&fsm); continue; } // messages need bbc2 min
        if (fsm.data_size > STUFFED_INFO_MAX) { stats_rx_i_rej_tx(); (void)sendREJ(g_rxExpectedNs); fsm_reset(&fsm); continue; }

        unsigned char destuffed[INFO_MAX + 1];
        int m = destuffBytes(fsm.data, fsm.data_size, destuffed, (int)sizeof(destuffed));
        if (m < 1) { stats_rx_i_rej_tx(); (void)sendREJ(g_rxExpectedNs); fsm_reset(&fsm); continue; }

        const int payload_len = m - 1;
        const unsigned char bcc2_rx   = destuffed[m - 1];
        const unsigned char bcc2_calc = compute_bcc2(destuffed, payload_len);
        if (bcc2_rx != bcc2_calc) { stats_rx_i_rej_tx(); (void)sendREJ(g_rxExpectedNs); fsm_reset(&fsm); continue; }

        const unsigned char ns = (C == C_I0) ? 0 : 1;
        if (ns != g_rxExpectedNs) { stats_rx_i_dup(); (void)sendRR(g_rxExpectedNs); fsm_reset(&fsm); continue; } //Duplicate or out-of-order
        
        if (payload_len > 0) memcpy(packet, destuffed, payload_len);

        stats_rx_i_ok();
        (void)sendRR(g_rxExpectedNs ^ 1);
        g_rxExpectedNs ^= 1;
        return payload_len;
    }
    return 0;
}



////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose() {
    // The logic here:
    // Tx: send DISC -> wait DISC -> send UA -> close
    // Rx: wait DISC -> send DISC -> wait UA -> close

    // TRANSMITTER
    if (g_cfg.role == LlTx) {
        unsigned char disc[5];
        buildCtrlFrame(disc, A_TX, C_DISC);

        FrameFSM fsm;
        fsm_init(&fsm);

        int r = sendFrame(disc, 5, C_DISC, &fsm, g_cfg.timeout, g_cfg.nRetransmissions);
        if (r == C_DISC) {
            // Other guy answered DISC; send final UA
            unsigned char ua[5];
            buildCtrlFrame(ua, A_RX, C_UA);
            if (sendFrame(ua, 5, C_UA, NULL, 0, 1) < 0) {
                printf("[llclose][Tx] Error sending UA.\n");
                closeSerialPort();
                return -2;
            }
            printf("[llclose][Tx] DISC↔DISC completed. UA sent. Link closed.\n");
            closeSerialPort();
            g_txNs = 0; g_rxExpectedNs = 0; g_disc_already_seen = 0;
            stats_print();
            return 0;
        }
        if (r == -2) {
            printf("[llclose][Tx] I/O error during close.\n");
            closeSerialPort();
            return -2;
        }

        printf("[llclose][Tx] Timeout or unexpected control 0x%02X during close.\n", r & 0xFF);
        closeSerialPort();
        return -1;
    }

    // RECEIVER
    if (g_cfg.role == LlRx) {
        // Wait for DISC from the transmitter
        FrameFSM fsm;
        fsm_init(&fsm);

        // If llread already saw DISC, skip waiting and answer now.
        if (g_disc_already_seen) {
            unsigned char disc[5];
            buildCtrlFrame(disc, A_RX, C_DISC);

            int r = sendFrame(disc, 5, C_DISC, &fsm, g_cfg.timeout, g_cfg.nRetransmissions);
            if (r == C_UA) {
                printf("[llclose][Rx] UA received. Link closed.\n");
                closeSerialPort();
                g_txNs = 0; g_rxExpectedNs = 0; g_disc_already_seen = 0;
                stats_print();
                return 0;
            }
            if (r == -1) {
                printf("[llclose][Rx] Timeout waiting for UA after DISC. Forcing close.\n");
                closeSerialPort();
                g_txNs = 0; g_rxExpectedNs = 0; g_disc_already_seen = 0;
                stats_print();
                return -1;
            }
            if (r == -2) {
                printf("[llclose][Rx] I/O error while waiting for UA. Forcing close.\n");
                closeSerialPort();
                return -2;
            }
            // Any other control is unexpected -> force close
            printf("[llclose][Rx] Unexpected control 0x%02X while waiting for UA. Forcing close.\n", (unsigned)(r & 0xFF));
            closeSerialPort();
            g_txNs = 0; g_rxExpectedNs = 0; g_disc_already_seen = 0;
            stats_print();
            return -1;
        }


        unsigned char byte;
        while (1) {
            int rb = readByteSerialPort(&byte);
            if (rb < 0) {
                printf("[llclose][Rx] Read error while waiting for DISC.\n");
                closeSerialPort();
                return -2;
            }
            if (rb == 1 && fsm_feed(&fsm, byte)) {
                if (fsm.control == C_DISC && fsm.address == A_TX) {
                    // Reply with DISC
                    unsigned char disc[5];
                    buildCtrlFrame(disc, A_RX, C_DISC);
                    
                    fsm_reset(&fsm);
                    int r = sendFrame(disc, 5, C_DISC, &fsm, g_cfg.timeout, g_cfg.nRetransmissions);
                    if (r == C_UA) {
                        stats_frame_recv();
                        stats_rx_ctrl_seen();
                        stats_rx_ctrl_ok();
                        printf("[llclose][Rx] UA received. Link closed.\n");
                        closeSerialPort();
                        g_txNs = 0; g_rxExpectedNs = 0; g_disc_already_seen = 0;
                        stats_print();
                        return 0;
                    }
                    if (r == -1) {
                        printf("[llclose][Rx] Timeout waiting for UA after DISC. Forcing close.\n");
                        closeSerialPort();
                        g_txNs = 0; g_rxExpectedNs = 0; g_disc_already_seen = 0;
                        stats_print();
                        return -1;
                    }
                    if (r == -2) {
                        printf("[llclose][Rx] I/O error while waiting for UA. Forcing close.\n");
                        closeSerialPort();
                        return -2;
                    }
                    printf("[llclose][Rx] Unexpected control 0x%02X while waiting for UA. Forcing close.\n", (unsigned)(r & 0xFF));
                    closeSerialPort();
                    g_txNs = 0; g_rxExpectedNs = 0; g_disc_already_seen = 0;
                    stats_print();
                    return -1;
                }
                // Not DISC; reset state machine and continue waiting
                fsm_reset(&fsm);
            }
        }
    }

    closeSerialPort();
    return -1;
}