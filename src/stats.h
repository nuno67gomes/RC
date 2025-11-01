#ifndef STATS_H
#define STATS_H

#include <time.h>
#include "link_layer.h" 

typedef struct {
    // Common
    unsigned long frames_sent_total;
    unsigned long frames_received_total;
    unsigned long alarms_fired;

    // TX-only
    unsigned long i_frames_sent;
    unsigned long i_frames_acked;
    unsigned long i_frames_retr;
    unsigned long i_frames_rejs_received;
    
    unsigned long ctrl_frames_sent;
    unsigned long ctrl_frames_acked;
    unsigned long ctrl_frames_retr;

    // RX-only
    unsigned long i_frames_received;
    unsigned long i_frames_ok;
    unsigned long i_frames_dup_received;
    unsigned long i_frames_rejs_sent;

    unsigned long ctrl_frames_received;
    unsigned long ctrl_frames_ok;


    struct timespec t_start, t_end;
    LinkLayerRole role;
} LlStats;

// lifecycle
void stats_start(LinkLayerRole role);
void stats_stop(void);
double stats_elapsed_seconds(void);
void stats_print(void);

// generic
void stats_frame_sent(void);
void stats_frame_recv(void);
void stats_alarm(void);

// TX — data (I)
void stats_tx_i_sent(void);
void stats_tx_i_retr(void);
void stats_tx_i_acked(void);
void stats_tx_i_rej_rx(void);

// TX — control
void stats_tx_ctrl_sent(void);
void stats_tx_ctrl_retr(void);
void stats_tx_ctrl_ua_rx(void);

// RX — data (I)
void stats_rx_i_seen(void);
void stats_rx_i_ok(void);
void stats_rx_i_dup(void);
void stats_rx_i_rej_tx(void);

// RX — control
void stats_rx_ctrl_seen(void);
void stats_rx_ctrl_ok(void);

#endif
