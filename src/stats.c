#include "stats.h"
#include <stdio.h>
#include <string.h>
#include <time.h>

static LlStats g_stats;

static double diff_s(struct timespec a, struct timespec b) {
    long s   = b.tv_sec  - a.tv_sec;
    long nsc = b.tv_nsec - a.tv_nsec;
    if (nsc < 0) { --s; nsc += 1000000000L; }
    return (double)s + (double)nsc / 1e9;
}

// lifecycle
void stats_start(LinkLayerRole role) {
    memset(&g_stats, 0, sizeof(g_stats));
    g_stats.role = role;
    clock_gettime(CLOCK_MONOTONIC, &g_stats.t_start);
}
void stats_stop(void) { clock_gettime(CLOCK_MONOTONIC, &g_stats.t_end); }
double stats_elapsed_seconds(void) { return diff_s(g_stats.t_start, g_stats.t_end); }

void stats_print(void) {
    stats_stop();
    double elapsed = stats_elapsed_seconds();

    printf("\n=========== STATISTICS ==========\n");
    printf("Role: %s\n", (g_stats.role == LlTx ? "Transmitter" : "Receiver"));
    printf("Elapsed: %.3f s\n\n", elapsed);

    if (g_stats.role == LlTx) {
        printf("-- Transmitter --\n");
        printf("Frames sent:%lu\n", g_stats.frames_sent_total);
        printf("Alarms fired:%lu\n", g_stats.alarms_fired);
        printf("\n");
        printf("I frames sent:%lu\n", g_stats.i_frames_sent);
        printf("I frames ACKed:%lu\n", g_stats.i_frames_acked);
        printf("I retransmissions:%lu\n", g_stats.i_frames_retr);
        printf("REJ received:%lu\n", g_stats.i_frames_rejs_received);
        printf("\n");
        printf("Control frames sent:%lu\n", g_stats.ctrl_frames_sent);
        printf("Control frames ACKed:%lu\n", g_stats.ctrl_frames_acked);
        printf("Control retransmissions:%lu\n", g_stats.ctrl_frames_retr);

        printf("\n");
        if (g_stats.i_frames_sent) {
            double succ = (double)g_stats.i_frames_acked / (double)g_stats.i_frames_sent * 100.0;
            double fps_acked  = (elapsed > 0.0) ? (double)g_stats.i_frames_acked / elapsed : 0.0;
            double fps = (elapsed > 0.0) ? (double)g_stats.frames_sent_total / elapsed : 0.0;
            printf("I success rate:%.1f%%\n", succ);
            printf("I frames/sec:%.2f\n", fps);
            printf("I frames/sec (ACKed):%.2f\n", fps_acked);
        }
    } else {
        printf("-- Receiver --\n");
        printf("Frames received:%lu\n", g_stats.frames_received_total);
        printf("\n");
        printf("I frames received:%lu\n", g_stats.i_frames_received);
        printf("I frames accepted:%lu\n", g_stats.i_frames_ok);
        printf("I duplicates received:%lu\n", g_stats.i_frames_dup_received);
        printf("REJ sent:%lu\n", g_stats.i_frames_rejs_sent);
        printf("\n");
        printf("Control frames received:%lu\n", g_stats.ctrl_frames_received);
        printf("Control handshakes ok:%lu\n", g_stats.ctrl_frames_ok);
    }
    printf("=================================\n\n");
}

// generic
void stats_frame_sent(void){ g_stats.frames_sent_total++; }
void stats_frame_recv(void){ g_stats.frames_received_total++; }
void stats_alarm(void){ g_stats.alarms_fired++; }

// TX — data (I)
void stats_tx_i_sent(void){ g_stats.i_frames_sent++; }
void stats_tx_i_retr(void){ g_stats.i_frames_retr++; }
void stats_tx_i_acked(void){ g_stats.i_frames_acked++; }
void stats_tx_i_rej_rx(void){ g_stats.i_frames_rejs_received++; }

// TX — control
void stats_tx_ctrl_sent(void){ g_stats.ctrl_frames_sent++; }
void stats_tx_ctrl_retr(void){ g_stats.ctrl_frames_retr++; }
void stats_tx_ctrl_ua_rx(void){ g_stats.ctrl_frames_acked++; }

// RX — data (I)
void stats_rx_i_seen(void){ g_stats.i_frames_received++; }
void stats_rx_i_ok(void){ g_stats.i_frames_ok++; }
void stats_rx_i_dup(void){ g_stats.i_frames_dup_received++; }
void stats_rx_i_rej_tx(void){ g_stats.i_frames_rejs_sent++; }

// RX — control
void stats_rx_ctrl_seen(void){ g_stats.ctrl_frames_received++; }
void stats_rx_ctrl_ok(void){ g_stats.ctrl_frames_ok++; }
