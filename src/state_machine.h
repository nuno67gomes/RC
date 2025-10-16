#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <stdint.h>

#define FLAG 0x7E
#define ESC  0x7D

#define A_TX 0x03
#define A_RX 0x01

#define C_SET  0x03
#define C_UA   0x07
#define C_DISC 0x0B
#define C_RR0  0x05
#define C_RR1  0x85
#define C_REJ0 0x01
#define C_REJ1 0x81
#define C_I0   0x00
#define C_I1   0x40

typedef enum {
    STATE_START,
    STATE_FLAG_RCV,
    STATE_A_RCV,
    STATE_C_RCV,
    STATE_BCC1_OK,
    STATE_DATA,
    STATE_STOP
} FrameState;

typedef struct {
    FrameState state;
    uint8_t address;
    uint8_t control;
    uint8_t bcc1;
    uint8_t data[1024];
    int data_size;
    int escape_next;
} FrameFSM;

void fsm_init(FrameFSM *fsm);
int fsm_feed(FrameFSM *fsm, uint8_t byte);
int fsm_is_done(const FrameFSM *fsm);
void fsm_reset(FrameFSM *fsm);

#endif
