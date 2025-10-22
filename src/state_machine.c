#include "state_machine.h"

#define IS_IFRAME(c) ((c) == C_I0 || (c) == C_I1)
#define IS_SUPERVISION(c) ((c) == C_RR0 || (c) == C_RR1 || (c) == C_REJ0 || (c) == C_REJ1)
#define IS_UNNUMBERED(c) ((c) == C_SET || (c) == C_UA || (c) == C_DISC)
#define IS_CONTROL_FRAME(c) (IS_SUPERVISION(c) || IS_UNNUMBERED(c))

void fsm_init(FrameFSM *fsm) {
    fsm->state = STATE_START;
    fsm->data_size = 0;
}

int fsm_feed(FrameFSM *fsm, uint8_t byte) {
    if (fsm->state == STATE_STOP) return 1; 

    switch (fsm->state) {
        case STATE_START:
            if (byte == FLAG) {
                fsm->state = STATE_FLAG_RCV;
                fsm->data_size = 0;
            }
            break;

        case STATE_FLAG_RCV:
            if (byte == FLAG) break;
            if(byte == A_RX || byte == A_TX){
                fsm->address = byte;
                fsm->state = STATE_A_RCV;
            } else {
                fsm->state = STATE_START;
            }
            break;

        case STATE_A_RCV:
            if (byte == FLAG) fsm->state = STATE_FLAG_RCV;
            else {
                fsm->control = byte;
                fsm->bcc1 = fsm->address ^ fsm->control;
                fsm->state = STATE_C_RCV;
            }
            break;

        case STATE_C_RCV:
            if (byte == FLAG) fsm->state = STATE_FLAG_RCV;
            else if (byte == fsm->bcc1) fsm->state = STATE_BCC1_OK;
            else fsm->state = STATE_START;
            break;

        case STATE_BCC1_OK:
            if (IS_IFRAME(fsm->control)) {
                if (byte == FLAG){
                    fsm->state = STATE_STOP;
                } else {
                    fsm->data[fsm->data_size++] = byte;
                    fsm->state = STATE_DATA;
                }
            }else if (IS_CONTROL_FRAME(fsm->control)){
                if (byte == FLAG) fsm->state = STATE_STOP;
                else fsm->state = STATE_START;
            }
            break;

        case STATE_DATA:
            if (byte == FLAG) fsm->state = STATE_STOP;
            else if (fsm->data_size < (int)sizeof(fsm->data)) {
                fsm->data[fsm->data_size++] = byte;
            } else {
                fsm->state = STATE_START;
                fsm->data_size = 0;
            }
            break;

        case STATE_STOP:
            break;
    }

    return fsm->state == STATE_STOP;
}

int fsm_is_done(const FrameFSM *fsm) {
    return fsm->state == STATE_STOP;
}

void fsm_reset(FrameFSM *fsm) {
    fsm_init(fsm);
}
