#pragma once

// Master knob: maximum unstuffed INFO field size for I-frames (bytes)
#ifndef INFO_MAX
#define INFO_MAX 1024   // change via -DINFO_MAX=2048, etc.
#endif

// Link framing constants
#define FRAME_OVERHEAD     5  // FLAG,A,C,BCC1,FLAG
#define STUFF_GROWTH_MAX   2  // worst-case byte-stuffing expansion
#define STUFFED_INFO_MAX  (STUFF_GROWTH_MAX * (INFO_MAX + 1))  // +1 for BCC2
#define FRAME_MAX         (FRAME_OVERHEAD + STUFFED_INFO_MAX)

// App-layer uses the same INFO_MAX budget
#define APP_MAX_INFO_LEN   INFO_MAX
#define APP_DATA_HDR_SIZE  3   // CTRL_DATA + LEN_HI + LEN_LO
#define APP_CHUNK         (APP_MAX_INFO_LEN - APP_DATA_HDR_SIZE)
