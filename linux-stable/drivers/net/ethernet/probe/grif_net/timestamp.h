#ifndef TIMESTAMP__H
#define TIMESTAMP__H

// Rx timestamp passed in 2 first 8-bytes words.
// So it need 16 additional bytes 
#define TS_SIZE      (16)

// But really used only 10 bytes. 
// Timestamp format is 80 bit HW ticks (8 ns).
#define TS_SIZE_REAL (10)

// For converting 8 ns HW ticks in ns.
#define TICKS_NS_SHIFT (3)

#endif 
