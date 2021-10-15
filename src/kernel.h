#ifndef __KERNEL_H
#define __KERNEL_H

// Definitions for BUTTON values to parse keyboard events.
const int BUTTON_DAW_UP    = 5;
const int BUTTON_DAW_DOWN  = 6;
const int BUTTON_DAW_LEFT  = 3;
const int BUTTON_DAW_RIGHT = 4;
const int BUTTON_DAW_CENT  = 2;
const int BUTTON_TAP_TEMPO = 0;
const int BUTTON_NOTEREPEAT= 1;
const int BUTTON_ARPEGIATE = 47;
const int BUTTON_LATCH     = 46;
const int BUTTON_FULL_LEVEL= 40;
const int BUTTON_16_LEVEL  = 41;
const int BUTTON_BANK_A    = 42;
const int BUTTON_BANK_B    = 43;
const int BUTTON_BANK_C    = 44;
const int BUTTON_BANK_D    = 45;

const int BUTTON_KNOB_LEFT = 70;
const int BUTTON_KNOB_RIGHT= 71;
const int BUTTON_KNOB_PUSH = 8;
const int BUTTON_SEL_UP    = 14;
const int BUTTON_SEL_DOWN  = 15;
const int BUTTON_SEL_LEFT  = 12;
const int BUTTON_SEL_RIGHT = 13;

const int BUTTON_PRESET    = 16;
const int BUTTON_EDIT      = 17;
const int BUTTON_GLOBAL    = 18;
const int BUTTON_PROG_CHA  = 19;
const int BUTTON_PREVIEW   = 30;
const int BUTTON_LOOP      = 29;
const int BUTTON_REWIN     = 24;
const int BUTTON_FFWARD    = 25;
const int BUTTON_STOP      = 26;
const int BUTTON_PLAY      = 27;
const int BUTTON_REC       = 28;
const int BUTTON_OCT_DOWN  = 20;
const int BUTTON_OCT_UP    = 21;
const int BUTTON_CONTROL_A = 9;
const int BUTTON_CONTROL_B = 10;
const int BUTTON_CONTROL_C = 11;
const int BUTTON_TIME_DIV  = 22;

const int BUTTON_S1        = 32;
const int BUTTON_S2        = 33;
const int BUTTON_S3        = 34;
const int BUTTON_S4        = 35;
const int BUTTON_S5        = 36;
const int BUTTON_S6        = 37;
const int BUTTON_S7        = 38;
const int BUTTON_S8        = 39;




int start(void);








#define DELAY 1500  // in millis

#endif
