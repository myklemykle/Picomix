// we can let individual includers of this file defined SDEBUG or not, and redef for each one ...
// if this gets more comples, revisit that  ...
// #ifndef __DBG_MACROS_H
// #define __DBG_MACROS_H


#ifdef SDEBUG
// Teensy will eventually hang on a clogged output buffer if we Serial.print() without USB plugged in.
// is there some more normal solution for this?  Seems like this would be a common problem.
// https://gcc.gnu.org/onlinedocs/cpp/Variadic-Macros.html 
#define Dbg_println(...) if(Serial) Serial.println(__VA_ARGS__)
#define Dbg_printf(...) if(Serial) Serial.printf(__VA_ARGS__)
#define Dbg_print(...) if(Serial) Serial.print(__VA_ARGS__)
#define Dbg_flush(X) if(Serial) Serial.flush()

#else
#define Dbg_println(...) {}
#define Dbg_printf(...) {}
#define Dbg_print(...) {}
#define Dbg_flush(X) {}
#endif

// maybe some more sophisitcated eception handling/logging someday?
// #define Dbg_warn Dbg_printf
#define Dbg_warn(...) if(Serial) { Serial.printf(__VA_ARGS__); Serial.flush(); }




//#endif
