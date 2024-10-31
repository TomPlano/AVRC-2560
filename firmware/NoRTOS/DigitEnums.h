#ifndef DIGITENUMS_H
#define DIGITENUMS_H


#define ASCII_NULL 0
#define ASCII_0 48
#define ASCII_9 57
#define ASCII_A 65
#define ASCII_F 70// Wire Master Writer

#define ASCII_Z 90
#define ASCII_HEX_SKIP (ASCII_A - ASCII_9 - 1)
#define CHARSET_SIZE 86

  
  typedef enum digit_states{
    OFF_STATE,
    ON
  } digit_state;

  typedef enum digit_modes{
    OFF_MODE,
    DECI,
    HEXI,
    ALPHA
  } digit_mode;
  
  typedef enum digitOps{
    ADD,
    SUB
  }digitOp;

enum ascii_bound{
  ascii_null =  ASCII_NULL,
  ascii_0 = ASCII_0,
  decimal = ASCII_9,
  ascii_A = ASCII_A,
  hex = ASCII_F,
  alpha = ASCII_Z
};

#endif //DIGITENUMS_H
