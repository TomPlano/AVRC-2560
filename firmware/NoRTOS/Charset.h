#ifndef CHARSET_H
#define CHARSET_H

#include "DigitEnums.h"

  byte alphanum_charset[CHARSET_SIZE] = {
  0b00111111,0b010001,    //0
  0b110,0b1,              //1
  0b11011,0b100010,       //2
  0b1111,0b10,            //3
  0b000100110,0b100010,   //4
  0b000101101,0b100010,   //5
  0b000111101,0b100010,   //6
  0b00000001,0b10001,     //7
  0b111111,0b100010,      //8
  0b100111,0b100010,      //9
  0b0, 0b0,               //unsupported ascii gap
  0b0, 0b0,
  0b0, 0b0,
  0b0, 0b100010,      //- Dash
  0b0, 0b0,
  0b0, 0b0,
  0b0, 0b0,
  0b110111,0b100010,      //A
  0b10001111,0b1010,      //B
  0b111001,0b0,           //C
  0b10001111,0b1000,      //D
  0b111001,0b0100000,     //E
  0b110001,0b0100000,     //F
  0b111101,0b10,          //G
  0b110110,0b100010,      //H
  0b10001001,0b1000,      //I
  0b00011110, 0b0,        //J
  0b00110000,0b100101,    //K
  0b111000,0b0,           //L
  0b01110110,0b1,         //M
  0b01110110,0b100,       //N
  0b00111111,0b0,         //O
  0b110011,0b100010,      //P
  0b00111111,0b100,       //Q
  0b110011,0b100110,      //R
  0b000101101,0b100010,   //S
  0b10000001,0b1000,      //T
  0b00111110,0b0,         //U
  0b110000,0b10001,       //V
  0b110110,0b10100,       //W
  0b01000000,0b010101,    //X
  0b01000000,0b001001,    //X
  0b1001,0b10001,         //Z
  };

  void  byte2string(byte val, char* str, byte len) {
    byte devisor = 1;
    byte clean_val = val;
    switch(len){
      case 2:
      devisor = 0x10;
      break;
      case 3:
      devisor = 10;
      break;
      default:
      len = 1;
      break;
    }
    for (byte i=len; i>0; --i ){
      byte idx = i-1;
      str[idx] = (clean_val % devisor);
      clean_val =  (clean_val - str[idx])/devisor;
      str[idx] += ASCII_0;
      str[idx] += ((len ==2)&& str[idx] >ASCII_9 && str[idx] <ASCII_A) ? ASCII_HEX_SKIP : 0;
    }
  }
  
  byte add(byte a,byte b){ return a+b;}
  byte sub(byte a,byte b){ return a-b;}

  int tableGapSkip_withCarry(byte* candidate_char, digitOp op,  digit_mode mode){
    bool carry =false;
    
    //add or subtract 1
    switch(op){
      case ADD:
          *candidate_char+=1;
      case SUB:
          *candidate_char-=1;
      default:
        break;
     }
    //set up the modulus
    ascii_bound rollover = ascii_null;
    switch (mode){
      case DECI:
        rollover = decimal;
        break;
      case HEXI:
        rollover = hex;
        break;
      case ALPHA:
        rollover = alpha;
        break;
      case OFF_MODE:
      default:
        break;
    };

    //compute the new char and set carry
    //underflow 
    if(*candidate_char>=ASCII_NULL && *candidate_char<ASCII_0){
      *candidate_char = (op == ADD)? ASCII_0 : rollover;
      carry = true;
    }
    //midskip
    if(*candidate_char>ASCII_9 && *candidate_char<ASCII_A){
      *candidate_char = (op == ADD)? ASCII_A : ASCII_9;
    }
    return carry;
  }

#endif /*CHARSET_H*/
