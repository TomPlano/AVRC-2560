#ifndef DIGITDISPLAY_H
#define DIGITDISPLAY_H

#include "Charset.h"
#include "DigitEnums.h"


class DigitDisplay {
  public:
  DigitDisplay(byte* in_digitLEDSpace){
    digitLEDSpace = in_digitLEDSpace;
    populateDisplay();    
  }

  void togglePower(){
    switch(current_state){
      case OFF_STATE:
          current_mode = on_default_current_mode;
          displayed_char = on_default_char;
          displayed_LED_low = on_default_LED_low;
          displayed_LED_hi = on_default_LED_hi;
          current_state = ON;   
          collectBitsForDisplay();
        break;
      
      case ON:
          current_mode = off_default_current_mode;
          displayed_char = off_default_char;
          displayed_LED_low = off_default_LED_low;
          displayed_LED_hi = off_default_LED_hi;
          current_state = OFF_STATE;
        break;
    }
    populateDisplay();    
  }

  void setMode(digit_mode new_mode){
    current_mode = new_mode;
  }
  
  void setChar(char ch) {
    displayed_char = ch;
    collectBitsForDisplay();
    populateDisplay();    
  }
  bool incdec(digitOp op){
    //add 1 to ch, avoid the invalid table region, return carry
    bool carry = tableGapSkip_withCarry(&displayed_char, op, current_mode);
    collectBitsForDisplay();
    populateDisplay();    
    return carry;
  }

  void collectBitsForDisplay(){
    if (current_state!=OFF_STATE){
      byte ascii_char = displayed_char - ASCII_0;
      displayed_LED_low = alphanum_charset[2*ascii_char];
      displayed_LED_hi = alphanum_charset[(2*ascii_char)+1];
    }
  }

  void populateDisplay(){
  //  read table based on stored char, popultate 2 bytes starting at digitLEDSpace
    *digitLEDSpace = displayed_LED_low;
    *(digitLEDSpace+1) = displayed_LED_hi;
  }
  

  //off defaults
  digit_mode off_default_current_mode = OFF_MODE; 
  char off_default_char = '-';
  byte off_default_LED_low = alphanum_charset[26];
  byte off_default_LED_hi = alphanum_charset[27];

  //on defaults
  digit_mode on_default_current_mode = HEXI; 
  char on_default_char = '0';
  byte on_default_LED_low = alphanum_charset[0];
  byte on_default_LED_hi = alphanum_charset[1];

  //init digit state
  byte* digitLEDSpace = NULL;
  
  digit_state current_state = OFF_STATE;
  digit_mode current_mode = off_default_current_mode;
  char displayed_char = off_default_char;
  byte displayed_LED_low = off_default_LED_low;
  byte displayed_LED_hi = off_default_LED_hi;


  private:
};

#endif //DIGITDISPLAY_H
