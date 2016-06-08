#include <avr/io.h>

// PIN D

// PB0  O - Chip Select, Temp Sensor Active low
// PB1  O - STBY on CAN bus controller.
// PB2  I - Ext 5
// PB3  I - Ext 6
// PB4  I - MISO - Temp.
// PB5  O - SCLK - Temp
// PB6  X - XTAL
// PB7  X - XTAL

// PC:
// ADC0 A - IS A
// ADC1 A - V  A
// ADC2 A - IS B
// ADC3 A - V  B  (Ref)
// ADC4 I - Ext 3 (SDL)
// ADC5 I - Ext 4 (SCL)
// ADC6 A - IS C
// ADC7 A - V  C

// PD0  I - UART RXD
// PD1  O - UART TXD
// PD2  O - IN  A
// PD3  O - INH A
// PD4  O - IN  B
// PD5  O - INH B
// PD6  O - IN  C
// PD7  O - INH C


// Commutation Sequence

//  A B
//  A C
//  B C
//  B A
//  C A
//  C B

#define MA(x) ((uint8_t) (2 + x)<<2)
#define MB(x) ((uint8_t) (2 + x)<<4)
#define MC(x) ((uint8_t) (2 + x)<<6)

// Commutation sequence, with on , off values for PWM.

const uint8_t g_commutationSequence[6][2] = {
    { MB(0), MA(1)|MB(0)  },
    { MC(0), MA(1)|MC(0)  },
    { MC(0), MB(1)|MC(0)  },
    { MA(0), MB(1)|MA(0)  },
    { MA(0), MC(1)|MA(0)  },
    { MB(0), MC(1)|MB(0)  },
};

const uint8_t g_breakSequence[6][2] = {
    { MB(0), MA(0)|MB(0)  },
    { MC(0), MA(0)|MC(0)  },
    { MC(0), MB(0)|MC(0)  },
    { MA(0), MB(0)|MA(0)  },
    { MA(0), MC(0)|MA(0)  },
    { MB(0), MC(0)|MB(0)  },
};

uint16_t g_phase = 0; //!< Current motor position. 0 to 360 degrees

void InitIO()
{
  // Lets get things going full speed

  CLKPR = _BV(CLKPCE); // Set clock change enable bit.
  CLKPR = 0;      // Set division to 1.

  // IO.  DDR  1=Output 0=Input
  // See pg 107 in datasheet.

  // Port B
  // Pull inputs PB5 and PB6 high,
  PORTB = _BV(PB0) | _BV(PB2) | _BV(PB3) ;
  DDRB =  _BV(DDB0) | _BV(DDB1) | _BV(DDB5);

  // Port C is all inputs.
  // Pull inputs PB4 and PB5 high.
  PORTC = _BV(PB4) | _BV(PB5);
  DDRC = 0;

  // Port D, motor control. Everything off to start with.
  PORTD = 0;
  DDRD  = _BV(DDD1) | _BV(DDD2) | _BV(DDD3) | _BV(DDD4) | _BV(DDD5) | _BV(DDD6) | _BV(DDD7);

}



int main()
{
  InitIO();

  return 0;
}
