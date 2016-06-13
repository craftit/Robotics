#include <avr/io.h>
#include <avr/interrupt.h>

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

uint16_t g_position; //!< Current motor position. 0 to 360 degrees
uint8_t g_phase = 0;    //!< Commutation phase.
uint8_t g_pwmState = 0; //!<

ISR(TIMER1_OVF_vect)
{
  // BOTTOM Value reached.
#if 1
  if(g_pwmState == 0) {
    PORTB=0x0e;
    g_pwmState = 1;
  } else {
    PORTB=0x0;
    g_pwmState = 0;
  }
#endif

}


ISR(TIMER1_COMPA_vect)
{
   // TOP Value reached.
  //g_pwmState = 0;
//  PORTB=0x0;
}

ISR(TIMER1_COMPB_vect)
{
  //PORTD = g_commutationSequence[g_phase][g_pwmState];
}



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
  DDRB =  _BV(DDB0) | _BV(DDB1) | _BV(DDB5) | _BV(DDB2) | _BV(DDB3); // 2 and 3 are ext outputs

  // Port C is all inputs.
  // Pull inputs PB4 and PB5 high.
  PORTC = _BV(PB4) | _BV(PB5);
  DDRC = 0;

  // Port D, motor control. Everything off to start with.
  PORTD = 0;
  DDRD  = _BV(DDD1) | _BV(DDD2) | _BV(DDD3) | _BV(DDD4) | _BV(DDD5) | _BV(DDD6) | _BV(DDD7);


  // Setup the counter/time

  TCNT1 = 0;    // Start at 0
  OCR1A = 1023; // TOP Value
  OCR1B = 200;    // Change at.

  // Enable interrupts
  TIMSK1 = _BV(OCIE1B) | _BV(OCIE1A) | _BV(TOIE1);

  // Configure timer.
  // We want mode 9,  phase and frequency correct, TOP in OCR1A
  // Pg 188 in datasheet.
  //TCCR1A = _BV(WGM10);
  //TCCR1B = _BV(WGM13) | _BV(CS10);

  //TCCR1A = ;
  TCCR1A = _BV(WGM11) | _BV(WGM10);
  TCCR1B = _BV(CS10);


}


int main()
{
  InitIO();

  while(true) {
    //
    // BOTTOM Value reached.
#if 0
    if(g_pwmState == 0) {
      PORTB=0x0e;
      g_pwmState = 1;
    } else {
      PORTB=0x0;
      g_pwmState = 0;
    }
#endif
  }
  return 0;
}
