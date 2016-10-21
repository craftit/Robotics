#include <avr/io.h>
#include <avr/interrupt.h>

#define F_CPU 20000000UL  // 1 MHz

#include <util/delay.h>
#include "Fifo.hh"

const uint8_t g_charSTX = 0x02;
const uint8_t g_charETX = 0x03;


// PIN D

// PB0  O - Chip Select, Temp Sensor. Active low
// PB1  O - STBY on CAN bus controller.  High = Standby, Low = Normal
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

#if 0
const uint8_t g_commutationSequence[6][2] = {
    { 0, MA(1)|MB(0)  },
    { 0, MA(1)|MC(0)  },
    { 0, MB(1)|MC(0)  },
    { 0, MB(1)|MA(0)  },
    { 0, MC(1)|MA(0)  },
    { 0, MC(1)|MB(0)  },
};
#elif 0
const uint8_t g_commutationSequence[6][2] = {
    { MA(1), MA(1)|MB(0)  },
    { MA(1), MA(1)|MC(0)  },
    { MB(1), MB(1)|MC(0)  },
    { MB(1), MB(1)|MA(0)  },
    { MC(1), MC(1)|MA(0)  },
    { MC(1), MC(1)|MB(0)  },
};
#else
const uint8_t g_commutationSequence[6][2] = {
    { MB(0), MA(1)|MB(0)  },
    { MC(0), MA(1)|MC(0)  },
    { MC(0), MB(1)|MC(0)  },
    { MA(0), MB(1)|MA(0)  },
    { MA(0), MC(1)|MA(0)  },
    { MB(0), MC(1)|MB(0)  },
};
#endif

const uint8_t g_breakSequence[6][2] = {
    { MB(0), MA(0)|MB(0)  },
    { MC(0), MA(0)|MC(0)  },
    { MC(0), MB(0)|MC(0)  },
    { MA(0), MB(0)|MA(0)  },
    { MA(0), MC(0)|MA(0)  },
    { MB(0), MC(0)|MB(0)  },
};

#define ADC_IA 0
#define ADC_VA _BV(MUX0)
#define ADC_IB _BV(MUX1)
#define ADC_VB (_BV(MUX0) | _BV(MUX1))
#define ADC_IC (_BV(MUX1) | _BV(MUX2))
#define ADC_VC (_BV(MUX0) | _BV(MUX1) | _BV(MUX2))

#define ADC_PA (_BV(MUX2))
#define ADC_PB (_BV(MUX0) | _BV(MUX2))

const uint8_t g_currentADMux[6][2] =
{
  { ADC_IA, ADC_VC },
  { ADC_IA, ADC_VB },
  { ADC_IB, ADC_VA },
  { ADC_IB, ADC_VC },
  { ADC_IC, ADC_VB },
  { ADC_IC, ADC_VA }
};


//const uint8_t g_admuxConfg = _BV(REFS0) | _BV(REFS1); // Select 1.1 Volt ref.
const uint8_t g_admuxConfg = _BV(REFS0) ; // Select 5 Volt ref.

uint16_t g_position; //!< Current motor position. 0 to 360 degrees

uint8_t g_drivePhase = 0;    //!< Commutation phase.
uint8_t g_phaseAdc = 0; //!< Commutation phase.
uint8_t g_sampleAdc = 0; //!< Sample adc
uint8_t g_lastAdc = 0;   //!< MUX state at start of last conversion.
uint8_t g_pwmState = 0; //!<

volatile uint8_t g_motorOff = 0; //!< PWM Off.
volatile uint8_t g_motorOn = 0;  //!< PWM On.
uint8_t g_motorNext = 0; //!< Next setting for motor control lines.

enum MotorModeT {
  MM_EmergencyBreak,
  MM_FreeWheel,
  MM_StepDrive
} g_motorMode = MM_FreeWheel;

FifoC<0x1f> g_txBuff;

ISR(TIMER0_COMPA_vect)
{
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

// Bottom

ISR(TIMER1_OVF_vect)
{
  // BOTTOM Value reached.
  g_motorNext = g_motorOff;
  ADCSRA |= _BV(ADSC);
  g_lastAdc = ADMUX;
  //if(g_phase == 0)
  //PORTB=_BV(PB0);
}

// Top

ISR(TIMER1_COMPA_vect)
{
  g_motorNext = g_motorOn;
  ADCSRA |= _BV(ADSC);
  g_lastAdc = ADMUX;
  //if(g_phase == 0) PORTB=0x0c | _BV(PB0);
  //PORTB=0x0;
}

// Change.

ISR(TIMER1_COMPB_vect)
{
  // Set the new set of control signals.
  PORTD = g_motorNext;
}



void InitIO()
{
  // ---------------------------------------
  // Lets get things going full speed

  CLKPR = _BV(CLKPCE); // Set clock change enable bit.
  CLKPR = 0;      // Set division to 1.

  // ---------------------------------------
  // Setup ports
  // IO.  DDR  1=Output 0=Input
  // See pg 107 in datasheet.

  // Port B
  // Pull inputs PB5 and PB6 high,
  PORTB = _BV(PB0) | _BV(PB2) | _BV(PB3) ;
  DDRB =  _BV(DDB0) | _BV(DDB1) | _BV(DDB5) | _BV(DDB2) | _BV(DDB3); // 2 and 3 are ext outputs

  // Port C is all inputs.
  // Pull inputs PB4 and PB5 high.
  PORTC = 0; //_BV(PB4) | _BV(PB5);
  DDRC = 0;

  // Port D, motor control. Everything off to start with.
  PORTD = 0;
  DDRD  = _BV(DDD1) | _BV(DDD2) | _BV(DDD3) | _BV(DDD4) | _BV(DDD5) | _BV(DDD6) | _BV(DDD7);


  // ---------------------------------------
  // Setup the counter/time

  TCNT1 = 0;    // Start at 0
  OCR1A = 511; // TOP Value
  OCR1B = 5;    // Change at.

  // Enable interrupts
  TIMSK1 = _BV(OCIE1B) | _BV(OCIE1A) | _BV(TOIE1);

  // Configure timer.
  // We want mode 9,  phase and frequency correct, TOP in OCR1A
  // Pg 188 in datasheet.
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(WGM13) | _BV(CS10);

  //TCCR1A = ;
  //TCCR1A = _BV(WGM11) | _BV(WGM10);
  //TCCR1B = _BV(CS10);

#if 1
  // ---------------------------------------
  // Setup timer 0 for 100Hz
  OCR0A = 40;
  //OCR0A = 195; // Give a 100Hz clock at 20Mhz

  TCCR0A = _BV(WGM01);  // Enable CTC, no output.
  TCCR0B = _BV(CS02) | _BV(CS00); // Set clock to clkio/1024
  TIMSK0 = 0;//_BV(OCIE0A); // Enable interrupt
#endif

  // ---------------------------------------
  // Setup timer 2

  OCR2A = 195;
  TCCR2A = _BV(WGM21);
  TCCR2B = _BV(CS22) | _BV(CS21) | _BV(CS20);
  TIMSK2 = 0;//_BV(OCIE0A); // Disable interrupt

  // ---------------------------------------
  // Setup UART.

  // 20MHz
  // 21:  57600
  // 10: 115200
  //  4: 250000

  UBRR0H = 0;
  UBRR0L = 10;

  // 8-bit data, 1-stop bit no parity.
  UCSR0A = 0;
  UCSR0C =  _BV(UCSZ01) | _BV(UCSZ00);
  UCSR0B =  _BV(RXEN0) | _BV(TXEN0); // |_BV(RXCIE0) | _BV(UDRIE0);

  // ---------------------------------------
  // Setup ADC

  DIDR0 = _BV(ADC0D) | _BV(ADC1D) | _BV(ADC2D) | _BV(ADC3D) |_BV(ADC4D) | _BV(ADC5D) | _BV(PD6) | _BV(PD7); // Disable digital inputs.
  ADMUX = g_admuxConfg; //_BV(REFS0) | _BV(REFS1);

  ADCSRB = _BV(ADTS1) | _BV(ADTS2); // Trigger on time counter 1 overflow.

  // Divide system clock by 32, enable ADC .
  // ; Start conversion bit.
  ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS0);// | _BV(ADATE); // Trigger on Counter timer overflow event.
  g_phaseAdc = 0;

}

namespace PositionN
{
  const int16_t m_rotSteps = 252; // steps in 1 rotation.
  const int16_t m_stepSize = 252/42; // Steps in a motor tick.


  int16_t m_velocityLimit = 10; // Velocity limit.

  int16_t m_position = 0;
  int16_t m_targetPosition = 0;  // Target position.

  int16_t m_velocity = 0; // Current velocity estimate.
  int16_t m_vDeltaLimit = 100; // Current velocity estimate.
  int16_t m_targetVelocity = 0;  // Target position.

  int8_t m_direction = 0;  // Direction of rotation.
  int8_t m_tickDelay = 0;

  void ComputeSpeed()
  {
    int16_t diff = m_position - m_targetPosition;

    // Compute target velocity.
    int16_t targetVel = diff;
    if(targetVel > m_velocityLimit)
      targetVel = m_velocityLimit;
    if(targetVel < -m_velocityLimit)
      targetVel = -m_velocityLimit;

    m_targetVelocity = targetVel;  // Target position.

    int16_t vdiff = m_velocity - m_targetVelocity;

    if(diff == 0) {
      m_direction = 0;
    } else {
      if(diff > 0) {
        m_direction = 1;
      } else {
        m_direction = -1;
      }
    }

  }

  int8_t Delay() {
    return m_tickDelay;
  }

  void TickMotor()
  {
    g_drivePhase += m_direction;

    if(m_direction == 0) {
      g_motorOff = 0;
      g_motorOn = MA(0)|MB(0)|MC(0);
      return ;
    }
    if(m_direction > 0) {
      m_position += m_stepSize;
      if(g_drivePhase > 5)
        g_drivePhase = 0;
    } else {
      m_position -= m_stepSize;
      if(g_drivePhase < 0)
        g_drivePhase = 5;
    }


    uint8_t nextOff =  g_commutationSequence[g_drivePhase][0];
    uint8_t nextOn = g_commutationSequence[g_drivePhase][1];

    g_motorOff = nextOff;
    g_motorOn = nextOn;

  }



};



void SendPacket(char *buff,int len)
{
  if((5 + len) > g_txBuff.Space()) {
    // Count dropped packets?
    return;
  }

  g_txBuff.PutNoLock(g_charSTX);
  int crc = len + 0x55;
  g_txBuff.PutNoLock(len);
  for(int i =0;i < len;i++) {
    uint8_t data = buff[i];
    g_txBuff.PutNoLock(data);
    crc += data;
  }
  g_txBuff.PutNoLock(crc);
  g_txBuff.PutNoLock(crc >> 8);
  g_txBuff.PutNoLock(g_charETX);

}

extern uint16_t fxpt_atan2(const int16_t y, const int16_t x);

uint16_t g_lastCurrent = 0;
uint16_t g_lastVoltage = 0;
int16_t g_lastPhaseA = 0;
int16_t g_lastPhaseB = 0;

void ReadADC()
{
  ADCSRA = ADCSRA | _BV(ADIF);
  uint16_t data = ADC;

  // Setup the next conversion.

  // Sync the sample with PWM phase.
  if(g_motorNext == g_motorOff)
    g_sampleAdc |= 2;
  if(g_sampleAdc & 1) {
    ADMUX = g_admuxConfg | g_currentADMux[g_drivePhase][(g_sampleAdc >>1) & 1]; // Pick the next signal.
  } else {
    ADMUX = g_admuxConfg | (((g_sampleAdc & 1) == 0) ? ADC_PA : ADC_PB);
  }
  uint8_t thePhase = g_phaseAdc;
  g_phaseAdc = g_drivePhase;

  g_sampleAdc++;
  switch(g_sampleAdc) {
  case 1:
    g_lastCurrent = data;

    // Sort out the current ASAP.
    if(g_motorMode == MM_StepDrive) {

      int nextVal = 0;
      const int targetCurrent = 20; // 20
      int err = targetCurrent - (int) g_lastCurrent;

      static int sum = 0;
      sum = (15 * sum + 16 * err)/16;

      nextVal = 40 + (sum/16);    // Change at.
      if(nextVal < 30)
        nextVal = 30;
      if(nextVal > 120)
        nextVal = 120;

      OCR1B = nextVal; //nextVal;
    }
    return;
  case 2:
    g_lastPhaseA = ((int16_t) data)-512;
    return ;
  case 3:
    g_lastVoltage = ((int16_t) data)-512;
    return ;
  case 4:
    g_lastPhaseB = data;
    g_sampleAdc = 0;
    break;
  }

  uint16_t phase = fxpt_atan2(g_lastPhaseA,g_lastPhaseB);
  {
    char buff[16];
    int at = 0;
    buff[at++] = 1; // Address
    buff[at++] = 2; // Type
    buff[at++] = g_lastCurrent; // 2 Current
    buff[at++] = g_lastCurrent >> 8;
    buff[at++] = g_lastVoltage; // 4 Voltage
    buff[at++] = g_lastVoltage>>8;
    buff[at++] = thePhase;      // 6
    buff[at++] = OCR1B;         // 7
    buff[at++] = g_lastPhaseA;  // 8
    buff[at++] = g_lastPhaseA >> 8;
    buff[at++] = g_lastPhaseB;  // 10
    buff[at++] = g_lastPhaseB >> 8;
    buff[at++] = phase;         // 12
    buff[at++] = phase >> 8;
    SendPacket(buff,at);
  }

}

void SendSync()
{
  char buff[6];
  int at = 0;
  buff[at++] = 1; // Address
  buff[at++] = 3; // Type
  SendPacket(buff,at);

}

void TurnMotor()
{
  g_drivePhase++;
  if(g_drivePhase > 5)
    g_drivePhase = 0;

  uint8_t nextOff =  g_commutationSequence[g_drivePhase][0];
  uint8_t nextOn = g_commutationSequence[g_drivePhase][1];

  g_motorOff = nextOff;
  g_motorOn = nextOn;

}

void BreakMotor()
{
  g_motorOff = 0;
  g_motorOn = MA(0)|MB(0)|MC(0);

  OCR1B = 250;    // Change at.
}


int main()
{
  InitIO();

  // Make sure interrupts are enabled.
  sei();

  g_drivePhase = 0;
  g_motorOff = 0;
  g_motorOn = 0;

  PORTB= _BV(PB0);// | _BV(PB1);

  OCR1B = 30;    // Change at.

  OCR0A = 120; // 195

  g_motorMode = MM_StepDrive;

  while(true) {
    // Step flag.
    if(TIFR0 & _BV(OCF0A)) {
      // Clear bit.
      TIFR0 = _BV(OCF0A);

      switch(g_motorMode)
      {
      case MM_EmergencyBreak:
        BreakMotor();
        break;
      case MM_FreeWheel:
        g_motorOff = 0;
        g_motorOn = 0;
        break;
      case MM_StepDrive:
        TurnMotor();
        break;
      }
      continue;
    }

    // 100Hz flag.
    if(TIFR2 & _BV(OCF2A)) {
      // Clear bit.
      TIFR2 = _BV(OCF2A);
      static int count = 0;
      static int at = 195;
      count++;
      if(count > 10) {
#if 0
        count = 0;
        if(at > 50) {
          at--;
          OCR0A = at;
          TCNT0 = 0;
        }
#endif
      }
    }

    // Check ADC Value.
    if(ADCSRA & _BV(ADIF)) {
      // Clear data flag.
      ReadADC();
      continue;
    }
    // Ready to receive a byte?
    if ( (UCSR0A & _BV(RXC0)) ) {
      /* Get and return received data from buffer */
      uint8_t data = UDR0;
      continue;
    }
    // Ready to transmit a byte?
    if( ( UCSR0A & _BV(UDRE0)) ) {
     /// UDR0 = 0x55;
#if 1
      if(!g_txBuff.IsEmpty()) {
        /* Put data into buffer, sends the data */
        UDR0 = g_txBuff.Get();
        continue;
      }
#endif
    }

  }


#if 0
  while(true) {
    uint8_t nextOff =  g_commutationSequence[g_drivePhase][0];
    uint8_t nextOn = g_commutationSequence[g_drivePhase][1];

    g_drivePhase++;
    if(g_drivePhase > 5)
      g_drivePhase = 0;

    g_motorOff = nextOff;
    g_motorOn = nextOn;

    _delay_ms(10);
  }
#endif
  return 0;
}
