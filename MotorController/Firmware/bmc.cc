#include <avr/interrupt.h>
#include <avr/io.h>


#define F_CPU 20000000UL  // 1 MHz

#include <util/delay.h>
#include "Fifo.hh"

extern uint16_t fxpt_atan2(const int16_t y, const int16_t x);

const uint8_t g_charSTX = 0x02;
const uint8_t g_charETX = 0x03;

void TurnMotor();

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

#define ADC_PB (_BV(MUX2))
#define ADC_PA (_BV(MUX0) | _BV(MUX2))

const uint8_t g_currentADMux[6][2] =
{
  { ADC_IA, ADC_VC },
  { ADC_IA, ADC_VB },
  { ADC_IB, ADC_VA },
  { ADC_IB, ADC_VC },
  { ADC_IC, ADC_VB },
  { ADC_IC, ADC_VA }
};

const uint16_t g_phaseAngle[6] = {
   11270,  // 0
   24954, // 1
   34738, // 2
   45509, // 3
   60539, // 4
    2517, // 5
};


uint16_t g_phaseTopAngle[6] = {
   17997,  // 0
   29405, // 1
   39973, // 2
   51778, // 3
   63200, // 4
   7220,  // 5
};

//const uint8_t g_admuxConfg = _BV(REFS0) | _BV(REFS1); // Select 1.1 Volt ref.
const uint8_t g_admuxConfg = _BV(REFS0) ; // Select 5 Volt ref.


enum MotorModeT {
  MM_EmergencyBreak,
  MM_FreeWheel,
  MM_SensorDrive,
  MM_StepDrive
} g_motorMode = MM_FreeWheel;

FifoC<0x1f> g_txBuff;

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

uint8_t g_phase = 0;    //!< Commutation phase.

volatile uint8_t g_motorOff = 0; //!< PWM Off.
volatile uint8_t g_motorOn = 0;  //!< PWM On.

uint8_t g_motorNext = 0; //!< Next setting for motor control lines.
uint8_t g_pwmState  = 0;
// Bottom

ISR(TIMER0_OVF_vect,ISR_NAKED)
{
  // BOTTOM Value reached.
  __asm__ __volatile__ ("push r24" ::);
  g_motorNext = g_motorOff;
  __asm__ __volatile__ ("pop r24" ::);
  //ADCSRA |= _BV(ADSC);  // Trigger next ADC conversion.
  //PORTB=0x0e;
  reti();
}

// Top

ISR(TIMER0_COMPA_vect,ISR_NAKED)
{
  __asm__ __volatile__ ("push r24" ::);
  g_motorNext = g_motorOn;
  __asm__ __volatile__ ("pop r24" ::);
  //ADCSRA |= _BV(ADSC); // Trigger next ADC conversion.
  //PORTB=0x0;
  reti();
}

// Change.

ISR(TIMER0_COMPB_vect)
{
  // Set the new set of control signals.
  PORTD = g_motorNext;
}

uint16_t g_lastCurrent = 0;
uint16_t g_lastVoltage = 0;
int16_t g_lastPhaseA = 0;
int16_t g_lastPhaseB = 0;
uint8_t g_measure = 0;

uint8_t g_adcNewData = 0;

void MotorPhase(uint16_t phase) {
  uint16_t last = g_phaseTopAngle[5];
  int i = 0;
  for(;i < 6;i++) {
    uint16_t next = g_phaseTopAngle[i];
    if(last < next) {
      if(last < phase && phase <= next) {
        break;
      }
    } else {
      if(last < phase || phase <= next) {
        break;
      }
    }
    last = next;
  }
#if 1
  i++;
  if(i > 5) i = 0;
#else
  if(i == 0) i = 6;
  i--;

#endif
  g_phase = i;
  uint8_t nextOff =  g_commutationSequence[g_phase][0];
  uint8_t nextOn = g_commutationSequence[g_phase][1];

  g_motorOff = nextOff;
  g_motorOn = nextOn;

}

ISR(ADC_vect,ISR_NOBLOCK)
{
  // Make sure interrupts are enabled for PWM
  PORTB=0x0e;
  uint16_t data = ADC;

  int8_t nextSampleMotorOn = 0;
  switch(g_measure)
  {
    case 0:
      g_lastCurrent = data;

      // Sort out the current ASAP.
      if(g_motorMode == MM_StepDrive) {

#if 0
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
#endif
      }
      nextSampleMotorOn = 0;
      break;
    case 1:
      g_lastVoltage = data;
      nextSampleMotorOn = 0;
      break;
    case 2: {
      g_lastPhaseA = ((int16_t) data)-512;
      if(g_motorMode == MM_SensorDrive) {
        uint16_t phase = fxpt_atan2(g_lastPhaseA,g_lastPhaseB);
        MotorPhase(phase);
      }
      nextSampleMotorOn = 1;
      g_adcNewData = 1;
    } break;
    case 3: {
      g_lastPhaseB = ((int16_t) data)-512;
      if(g_motorMode == MM_SensorDrive) {
        uint16_t phase = fxpt_atan2(g_lastPhaseA,g_lastPhaseB);
        MotorPhase(phase);
      }
      nextSampleMotorOn = 1;
      g_adcNewData = 1;
    } break;
  }

  // Setup next conversion.

  if(nextSampleMotorOn) { // Motor on ?
    static int sampleState = 0;
    if(sampleState) {
      g_lastCurrent = data;
      g_measure = 0;
      sampleState = 0;
    } else {
      g_lastVoltage = data;
      g_measure = 1;
      sampleState = 1;
    }
    ADMUX = g_admuxConfg | g_currentADMux[g_phase][g_measure];
    ADCSRB = _BV(ADTS1) | _BV(ADTS0); // Trigger on time counter 0 match A (Off)

    //PORTB= _BV(PB0);

    //PORTB=0x0e;
  } else {
    static int sampleA = 0;
    // PWM was on.
    if(sampleA) {
      sampleA = 0;
      ADMUX = g_admuxConfg | ADC_PA;
      g_measure = 2;
      //PORTB= _BV(PB0) | _BV(PB3);
    } else {
      sampleA = 1;
      ADMUX = g_admuxConfg | ADC_PB;
      g_measure = 3;
      //PORTB= _BV(PB0) | _BV(PB2);
    }
    //PORTB=0x0;
    ADCSRB = _BV(ADTS2); // Trigger on time counter 0 overflow. (On)

  }
  PORTB=0x00;

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


#if 1
  // ---------------------------------------
  // Setup timer 0 PWM
  OCR0A = 150;  // Frequency. 255 = 5 KHz
  OCR0B = 60;  // Min value 3

  TCCR0A = _BV(WGM00) ;  // Phase correct PWM.
  //| _BV(CS00)
  TCCR0B = _BV(CS01) | _BV(WGM02); // Set clock to clkio/8
  TIMSK0 = _BV(OCIE0A) | _BV(OCIE0B) | _BV(TOIE0) ; // Enable interrupts
#endif

  // ---------------------------------------
  // Setup the counter/time
#if 0
  TCNT1 = 0;    // Start at 0
  OCR1A = 511; // TOP Value
  OCR1B = 5;    // Change at.

  // Enable interrupts
  TIMSK1 = 0; //_BV(OCIE1B) | _BV(OCIE1A) | _BV(TOIE1);

  // Configure timer.
  // We want mode 9,  phase and frequency correct, TOP in OCR1A
  // Pg 188 in datasheet.
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(WGM13) | _BV(CS10); // clkio/

  //TCCR1A = ;
  //TCCR1A = _BV(WGM11) | _BV(WGM10);
  //TCCR1B = _BV(CS10);
#endif

  // ---------------------------------------
  // Setup timer 2 for 100Hz

  OCR2A = 40;
  TCCR2A = _BV(WGM21);
  TCCR2B = _BV(CS02) | _BV(CS00); // Set clock to clkio/1024
  //TCCR2B = _BV(CS22) | _BV(CS21) | _BV(CS20);
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
#if 1
  DIDR0 = _BV(ADC0D) | _BV(ADC1D) | _BV(ADC2D) | _BV(ADC3D) |_BV(ADC4D) | _BV(ADC5D) | _BV(PD6) | _BV(PD7); // Disable digital inputs.
  ADMUX = g_admuxConfg; //_BV(REFS0) | _BV(REFS1);

  //ADCSRB = _BV(ADTS2); // Trigger on time counter 0 overflow. (On)
  //ADCSRB = _BV(ADTS1) | _BV(ADTS0); // Trigger on time counter 0 match A (Off)

  ADMUX = g_admuxConfg | ADC_PB;
  g_measure = 3;
  ADCSRB = _BV(ADTS1) | _BV(ADTS0); // Trigger on time counter 0 match A (Off)

  // Divide system clock by 32, enable ADC .
  ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS0) | _BV(ADIE) | _BV(ADATE); // Trigger on Counter timer overflow event.


#endif
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
  g_phase++;
  if(g_phase > 5)
    g_phase = 0;

  uint8_t nextOff =  g_commutationSequence[g_phase][0];
  uint8_t nextOn = g_commutationSequence[g_phase][1];

  g_motorOff = nextOff;
  g_motorOn = nextOn;

}

void BreakMotor()
{
  g_motorOff = 0;
  g_motorOn = MA(0)|MB(0)|MC(0);

  //OCR1B = 250;    // Change at.
}


int main()
{
  InitIO();

  // Make sure interrupts are enabled.
  sei();

#if 1
  // Compute angle boundaries.
  for(int i = 0;i < 6;i++) {
    int ni = i+1;
    if(ni > 5) ni = 0;
    uint32_t curr = g_phaseAngle[i];
    uint32_t next = g_phaseAngle[ni];
    if(next > curr) {
      g_phaseTopAngle[i] = (curr + next)/2;
    } else {
      uint32_t sum = (curr + next + 65535)/2;
      g_phaseTopAngle[i] = sum;
    }
  }
#endif

  g_phase = 0;
  g_motorOff = 0;
  g_motorOn = 0; //MB(0);

  PORTB= _BV(PB0);// | _BV(PB1);

  //OCR1B = 30;    // Change at.
  //ADCSRA |= _BV(ADSC); // Trigger next ADC conversion.

  //OCR0B = 50;
  //TurnMotor();

  g_motorMode = MM_SensorDrive;
  //g_motorMode = MM_StepDrive;

#if 1
  while(true) {

    // 100Hz flag.
    if(TIFR2 & _BV(OCF2A)) {
      // Clear bit.
      TIFR2 = _BV(OCF2A);
      static int count = 0;
      static int at = 195;
      count++;
      if(count > 1000) {
        count = 0;
        if(g_motorMode == MM_StepDrive)
          TurnMotor();
#if 0
        if(at > 50) {
          at--;
          OCR0A = at;
          TCNT0 = 0;
        }
#endif
      }
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

    if(g_adcNewData) {
      g_adcNewData = 0;
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
        buff[at++] = g_phase;      // 6
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


  }
#endif


#if 0
  while(true) {
    TurnMotor();
    _delay_ms(10);
  }
#endif
  return 0;
}
