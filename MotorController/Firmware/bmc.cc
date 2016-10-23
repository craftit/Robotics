
#include <avr/interrupt.h>
#include <avr/io.h>

#define F_CPU 20000000UL  // 20 MHz

#include <util/delay.h>

#include "coms.hh"

extern uint16_t fxpt_atan2(const int16_t y, const int16_t x);
extern uint16_t fxpt_atan2fast(int16_t y, int16_t x);

#if 0
#define DBTIMING(x) x
#else
#define DBTIMING(x)
#endif

#if 1
#define DBMISC(x) x
#else
#define DBMISC(x)
#endif

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

// PB - ADC4
// PA - ADC5

const uint8_t g_currentADMux[6][2] =
{
  { ADC_IA, ADC_VC },
  { ADC_IA, ADC_VB },
  { ADC_IB, ADC_VA },
  { ADC_IB, ADC_VC },
  { ADC_IC, ADC_VB },
  { ADC_IC, ADC_VA }
};

uint16_t g_phaseAngle[6] = {
   11270,  // 0
   24954, // 1
   34738, // 2
   45509, // 3
   60539, // 4
    2517, // 5
};


uint16_t g_phaseTopAngle[6] = {
   23462, // 0
   29388, // 1
   38443, // 2
   58119, // 3
   1536,  // 4
   8966,  // 5
};

//const uint8_t g_admuxConfg = _BV(REFS0) | _BV(REFS1); // Select 1.1 Volt ref.
const uint8_t g_admuxConfg = _BV(REFS0) ; // Select 5 Volt ref.

enum ServoTaskT {
  ST_Idle,
  ST_Calibrate,
  ST_StatusReport
} g_servoTask;


enum MotorModeT {
  MM_Break,
  MM_FreeWheel,
  MM_SensorDrive,
  MM_SensorServo,
  MM_StepDrive
} g_motorMode = MM_FreeWheel;

const uint8_t g_minPWM = 4;
const uint8_t g_maxPWM = 120; // 250

uint8_t g_drivePhase = 0;    //!< Commutation phase.
uint16_t g_atPhaseAngle = 0;    //!< Commutation phase.

volatile uint8_t g_motorOff = 0; //!< PWM Off.
volatile uint8_t g_motorOn = 0;  //!< PWM On.

uint8_t g_motorNext = 0; //!< Next setting for motor control lines.

ISR(TIMER0_OVF_vect,ISR_NAKED)
{
  // BOTTOM Value reached.
  __asm__ __volatile__ ("push r24" ::);
  g_motorNext = g_motorOff;
  DBTIMING(PINB=0x08);
  DBMISC(PINB=0x08);
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
  DBTIMING(PINB=0x08);
  DBMISC(PINB=0x08);
  __asm__ __volatile__ ("pop r24" ::);
  //ADCSRA |= _BV(ADSC); // Trigger next ADC conversion.
  //PORTB=0x0;
  reti();
}

// Change.
#if 0
ISR(TIMER0_COMPB_vect,ISR_NAKED)
{
  // Set the new set of control signals.
  __asm__ __volatile__ ("push r24" ::);
  PORTD = g_motorNext;
  __asm__ __volatile__ ("pop r24" ::);
  reti();
}
#else
ISR(TIMER0_COMPB_vect)
{
  // Set the new set of control signals.
  PORTD = g_motorNext;
  DBMISC(PINB=0x04);
}

#endif

uint16_t g_lastCurrent = 0;
uint16_t g_lastVoltage = 0;
int16_t g_lastPhaseA = 0;
int16_t g_lastPhaseB = 0;
int8_t g_atPhase = 0;
uint8_t g_measure = 0;

uint8_t g_adcNewData = 0;

int32_t g_position = 0;
int32_t g_goalPosition = 100;
int g_targetCurrent = 30;

void DrivePhase(uint8_t phase) {
  g_drivePhase = phase;
  uint8_t nextOff =  g_commutationSequence[g_drivePhase][0];
  uint8_t nextOn = g_commutationSequence[g_drivePhase][1];

  g_motorOff = nextOff;
  g_motorOn = nextOn;
}

void TurnMotor()
{
  g_drivePhase++;
  if(g_drivePhase > 5)
    g_drivePhase = 0;

  DrivePhase(g_drivePhase);
}

void MotorPhase() {

  uint16_t rawPhase = fxpt_atan2fast(g_lastPhaseA,g_lastPhaseB);

  g_atPhaseAngle = rawPhase;

  uint16_t last = g_phaseAngle[5];
  int phase = 0;
  for(;phase < 6;phase++) {
    uint16_t next = g_phaseAngle[phase];
    if(last < next) {
      if(last < rawPhase && rawPhase <= next) {
        break;
      }
    } else {
      if(last < rawPhase || rawPhase <= next) {
        break;
      }
    }
    last = next;
  }


  int8_t change = phase - g_atPhase;
  g_atPhase = phase;

  if(change > 3) change -= 6;
  if(change < -3) change += 6;

  g_position += change;

  if(g_motorMode == MM_SensorDrive) {
    phase += 2;
    if(phase > 5)
      phase -= 6;
    DrivePhase(phase);
    return ;
  }

  if(g_motorMode == MM_SensorServo) {
    int32_t positionError = g_goalPosition - g_position;
    if(positionError > 127) positionError = 127;
    if(positionError < -127) positionError = -127;

    int8_t drive = positionError;

    if(drive == 0) {
      g_motorOff = 0;
      g_motorOn = 0;
    } else {
      uint8_t pwmWidth;
      if(drive > 0) {
        phase += 2;
        if(phase > 5) phase -= 6;
        pwmWidth = drive;
      } else {
        phase -= 2;
        if(phase < 0) phase += 6;
        pwmWidth = -drive;
      }
      if(pwmWidth < g_minPWM) pwmWidth = g_minPWM;
      if(pwmWidth > g_maxPWM) pwmWidth = g_maxPWM;

      DrivePhase(phase);
    }
  }
}

// Handle result of ADC conversion.
// Make sure interrupts are enabled for PWM as soon as possible.

ISR(ADC_vect,ISR_NOBLOCK)
{
  DBTIMING(PINB=0x04);
  uint16_t data = ADC;
  int8_t lastMeasure = g_measure;

  // Setup next conversion before doing anything else so if
  // the phase computation overruns the next conversion it doesn't cause us
  // to skip.  This does mean this function may be called twice concurrently

  switch(g_measure)
  {
    case 1: // Motor off next sample
    case 0:{
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
      ADCSRB = _BV(ADTS1) | _BV(ADTS0); // Trigger on time counter 0 match A (Off)
    } break;
    case 2: // Motor on next sample.
    case 3: {
      static int sampleState = 0;
      if(sampleState) {
        g_measure = 0;
        sampleState = 0;
      } else {
        g_measure = 1;
        sampleState = 1;
      }
      ADMUX = g_admuxConfg | g_currentADMux[g_drivePhase][g_measure];
      ADCSRB = _BV(ADTS2); // Trigger on time counter 0 overflow. (Currently On)
    } break;
  }


  switch(lastMeasure)
  {
    case 0:
      g_lastCurrent = data;

      // Sort out the current.
      if(g_motorMode == MM_StepDrive) {

#if 0
        int nextVal = 0;
        int err = g_targetCurrent - (int) g_lastCurrent;

        static int sum = 0;
        sum = (15 * sum + 16 * err)/16;

        nextVal = 40 + (sum/16);    // Change at.
        if(nextVal < g_minPWM)
          nextVal = g_minPWM;
        if(nextVal > g_maxPWM)
          nextVal = g_maxPWM;

        OCR1B = nextVal; //nextVal;
#endif
      }
      break;
    case 1:
      g_lastVoltage = data;
      break;
    case 2: {
      g_lastPhaseA = ((int16_t) data)-512;
      MotorPhase();
      g_adcNewData = 1;
    } break;
    case 3: {
      g_lastPhaseB = ((int16_t) data)-512;
      MotorPhase();
      g_adcNewData = 1;
    } break;
  }

  DBTIMING(PINB=0x04);

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
  OCR0A = 255;  // 255 = 5 KHz
  OCR0B = 80;  // Min value 3

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
#if 1
  OCR2A = 40;
  TCCR2A = _BV(WGM21);
  TCCR2B = _BV(CS02) | _BV(CS00); // Set clock to clkio/1024
  //TCCR2B = _BV(CS22) | _BV(CS21) | _BV(CS20);
  TIMSK2 = 0;//_BV(OCIE0A); // Disable interrupt
#endif
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

  ADMUX = g_admuxConfg | ADC_PB;
  g_measure = 3;
  ADCSRB = _BV(ADTS1) | _BV(ADTS0); // 40KHz Trigger on time counter 0 match A
  //ADCSRB = _BV(ADTS2); // Trigger on time counter 0 match A

  // Divide system clock by 32, enable ADC .
  ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS0) | _BV(ADIE) | _BV(ADATE); // Trigger on Counter timer overflow event.

#endif
}



void BreakMotor()
{
  g_motorOff = 0;
  g_motorOn = MA(0)|MB(0)|MC(0);

  //OCR1B = 250;    // Change at.
}

void GotoPosition(uint32_t pos)
{
  g_goalPosition = pos;
}

uint16_t g_taskStep;

void StartCalibration()
{
  g_taskStep = 0;
  g_motorMode = MM_StepDrive;
  g_servoTask = ST_Calibrate;
}

void UpdateBoundaries()
{
#if 0
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
}

void CalibrateStep()
{
  static int counter = 0;
  static int cycle = 0;
  static int32_t g_phaseSum[6];

  switch(g_taskStep) {
  case 0: {
    g_targetCurrent = 10; // Set motor current target.
    g_motorMode = MM_StepDrive;
    DrivePhase(0);
    cycle = 0;
    g_taskStep = 1;
    for(int i = 0;i < 6;i++)
      g_phaseSum[i] = 0;
    counter = 0;
  } break;
  case 1: { // Go through steps forward.
    counter++;
    if(counter > 2000) { // Wait for position to settle.
      counter = 0;
      uint8_t nextPhase = g_drivePhase;
      g_phaseSum[nextPhase] += g_atPhaseAngle;
      nextPhase++;
      if(nextPhase > 5) {
        nextPhase = 0;
#if 1
        cycle++;
        if(cycle >= 8) {
          g_taskStep = 2;
          break;
        }
#else
        DBMISC(PINB=0x04);
        g_taskStep = 2;
#endif
      }
      DrivePhase(nextPhase);
    }
  } break;
#if 1
  default: {
    for(int i =0;i < 6;i++)
      g_phaseAngle[i] = g_phaseSum[i] / 8;// >> 2;//>>3;

    // Finished.
    UpdateBoundaries();
    // Finished.
    g_motorMode = MM_SensorDrive;
    g_servoTask = ST_Idle;
  } break;
#endif
  }
}

void StartStatusReport(int16_t steps)
{
  g_servoTask = ST_StatusReport;
  g_taskStep = steps;
}

bool SendStatus() {
  if(!g_adcNewData)
    return false;
  g_adcNewData = 0;
  {
    char buff[16];
    int at = 0;
    buff[at++] = 1; // Address
    buff[at++] = 2; // Type
    buff[at++] = g_lastCurrent; // 2 Current
    buff[at++] = g_lastCurrent >> 8;
    buff[at++] = g_lastVoltage; // 4 Voltage
    buff[at++] = g_lastVoltage>>8;
    buff[at++] = g_drivePhase;      // 6
    buff[at++] = OCR1B;         // 7
    buff[at++] = g_lastPhaseA;  // 8
    buff[at++] = g_lastPhaseA >> 8;
    buff[at++] = g_lastPhaseB;  // 10
    buff[at++] = g_lastPhaseB >> 8;
    buff[at++] = g_atPhaseAngle;         // 12
    buff[at++] = g_atPhaseAngle >> 8;
    SendPacket(buff,at);
  }
  return true;
}

int main()
{
  InitIO();

  // Make sure interrupts are enabled.
  sei();

  UpdateBoundaries();

  g_drivePhase = 0;
  g_motorOff = 0;
  g_motorOn = 0; //MB(0);

  PORTB= _BV(PB0);// | _BV(PB1);

#if 0
  g_motorMode = MM_StepDrive;
  DrivePhase(0);
#else
  g_motorMode = MM_SensorDrive;
#endif

  //StartCalibration();

#if 1
  while(true) {

#if 1
    // 100Hz flag.
    if(TIFR2 & _BV(OCF2A)) {
      // Clear bit.
      TIFR2 = _BV(OCF2A);
      //DBMISC(PINB=0x08);

      switch(g_servoTask) {
      case ST_Idle: break;
      case ST_Calibrate: CalibrateStep(); break;
      case ST_StatusReport: break;
      default: g_servoTask = ST_Idle;
      }
    }
#endif

    // Ready to receive a byte?
    if ( (UCSR0A & _BV(RXC0)) ) {
      uint8_t data = UDR0;
      RecievedByte(data);
      continue;
    }

    // Ready to transmit a byte?
    if( ( UCSR0A & _BV(UDRE0)) ) {
     /// UDR0 = 0x55;
      if(!g_txBuff.IsEmpty()) {
        /* Put data into buffer, sends the data */
        UDR0 = g_txBuff.Get();
        continue;
      }
    }

    SendStatus();
    if(g_servoTask == ST_StatusReport) {
      if(SendStatus()) {
        g_taskStep--;
        if(g_taskStep == 0)
          g_servoTask = ST_Idle;
      }
    }

  }
#endif


  return 0;
}
