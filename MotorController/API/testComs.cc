
#include "Ravl/Option.hh"
#include "Ravl/OS/SerialIO.hh"
#include "Ravl/SysLog.hh"


class SerialDecodeC
{
public:

  //! Accept a byte
  void AcceptByte(RavlN::ByteT sendByte);

  //! Process received packet.
  void ProcessPacket();

  int m_state = 0;
  int m_checkSum = 0;
  int m_packetLen = 0;
  RavlN::ByteT m_data[255];
  int m_at = 0;
  const uint8_t m_charSTX = 0x02;
  const uint8_t m_charETX = 0x03;

  // Packet structure.
  // x    STX
  // x    Len - Of data excluding STX,ETX and Checksum.
  // 0    Address
  // 1    Type
  // 2..n data.
  // n    2-CRC
  // n    ETX.
};

void SerialDecodeC::AcceptByte(RavlN::ByteT sendByte)
{
  switch(m_state)
  {
  case 0: // Wait for STX.
    if(sendByte == m_charSTX)
      m_state = 1;
    // Else remain in state 0.
    break;
  case 1: // Packet length.
    m_packetLen = sendByte;
    m_at = 0;
    m_checkSum = 0x55 + m_packetLen;
    m_state = 2;
    break;
  case 2: // Data
    m_checkSum += sendByte;
    m_data[m_at] = sendByte;
    m_at++;
    if(m_at >= m_packetLen)
      m_state = 3;
    break;
  case 3: { // CRC 1
    RavlN::ByteT cs1 = (m_checkSum & 0xff);
    //RavlDebug("Checksum1 : %d %d ",(int)  cs1 , (int) sendByte);
    if(cs1 != sendByte) {
      RavlDebug("Checksum failed. ");
      if(sendByte == m_charSTX)
        m_state = 1;
      else
        m_state = 0;
      break;
    }

    m_state = 4;
  } break;
  case 4: { // CRC 1
    RavlN::ByteT cs2 = ((m_checkSum >> 8) & 0xff);
    //RavlDebug("Checksum2 : %d %d ",(int) ((m_checkSum >> 8) & 0xff) , (int) sendByte);
    if(cs2 != sendByte) {
      RavlDebug("Checksum failed. ");
      if(sendByte == m_charSTX)
        m_state = 1;
      else
        m_state = 0;
      break;
    }

    m_state = 5;
  } break;
  case 5: // ETX.
    if(sendByte == m_charETX) {
      //RavlDebug("Got packet!");
      ProcessPacket();
    } else {
      RavlDebug("Packet corrupted.");
    }
    m_state = 0;
    break;
  }
}

//! Process received packet.
void SerialDecodeC::ProcessPacket()
{
  // m_data[0] //
  switch(m_data[1])
  {
  case 2: { // ADC Data.
    int current = ((int) m_data[2])  + (((int) m_data[3]) << 8);
    int volt = ((int) m_data[4])  + (((int) m_data[5]) << 8);
    int phaseA = (int16_t) (((int) m_data[8])  + (((int) m_data[9]) << 8));
    int phaseB = (int16_t) (((int) m_data[10])  + (((int) m_data[11]) << 8));
    int phase = ((int) m_data[12])  + (((int) m_data[13]) << 8);

    //(int) m_data[6];

    RavlDebug("I:%4d V:%4d  Phase:%d  PWM:%d  A:%d B:%d Ph:%d ",current,volt,(int) m_data[6],(int) m_data[7],phaseA,phaseB,phase);
  } break;
  default:
    RavlDebug("Unexpected packet type %d ",(int) m_data[1]);
    break;
  }
}



int main(int nargs,char **argv)
{
  RavlN::OptionC opt(nargs,argv);

  opt.Check();

  RavlN::SerialCtrlC serialPort;
  const char *port = "/dev/tty.usbmodem00156944";
  //const char *port = "/dev/tty.usbserial";
  serialPort.Open(port,"RDWR");

  int baudRate = 115200;
  //int baudRate = 57600;
  //int baudRate = 38400;
  if(!serialPort.Setup(baudRate,baudRate,1,RavlN::SerialCtrlC::PARITY_NONE,8)) {
    RavlError("Failed to setup coms port. ");
    return 1;
  }
  serialPort.SetReadTimeout(2.0);
  serialPort.SetWriteTimeout(2.0);
  serialPort.SetNonBlocking(false);


  SerialDecodeC decoder;
  while(true) {
    RavlN::ByteT sendByte = 0x55;
    //serialPort.Write((char *)&sendByte,1);
    serialPort.Read((char *)&sendByte,1);
    decoder.AcceptByte(sendByte);

    //RavlDebug("Data : %d ",(int) sendByte);
#if 0
    RavlN::ByteT recvByte;
    if(serialPort.Read((char *) &recvByte,1) != 1) {
      continue;
    }
#endif



  }
  return 0;
}
