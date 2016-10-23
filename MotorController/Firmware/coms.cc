
#include <stdint.h>
#include "coms.hh"

const uint8_t g_charSTX = 0x02;
const uint8_t g_charETX = 0x03;

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

void SendSync()
{
  char buff[6];
  int at = 0;
  buff[at++] = 1; // Address
  buff[at++] = 3; // Type
  SendPacket(buff,at);

}

// Error codes
//  1 - Unexpected packet.
//  2 - Packet unexpected length.

void SendError(uint8_t code,uint8_t data)
{
  char buff[16];
  int at = 0;
  buff[at++] = 1; // Address
  buff[at++] = 4; // Error.
  buff[at++] = code; // Unexpected packet type.
  buff[at++] = data; // Type of packet.

  SendPacket(buff,at);

}

class SerialDecodeC
{
public:

  //! Accept a byte
  void AcceptByte(uint8_t sendByte);

  //! Process received packet.
  void ProcessPacket();

  int m_state = 0;
  int m_checkSum = 0;
  int m_packetLen = 0;
  uint8_t m_data[255];
  int m_at = 0;

  // Packet structure.
  // x    STX
  // x    Len - Of data excluding STX,ETX and Checksum.
  // 0    Address
  // 1    Type
  // 2..n data.
  // n    2-CRC
  // n    ETX.
} g_comsDecode;

void SerialDecodeC::AcceptByte(uint8_t sendByte)
{
  switch(m_state)
  {
  case 0: // Wait for STX.
    if(sendByte == g_charSTX)
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
    uint8_t cs1 = (m_checkSum & 0xff);
    //RavlDebug("Checksum1 : %d %d ",(int)  cs1 , (int) sendByte);
    if(cs1 != sendByte) {
      //RavlDebug("Checksum failed. ");
      if(sendByte == g_charSTX)
        m_state = 1;
      else
        m_state = 0;
      break;
    }

    m_state = 4;
  } break;
  case 4: { // CRC 1
    uint8_t cs2 = ((m_checkSum >> 8) & 0xff);
    //RavlDebug("Checksum2 : %d %d ",(int) ((m_checkSum >> 8) & 0xff) , (int) sendByte);
    if(cs2 != sendByte) {
      //RavlDebug("Checksum failed. ");
      if(sendByte == g_charSTX)
        m_state = 1;
      else
        m_state = 0;
      break;
    }

    m_state = 5;
  } break;
  case 5: // ETX.
    if(sendByte == g_charETX) {
      //RavlDebug("Got packet!");
      ProcessPacket();
    } else {
      //RavlDebug("Packet corrupted.");
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
  case 0: { // Ping.
    char buff[16];
    int at = 0;
    buff[at++] = 1; // Address
    buff[at++] = 1; // Type, ping reply.
    SendPacket(buff,at);
  } break;

  case 1: { // Ping reply.
    // Drop it
    break;
  }
  case 2: { // ADC Data.
    // Drop it
  } break;

  case 3: { // Sync.
    // Drop it
  } break;

  case 4: { // Error.
    // Drop it
  } break;

  case 5: { // Goto position.
    if(m_packetLen != 6) {
      SendError(2,m_data[1]);
      break;
    }

    uint32_t pos = ((uint32_t) m_data[2])  +
                    (((uint32_t) m_data[3]) << 8) +
                    (((uint32_t) m_data[4]) << 16) +
                    (((uint32_t) m_data[5]) << 24);
    GotoPosition(pos);
  } break;

  case 6: { // Calibrate request.
    StartCalibration();
  } break;
  case 7: { // Status report
    if(m_packetLen != 4) {
      SendError(2,m_data[1]);
      break;
    }
    uint16_t steps = ((uint16_t) m_data[2])  +
                    (((uint16_t) m_data[3]) << 8);
    StartStatusReport(steps);
  } break;
  default: {
    SendError(1,m_data[1]);
    //RavlDebug("Unexpected packet type %d ",(int) m_data[1]);
  } break;
  }
}

void RecievedByte(uint8_t value)
{
  g_comsDecode.AcceptByte(value);
}

