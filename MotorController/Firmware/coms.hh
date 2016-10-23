#ifndef COMS_HH
#define COMS_HH 1

#include "Fifo.hh"

void RecievedByte(uint8_t value);
void SendPacket(char *buff,int len);

extern FifoC<0x1f> g_txBuff;

void GotoPosition(uint32_t pos);
void StartCalibration();
void StartStatusReport(int16_t steps);

#endif
