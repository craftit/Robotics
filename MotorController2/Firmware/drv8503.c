
#include "ch.h"
#include "hal.h"

/*
 * Low speed SPI configuration (328.125kHz, CPHA=0, CPOL=0, MSb first).
 */
static const SPIConfig ls_spicfg = {
  NULL,
  GPIOC,
  9,
  SPI_CR1_BR_2 | SPI_CR1_BR_1  | SPI_CR1_CPHA
};
//| SPI_CR1_DFF

uint16_t Drv8503SetRegister(uint16_t cmd) {
  uint8_t data[2];
  uint8_t txbuff[2];
  txbuff[0] = cmd;
  txbuff[1] = 0;

  spiAcquireBus(&SPID3);              /* Acquire ownership of the bus.    */
  spiStart(&SPID3, &ls_spicfg);       /* Setup transfer parameters.       */
  spiSelect(&SPID3);                  /* Slave Select assertion.          */
  spiExchange(&SPID3, 1,
              &txbuff, &data);          /* Atomic transfer operations.      */
  spiUnselect(&SPID3);                /* Slave Select de-assertion.       */
  spiReleaseBus(&SPID3);              /* Ownership release.               */
  return data[0];
}

uint16_t Drv8503ReadRegister(uint16_t addr) {
  uint8_t cmd[2] ;
  cmd[0] = (1 << 7) | (addr << 3);
  cmd[1] = 0;
  uint8_t data[2];
  data[0] = 0xff;
  data[1] = 0xff;
  spiAcquireBus(&SPID3);              /* Acquire ownership of the bus.    */
  spiStart(&SPID3, &ls_spicfg);       /* Setup transfer parameters.       */
  spiSelect(&SPID3);                  /* Slave Select assertion.          */
  spiExchange(&SPID3, 2,
              &cmd, &data);          /* Atomic transfer operations.      */
  spiUnselect(&SPID3);                /* Slave Select de-assertion.       */
  spiReleaseBus(&SPID3);              /* Ownership release.               */
  return (uint16_t) data[0] << 8 | (uint16_t) data[1];
}

uint16_t Drv8503ReadStatus()
{
  uint16_t ret= Drv8503ReadRegister(0x5);

  return ret;
}

void InitDrv8503()
{


}
