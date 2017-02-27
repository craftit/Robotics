
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

uint16_t Drv8503SetRegister(uint16_t addr,uint16_t value) {
  uint8_t data[2];
  uint8_t txbuff[2];
  txbuff[0] = addr << 3 | (value >> 8);
  txbuff[1] = value & 0xff;

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

uint16_t Drv8503ReadStatus(void)
{
  uint16_t ret= Drv8503ReadRegister(0x5);

  return ret;
}

#define DRV8503_FLIP_OTSD  (1U<<10)
#define DRV8503_EN_SNS_CLAMP (1U<<7)
#define DRV8503_CLR_FLTS (1U<<2)

void Drv8503Init(void)
{
  // Make sure the output of the sense amplifiers is clamped to 3.3V clear any faults
  // before being enabled
  Drv8503SetRegister(9,DRV8503_FLIP_OTSD | DRV8503_EN_SNS_CLAMP | DRV8503_CLR_FLTS);

}


uint16_t Drv8503Test(void)
{
  // Turn everything off
  palClearPad(GPIOA, GPIOC_PIN8);  // HC
  palClearPad(GPIOA, GPIOC_PIN9);  // HB
  palClearPad(GPIOA, GPIOC_PIN10); // HA
  palClearPad(GPIOB, GPIOC_PIN13); // LC
  palClearPad(GPIOB, GPIOC_PIN14); // LB
  palClearPad(GPIOB, GPIOC_PIN15); // LA

  palSetPad(GPIOC, GPIOC_PIN13); // Wake
  palSetPad(GPIOC, GPIOC_PIN14); // Gate enable

  palSetPad(GPIOA, GPIOC_PIN10); // HA
  palSetPad(GPIOB, GPIOC_PIN14); // LB


  return 0;
}
