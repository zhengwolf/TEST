#include "tdc.h"

void TDC_Initialize(TDC *tdc)
{
  *tdc->SSN = 1;
  
  __HAL_SPI_ENABLE(tdc->SPI);

  *tdc->EN_START = 1;
  *tdc->EN_STOP1 = 1;
  *tdc->EN_STOP2 = 1;

  *tdc->RSTN = 1;
  *tdc->RSTN = 0;
  *tdc->RSTN = 1;
}

void TDC_SendCommand(TDC *tdc, uint8_t command)
{
  *tdc->SSN = 0;

  uint8_t opcode = command;
  while (__HAL_SPI_GET_FLAG(tdc->SPI, SPI_FLAG_TXE) == 0);
  (tdc->SPI)->Instance->DR = opcode;
  while (__HAL_SPI_GET_FLAG(tdc->SPI, SPI_FLAG_RXNE) == 0);
  (tdc->SPI)->Instance->DR;

  *tdc->SSN = 1;
}

void TDC_WriteConfigurationRegister(TDC *tdc, uint8_t address, uint32_t value)
{
  *tdc->SSN = 0;
  
  uint8_t opcode = 0x80 | (address & 0x7);
  while (__HAL_SPI_GET_FLAG(tdc->SPI, SPI_FLAG_TXE) == 0);
  (tdc->SPI)->Instance->DR = opcode;
  while (__HAL_SPI_GET_FLAG(tdc->SPI, SPI_FLAG_RXNE) == 0);
  (tdc->SPI)->Instance->DR;
  
  for (uint8_t i = sizeof(value); i > 0; i--) {
    while (__HAL_SPI_GET_FLAG(tdc->SPI, SPI_FLAG_TXE) == 0);
    (tdc->SPI)->Instance->DR = *(((uint8_t*)&value) + i - 1);
    while (__HAL_SPI_GET_FLAG(tdc->SPI, SPI_FLAG_RXNE) == 0);
    (tdc->SPI)->Instance->DR;
  }
  
  *tdc->SSN = 1;
}

bool TDC_TestCommunication(TDC *tdc, uint8_t expected)
{
  uint8_t actual = 0;
  
  *tdc->SSN = 0;
  
  uint8_t opcode = GP21_READ_REG1_REGISTER;
  while (__HAL_SPI_GET_FLAG(tdc->SPI, SPI_FLAG_TXE) == 0);
  (tdc->SPI)->Instance->DR = opcode;
  while (__HAL_SPI_GET_FLAG(tdc->SPI, SPI_FLAG_RXNE) == 0);
  (tdc->SPI)->Instance->DR;

  while (__HAL_SPI_GET_FLAG(tdc->SPI, SPI_FLAG_TXE) == 0);
  (tdc->SPI)->Instance->DR = 0;
  while (__HAL_SPI_GET_FLAG(tdc->SPI, SPI_FLAG_RXNE) == 0);
  actual = (tdc->SPI)->Instance->DR;
  
  *tdc->SSN = 1;
  
  if (actual == expected)
    return true;
  else
    return false;
}

void TDC_WaitForInterrupt(TDC *tdc)
{
  while (*tdc->INTN);
}

bool TDC_CheckInterruptStatus(TDC *tdc)
{
  return (*tdc->INTN == 0) ? true : false;
}

uint16_t TDC_ReadStatusRegister(TDC *tdc)
{
  uint16_t status = 0;
  
  *tdc->SSN = 0;
  
  uint8_t opcode = GP21_READ_STAT_REGISTER;
  while (__HAL_SPI_GET_FLAG(tdc->SPI, SPI_FLAG_TXE) == 0);
  (tdc->SPI)->Instance->DR = opcode;
  while (__HAL_SPI_GET_FLAG(tdc->SPI, SPI_FLAG_RXNE) == 0);
  (tdc->SPI)->Instance->DR;
  
  for (uint8_t i = sizeof(status); i > 0; i--) {
    while (__HAL_SPI_GET_FLAG(tdc->SPI, SPI_FLAG_TXE) == 0);
    (tdc->SPI)->Instance->DR = 0;
    while (__HAL_SPI_GET_FLAG(tdc->SPI, SPI_FLAG_RXNE) == 0);
    *(((uint8_t*)&status) + i - 1) = (tdc->SPI)->Instance->DR;
  }
  
  *tdc->SSN = 1;
  
  return status;
}

uint32_t TDC_ReadResultRegister(TDC *tdc, uint8_t address)
{
  uint32_t result = 0;
  
  *tdc->SSN = 0;
  
  uint8_t opcode = 0xB0 | (address & 0x3);
  while (__HAL_SPI_GET_FLAG(tdc->SPI, SPI_FLAG_TXE) == 0);
  (tdc->SPI)->Instance->DR = opcode;
  while (__HAL_SPI_GET_FLAG(tdc->SPI, SPI_FLAG_RXNE) == 0);
  (tdc->SPI)->Instance->DR;
  
  for (uint8_t i = sizeof(result); i > 0; i--) {
    while (__HAL_SPI_GET_FLAG(tdc->SPI, SPI_FLAG_TXE) == 0);
    (tdc->SPI)->Instance->DR = 0;
    while (__HAL_SPI_GET_FLAG(tdc->SPI, SPI_FLAG_RXNE) == 0);
    *(((uint8_t*)&result) + i - 1) = (tdc->SPI)->Instance->DR;
  }
  
  *tdc->SSN = 1;
  
  return result;
}
