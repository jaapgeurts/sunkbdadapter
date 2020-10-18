#include "uart.h"

#include <xc.h>

void UART_Init(void)
{


  // 48Mhz @ 1200 baud
  BRG16 = 1;
  BRGH = 0;
  SPBRG = 2499;

  SYNC = 0; // asynchronous mode
  TXEN = 1; // transmit enable
  CREN = 1; // Continuous Receive Enable
  SPEN = 1; // Serial Port Enable
}

char UART_TX_Empty()
{
  return TRMT;
}

char UART_Data_Ready(void)
{
  return RCIF;
}

bool UART_Read(uint8_t *data)
{

  if (OERR) {
    CREN = 0;
    CREN = 1;
  }
  if (RCIF) {
    *data = RCREG;
    return true;
  }

  // nothing was received
  return false;
}

void UART_Read_Text(char *Output, unsigned int length)
{
  int i;
  for (int i = 0; i < length; i++) {
    while (UART_Read((uint8_t *) Output) == 0)
      ;
    Output++;
  }
}

void UART_Write(char data)
{
  while (!TXIF)
    ;
  TXREG = data;
}

void UART_Write_String(char *text)
{
  while (*text != '\0')
    UART_Write(*text++);
}

void UART_Write_Buf(const uint8_t *data, const int len)
{
  for (int i = 0; i < len; i++)
    UART_Write(*data++);
}
