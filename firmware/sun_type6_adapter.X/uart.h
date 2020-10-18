#ifndef __UART_H
#define __UART_H

#include <stdint.h>
#include <stdbool.h>


void UART_Init(void);

char UART_TX_Empty(void);

char UART_Data_Ready(void);

bool UART_Read(uint8_t *data);

void UART_Read_Text(char *Output, unsigned int length);

void UART_Write(char data);

void UART_Write_String(char *text);

void UART_Write_Buf(const uint8_t *data, const int len);

#endif
