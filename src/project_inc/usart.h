#ifndef USART_H
#define USART_H

//buffer uart
#define rxBUF_SZ 256
#define txBUF_SZ 256

//uart structure
typedef struct {
  unsigned char rxbuffer[rxBUF_SZ];
  unsigned char txbuffer[txBUF_SZ];
  unsigned int rxready;
  unsigned int txready;
  unsigned int rxcnt;
  unsigned int txcnt;
  unsigned int txlen;	// Need for Answer is longer multiple read
}USART_OBJ;

/* Global variable declarations */
USART_OBJ It_Com;

void Usart2Put(uint8_t ch);
void Usart3Put(uint8_t ch);
uint8_t Usart3Get(void);
void SendBufferedDataFrame(USART_OBJ* It_Data, void* Header, void* Value1, void* Value2, void* Value3, uint32_t Bufferdeep);

#endif /*USART_H */
