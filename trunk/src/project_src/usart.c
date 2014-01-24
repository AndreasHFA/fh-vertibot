#include <stm32f30x.h>
#include <stm32f30x_conf.h>
#include <usart.h>

void Usart2Put(uint8_t ch)
{
      USART_SendData(USART2, (uint8_t) ch);
      //Loop until the end of transmission
      while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
      {
      }
}

/*
 * USART3 use Interrupt
 * */

void Usart3Put(uint8_t ch)
{
      USART_SendData(USART3, (uint8_t) ch);
      //Loop until the end of transmission
      while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
      {
      }
}

uint8_t Usart3Get(){
	uint8_t ch;
	while(USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == RESET)
	{
	}
	ch = USART_ReceiveData(USART3);
	return ch;
}

void SendBufferedDataFrame(USART_OBJ* It_Data ,void* Header, void* Value1, void* Value2, void* Value3, uint32_t Bufferdeep){
	uint32_t offset;
	static uint32_t loopcnt = 0;

	if (Bufferdeep < 1) {
		Bufferdeep = 1;
	}

	if (loopcnt < (Bufferdeep-1)) {
		offset = loopcnt * 16;
		memcpy(&It_Data->txbuffer[offset + 0], Header, 4); // 0xFF7F3F1F => 4286529311 => [255][127][63][31]Header
		memcpy(&It_Data->txbuffer[offset + 4], Value1, 4);
		memcpy(&It_Data->txbuffer[offset + 8], Value2, 4);
		memcpy(&It_Data->txbuffer[offset + 12], Value3, 4);
		loopcnt++;
	} else {
		offset = loopcnt * 16;
		memcpy(&It_Data->txbuffer[offset + 0], Header, 4); // 0xFF7F3F1F => 4286529311 => [255][127][63][31]Header
		memcpy(&It_Data->txbuffer[offset + 4], Value1, 4);
		memcpy(&It_Data->txbuffer[offset + 8], Value2, 4);
		memcpy(&It_Data->txbuffer[offset + 12], Value3, 4);
		It_Data->txlen = (loopcnt+1)*16;
		loopcnt = 0;
		USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
	}
}


/*
 *  KeinePufferung
 *  CtrlStates.gas, CtrlStates.nick, CtrlStates.roll,
 */
/*
if (loopcnt >= 10) {
	uint32_t Header = 0xFF7F3F1F;
	InttoBuffer(&It_Com.txbuffer[0], Header); // 0xFF7F3F1F => 4286529311 => [255][127][63][31]Header
	InttoBuffer(&It_Com.txbuffer[4], xTemp);		//xTemp);
	InttoBuffer(&It_Com.txbuffer[8], yTemp);		// yTemp);
	InttoBuffer(&It_Com.txbuffer[12], zTemp);		//zTemp);

	loopcnt = 0;
	It_Com.txlen = 16;
	USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
} else {
	loopcnt++;
}
*/

/*
 *  Mit Pufferung
 */
/*
uint32_t Header = 0xFF7F3F1F;
InttoBuffer(&It_Com.rxbuffer[(loopcnt + 0)], Header); // 0xFF7F3F1F => 4286529311 => [255][127][63][31]Header
InttoBuffer(&It_Com.rxbuffer[(loopcnt + 4)], xTemp);
InttoBuffer(&It_Com.rxbuffer[(loopcnt + 8)], yTemp);
InttoBuffer(&It_Com.rxbuffer[(loopcnt + 12)],zTemp);

if (loopcnt > (9 * 16) ) {
	memcpy((void *) &It_Com.txbuffer[0], (void *) &It_Com.rxbuffer[0], (loopcnt));
	It_Com.txlen = loopcnt;
	loopcnt = 0;
	USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
}else{
	loopcnt= loopcnt + 16; // Datenblock
}
*/
