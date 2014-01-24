#include <MotorPWM.h>



void PWM_Motor(uint8_t Motor, uint16_t value)
{
	switch (Motor)
	{
	case 0:
		TIM_SetCompare1(TIM3, value);
		break;
	case 1:
		TIM_SetCompare2(TIM3, value);
		break;
	case 2:
		TIM_SetCompare3(TIM3, value);
		break;
	case 3:
		TIM_SetCompare4(TIM3, value);
		break;
	case 4:
		TIM_SetCompare1(TIM2, value);
		break;
	case 5:
		TIM_SetCompare2(TIM2, value);
		break;
	}
}

void Enable_Motor(){
	//GPIO_WriteBit(GPIOB, GPIO_Pin_0 | GPIO_Pin_1 , Bit_SET);
	GPIO_WriteBit(GPIOB, GPIO_Pin_0 , Bit_RESET);
	GPIO_WriteBit(GPIOB, GPIO_Pin_1 , Bit_RESET);
}

void Disable_Motor(){
	//GPIO_WriteBit(GPIOB, GPIO_Pin_0 | GPIO_Pin_1 , Bit_RESET);
	GPIO_WriteBit(GPIOB, GPIO_Pin_0 | GPIO_Pin_1 , Bit_SET);
}
