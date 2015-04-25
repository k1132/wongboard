/*
*
* Copyright (C) Patryk Jaworski <regalis@regalis.com.pl>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
*/

/**
  * @brief Internal High Speed oscillator for USB (HSI48) value.
  */
#include <stm32f0xx.h>
 
#define LED_PIN 5
#define LED_ON() GPIOA->BSRR |= GPIO_BSRR_BS_5;
#define LED_OFF() GPIOA->BSRR |= GPIO_BSRR_BR_5;

 
int main() {
	/* Enbale GPIOA clock */
	RCC->AHBENR |= (1 << 17);
	//RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	//RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	/* Configure GPIOA pin 5 as output */
	//GPIOA->MODER |= (1 << (LED_PIN << 1));
	/* Configure GPIOA pin 5 in max speed */
	//GPIOA->OSPEEDR |= (3 << (LED_PIN << 1));
	GPIOA->MODER |= (1 << 10);
	GPIOA->ODR   |= (1 << 5);
	
	
	//give clock to Port A
	//RCC->AHBENR |= (1 << 17);


	//int pin = 5;
	//pin A5 is bits 11 and 10
	//GPIOA->MODER |= (1 << 10);
	//GPIOA->ODR   |= ~(1 << 5);

	
	/* Turn on the LED */
	//LED_ON();
	int index;
	while(1) {
		for (index = 0; index < 300000; index++);
    	GPIOA->ODR   |= (1 << 5);
    	for (index = 0; index < 300000; index++);
    	GPIOA->ODR   &= ~(1 << 5);
	};
	
	return 0;
		
		
}