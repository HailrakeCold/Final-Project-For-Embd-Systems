
#include "stm32l552xx.h"
#include "stdio.h"
#include "math.h"
// Some helper macros
#define bitset(word,   idx)  ((word) |=  (1<<(idx))) //Sets the bit number <idx> -- All other bits are not affected.
#define bitclear(word, idx)  ((word) &= ~(1<<(idx))) //Clears the bit number <idx> -- All other bits are not affected.
#define bitflip(word,  idx)  ((word) ^=  (1<<(idx))) //Flips the bit number <idx> -- All other bits are not affected.
#define bitcheck(word, idx)  ((word>>idx) & 1      ) //Checks the bit number <idx> -- 0 means clear; !0 means set.

// Helping functions
void setClks();
void LPUART1init(void);
void LPUART1write(int c);
int  LPUART1read(void);
void myprint(char msg[]);
void delayMs(int n);
void delay_us(uint32_t val);

void playNote(int freq);

volatile int maxAmp;			// max int# that output wave can reach

volatile float sineWaveArr[10];
volatile float squareWaveArr[10];

volatile int tremStep;			// keeps track of trem wave step
volatile float tremRate;		// tremolo rate in Hz
volatile int tremDepth;			// tremolo depth 0-100%  0-100

volatile float tremEffect;		// end amplitude modifier variable



int main (void) {

	setClks();

	/*
	 * Final design pins
	 * 12 Keys C4-C5:
	 *  C		PB11
	 *  C#		PE0
	 *  D		PB10
	 *  D#		PE15
	 *  E		PE14
	 *  F		PE12
	 *  F#		PB0
	 *  G		PE10
	 *  G#		PE7
	 *  A		PE8
	 *  A#		PE13
	 *  B		PE11
	 *  C		PE9
	 *
	 * ADC pins for Rate and Decay:
	 * 	- PA0 ADC1_in5
	 * 	- PA1 ADC1_IN6
	 *
	 * DAC out pin
	 *  - PA4 DAC1_OUT1
	 *
	 * TIM5 for delay_us
	 *  - TIM5	can count us between tone steps
	 *
	 * TIM for trem interrupts
	 *  - TIM2	can count ms between trem steps, and activate ADC conversion
	 *
	 */

	// GPIOB PB0 PB11 PB10
	RCC->AHB2ENR |= 1 << 1;

	GPIOB->MODER &= ~(0b1111 << 20);	// set input mode PB11 PB10
	GPIOB->MODER &= ~(0b11);			// PB0

	GPIOB->PUPDR &= ~(0b1111 << 20);
	GPIOB->PUPDR |=  (0b1010 << 20);	// set pull down resistor PB11 PB10
	GPIOB->PUPDR &= ~(0b11);
	GPIOB->PUPDR |=  (0b10);			// PB0

	// GPIOE PE0 PE7 PE8 PE9 PE10 PE11 PE12 PE13 PE14 PE15
	RCC->AHB2ENR |= 1 << 4;

	GPIOE->MODER &= ~(0xFFFFFFFF);		// just set all to input lmao

	GPIOE->PUPDR &= ~(0xFFFFFFFF);
	GPIOE->PUPDR |=  (0xAAAAAAAA);		// set all to pull down



	// setup TIM2 for trem interrupts
	maxAmp = 372;		// max amplitude for sine wave
	tremRate = 1;		// trem rate in Hz
	tremDepth = 100;		// trem depth 1-100
	tremStep = 0;
	tremEffect = 1;


	for (int i = 0; i < 10; i++) {
		sineWaveArr[i] = sin(2*3.14*i/10);
	}

	for (int i = 0; i < 10; i++) {
		if (i < 5) {
			squareWaveArr[i] = 1;
		} else {
			squareWaveArr[i] = 0;
		}
	}


	RCC->APB1ENR1 |= 1<<0;			// enable timer clock
	TIM2->PSC = 16 - 1;       	// Divided 16MHz source clk by 16, for 1us tick
	TIM2->ARR = (10000/tremRate) - 1;  	// Count 1us 10,000 times	with 100 steps per wave period, this is 1Hz
	TIM2->CNT = 0;               // Clear counter
	TIM2->DIER |= 1;             // Set Update Interrupt Enable
	TIM2->CR1 |= 0b100;               // update bruh what even

	// enable TIM2 interrupt in NVIC
	NVIC_SetPriority(TIM2_IRQn, 0); 	// Priority for TIM2
	NVIC_EnableIRQ(TIM2_IRQn);



	// Enable ADC Clock
	bitset(RCC->AHB2ENR, 13);  // Enable ADC clock
	RCC->CCIPR1 |=0x3<<28;     // Route SYSCLK (HCLK) to ADC

	// Turn on ADC Voltage Regulator
	bitclear(ADC1->CR, 29);  // Get out of deep power down mode
	bitset(ADC1->CR, 28);

	// Wait for the voltage regulator to stabilize
	delay_us(10000);

	// Set up ADC1
	ADC1->SQR1 = (0x5<<6)|(0x6<<12)|(1);		// 2 channels to read, channel 5 first then 6

	ADC1->CR   |= 1;            				// Enable ADC

	TIM2->CR1 |= 0b1;				// enable tim2 breaks shit?



	// enable DAC clk
	RCC->APB1ENR1 |= 1 << 29;
	DAC1->CR |= 1;		// enable DAC uhdoy


	while (1) {
		/*
		 * 12 Keys C2-C3:
		 *  C		PB11
		 *  C#		PE0
		 *  D		PB10
		 *  D#		PE15
		 *  E		PE14
		 *  F		PE12
		 *  F#		PB0
		 *  G		PE10
		 *  G#		PE7
		 *  A		PE8
		 *  A#		PE13
		 *  B		PE11
		 *  C		PE9
		 */
		int gpioe = GPIOE->IDR;
		int gpiob = GPIOB->IDR;
		int buttonsPressed = ((gpiob & (1<<11)) >> 11)		// 0	C
						   | ((gpioe & (1<< 0)) <<  1)		// 1	C#
						   | ((gpiob & (1<<10)) >>  8)		// 2	D
						   | ((gpioe & (1<<15)) >> 12)		// 3	D#
						   | ((gpioe & (1<<14)) >> 10)		// 4	E
						   | ((gpioe & (1<<12)) >>  7)		// 5	F
						   | ((gpiob & (1<< 0)) <<  6)		// 6	F#
						   | ((gpioe & (1<<10)) >>  3)		// 7	G
						   | ((gpioe & (1<< 7)) <<  1)		// 8	G#
						   | ((gpioe & (1<< 8)) <<  1)		// 9	A
						   | ((gpioe & (1<<13)) >>  3)		// 10	A#
						   | ((gpioe & (1<<11)) <<  0)		// 11	B
						   | ((gpioe & (1<< 9)) <<  3);		// 12	C

		if ((buttonsPressed & (1<<12)) >> 12) {
			// play high C 261Hz
			playNote(261/2);
		} else if ((buttonsPressed & (1<<11)) >> 11) {
			// play B  246Hz
			playNote(246/2);
		} else if ((buttonsPressed & (1<<10)) >> 10) {
			// play A# 233Hz
			playNote(233/2);
		} else if ((buttonsPressed & (1<< 9)) >>  9) {
			// play A  220Hz
			playNote(220/2);
		} else if ((buttonsPressed & (1<< 8)) >>  8) {
			// play G# 208Hz
			playNote(208/2);
		} else if ((buttonsPressed & (1<< 7)) >>  7) {
			// play G  196Hz
			playNote(196/2);
		} else if ((buttonsPressed & (1<< 6)) >>  6) {
			// play F# 185Hz
			playNote(185/2);
		} else if ((buttonsPressed & (1<< 5)) >>  5) {
			// play F  174Hz
			playNote(174/2);
		} else if ((buttonsPressed & (1<< 4)) >>  4) {
			// play E  164Hz
			playNote(164/2);
		} else if ((buttonsPressed & (1<< 3)) >>  3) {
			// play D# 156Hz
			playNote(156/2);
		} else if ((buttonsPressed & (1<< 2)) >>  2) {
			// play D  147Hz
			playNote(147/2);
		} else if ((buttonsPressed & (1<< 1)) >>  1) {
			// play C# 138Hz
			playNote(138/2);
		} else if ((buttonsPressed & 1)) {
			// play C  131Hz
			playNote(131/2);
		}

	}

//	while (1) {												// WORKS FOR A
//		int buttonPressed = ((GPIOD->IDR & 1 << 8) >> 8);
//
//		if (buttonPressed) {
//			// play 1 period of sine wave
//			for (int i = 0; i < 10; i++) {
////				int dac_val_12b = (4095/2)*(1+sin(2*3.14*i/10));
//				int dac_val_12b = (maxAmp/2) + ((maxAmp/2) * squareWaveArr[i] * tremEffect);
//				DAC1->DHR12R1 = dac_val_12b;
//				delay_us(226);
//			}
//		}
//	}

}

void playNote(int freq) {

	// calculate step period
	// 10 steps per wave period
	// 1 / (freq*10) = step period in seconds
	// 100,000 / (freq) = step period in us

	int stepPeriod = 100000 / freq;

	for (int i = 0; i < 10; i++) {
		// desmos function to set DAC
		int dac_val_12b = (maxAmp/2) + ((maxAmp/2) * squareWaveArr[i] * tremEffect);
		DAC1->DHR12R1 = dac_val_12b;
		// delay for step period
		delay_us(stepPeriod);
	}
}

// TIM2 interrupt
void TIM2_IRQHandler(){
	int readRate;
	int readDepth;

	// set trem effect for wave generator
	if (tremStep > 99) {
		tremStep = 0;		// reset tremStep when OOB
	}

//	float endEffect = (maxAmp/2) + ((maxAmp/2) * cos(2*3.14*tremStep/99));		// calculate tremEffect		| combine
	float endEffect = 1 - ((float)tremDepth/200) + (((float)tremDepth/200) * cos(2*3.14*tremStep/99));
	tremEffect = endEffect; 												// set tremEffect			|

	tremStep++;					// increment step

	// read data from pots
	ADC1->CR |= (1 << 2);	// ADSTART
	while((ADC1->ISR & (1<<2)) == 0);		// wait for EOC flag
	readRate = ADC1->DR;					// read DR for ADC1_IN5
	while((ADC1->ISR & (1<<2)) == 0);		// wait for EOC flag
	readDepth = ADC1->DR;					// read DR for ADC1_IN6
	while((ADC1->CR & (1<<2)) != 0);		// wait for ADSTART to fall

	// convert ADC vals to depth vals
	tremRate = ((float)(readRate * 5) / 4095) + 1;		// rate range = 0Hz - 5Hz
	tremDepth = (readDepth * 100) / 4095;	// depth range = 0 - 100

	// set TIM2 ARR
	TIM2->CR1 &= ~(1);						// disable TIM2
	while((TIM2->CR1 & (1)) == 1);			// wait til TIM is disabled
	int setRate = (10000/(tremRate + 1));		// convert from Hz to ms period
	TIM2->ARR = setRate - 1;				// set ARR
	TIM2->CNT = 0;
	TIM2->CR1 |= 1;							// enable TIM2


	TIM2->SR &= ~1; 		// clear flag
}

void delay_us(uint32_t val){
    // Using TIM5: A delay function that can delay 1usec to 10 sec

	RCC->APB1ENR1 |= 1<<3;		// enable TIM5 clock
	TIM5->PSC = 16 - 1; 		// set Prescaler to 16, making each cycle 1us
	TIM5->ARR = val - 1;		// set ARR to val
	TIM5->CNT = 0;				// clear current counter value
	TIM5->DIER |= 1;			// enable Update Interrupt Flag
	TIM5->CR1 |= 0b10101;		// enable, count down
	TIM5->SR &= ~1;				// reset the stupid UIF

	while(bitcheck(TIM5->SR, 0) == 0);
}

void setClks(){
	RCC->APB1ENR1 |=1<<28;   // Enable the power interface clock by setting the PWREN bits
	RCC->APB1ENR2 |=0x1;     // Enable LPUART1EN clock
	RCC->CCIPR1   |=0x800;   // 01 for HSI16 to be used for LPUART1
	RCC->CCIPR1   &= ~(0x400);
	RCC->CFGR     |=0x1;     // Use HSI16 as SYSCLK
	RCC->CR       |=0x161;   // MSI clock enable; MSI=4 MHz; HSI16 clock enable
}
void LPUART1init() {

	// enable clocks
//	bitset(RCC->APB1ENR1, 28);  // Enable Clock to PWR Interface
	bitset(RCC->AHB2ENR,   6);  // Enable Clock to GPIOG
//	bitset(RCC->APB1ENR2,  0);  // Enable Clock to LPUART
//	bitset(RCC->CCIPR1,   11);  // Select the high speed internal (HSI) oscillator as the clock to LPUART1 (16MHz)
//	bitclear(RCC->CCIPR1, 10);  //
//	bitset(RCC->CR, 8);         // HSI16 clock enable

	// enable power going to port G
	bitset(PWR->CR2, 9);        // Enable GPIOG power

	// config GPIOG
	//set GPIOG.7 to AF
	bitset(GPIOG->MODER,    15);  // Setting 0b10 in pin 7 two bit mode cfgs
	bitclear(GPIOG->MODER,  14);

	bitset(GPIOG->AFR[0],   31);  // Programming 0b1000
	bitclear(GPIOG->AFR[0], 30);
	bitclear(GPIOG->AFR[0], 29);
	bitclear(GPIOG->AFR[0], 28);

	//set GPIOG.8 to AF
	bitset(GPIOG->MODER,    17);  // Setting 0b10 in pin 8 two bit mode cfgs
	bitclear(GPIOG->MODER,  16);

	bitset(  GPIOG->AFR[1], 3);  // Programming 0b1000
	bitclear(GPIOG->AFR[1], 2);
	bitclear(GPIOG->AFR[1], 1);
	bitclear(GPIOG->AFR[1], 0);

	LPUART1-> PRESC = 0;

	// set baud rate and enable TX and RX LPUART
	// BRR = 256*16000000/115200 =
	LPUART1->BRR = 35555;
	LPUART1->CR1 = 0xD | (1<<5); // Enable Receive Data Not Empty Interrupt (RXNEIE)

}

void myprint (char msg[]){
    uint8_t idx=0;
    while(msg[idx]!='\0' ){
    	LPUART1write(msg[idx++]);
    }
}

/* Write a character to LPUART1 */
void LPUART1write (int ch) {
    while (!(LPUART1->ISR & 0x0080)); // wait until Tx buffer empty
    LPUART1->TDR = (ch & 0xFF);
}

/* Read a character from LPUART1 */
int LPUART1read(void) {
    while (!(LPUART1->ISR & 0x0020)) {}   // wait until char arrives
    return LPUART1->RDR;
}

void delayMs(int n) {
    int i;

    /* Configure SysTick */
    SysTick->LOAD = 16000;  /* reload with number of clocks per millisecond */
    SysTick->VAL = 0;       /* clear current value register */
    SysTick->CTRL = 0x5;    /* Enable the timer */

    for(i = 0; i < n; i++) {
        while((SysTick->CTRL & 0x10000) == 0) /* wait until the COUNTFLAG is set */
            { }
    }
    SysTick->CTRL = 0;      /* Stop the timer (Enable = 0) */
}
