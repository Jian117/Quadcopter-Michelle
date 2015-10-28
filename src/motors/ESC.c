//Using default Internal Clock of 4 Mhz
//To use TMR1 at 50 Hz and 1:1 prescaler, set offset to 45535.
//TMR0 can vary from 7 to 132, or 2 ms to 1 ms, at 1:1 prescaler. 
//Use TMR0 to control pulse width, even though TMR1 has much higher resolution, because TMR1 offset is 16 bits and may require lots of time to compute proper TMR1 offset from ADC value.

#include 
__CONFIG(INTIO & WDTDIS & PWRTEN & MCLRDIS & UNPROTECT \
& UNPROTECT & BORDIS & IESODIS & FCMDIS);

unsigned int pulse = 0;

//Interrupt function
static void interrupt isr(void){
	if(T0IF){
		RC0 = 0;
	}
	if(TMR1IF){
		TMR1IF = 0;
		//Start 50 Hz cycle period.
		TMR1L = 0b11011111;
		TMR1H = 0b10110001;
		//Turn on variable pulse.
		RC0 = 1;
		if(RC1 == 1){ //safety switch (must be pressed).
			TMR0 = pulse;
			T0IF = 0;
		}
		else{ //off.
			TMR0 = 132;
			T0IF = 0;
		}
	}
}

void main(void){
	//Setting ports.
	TRISA = 0x00001001; // Set A0 (for ADC) and A3 (MCLR) to input, others output
	PORTA = 0x00;
	TRISC = 0b00000010; //Set C1 to input for safety on switch.
	PORTC = 0x00;
	//ADC configuration.
	ADCON1 = 0x00;//So that ADC conversion clock is set to Fosc/2.
	ADCON0 = 0b00000001;//ADC enabled.
	ANSEL = 0x01; // Set A0 to analog, others digital
	
	//Timer configuration 
	OPTION = 0b00000010; // Timer0 configuration, 1:8 prescaler.
	TMR1L = 0b11011111;
	TMR1H = 0b10110001;
	T1CON = 0b00000001; // Timer1 configuration, including 1:1 Prescaler and TMR1ON = 1.
	
	//Timer interrupt enabling.
	T0IE = 1;
	TMR1IE = 1; // Timer1 interrupt enable, needed with PEIE and GIE bits set.
	PEIE = 1; // Peripheral interrupt enable, for Timer1.
	
	//General interrupt enabling.
	GIE = 1; // Global interrupt enable, for Timer1 and Timer0.

	while(1){
		GODONE=1; // initiate conversion.
		while(GODONE) continue; // Wait for conversion to finish. 
		pulse = 7+ADRESH*125/255;
	}
}
