/*
 * LabPS_uContr.c
 *
 * Created: 15.03.2020 10:27:08
 * Author : maks
 */ 

#include <avr/io.h>
#define F_CPU 8000000UL
#include <util/delay.h>
//---------------------------------------------------------------//
#define RELAY1_OUT	PORTA2
#define RELAY2_OUT	PORTA3

#define SWITCH_TO_I()		(PORTA &= ~(1 << RELAY1_OUT) & ~(1 << RELAY2_OUT))
#define SWITCH_TO_II()		(PORTA |= (1 << RELAY1_OUT)) ; (PORTA &= ~(1 << RELAY2_OUT))
#define SWITCH_TO_III()		(PORTA |= (1 << RELAY1_OUT) | (1 << RELAY2_OUT))

#define VFB_ADC_CH	0
#define TEMP_ADC_CH	1

#define FAN_PWM_MIN	15
#define FAN_PWM_MAX	255

#define TEMP_MAX	390   // ~110C
#define TEMP_MIN	490 // ~40C

#define HYST			10	//1.0V

#define I_STEP_LIMIT	120 //12.0V
#define II_STEP_LIMIT	250 //26.0V
//#define III_STEP_LIMIT	370	//37.0V


#define FAN_OUT		PORTA7



//---------------------------------------------------------------//

typedef struct {
	
	uint16_t readVal;
	
	uint16_t N;
	uint16_t k;
	
	uint16_t sum;
	uint16_t avg;
	uint16_t rm; 
	uint16_t lastVal;
	
	uint16_t out;
	
}fiterVariables;

typedef enum{
	firstStep = 0,
	secondStep,
	thirdStep
}steps;

fiterVariables vfbADC;
fiterVariables tempADC;

uint16_t readADC(uint8_t input);
void setFanPwm(int16_t pwmVal);
void filter(fiterVariables *fVar);
uint16_t scaleVfb(uint16_t val);

void sysInit(void){
	
	DDRA |= (1 << DDA2) | (1 << DDA3) | (1 << DDA7);
	
	TCCR0A = 0;
	TCCR0A |= (1 << WGM01) | (1 << WGM00); // Fast PWM mode
	TCCR0A |= (1 << COM0B1) ; // Non-inverting mode (clear OC0B on compare match)
	TCCR0B = 0;
//	TCCR0B |= (1 << CS01) | (1 << CS00); // Division by 64; 8MHz / 64 / 256 = 500Hz(PWM frequency)
	TCCR0B |= (1 << CS00);
	ADCSRA |= (1 << ADEN);	//Enable ADC
//	ADCSRA |= (1 << ADIE);	//Enable ADC interrupt
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1); // ADC prescaler 64 (125kHz)
	
//	ADCSRA |= (1 << ADSC);	//Start first conversion
	
	//Initialize Kalman filter for feedBack voltage ADC readings
	vfbADC.N = 10;
	vfbADC.k = 1;
	vfbADC.readVal = readADC(VFB_ADC_CH);
	vfbADC.avg = vfbADC.readVal;
	vfbADC.lastVal = vfbADC.readVal; 
	
	//Initialize Kalman filter for temperature ADC readings
	tempADC.N = 10;
	tempADC.k = 1;
	tempADC.readVal = readADC(TEMP_ADC_CH);
	tempADC.avg = tempADC.readVal;
	tempADC.lastVal = tempADC.readVal;
	tempADC.sum = tempADC.readVal;
	
	SWITCH_TO_I();
	
}

uint16_t Vfb, Temp;
int32_t fanSpeed;
steps currentStep = firstStep;

int main(void){
    sysInit();
	
    while (1) {
		
		
		vfbADC.readVal = readADC(VFB_ADC_CH);
		filter(&vfbADC);
		Vfb = scaleVfb(vfbADC.out);

		Vfb = scaleVfb(readADC(VFB_ADC_CH));

		switch (currentStep){
			
			case firstStep:{
				if (Vfb > I_STEP_LIMIT){
					if (Vfb <= II_STEP_LIMIT){
						SWITCH_TO_II();
						currentStep = secondStep;
					}
					else if (Vfb > II_STEP_LIMIT){
						SWITCH_TO_III();
						currentStep = thirdStep;
					}
				}
				break;
			}
			
			case secondStep:{
				
				if (Vfb > II_STEP_LIMIT){
					SWITCH_TO_III();
					currentStep = thirdStep;
				}
				else if(Vfb <= (I_STEP_LIMIT - HYST)) {
					SWITCH_TO_I();
					currentStep = firstStep;
				}
				break;
			}
		
			case thirdStep:{
				
				if (Vfb <= (II_STEP_LIMIT - HYST)){
					if (Vfb > I_STEP_LIMIT){
						SWITCH_TO_II();
						currentStep = secondStep;
					}
					else if (Vfb <= (I_STEP_LIMIT - HYST)){
						SWITCH_TO_I();
						currentStep = firstStep;
					}
				}

				break;
			}
			default:break;
		}
		
	
		tempADC.readVal = readADC(TEMP_ADC_CH);
		filter(&tempADC);
		if (tempADC.out < TEMP_MIN){
			if (tempADC.out > TEMP_MAX){
				fanSpeed = (((int32_t)tempADC.out - TEMP_MIN) * (FAN_PWM_MAX - FAN_PWM_MIN) / (TEMP_MAX - TEMP_MIN) + FAN_PWM_MIN);
			}
			else{
				fanSpeed = FAN_PWM_MAX;
			}
			
		}
		else{
			fanSpeed = 0;
			PORTA &= ~(1 << FAN_OUT);
		}
		setFanPwm(fanSpeed);

    }
}


uint16_t readADC(uint8_t input){
	ADMUX = input;					//Write input argument to ADMUX register to select channel(only with default reference!)
	_delay_us(10);
	ADCSRA |= (1 << ADSC);			//Start the conversion
	while (!(ADCSRA & (1 << ADIF)));//Wait until conversion is completed
	ADCSRA |= (1 << ADIF);			//Reset flag(by writing 1)
	return ADCW;					//Return value
}


void setFanPwm(int16_t pwmVal){
	if (pwmVal >= UINT8_MAX){
		pwmVal = UINT8_MAX;
	}
	else if (pwmVal <= 0){
		pwmVal = 0;
	}
	
	OCR0B = pwmVal;
}

uint16_t scaleVfb(uint16_t val){
	return (uint16_t)((uint32_t)val * 550 / 1023);
}

void filter(fiterVariables *fVar){
	
	fVar->sum = fVar->lastVal * fVar->k + fVar->avg * (fVar->N - fVar->k) + fVar->rm;
	
	fVar->avg = fVar->sum / fVar->N;
	fVar->rm = fVar->sum - fVar->avg * fVar->N;
	

	fVar->lastVal = fVar->readVal;
	
	fVar->out =  fVar->avg;
}
