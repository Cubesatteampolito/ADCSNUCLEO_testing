/* Magnetorquers driver */
#include "magnetorquers_driver.h"


const float Rsense[] = {1973,2028,1962,1992,1979};				// Rsense value for each motor driver - TODO only keep 3 values for magnetorquers
const float Aipropri = 1575e-6; 								// Adimensionale - Mirror ratio of driver current mirror circuit
const float Rmagnetorquer[] = {30.5,30.5,142}; 					// Resistance (Ohm) of each magnetorquer

// bool int_flag1 = 0,int_flag2 = 0; //unused



/* Magnetorquers initialization 
*  Duty cycle must be written in %
*  Direction: 1 = Forward; 0 = Reverse
*  PWM frequency can vary between 4Hz and 200Khz
*/
void init_actuator_handler(
	Actuator_struct *act,						//
	TIM_HandleTypeDef* htim,					//
	uint32_t pwm_channel1, 						//
	uint32_t pwm_channel2,						//
	uint32_t pwm_freq,							//
	uint8_t duty_cycle							//
){
	act->htim=htim;
	act->pwm_channel1=pwm_channel1;
	act->pwm_channel2=pwm_channel2;
	act->duty_cycle = duty_cycle;
	act->dir = 1; 								//Initially set to Forward

	HAL_TIM_PWM_Stop(act->htim,pwm_channel1);
	HAL_TIM_PWM_Stop(act->htim,pwm_channel2);

	if(pwm_freq > 200000) pwm_freq = 200000;	// Safety clamps
	else if(pwm_freq < 4) pwm_freq = 4;
	uint32_t prescaler = (40000000 / (pwm_freq * (act->htim->Init.Period + 1))) - 1;

	/* Update the prescaler */
	__HAL_TIM_SET_PRESCALER(act->htim, prescaler);
	
	uint32_t update_value = (uint32_t)roundf((float)(act->htim->Instance->ARR) * (act->duty_cycle * 0.01));

	//if(update_value > act->htim->Instance->ARR)	update_value = act->htim->Instance->ARR;
	__HAL_TIM_SET_COMPARE(act->htim, act->pwm_channel1, (uint32_t)(roundf(act->htim->Instance->ARR)));
	__HAL_TIM_SET_COMPARE(act->htim, act->pwm_channel2, (uint32_t)(roundf(update_value)));
}

/* Read magnetorquers current */
void get_actuator_current(
	ADC_HandleTypeDef *hadc,			//
	volatile float voltagebuf[],		//
	volatile float currentbuf[],		//
	uint8_t Channels_mask[]				//
){
	volatile uint16_t adc_raw[NUM_DRIVERS];
	if(Channels_mask[0] == 1)			// TODO: remove unused channels cases
	{
		ADC_Select_CH1(hadc);
		HAL_ADC_Start(hadc);
		if (HAL_ADC_PollForConversion(hadc, 10) != HAL_OK)
		{
			Error_Handler();
		}
		else
		{
			adc_raw[0] = HAL_ADC_GetValue(hadc);
			voltagebuf[0] = (volatile float)adc_raw[0] * (3.3/(pow(2,12) - 1));
			currentbuf[0] = (volatile float)(voltagebuf[0]/(Rsense[0]*Aipropri));
			#if ( DEBUG_MSGS == 1 )
				printf("Channel 1 digits: %d,voltage value:  %f v, current value: %f A \r\n", adc_raw[0],voltagebuf[0],currentbuf[0]);
			#endif
		}
		HAL_ADC_Stop(hadc);
	}
	if(Channels_mask[1] == 1)
	{
		ADC_Select_CH2(hadc);
		HAL_ADC_Start(hadc);
		if (HAL_ADC_PollForConversion(hadc, 10) != HAL_OK)
		{
			Error_Handler();
		}
		else
		{
			adc_raw[1] = HAL_ADC_GetValue(hadc);
			voltagebuf[1] = (volatile float)adc_raw[1] * (3.3/(pow(2,12) - 1));
			currentbuf[1] = (volatile float)(voltagebuf[1]/(Rsense[1]*Aipropri));
			#if ( DEBUG_MSGS == 1 )
				printf("Channel 2 digits: %d,voltage value:  %f v, current value: %f A \r\n",adc_raw[1],voltagebuf[1],currentbuf[1]);
			#endif
		}
		HAL_ADC_Stop(hadc);
	}
	if(Channels_mask[2] == 1)
	{
		ADC_Select_CH3(hadc);
		HAL_ADC_Start(hadc);
		if (HAL_ADC_PollForConversion(hadc, 10) != HAL_OK)
		{
			Error_Handler();
		}
		else
		{
			adc_raw[2] = HAL_ADC_GetValue(hadc);
			voltagebuf[2] = (volatile float)adc_raw[2] * (3.3/(pow(2,12) - 1));
			currentbuf[2] = (volatile float)(voltagebuf[2]/(Rsense[2]*Aipropri));
			#if ( DEBUG_MSGS == 1 )
				printf("Channel 3 digits: %d,voltage value:  %f v, current value: %f A \r\n",adc_raw[2],voltagebuf[2],currentbuf[2]);
			#endif
		}
		HAL_ADC_Stop(hadc);
	}
	if(Channels_mask[3] == 1)
	{
		ADC_Select_CH4(hadc);
		HAL_ADC_Start(hadc);
		if (HAL_ADC_PollForConversion(hadc, 10) != HAL_OK)
		{
			Error_Handler();
		}
		else
		{
			adc_raw[3] = HAL_ADC_GetValue(hadc);
			voltagebuf[3] = (volatile float)adc_raw[3] * (3.3/(pow(2,12) - 1));
			currentbuf[3] = (volatile float)(voltagebuf[3]/(Rsense[3]*Aipropri));
			#if ( DEBUG_MSGS == 1 )
				printf("Channel 4 digits: %d,voltage value:  %f v, current value: %f A \r\n",adc_raw[3],voltagebuf[3],currentbuf[3]);
			#endif
		}
		HAL_ADC_Stop(hadc);
	}
	if(Channels_mask[4] == 1)
	{
		ADC_Select_CH16(hadc);
		HAL_ADC_Start(hadc);
		if (HAL_ADC_PollForConversion(hadc,10) != HAL_OK)
		{
			Error_Handler();
		}
		else
		{
			adc_raw[4] = HAL_ADC_GetValue(hadc);
			voltagebuf[4] = (volatile float)adc_raw[4] * (3.3/(pow(2,12) - 1));
			currentbuf[4] = (volatile float)(voltagebuf[4]/(Rsense[4]*Aipropri));
			#if ( DEBUG_MSGS == 1 )
				printf("Channel 16 digits: %d,voltage value:  %f v, current value: %f A \r\n",adc_raw[4],voltagebuf[4],currentbuf[4]);
			#endif
		}
		HAL_ADC_Stop(hadc);
	}
}


/* Duty cycle updated for given magnetorquer 
*  duty is passed in percentage
*/
void update_duty_dir(
	Actuator_struct *act,			
	float duty,						
	bool dir						
){
	if(duty != 0)					// TODO: isn't 0 a possible value?
	{
		if(duty < 0) duty = duty * -1;
		act->duty_cycle = duty;

		/* The duty cycle value is a percentage of the reload register value (ARR). Rounding is used.*/
		uint32_t update_value = (uint32_t)roundf((float)(act->htim->Instance->ARR) * (duty * 0.01));

		/* In case of the duty cycle being calculated is higher than the reload register, fix it to the reload register */
		if(update_value > act->htim->Instance->ARR)	update_value = act->htim->Instance->ARR;

		if(dir)
		{
													//IN1 -> 100% PWM
													//IN2 -> Duty Cycle PWM
			act->dir = 1; 							// Set direction to Forward
			#if ( DEBUG_MSGS == 1 )
				printf("Direction change: Forward\r\n");
			#endif
			/* Assign the new value of duty cycle to the capture compare register. */
			// IN1
			__HAL_TIM_SET_COMPARE(act->htim, act->pwm_channel1, (uint32_t)(roundf(act->htim->Instance->ARR)));
			// IN2
			__HAL_TIM_SET_COMPARE(act->htim, act->pwm_channel2, (uint32_t)(roundf(update_value)));
		}
		else
		{
													//IN1 -> Duty Cycle PWM
													//IN2 -> 100% PWM
			act->dir = 0;							// Set direction to Reserve
			#if ( DEBUG_MSGS == 1 )
				printf("Direction change: Reverse\r\n");
			#endif
			/* Assign the new value of duty cycle to the capture compare register. */
			//IN1
			__HAL_TIM_SET_COMPARE(act->htim, act->pwm_channel1, (uint32_t)(roundf(update_value)));
			//IN2
			__HAL_TIM_SET_COMPARE(act->htim, act->pwm_channel2, (uint32_t)(roundf(act->htim->Instance->ARR)));
		}
	}
	else
	{
		#if ( DEBUG_MSGS == 1 )
			printf("Error: unexpected duty cycle\r\n");
		#endif
	}
}

/* Start the actuator passed as argument */
void actuator_START(Actuator_struct *act){
	HAL_TIM_PWM_Start(act->htim, act->pwm_channel1);	//Start PWM signal 1
	HAL_TIM_PWM_Start(act->htim, act->pwm_channel2); 	//Start PWM signal 2
	#if ( DEBUG_MSGS == 1 )
		printf("PWM signals started\r\n");
	#endif
}

/* Stop the actuator passed as argument */
void actuator_STOP(Actuator_struct *act){
	HAL_TIM_PWM_Stop(act->htim, act->pwm_channel1); 	//Stop PWM signal 1
	HAL_TIM_PWM_Stop(act->htim, act->pwm_channel2); 	//Stop PWM signal 2
	#if ( DEBUG_MSGS == 1 )
		printf("PWM signals stopped\r\n");
	#endif
}

/* Choose CHx for the next sampling */
void ADC_Select_CH1 (ADC_HandleTypeDef *hadc)
{
	  ADC_ChannelConfTypeDef sConfig = {0};
	  /*
	  * Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	  */
	  sConfig.Channel = ADC_CHANNEL_1;
	  sConfig.Rank = ADC_REGULAR_RANK_1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
	  sConfig.SingleDiff = ADC_SINGLE_ENDED;
	  sConfig.OffsetNumber = ADC_OFFSET_NONE;
	  if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

void ADC_Select_CH2 (ADC_HandleTypeDef *hadc)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	  /*
	  * Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	  */
	  sConfig.Channel = ADC_CHANNEL_2;
	  sConfig.Rank = ADC_REGULAR_RANK_1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5 ;
	  sConfig.SingleDiff = ADC_SINGLE_ENDED;
	  	  sConfig.OffsetNumber = ADC_OFFSET_NONE;
	  if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

void ADC_Select_CH3 (ADC_HandleTypeDef *hadc)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	  /*
	  * Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	  */
	  sConfig.Channel = ADC_CHANNEL_3;
	  sConfig.Rank = ADC_REGULAR_RANK_1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
	  sConfig.SingleDiff = ADC_SINGLE_ENDED;
	  	  sConfig.OffsetNumber = ADC_OFFSET_NONE;
	  if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

void ADC_Select_CH4 (ADC_HandleTypeDef *hadc)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	  /*
	  * Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	  */
	  sConfig.Channel = ADC_CHANNEL_4;
	  sConfig.Rank = ADC_REGULAR_RANK_1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
	  sConfig.SingleDiff = ADC_SINGLE_ENDED;
	  	  sConfig.OffsetNumber = ADC_OFFSET_NONE;
	  if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

void ADC_Select_CH16 (ADC_HandleTypeDef *hadc)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	  /*
	  * Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	  */
	  sConfig.Channel = ADC_CHANNEL_16;
	  sConfig.Rank = ADC_REGULAR_RANK_1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5 ;sConfig.SingleDiff = ADC_SINGLE_ENDED;
	  sConfig.OffsetNumber = ADC_OFFSET_NONE;
	  if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}
