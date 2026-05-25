#ifndef HW_VEGA_CORE_H_
#define HW_VEGA_CORE_H_

#ifdef HW_VEGA_V1_0
    #define HW_NAME					"Vega_V1_0"
#else
	#error "Must define hardware type"
#endif

// HW properties
#define HW_USE_25MHZ_EXT_CLOCK
#define HW_HAS_3_SHUNTS
#define HW_HAS_PHASE_SHUNTS
#define HW_HAS_PHASE_FILTERS

// Macros
#define LED_GREEN_GPIO			GPIOB
#define LED_GREEN_PIN			3
#define LED_RED_GPIO			GPIOB
#define LED_RED_PIN				4

#define LED_GREEN_ON()			palSetPad(LED_GREEN_GPIO, LED_GREEN_PIN)
#define LED_GREEN_OFF()			palClearPad(LED_GREEN_GPIO, LED_GREEN_PIN)
#define LED_RED_ON()			palSetPad(LED_RED_GPIO, LED_RED_PIN)
#define LED_RED_OFF()			palClearPad(LED_RED_GPIO, LED_RED_PIN)

#define SCTL_RESET_GPIO			GPIOC
#define SCTL_RESET_PIN			9

#define SCTL_RESET_ON()			palClearPad(SCTL_RESET_GPIO, SCTL_RESET_PIN)
#define SCTL_RESET_OFF()		palSetPad(SCTL_RESET_GPIO, SCTL_RESET_PIN)

#define SCTL_INT_GPIO          GPIOA
#define SCTL_INT_PIN           15

#define GDRV_EN_GPIO			GPIOB
#define GDRV_EN_PIN			    12

#define ENABLE_GATE()			palSetPad(GDRV_EN_GPIO, GDRV_EN_PIN)
#define DISABLE_GATE()			palClearPad(GDRV_EN_GPIO, GDRV_EN_PIN)

#define IS_DRV_FAULT()			check_drv_fault()

// Phase Filter
#define PHASE_FILTER_U_GPIO		GPIOC
#define PHASE_FILTER_U_PIN		12
#define PHASE_FILTER_V_GPIO		GPIOC
#define PHASE_FILTER_V_PIN		11
#define PHASE_FILTER_W_GPIO		GPIOC
#define PHASE_FILTER_W_PIN		10

#define PHASE_FILTER_ON()                                   \
    do {                                                    \
        palSetPad(PHASE_FILTER_U_GPIO, PHASE_FILTER_U_PIN); \
        palSetPad(PHASE_FILTER_V_GPIO, PHASE_FILTER_V_PIN); \
        palSetPad(PHASE_FILTER_W_GPIO, PHASE_FILTER_W_PIN); \
    } while (0)
#define PHASE_FILTER_OFF()                                    \
    do {                                                      \
        palClearPad(PHASE_FILTER_U_GPIO, PHASE_FILTER_U_PIN); \
        palClearPad(PHASE_FILTER_V_GPIO, PHASE_FILTER_V_PIN); \
        palClearPad(PHASE_FILTER_W_GPIO, PHASE_FILTER_W_PIN); \
    } while (0)

// Current Filter
#define CURRENT_FILTER_U_GPIO     GPIOC
#define CURRENT_FILTER_U_PIN      13
#define CURRENT_FILTER_V_GPIO     GPIOC
#define CURRENT_FILTER_V_PIN      14
#define CURRENT_FILTER_W_GPIO     GPIOC
#define CURRENT_FILTER_W_PIN      15

#define CURRENT_FILTER_ON()                                     \
    do {                                                        \
        palSetPad(CURRENT_FILTER_U_GPIO, CURRENT_FILTER_U_PIN); \
        palSetPad(CURRENT_FILTER_V_GPIO, CURRENT_FILTER_V_PIN); \
        palSetPad(CURRENT_FILTER_W_GPIO, CURRENT_FILTER_W_PIN); \
    } while (0)

#define CURRENT_FILTER_OFF()                                      \
    do {                                                          \
        palClearPad(CURRENT_FILTER_U_GPIO, CURRENT_FILTER_U_PIN); \
        palClearPad(CURRENT_FILTER_V_GPIO, CURRENT_FILTER_V_PIN); \
        palClearPad(CURRENT_FILTER_W_GPIO, CURRENT_FILTER_W_PIN); \
    } while (0)

/*
 * ADC Vector
 *
 * 0  (1):	IN10	CURR1 - U
 * 1  (2):	IN11	CURR2 - V
 * 2  (3):	IN12	CURR3 - W
 * 
 * 3  (1):	IN0	    SENS1 - U
 * 4  (2):	IN1	    SENS2 - V
 * 5  (3):	IN2	    SENS3 - W
 * 
 * 6  (1):  IN15    ADC_IND_EXT2 - COS   
 * 7  (2):	IN14    ADC_IND_EXT  - SIN
 * 8  (3):	IN13    ADC_IND_TEMP_MOTOR
 * 
 * 9  (1):	IN5     ADC_IND_TEMP_MOS_L_U
 * 10 (2):	IN4     ADC_IND_TEMP_MOS_H_U
 * 11 (3):	IN3     ADC_IND_VIN_SENS
 * 
 * 12 (1):	IN9     ADC_IND_TEMP_MOS_H_V
 * 13 (2):	IN8     ADC_IND_TEMP_MOS_L_V
 * 14 (3)	IN13    ADC_IND_TEMP_MOTOR - Free slot
 * 
 * 15 (1):	IN6     ADC_IND_TEMP_MOS_H_W
 * 16 (2):	IN7     ADC_IND_TEMP_MOS_L_W
 * 17 (3):	IN3     ADC_IND_VIN_SENS - Free slot
 */
#define HW_ADC_CHANNELS			18
#define HW_ADC_INJ_CHANNELS		3
#define HW_ADC_NBR_CONV			6

// ADC Indicies
#define ADC_IND_CURR1			0
#define ADC_IND_CURR2			1
#define ADC_IND_CURR3			2
#define ADC_IND_SENS1			3
#define ADC_IND_SENS2			4
#define ADC_IND_SENS3			5
#define ADC_IND_EXT2			6
#define ADC_IND_EXT		    	7
#define ADC_IND_TEMP_MOTOR	    8
#define ADC_IND_TEMP_MOS_L_U    9
#define ADC_IND_TEMP_MOS_H_U    10
#define ADC_IND_VIN_SENS		11
#define ADC_IND_TEMP_MOS_H_V    12
#define ADC_IND_TEMP_MOS_L_V    13
#define ADC_IND_TEMP_MOS_H_W    15
#define ADC_IND_TEMP_MOS_L_W    16

// Set general MOS temperature to ADC IND of central sensor
#define ADC_IND_TEMP_MOS ADC_IND_TEMP_MOS_H_V

// ADC macros and settings
#define VOLTAGE_TRANSFER_FUNCTION     225.816 //[V/V] Needs to account for voltage divider and internal IC transfere function
#define CURRENT_TRANSFER_FUNCTION     0.0099 //[V/A]

// Component parameters (can be overridden) - Definitions are used across software, so terminology has to be kept
#ifndef V_REG
#define V_REG							3.3
#endif
#ifndef VIN_R1
#define VIN_R1							(VOLTAGE_TRANSFER_FUNCTION - 1.0)
#endif
#ifndef VIN_R2
#define VIN_R2							1.0
#endif
#ifndef CURRENT_AMP_GAIN
#define CURRENT_AMP_GAIN				CURRENT_TRANSFER_FUNCTION
#endif
#ifndef CURRENT_SHUNT_RES
#define CURRENT_SHUNT_RES				1.0 // Unity gain so we use a single transfer function defined as CURRENT_TRANSFER_FUNCTION
#endif

#define ENCODER_SIN_VOLTS		ADC_VOLTS(ADC_IND_EXT)
#define ENCODER_COS_VOLTS		ADC_VOLTS(ADC_IND_EXT2)

// Input voltage
#define GET_INPUT_VOLTAGE()		((V_REG / 4095.0) * (float)ADC_Value[ADC_IND_VIN_SENS] * ((VIN_R1 + VIN_R2) / VIN_R2))

// Voltage on ADC channel
#define ADC_VOLTS(ch)			((float)ADC_Value[ch] / 4096.0 * V_REG)

// UCC21755 aPWM
#define UCC_CLAMP_DUTY(val)     ((val) / 3.3f < 0.10f ? 0.10f : ((val) / 3.3f > 0.88f ? 0.88f : (val) / 3.3f)) // Clamp voltage to valid range
#define UCC_V_AIN(duty)         (4.50f - (((duty) - 0.10f) / 0.78f) * 3.90f) // Reconstructs V_AIN from clamped duty cycle - mapping function 88% = 0.6V , 10% = 4.5V
#define UCC_NTC_RES(adc_ind)    (((UCC_V_AIN(UCC_CLAMP_DUTY(ADC_VOLTS(adc_ind)))) / 0.000200f) - 1000.0f) // Calculates NTC resistance using the internal 200uA current source, subtract filter resistor

// Temperature Sensors
#define NTC_TEMP_CONV(adc_ind)  (1.0 / ((logf(UCC_NTC_RES(adc_ind) / 10000.0) / 3435.0) + (1.0 / 298.15)) - 273.15)
#define NTC_TEMP(adc_ind)		mos_get_high_temp()

#define NTC_RES_MOTOR(adc_val)	(10000.0 / ((4095.0 / (float)adc_val) - 1.0)) // Motor temp sensor on low side
#define NTC_TEMP_MOTOR(beta)	(1.0 / ((logf(NTC_RES_MOTOR(ADC_Value[ADC_IND_TEMP_MOTOR]) / 10000.0) / beta) + (1.0 / 298.15)) - 273.15)

#define NTC_TEMP_MOS1()         mos_phase_get_high_temp(ADC_IND_TEMP_MOS_H_U, ADC_IND_TEMP_MOS_L_U)
#define NTC_TEMP_MOS2()         mos_phase_get_high_temp(ADC_IND_TEMP_MOS_H_V, ADC_IND_TEMP_MOS_L_V)
#define NTC_TEMP_MOS3()         mos_phase_get_high_temp(ADC_IND_TEMP_MOS_H_W, ADC_IND_TEMP_MOS_L_W)

// UART Peripheral - Not used same pins as I2C
#define HW_UART_DEV				SD3
#define HW_UART_GPIO_AF			GPIO_AF_USART3
#define HW_UART_TX_PORT			GPIOB
#define HW_UART_TX_PIN			10
#define HW_UART_RX_PORT			GPIOB
#define HW_UART_RX_PIN			11

// ICU Peripheral for servo decoding and PWM Output
#define HW_USE_SERVO_TIM4
#define HW_ICU_TIMER			TIM4
#define HW_ICU_TIM_CLK_EN()		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE)
#define HW_ICU_DEV				ICUD4
#define HW_ICU_CHANNEL			ICU_CHANNEL_2
#define HW_ICU_GPIO_AF			GPIO_AF_TIM4
#define HW_ICU_GPIO				GPIOB
#define HW_ICU_PIN				7

// I2C Peripheral
#define HW_I2C_DEV				I2CD2
#define HW_I2C_GPIO_AF			GPIO_AF_I2C2
#define HW_I2C_SCL_PORT			GPIOB
#define HW_I2C_SCL_PIN			10
#define HW_I2C_SDA_PORT			GPIOB
#define HW_I2C_SDA_PIN			11

// Hall/encoder pins
#define HW_HALL_ENC_GPIO1		GPIOC
#define HW_HALL_ENC_PIN1		6
#define HW_HALL_ENC_GPIO2		GPIOC
#define HW_HALL_ENC_PIN2		7
#define HW_HALL_ENC_GPIO3		GPIOC
#define HW_HALL_ENC_PIN3		8
#define HW_ENC_TIM				TIM3
#define HW_ENC_TIM_AF			GPIO_AF_TIM3
#define HW_ENC_TIM_CLK_EN()		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE)
#define HW_ENC_EXTI_PORTSRC		EXTI_PortSourceGPIOC
#define HW_ENC_EXTI_PINSRC		EXTI_PinSource8
#define HW_ENC_EXTI_CH			EXTI9_5_IRQn
#define HW_ENC_EXTI_LINE		EXTI_Line8
#define HW_ENC_EXTI_ISR_VEC		EXTI9_5_IRQHandler
#define HW_ENC_TIM_ISR_CH		TIM3_IRQn
#define HW_ENC_TIM_ISR_VEC		TIM3_IRQHandler

// SPI pins - Not used same pins as some ADC Temperature sensors and I2C
#define HW_SPI_DEV				SPID1
#define HW_SPI_GPIO_AF			GPIO_AF_SPI1
#define HW_SPI_PORT_NSS			GPIOB
#define HW_SPI_PIN_NSS			11
//#define HW_SPI_PORT_SCK			GPIOA
//#define HW_SPI_PIN_SCK			5
#define HW_SPI_PORT_MOSI		GPIOA
#define HW_SPI_PIN_MOSI			7
#define HW_SPI_PORT_MISO		GPIOA
#define HW_SPI_PIN_MISO			6

// CAN device and port for CAN2
#define HW_CAN2_RX_PORT			GPIOB
#define HW_CAN2_RX_PIN			5
#define HW_CAN2_TX_PORT			GPIOB
#define HW_CAN2_TX_PIN			6
#define HW_CAN2_GPIO_AF			GPIO_AF_CAN2
#define HW_CAN2_DEV				CAND2

// Measurement macros
#define ADC_V_L1				ADC_Value[ADC_IND_SENS1]
#define ADC_V_L2				ADC_Value[ADC_IND_SENS2]
#define ADC_V_L3				ADC_Value[ADC_IND_SENS3]
#define ADC_V_ZERO				(ADC_Value[ADC_IND_VIN_SENS] / 2)

// Macros
#define READ_HALL1()			palReadPad(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1)
#define READ_HALL2()			palReadPad(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2)
#define READ_HALL3()			palReadPad(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3)

// Override dead time. See the stm32f4 reference manual for calculating this value.
#define HW_DEAD_TIME_NSEC		250.0 // Conservative estimate 

// Default setting overrides
#ifndef MCCONF_L_MIN_VOLTAGE
#define MCCONF_L_MIN_VOLTAGE			20.0	// Minimum input voltage
#endif

#ifndef MCCONF_L_MAX_VOLTAGE
#define MCCONF_L_MAX_VOLTAGE			630.0	// Maximum input voltage
#endif

#ifndef MCCONF_DEFAULT_MOTOR_TYPE
#define MCCONF_DEFAULT_MOTOR_TYPE		MOTOR_TYPE_FOC
#endif

#ifndef MCCONF_L_MAX_ABS_CURRENT
#define MCCONF_L_MAX_ABS_CURRENT		90.0	// The maximum absolute current above which a fault is generated
#endif

#ifndef MCCONF_FOC_SAMPLE_V0_V7
#define MCCONF_FOC_SAMPLE_V0_V7			false	// Run control loop in both v0 and v7 (requires phase shunts)
#endif

#ifndef MCCONF_L_IN_CURRENT_MAX
#define MCCONF_L_IN_CURRENT_MAX			80.0	// Input current limit in Amperes (Upper)
#endif

#ifndef MCCONF_L_IN_CURRENT_MIN
#define MCCONF_L_IN_CURRENT_MIN			-40.0	// Input current limit in Amperes (Lower)
#endif

#ifndef APPCONF_APP_TO_USE
#define APPCONF_APP_TO_USE				APP_NONE
#endif

// Setting limits
#define HW_LIM_CURRENT			-100.0, 100.0
#define HW_LIM_CURRENT_IN		-100.0, 100.0
#define HW_LIM_CURRENT_ABS		0.0, 100.0
#define HW_LIM_VIN				20.0, 650.0
#define HW_LIM_ERPM				-200e3, 200e3
#define HW_LIM_DUTY_MIN			0.0, 0.1
#define HW_LIM_DUTY_MAX			0.0, 0.99
#define HW_LIM_TEMP_FET			-40.0, 110.0
#define HW_LIM_FOC_CTRL_LOOP_FREQ	16000.0, 50000.0 // Dont use v0 and v7 with frequencies higher than 30kHz or the RTOS might crash

// HW-specific functions
float mos_get_high_temp(void);
float mos_phase_get_high_temp(uint8_t adc_ind_h, uint8_t adc_ind_l);
uint8_t check_drv_fault(void); 

#endif /* HW_VEGA_CORE_H_ */