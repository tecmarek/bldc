#include "hw.h"
#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "utils_math.h"
#include <math.h>
#include "mc_interface.h"
#include "lispif.h"
#include "lispbm.h"

// Variables
static volatile bool i2c_running = false;

// I2C configuration
static const I2CConfig i2cfg = {
		OPMODE_I2C,
		100000,
		STD_DUTY_CYCLE
};

void hw_init_gpio(void) {
	// GPIO clock enable
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	// LEDs
	palSetPadMode(LED_GREEN_GPIO, LED_GREEN_PIN, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
	palSetPadMode(LED_RED_GPIO, LED_RED_PIN, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);

	// SCTL reset
	palSetPadMode(SCTL_RESET_GPIO, SCTL_RESET_PIN, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
	SCTL_RESET_OFF();

	// GDRV enable
	palSetPadMode(GDRV_EN_GPIO, GDRV_EN_PIN, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
	ENABLE_GATE();

	// GPIOA Configuration: Channel 1 to 3 as alternate function push-pull
	palSetPadMode(GPIOA, 8, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOA, 10, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);

	palSetPadMode(GPIOB, 13, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOB, 14, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOB, 15, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);

	// Hall sensors
	palSetPadMode(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3, PAL_MODE_INPUT_PULLUP);

	// Phase filters
	palSetPadMode(PHASE_FILTER_U_GPIO, PHASE_FILTER_U_PIN, PAL_MODE_OUTPUT_OPENDRAIN);
	palSetPadMode(PHASE_FILTER_V_GPIO, PHASE_FILTER_V_PIN, PAL_MODE_OUTPUT_OPENDRAIN);
	palSetPadMode(PHASE_FILTER_W_GPIO, PHASE_FILTER_W_PIN, PAL_MODE_OUTPUT_OPENDRAIN);
	PHASE_FILTER_OFF();

	// Current filters
	palSetPadMode(CURRENT_FILTER_U_GPIO, CURRENT_FILTER_U_PIN, PAL_MODE_OUTPUT_OPENDRAIN);
	palSetPadMode(CURRENT_FILTER_V_GPIO, CURRENT_FILTER_V_PIN, PAL_MODE_OUTPUT_OPENDRAIN);
	palSetPadMode(CURRENT_FILTER_W_GPIO, CURRENT_FILTER_W_PIN, PAL_MODE_OUTPUT_OPENDRAIN);
	CURRENT_FILTER_OFF();

	// ADC Pins
	palSetPadMode(GPIOA, 0, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 1, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 2, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 3, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 4, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 5, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 6, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 7, PAL_MODE_INPUT_ANALOG);

	palSetPadMode(GPIOB, 0, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOB, 1, PAL_MODE_INPUT_ANALOG);

	palSetPadMode(GPIOC, 0, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 1, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 2, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 3, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 4, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 5, PAL_MODE_INPUT_ANALOG);

	//lispif_add_ext_load_callback(load_extensions); // ToDO check waht this is for
}

void hw_setup_adc_channels(void) {
	uint8_t sample_time = ADC_SampleTime_15Cycles;

	// ADC1 regular channels
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, sample_time);			// 0
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 2, sample_time);			// 3
	ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 3, sample_time);			// 6
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 4, sample_time);			// 9
	ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 5, sample_time);			// 12
	ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 6, sample_time);			// 15

	// ADC2 regular channels
	ADC_RegularChannelConfig(ADC2, ADC_Channel_11, 1, sample_time);			// 1
	ADC_RegularChannelConfig(ADC2, ADC_Channel_1, 2, sample_time);			// 4
	ADC_RegularChannelConfig(ADC2, ADC_Channel_14, 3, sample_time);			// 7
	ADC_RegularChannelConfig(ADC2, ADC_Channel_4, 4, sample_time);			// 10
	ADC_RegularChannelConfig(ADC2, ADC_Channel_8, 5, sample_time);			// 13
	ADC_RegularChannelConfig(ADC2, ADC_Channel_7, 6, sample_time);			// 16

	// ADC3 regular channels
	ADC_RegularChannelConfig(ADC3, ADC_Channel_12, 1, sample_time);			// 2
	ADC_RegularChannelConfig(ADC3, ADC_Channel_2, 2, sample_time);			// 5
	ADC_RegularChannelConfig(ADC3, ADC_Channel_13, 3, sample_time);			// 8
	ADC_RegularChannelConfig(ADC3, ADC_Channel_3, 4, sample_time);			// 11
	ADC_RegularChannelConfig(ADC3, ADC_Channel_13, 5, sample_time);			// 14
	ADC_RegularChannelConfig(ADC3, ADC_Channel_3, 6, sample_time);			// 17

	// Injected channels
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, 1, sample_time);
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 1, sample_time);
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, 1, sample_time);
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, 2, sample_time);
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 2, sample_time);
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, 2, sample_time);
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, 3, sample_time);
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 3, sample_time);
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, 3, sample_time);

}

void hw_start_i2c(void) {
	i2cAcquireBus(&HW_I2C_DEV);

	if (!i2c_running) {
		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);
		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		i2cStart(&HW_I2C_DEV, &i2cfg);
		i2c_running = true;
	}

	i2cReleaseBus(&HW_I2C_DEV);
}

void hw_stop_i2c(void) {
	i2cAcquireBus(&HW_I2C_DEV);

	if (i2c_running) {
		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN, PAL_MODE_INPUT);
		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN, PAL_MODE_INPUT);

		i2cStop(&HW_I2C_DEV);
		i2c_running = false;

	}

	i2cReleaseBus(&HW_I2C_DEV);
}

/**
 * Try to restore the i2c bus
 */
void hw_try_restore_i2c(void) {
	if (i2c_running) {
		i2cAcquireBus(&HW_I2C_DEV);

		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
		palSetPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);

		chThdSleep(1);

		for(int i = 0;i < 16;i++) {
			palClearPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
			chThdSleep(1);
			palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
			chThdSleep(1);
		}

		// Generate start then stop condition
		palClearPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);
		chThdSleep(1);
		palClearPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
		chThdSleep(1);
		palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
		chThdSleep(1);
		palSetPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);

		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		HW_I2C_DEV.state = I2C_STOP;
		i2cStart(&HW_I2C_DEV, &i2cfg);

		i2cReleaseBus(&HW_I2C_DEV);
	}
}

float mos_get_high_temp(void) {
	float uh = (1.0 / ((logf(NTC_RES(ADC_Value[ADC_IND_TEMP_MOS_H_U]) / 10000.0) / 3380.0) + (1.0 / 298.15)) - 273.15);
	float ul = (1.0 / ((logf(NTC_RES(ADC_Value[ADC_IND_TEMP_MOS_L_U]) / 10000.0) / 3380.0) + (1.0 / 298.15)) - 273.15);
	float vh = (1.0 / ((logf(NTC_RES(ADC_Value[ADC_IND_TEMP_MOS_H_V]) / 10000.0) / 3380.0) + (1.0 / 298.15)) - 273.15);
	float vl = (1.0 / ((logf(NTC_RES(ADC_Value[ADC_IND_TEMP_MOS_L_V]) / 10000.0) / 3380.0) + (1.0 / 298.15)) - 273.15);
	float wh = (1.0 / ((logf(NTC_RES(ADC_Value[ADC_IND_TEMP_MOS_H_W]) / 10000.0) / 3380.0) + (1.0 / 298.15)) - 273.15);
	float wl = (1.0 / ((logf(NTC_RES(ADC_Value[ADC_IND_TEMP_MOS_L_W]) / 10000.0) / 3380.0) + (1.0 / 298.15)) - 273.15);
	float res = uh;

	if (ul > res) res = ul;
	if (vh > res) res = vh;
	if (vl > res) res = vl;
	if (wh > res) res = wh;
	if (wl > res) res = wl;

	return res;
}

float mos_phase_get_high_temp(uint8_t adc_ind_h, uint8_t adc_ind_l) {
	float h = (1.0 / ((logf(NTC_RES(ADC_Value[adc_ind_h]) / 10000.0) / 3380.0) + (1.0 / 298.15)) - 273.15);
	float l = (1.0 / ((logf(NTC_RES(ADC_Value[adc_ind_l]) / 10000.0) / 3380.0) + (1.0 / 298.15)) - 273.15);
	float res = h;

	if (l > res) res = l;

	return res;
}