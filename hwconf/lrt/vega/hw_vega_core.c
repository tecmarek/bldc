#include "hw.h"
#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "utils_math.h"
#include <math.h>
#include "mc_interface.h"
#include "lispif.h"
#include "lispbm.h"
#include "terminal.h"
#include "commands.h"
#include "mcpwm.h"
#include "mcpwm_foc.h"
#include "mempools.h"
#include "timeout.h"

//TCAL6416
#define TCAL_ADDR						0x20
#define TCAL_TIMEOUT					MS2ST(5)

typedef enum {
	TCAL_Input_Port_0_ADDR 							= 0x00,
	TCAL_Input_Port_1_ADDR 							= 0x01,
	TCAL_Output_Port_0_ADDR 						= 0x02,
	TCAL_Output_Port_1_ADDR 						= 0x03,
	TCAL_Polarity_Inversion_Port_0_ADDR 			= 0x04,
	TCAL_Polarity_Inversion_Port_1_ADDR 			= 0x05,
	TCAL_Configuration_Port_0_ADDR 					= 0x06,
	TCAL_Configuration_Port_1_ADDR 					= 0x07,
	TCAL_Output_Drive_Strength_Port_00_ADDR 		= 0x40,
	TCAL_Output_Drive_Strength_Port_01_ADDR 		= 0x41,
	TCAL_Output_Drive_Strength_Port_10_ADDR 		= 0x42,
	TCAL_Output_Drive_Strength_Port_11_ADDR 		= 0x43,
	TCAL_Input_latch_register_Port_0_ADDR 			= 0x44,
	TCAL_Input_latch_register_Port_1_ADDR 			= 0x45,
	TCAL_PU_PD_enable_Port_0_ADDR 					= 0x46,
	TCAL_PU_PD_enable_Port_1_ADDR 					= 0x47,
	TCAL_PU_PD_selection_Port_0_ADDR 				= 0x48,
	TCAL_PU_PD_selection_Port_1_ADDR 				= 0x49,
	TCAL_Interrupt_mask_register_Port_0_ADDR 		= 0x4A,
	TCAL_Interrupt_mask_register_Port_1_ADDR 		= 0x4B,
	TCAL_Interrupt_status_register_Port_0_ADDR 		= 0x4C,
	TCAL_Interrupt_status_register_Port_1_ADDR 		= 0x4D,
	TCAL_Output_port_configuration_register_ADDR 	= 0x4F
} TCAL_RegistersAddrTypeDef;

typedef struct {
	uint8_t
		GPIO_0 : 1,
		GPIO_1 : 1,
		GPIO_2 : 1,
		GPIO_3 : 1,
		GPIO_4 : 1,
		GPIO_5 : 1,
		GPIO_6 : 1,
		GPIO_7 : 1;
} TCAL_GPIOTypeDef; 

typedef union {
	uint8_t generic;
	TCAL_GPIOTypeDef IO;
} TCAL_RegisterUnionTypeDef;


static mutex_t tcal_mtx;
TCAL_RegisterUnionTypeDef SCTL_GPIO_Port0 = {0};
TCAL_RegisterUnionTypeDef SCTL_GPIO_Port1 = {0};
TCAL_RegisterUnionTypeDef SCTL_INT_GPIO_Port0 = {0xFF};
TCAL_RegisterUnionTypeDef SCTL_INT_GPIO_Port1 = {0xFF};
TCAL_RegisterUnionTypeDef SCTL_INT_Status_GPIO_Port0 = {0};
TCAL_RegisterUnionTypeDef SCTL_INT_Status_GPIO_Port1 = {0};

// Terminal functions
static void terminal_cmd_get_sctl_state(int argc, const char **argv);
static void terminal_cmd_get_sctl_last_fault(int argc, const char **argv);
static void terminal_cmd_doublepulse(int argc, const char** argv);

// Variables
static volatile bool i2c_running = false;
static THD_WORKING_AREA(mux_thread_wa, 256);
static THD_FUNCTION(mux_thread, arg);
static volatile bool mux_thd_running = false;

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

	if (!mux_thd_running) {
		chMtxObjectInit(&tcal_mtx);
		chThdCreateStatic(mux_thread_wa, sizeof(mux_thread_wa), NORMALPRIO, mux_thread, NULL);
		mux_thd_running = true;
	}

	terminal_register_command_callback(
			"sctl_state",
			"Print SCTL state",
			0,
			terminal_cmd_get_sctl_state);

	terminal_register_command_callback(
			"sctl_fault",
			"Print SCTL last registered fault",
			0,
			terminal_cmd_get_sctl_last_fault);

	terminal_register_command_callback(
		"double_pulse",
		"Start a double pulse test",
		0,
		terminal_cmd_doublepulse);
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

uint8_t check_drv_fault(void) {
	if(palReadPad(SCTL_INT_GPIO, SCTL_INT_PIN) == PAL_LOW){
		return 1;
	}

	if(SCTL_GPIO_Port0.generic != 0xFF || SCTL_GPIO_Port1.generic != 0xFF){
		return 1;
	}

	return 0;
}

static void terminal_cmd_get_sctl_state(int argc, const char **argv) {
	(void)argc;
	(void)argv;

	chMtxLock(&tcal_mtx);

	commands_printf("SCTL State:");
	commands_printf("Signals in fault state will indicate as 0\n");

	commands_printf("Phase U - Fault High   : %d", SCTL_GPIO_Port0.IO.GPIO_0);
	commands_printf("Phase U - Fault Low    : %d", SCTL_GPIO_Port0.IO.GPIO_1);
	commands_printf("Phase U - Power Good   : %d", SCTL_GPIO_Port0.IO.GPIO_2);
	commands_printf("Phase U - Over Current : %d", SCTL_GPIO_Port0.IO.GPIO_3);

	commands_printf("Phase V - Fault High   : %d", SCTL_GPIO_Port0.IO.GPIO_4);
	commands_printf("Phase V - Fault Low    : %d", SCTL_GPIO_Port0.IO.GPIO_5);
	commands_printf("Phase V - Power Good   : %d", SCTL_GPIO_Port0.IO.GPIO_6);
	commands_printf("Phase V - Over Current : %d", SCTL_GPIO_Port0.IO.GPIO_7);

	commands_printf("Phase W - Fault High   : %d", SCTL_GPIO_Port1.IO.GPIO_3);
	commands_printf("Phase W - Fault Low    : %d", SCTL_GPIO_Port1.IO.GPIO_2);
	commands_printf("Phase W - Power Good   : %d", SCTL_GPIO_Port1.IO.GPIO_1);
	commands_printf("Phase W - Over Current : %d", SCTL_GPIO_Port1.IO.GPIO_0);

	commands_printf("Logic - SDC State      : %d", SCTL_GPIO_Port1.IO.GPIO_4);

	if(check_drv_fault()){
		commands_printf("Active DRV fault");
	}

	chMtxUnlock(&tcal_mtx);
}

static void print_sctl_fault(const char *label, int value, int int_status) {
	// Print value if GPIO is at 0 or if it triggered the interrupt
    if (value == 0 || int_status == 1) {
        commands_printf("%-22s : %d%s", 
                        label, 
                        value, 
                        (int_status == 1) ? " [t]" : "");
    }
}

static void terminal_cmd_get_sctl_last_fault(int argc, const char **argv) {
	(void)argc;
	(void)argv;

	chMtxLock(&tcal_mtx);

	commands_printf("SCTL last Fault:");
	commands_printf("Signals in fault state will indicate as 0. The trigger sources of the interrupt are marked with [t]\n");

    print_sctl_fault("Phase U - Fault High",   SCTL_INT_GPIO_Port0.IO.GPIO_0, SCTL_INT_Status_GPIO_Port0.IO.GPIO_0);
    print_sctl_fault("Phase U - Fault Low",    SCTL_INT_GPIO_Port0.IO.GPIO_1, SCTL_INT_Status_GPIO_Port0.IO.GPIO_1);
    print_sctl_fault("Phase U - Power Good",   SCTL_INT_GPIO_Port0.IO.GPIO_2, SCTL_INT_Status_GPIO_Port0.IO.GPIO_2);
    print_sctl_fault("Phase U - Over Current", SCTL_INT_GPIO_Port0.IO.GPIO_3, SCTL_INT_Status_GPIO_Port0.IO.GPIO_3);

    print_sctl_fault("Phase V - Fault High",   SCTL_INT_GPIO_Port0.IO.GPIO_4, SCTL_INT_Status_GPIO_Port0.IO.GPIO_4);
    print_sctl_fault("Phase V - Fault Low",    SCTL_INT_GPIO_Port0.IO.GPIO_5, SCTL_INT_Status_GPIO_Port0.IO.GPIO_5);
    print_sctl_fault("Phase V - Power Good",   SCTL_INT_GPIO_Port0.IO.GPIO_6, SCTL_INT_Status_GPIO_Port0.IO.GPIO_6);
    print_sctl_fault("Phase V - Over Current", SCTL_INT_GPIO_Port0.IO.GPIO_7, SCTL_INT_Status_GPIO_Port0.IO.GPIO_7);

    print_sctl_fault("Phase W - Fault High",   SCTL_INT_GPIO_Port1.IO.GPIO_3, SCTL_INT_Status_GPIO_Port1.IO.GPIO_3);
    print_sctl_fault("Phase W - Fault Low",    SCTL_INT_GPIO_Port1.IO.GPIO_2, SCTL_INT_Status_GPIO_Port1.IO.GPIO_2);
    print_sctl_fault("Phase W - Power Good",   SCTL_INT_GPIO_Port1.IO.GPIO_1, SCTL_INT_Status_GPIO_Port1.IO.GPIO_1);
    print_sctl_fault("Phase W - Over Current", SCTL_INT_GPIO_Port1.IO.GPIO_0, SCTL_INT_Status_GPIO_Port1.IO.GPIO_0);

    print_sctl_fault("Logic - SDC State",      SCTL_INT_GPIO_Port1.IO.GPIO_4, SCTL_INT_Status_GPIO_Port1.IO.GPIO_4);

	if(check_drv_fault()){
		commands_printf("Active DRV fault");
	}

	chMtxUnlock(&tcal_mtx);
}

msg_t tcal_write_reg(TCAL_RegistersAddrTypeDef reg, TCAL_RegisterUnionTypeDef *data) {
	uint8_t txbuf[2] = {reg, data->generic};

	i2cAcquireBus(&HW_I2C_DEV);
	msg_t status = i2cMasterTransmitTimeout(&HW_I2C_DEV, TCAL_ADDR, txbuf, 2, NULL, 0, TCAL_TIMEOUT);
	i2cReleaseBus(&HW_I2C_DEV);

	return status;
}

msg_t tcal_read_reg(TCAL_RegistersAddrTypeDef reg, TCAL_RegisterUnionTypeDef *data) {
	uint8_t txbuf[1] = {reg};

	i2cAcquireBus(&HW_I2C_DEV);
	msg_t status = i2cMasterTransmitTimeout(&HW_I2C_DEV, TCAL_ADDR, txbuf, 1, (uint8_t *)data, 1, TCAL_TIMEOUT);
	i2cReleaseBus(&HW_I2C_DEV);

	return status;
}

static THD_FUNCTION(mux_thread, arg) {
	(void)arg;

	chRegSetThreadName("SCTL");

	//msg_t status = MSG_OK;

	hw_start_i2c();
	chThdSleepMilliseconds(10);

	// Setup inputs
	TCAL_RegisterUnionTypeDef SCTL_PU_PD_state = {0xFF};

	tcal_write_reg(TCAL_PU_PD_enable_Port_0_ADDR, &SCTL_PU_PD_state);
	tcal_write_reg(TCAL_PU_PD_enable_Port_1_ADDR, &SCTL_PU_PD_state);

	TCAL_RegisterUnionTypeDef SCTL_PU_PD_direction_port_0 = {0};
	SCTL_PU_PD_direction_port_0.IO.GPIO_2 = 1;
	SCTL_PU_PD_direction_port_0.IO.GPIO_6 = 1;
	TCAL_RegisterUnionTypeDef SCTL_PU_PD_direction_port_1 = {0};
	SCTL_PU_PD_direction_port_1.IO.GPIO_1 = 1;
	SCTL_PU_PD_direction_port_1.IO.GPIO_5 = 1;
	SCTL_PU_PD_direction_port_1.IO.GPIO_6 = 1;
	SCTL_PU_PD_direction_port_1.IO.GPIO_7 = 1;

	tcal_write_reg(TCAL_PU_PD_selection_Port_0_ADDR, &SCTL_PU_PD_direction_port_0);
	tcal_write_reg(TCAL_PU_PD_selection_Port_1_ADDR, &SCTL_PU_PD_direction_port_1);

	// Invert polarity for Power Good pins
	TCAL_RegisterUnionTypeDef SCTL_inv_port_0 = {0};
	SCTL_inv_port_0.IO.GPIO_2 = 1;
	SCTL_inv_port_0.IO.GPIO_6 = 1;

	TCAL_RegisterUnionTypeDef SCTL_inv_port_1 = {0};
	SCTL_inv_port_1.IO.GPIO_1 = 1;

	tcal_write_reg(TCAL_Polarity_Inversion_Port_0_ADDR, &SCTL_inv_port_0);
	tcal_write_reg(TCAL_Polarity_Inversion_Port_1_ADDR, &SCTL_inv_port_1);

	// Setup latch
	TCAL_RegisterUnionTypeDef SCTL_active_latch_port_0 = {0xFF}; // All
	TCAL_RegisterUnionTypeDef SCTL_active_latch_port_1 = {0xFF};
	SCTL_active_latch_port_1.IO.GPIO_5 = 0;
	SCTL_active_latch_port_1.IO.GPIO_6 = 0;
	SCTL_active_latch_port_1.IO.GPIO_7 = 0;

	tcal_write_reg(TCAL_Input_latch_register_Port_0_ADDR, &SCTL_active_latch_port_0);
	tcal_write_reg(TCAL_Input_latch_register_Port_1_ADDR, &SCTL_active_latch_port_1);

	// Setup interrupt
	TCAL_RegisterUnionTypeDef SCTL_active_iterrupt_port_0 = {~SCTL_active_latch_port_0.generic}; // Invert as active mask state is 0
	TCAL_RegisterUnionTypeDef SCTL_active_iterrupt_port_1 = {~SCTL_active_latch_port_1.generic}; // Invert as active mask state is 0

	tcal_write_reg(TCAL_Interrupt_mask_register_Port_0_ADDR, &SCTL_active_iterrupt_port_0);
	tcal_write_reg(TCAL_Interrupt_mask_register_Port_1_ADDR, &SCTL_active_iterrupt_port_1);

	for (;;) {

		chMtxLock(&tcal_mtx);

		uint8_t tcal_interrupt = palReadPad(SCTL_INT_GPIO, SCTL_INT_PIN);

		if(tcal_interrupt == PAL_LOW){ // Interrupt active
			//Read interrupt status
			tcal_read_reg(TCAL_Interrupt_status_register_Port_0_ADDR, &SCTL_INT_Status_GPIO_Port0);
			tcal_read_reg(TCAL_Interrupt_status_register_Port_1_ADDR, &SCTL_INT_Status_GPIO_Port1);
		}

		// Reading input registers also clears interrupt
		tcal_read_reg(TCAL_Input_Port_0_ADDR, &SCTL_GPIO_Port0);
		tcal_read_reg(TCAL_Input_Port_1_ADDR, &SCTL_GPIO_Port1);

		if(tcal_interrupt == PAL_LOW){ // Interrupt active
			// Store interrupt snapshot
			SCTL_INT_GPIO_Port0 = SCTL_GPIO_Port0;
			SCTL_INT_GPIO_Port1 = SCTL_GPIO_Port1;
		}

		chMtxUnlock(&tcal_mtx);

		chThdSleepMilliseconds(250);
	}
}

static void terminal_cmd_doublepulse(int argc, const char** argv)
{
	(void)argc;
	(void)argv;

	int preface, pulse1, breaktime, pulse2;
	int utick;
	int deadtime = -1;

	TIM_TimeBaseInitTypeDef	 TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_BDTRInitTypeDef TIM_BDTRInitStructure;

	if (argc < 5) {
		commands_printf("Usage: double_pulse <preface> <pulse1> <break> <pulse2> [deadtime]");
		commands_printf("	preface: idle time in us");
		commands_printf("	 pulse1: high time of pulse 1 in us");
		commands_printf("	  break: break between pulses in us");
		commands_printf("	 pulse2: high time of pulse 2 in us");
		commands_printf("  deadtime: overwrite deadtime, in ns");
		return;
	}
	sscanf(argv[1], "%d", &preface);
	sscanf(argv[2], "%d", &pulse1);
	sscanf(argv[3], "%d", &breaktime);
	sscanf(argv[4], "%d", &pulse2);
	if (argc == 6) {
		sscanf(argv[5], "%d", &deadtime);
	}
	timeout_configure_IWDT_slowest();

	utick = (int)(SYSTEM_CORE_CLOCK / 1000000);
	mcpwm_deinit();
	mcpwm_foc_deinit();

	TIM_Cmd(TIM1, DISABLE);
	TIM_Cmd(TIM4, DISABLE);
	//TIM4 als Trigger Timer
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	TIM_TimeBaseStructure.TIM_Period = (SYSTEM_CORE_CLOCK / 20000);
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	TIM_SelectMasterSlaveMode(TIM4, TIM_MasterSlaveMode_Enable);
	TIM_SelectOutputTrigger(TIM4, TIM_TRGOSource_Enable);
	TIM4->CNT = 0;

	// TIM1
	// TIM1 clock enable
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	// Time Base configuration
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = (preface + pulse1) * utick;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	// Channel 1, 2 and 3 Configuration in PWM mode
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_Pulse = preface * utick;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;

	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

	TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM2);
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);

	TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_Inactive);
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);

	TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_Inactive);
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);
	TIM_GenerateEvent(TIM1, TIM_EventSource_COM);


	// Automatic Output enable, Break, dead time and lock configuration
	TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
	TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
	TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
	if (deadtime < 0) {
		TIM_BDTRInitStructure.TIM_DeadTime = conf_general_calculate_deadtime(HW_DEAD_TIME_NSEC, SYSTEM_CORE_CLOCK);
	} else {
		TIM_BDTRInitStructure.TIM_DeadTime = conf_general_calculate_deadtime(deadtime, SYSTEM_CORE_CLOCK);
	}
	TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
	TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
	TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;
	TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);

	TIM_CCPreloadControl(TIM1, ENABLE);
	TIM_ARRPreloadConfig(TIM1, ENABLE);

	TIM1->CNT = 0;
	TIM1->EGR = TIM_EGR_UG;

	TIM_SelectSlaveMode(TIM1, TIM_SlaveMode_Trigger);
	TIM_SelectInputTrigger(TIM1, TIM_TS_ITR3);
	TIM_SelectOnePulseMode(TIM1, TIM_OPMode_Single);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);

	TIM_Cmd(TIM1, ENABLE);
	//Timer 4 triggert Timer 1
	TIM_Cmd(TIM4, ENABLE);
	TIM_Cmd(TIM4, DISABLE);
	TIM1->ARR = (breaktime + pulse2) * utick;
	TIM1->CCR1 = breaktime * utick;
	while (TIM1->CNT != 0);
	TIM_Cmd(TIM4, ENABLE);

	chThdSleepMilliseconds(1);
	TIM_CtrlPWMOutputs(TIM1, DISABLE);
	mc_configuration* mcconf = mempools_alloc_mcconf();
	*mcconf = *mc_interface_get_configuration();

	switch (mcconf->motor_type) {
	case MOTOR_TYPE_BLDC:
	case MOTOR_TYPE_DC:
		mcpwm_init(mcconf);
		break;

	case MOTOR_TYPE_FOC:
		mcpwm_foc_init(mcconf, mcconf);
		break;

	default:
		break;
	}
	commands_printf("Done");
	mempools_free_mcconf(mcconf);
	return;
}