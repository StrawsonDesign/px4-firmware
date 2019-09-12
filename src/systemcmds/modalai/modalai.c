#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <px4_module.h>

#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

#include "chip.h"
#include "stm32_gpio.h"
#include "board_config.h"

#include <nuttx/board.h>
#include <arch/board/board.h>

#define _MK_GPIO_INPUT(def)  (((def) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_INPUT|GPIO_PULLUP))
#define _MK_GPIO_OUTPUT(def) (((def) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR))


// J1 - Primary MSS Communications Interface
#define J1_PIN2 _MK_GPIO_INPUT(GPIO_UART5_RX)
#define J1_PIN3 _MK_GPIO_OUTPUT(GPIO_UART5_TX)
#define J1_PIN4 _MK_GPIO_OUTPUT(GPIO_UART5_RTS)
#define J1_PIN6 _MK_GPIO_INPUT(GPIO_UART5_CTS)

// J2 - STM JTAG Programming Header

// J3 - USB 2.0 Full-Speed Downstream Device Port

// J4 - Spare MSS Comms
#define J4_PIN2 _MK_GPIO_OUTPUT(GPIO_USART2_RX)
#define J4_PIN3 _MK_GPIO_OUTPUT(GPIO_USART2_TX)
#define J4_PIN4 _MK_GPIO_OUTPUT(GPIO_USART2_RTS)
#define J4_PIN6 _MK_GPIO_OUTPUT(GPIO_USART2_CTS)

// J5 - TELEMETRY CONNECTOR
#define J5_PIN2 _MK_GPIO_OUTPUT(GPIO_UART7_TX)
#define J5_PIN3 _MK_GPIO_OUTPUT(GPIO_UART7_RX)
#define J5_PIN4 _MK_GPIO_OUTPUT(GPIO_UART7_CTS)
#define J5_PIN5 _MK_GPIO_OUTPUT(GPIO_UART7_RTS)

// J6 - EXPANSION CONNECTOR
#define J6_PIN2 _MK_GPIO_OUTPUT(GPIO_UART4_TX_5)
#define J6_PIN3 _MK_GPIO_OUTPUT(GPIO_UART4_RX_5)
#define J6_PIN4 _MK_GPIO_OUTPUT(GPIO_I2C3_SCL_2)
#define J6_PIN5 _MK_GPIO_OUTPUT(GPIO_I2C3_SDA_2)

// J7 - PWM Output Connector
#define J7_PIN2 _MK_GPIO_OUTPUT(GPIO_TIM1_CH4OUT_2)
#define J7_PIN3 _MK_GPIO_OUTPUT(GPIO_TIM1_CH3OUT_1)
#define J7_PIN4 _MK_GPIO_OUTPUT(GPIO_TIM1_CH2OUT_2)
#define J7_PIN5 _MK_GPIO_OUTPUT(GPIO_TIM1_CH1OUT_1)
#define J7_PIN6 _MK_GPIO_OUTPUT(GPIO_TIM4_CH2OUT_2)
#define J7_PIN7 _MK_GPIO_OUTPUT(GPIO_TIM4_CH3OUT_2)
#define J7_PIN8 _MK_GPIO_OUTPUT(GPIO_TIM4_CH1OUT_2)
#define J7_PIN9 _MK_GPIO_OUTPUT(GPIO_TIM4_CH4OUT_2)

// J8 - CAN 1 Peripheral Connector

// J9 - PPM (RC) IN
#define J9_PIN2 _MK_GPIO_INPUT(GPIO_TIM8_CH1IN_2)

// J10 - GPS CONNECTOR
#define J10_PIN2 _MK_GPIO_OUTPUT(GPIO_USART1_TX_3)
#define J10_PIN3 _MK_GPIO_OUTPUT(GPIO_USART1_RX_3)
#define J10_PIN4 _MK_GPIO_OUTPUT(GPIO_I2C1_SCL_2)
#define J10_PIN5 _MK_GPIO_OUTPUT(GPIO_I2C1_SDA_1)

// J12 - Spektrum RC Input Connector
#define J12_PIN1 GPIO_VDD_3V3_SPEKTRUM_POWER_EN
#define J12_PIN2 _MK_GPIO_OUTPUT(GPIO_USART6_TX_1)
#define J12_PIN3 _MK_GPIO_OUTPUT(GPIO_USART6_RX_1)

// J13 - I2C Display / Spare Sensor Connector
#define J13_PIN3 _MK_GPIO_OUTPUT(GPIO_I2C2_SDA_2)
#define J13_PIN4 _MK_GPIO_OUTPUT(GPIO_I2C2_SCL_2)
#define J13_PIN5 _MK_GPIO_OUTPUT(GPIO_PF3_EVENTOUT)

__EXPORT int modalai_main(int argc, char *argv[]);

static void print_usage(void)
{
	PRINT_MODULE_DESCRIPTION("ModalAI Test utility\n");


	PRINT_MODULE_USAGE_NAME_SIMPLE("modalai", "command");

	PRINT_MODULE_USAGE_COMMAND_DESCR("led", "LED Test");
	PRINT_MODULE_USAGE_COMMAND_DESCR("con", "Connector Output Test (as GPIO)");
}

static void print_usage_con_gpio_test(void)
{
	PRINT_MODULE_USAGE_NAME_SIMPLE("modalai con", "command");
	PRINT_MODULE_USAGE_COMMAND_DESCR("1",  "W<3,6> R<2,6>, <0-1>");
	PRINT_MODULE_USAGE_COMMAND_DESCR("4",  "W<2-4,6>, <0-1>");
	PRINT_MODULE_USAGE_COMMAND_DESCR("5",  "W<2-5>,   <0-1>");
	PRINT_MODULE_USAGE_COMMAND_DESCR("6",  "W<2-5>,   <0-1>");
	PRINT_MODULE_USAGE_COMMAND_DESCR("7",  "W<2-9>,   <0-1>");
	PRINT_MODULE_USAGE_COMMAND_DESCR("9",  "R<2>");
	PRINT_MODULE_USAGE_COMMAND_DESCR("10", "W<2-5>,   <0-1>");
	PRINT_MODULE_USAGE_COMMAND_DESCR("12", "W<1-3>,   <0-1>");
	PRINT_MODULE_USAGE_COMMAND_DESCR("13", "W<3-5>,   <0-1>");
}

static int con_gpio_test(uint8_t con, uint8_t pin, bool state)
{
	// validate
	switch (con) {
	// Primary MSS Communications Interface
	case 1:
		switch (pin) {
		case 2:
			stm32_configgpio(J1_PIN2);
			state = stm32_gpioread(J1_PIN2);
			break;

		case 3:
			stm32_configgpio(J1_PIN3);
			stm32_gpiowrite(J1_PIN3, state);
			break;

		case 4:
			stm32_configgpio(J1_PIN4);
			stm32_gpiowrite(J1_PIN4, state);
			break;

		case 6:
			stm32_configgpio(J1_PIN6);
			state = stm32_gpioread(J1_PIN6);
			break;

		default:
			print_usage_con_gpio_test();
			return -1;
		}

		break;

	// STM JTAG Programming Header
	case 2:
		print_usage_con_gpio_test();
		return -1;

	// USB 2.0 Full-Speed Downstream Device Port
	case 3:
		print_usage_con_gpio_test();
		return -1;

	// Spare MSS Communications Interface
	case 4:
		switch (pin) {
		case 2:
			stm32_configgpio(J4_PIN2);
			stm32_gpiowrite(J4_PIN2, state);
			break;

		case 3:
			stm32_configgpio(J4_PIN3);
			stm32_gpiowrite(J4_PIN3, state);
			break;

		case 4:
			stm32_configgpio(J4_PIN4);
			stm32_gpiowrite(J4_PIN4, state);
			break;

		case 6:
			stm32_configgpio(J4_PIN6);
			stm32_gpiowrite(J4_PIN6, state);
			break;

		default:
			print_usage_con_gpio_test();
			return -1;
		}

		break;

	// TELEMETRY CONNECTOR
	case 5:
		switch (pin) {
		case 2:
			stm32_configgpio(J5_PIN2);
			stm32_gpiowrite(J5_PIN2, state);
			break;

		case 3:
			stm32_configgpio(J5_PIN3);
			stm32_gpiowrite(J5_PIN3, state);
			break;

		case 4:
			stm32_configgpio(J5_PIN4);
			stm32_gpiowrite(J5_PIN4, state);
			break;

		case 5:
			stm32_configgpio(J5_PIN5);
			stm32_gpiowrite(J5_PIN5, state);
			break;

		default:
			print_usage_con_gpio_test();
			return -1;
		}

		break;

	// EXPANSION CONNECTOR
	case 6:
		switch (pin) {
		case 2:
			stm32_configgpio(J6_PIN2);
			stm32_gpiowrite(J6_PIN2, state);
			break;

		case 3:
			stm32_configgpio(J6_PIN3);
			stm32_gpiowrite(J6_PIN3, state);
			break;

		case 4:
			stm32_configgpio(J6_PIN4);
			stm32_gpiowrite(J6_PIN4, state);
			break;

		case 5:
			stm32_configgpio(J6_PIN5);
			stm32_gpiowrite(J6_PIN5, state);
			break;

		default:
			print_usage_con_gpio_test();
			return -1;
		}

		break;

	// PWM Output Connector
	case 7:
		switch (pin) {
		case 2:
			stm32_configgpio(J7_PIN2);
			stm32_gpiowrite(J7_PIN2, state);
			break;

		case 3:
			stm32_configgpio(J7_PIN3);
			stm32_gpiowrite(J7_PIN3, state);
			break;

		case 4:
			stm32_configgpio(J7_PIN4);
			stm32_gpiowrite(J7_PIN4, state);
			break;

		case 5:
			stm32_configgpio(J7_PIN5);
			stm32_gpiowrite(J7_PIN5, state);
			break;

		case 6:
			stm32_configgpio(J7_PIN6);
			stm32_gpiowrite(J7_PIN6, state);
			break;

		case 7:
			stm32_configgpio(J7_PIN7);
			stm32_gpiowrite(J7_PIN7, state);
			break;

		case 8:
			stm32_configgpio(J7_PIN8);
			stm32_gpiowrite(J7_PIN8, state);
			break;

		case 9:
			stm32_configgpio(J7_PIN9);
			stm32_gpiowrite(J7_PIN9, state);
			break;

		default:
			print_usage_con_gpio_test();
			return -1;
		}

		break;

	// CAN 1 Peripheral Connector
	case 8:
		print_usage_con_gpio_test();
		return -1;

	// PPM (RC) IN
	case 9:
		switch (pin) {
		case 2:
			stm32_configgpio(J9_PIN2);
			//stm32_gpiowrite(J9_PIN2, state);
			state = stm32_gpioread(J9_PIN2);
			break;

		default:
			print_usage_con_gpio_test();
			return -1;
		}

		break;

	// GPS CONNECTOR
	case 10:
		switch (pin) {
		case 2:
			stm32_configgpio(J10_PIN2);
			stm32_gpiowrite(J10_PIN2, state);
			break;

		case 3:
			stm32_configgpio(J10_PIN3);
			stm32_gpiowrite(J10_PIN3, state);
			break;

		case 4:
			stm32_configgpio(J10_PIN4);
			stm32_gpiowrite(J10_PIN4, state);
			break;

		case 5:
			stm32_configgpio(J10_PIN5);
			stm32_gpiowrite(J10_PIN5, state);
			break;

		default:
			print_usage_con_gpio_test();
			return -1;
		}

		break;

	// Micro SD Card Slot
	case 11:
		print_usage_con_gpio_test();
		return -1;

	// Spektrum RC Input Connector
	case 12:
		switch (pin) {
		case 1:
			VDD_3V3_SPEKTRUM_POWER_EN(state);
			break;

		case 2:
			__asm("nop");
			stm32_configgpio(J12_PIN2);
			stm32_gpiowrite(J12_PIN2, state);
			//state = stm32_gpioread(J12_PIN2);
			__asm("nop");
			break;

		case 3:
			stm32_configgpio(J12_PIN3);
			stm32_gpiowrite(J12_PIN3, state);
			break;

		default:
			print_usage_con_gpio_test();
			return -1;
		}

		break;

	// I2C DISPLAY / SPARE SENSOR CONNECTOR
	case 13:
		switch (pin) {
		case 3:
			stm32_configgpio(J13_PIN3);
			stm32_gpiowrite(J13_PIN3, state);
			break;

		case 4:
			stm32_configgpio(J13_PIN4);
			stm32_gpiowrite(J13_PIN4, state);
			break;

		case 5:
			stm32_configgpio(J13_PIN5);
			stm32_gpiowrite(J13_PIN5, state);
			break;

		default:
			print_usage_con_gpio_test();
			return -1;
		}

		break;
	}

	printf("GPIO - Con: %d, Pin: %d, State: %d\n", con, pin, state);
	return OK;
}

static int led_test(void)
{
	PX4_INFO("Running led test");

	stm32_configgpio(GPIO_nLED_2_RED);
	stm32_configgpio(GPIO_nLED_2_GREEN);
	stm32_configgpio(GPIO_nLED_2_BLUE);

	int i = 0;

	for (i = 0; i < 5; i++) {
		usleep(1000 * 100);
		stm32_gpiowrite(GPIO_nLED_2_RED, false);
		usleep(1000 * 100);
		stm32_gpiowrite(GPIO_nLED_2_RED, true);

		usleep(1000 * 100);
		stm32_gpiowrite(GPIO_nLED_2_GREEN, false);
		usleep(1000 * 100);
		stm32_gpiowrite(GPIO_nLED_2_GREEN, true);

		usleep(1000 * 100);
		stm32_gpiowrite(GPIO_nLED_2_BLUE, false);
		usleep(1000 * 100);
		stm32_gpiowrite(GPIO_nLED_2_BLUE, true);
	}

	return OK;
}

int modalai_main(int argc, char *argv[])
{
	if (argc <= 1) {
		print_usage();
		return 1;
	}

	if (!strcmp(argv[1], "led")) {
		return led_test();

	} else if (!strcmp(argv[1], "con")) {
		if (argc <= 2) {
			PRINT_MODULE_USAGE_COMMAND("con");
			PRINT_MODULE_USAGE_ARG("<1,4,5,6,7,9,10,12,13>", "Connector ID", false);
			PRINT_MODULE_USAGE_ARG("<uint>", "Pin Number", false);
			PRINT_MODULE_USAGE_ARG("0 | 1", "<output state> (defaults to 0)", false);
			return 1;
		}

		uint8_t con = 0;
		uint8_t pin = 0;
		bool state = false;

		if (argc > 2) {
			con = atoi(argv[2]);
		}

		if (argc > 3) {
			pin = atoi(argv[3]);
		}

		if (argc > 4) {
			state = atoi(argv[4]);
		}

		return con_gpio_test(con, pin, state);
	}

	print_usage();
	return -EINVAL;
}
