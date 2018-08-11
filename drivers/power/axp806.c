/*
 * (C) Copyright 2012
 * Henrik Nordstrom <henrik@henriknordstrom.net>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <command.h>
#include <asm/arch/pmic_bus.h>
#include <axp_pmic.h>

static u8 axp806_mvolt_to_cfg(int mvolt, int min, int max, int div)
{
	if (mvolt < min)
		mvolt = min;
	else if (mvolt > max)
		mvolt = max;

	return (mvolt - min) / div;
}

static int axp806_set_aldo2(int set_vol, int onoff)
{
	u8 reg_value;

	if(set_vol > 0) {
		set_vol = axp806_mvolt_to_cfg(set_vol, 700, 3300, 100);
		if(pmic_bus_read(AXP806_ALDO2_VOLTAGE, &reg_value)) {
			return -1;
		}
		reg_value = ((reg_value & 0xE0) | set_vol);
		if(pmic_bus_write(AXP806_ALDO2_VOLTAGE, reg_value)) {
			printf("sunxi pmu error : unable to set aldo2\n");
			return -1;
		}
	}
	if(onoff < 0) {
		return 0;
	}
	if(pmic_bus_read(AXP806_OUTPUT_CTRL_1, &reg_value)) {
		return -1;
	}
	if(onoff == 0) {
		reg_value &= ~(1 << 6);
	} else {
		reg_value |=  (1 << 6);
	}
	if(pmic_bus_write(AXP806_OUTPUT_CTRL_1, reg_value)) {
		printf("sunxi pmu error : unable to onoff aldo2\n");
		return -1;
	}
	printf("AXP806: Set ALDO2 %d, OK\n", set_vol);
	return 0;
}

int axp_set_aldo2(unsigned int mvolt)
{
	return axp806_set_aldo2(mvolt, 1);
}

int axp_init(void)
{
	u8 ver;
	int i, rc;

	rc = pmic_bus_init();
	if (rc)
		return rc;

	rc = pmic_bus_read(AXP806_CHIP_VERSION, &ver);
	if (rc)
		return rc;

	/* Low 4 bits is chip version */
	ver &= 0x0f;

	if (ver != 0x1)
		return -EINVAL;

	/* Mask all interrupts */
	rc = pmic_bus_write(AXP806_IRQ_ENABLE1, 0);
	if (rc)
		return rc;

	rc = pmic_bus_write(AXP806_IRQ_ENABLE2, 0x03);
	if (rc) {
		printf("AXP806: enable irq failed\n");
		return rc;
	}

	/*
	 * Turn off LDOIO regulators / tri-state GPIO pins, when rebooting
	 * from android these are sometimes on.
	 */
	rc = pmic_bus_write(AXP_GPIO0_CTRL, AXP_GPIO_CTRL_INPUT);
	if (rc)
		return rc;

	rc = pmic_bus_write(AXP_GPIO1_CTRL, AXP_GPIO_CTRL_INPUT);
	if (rc)
		return rc;

	rc = pmic_bus_write(AXP_GPIO2_CTRL, AXP_GPIO_CTRL_INPUT);
	if (rc)
		return rc;

	return 0;
}

int axp_key(void)
{
	u8 value;
	int rc = 0;

	rc = pmic_bus_read(AXP806_IRQ_STS2, &value);
	if(rc) {
		printf("AXP806: int status2 error\n");
		return 0;
	}
	if(value) {
		printf("\nAXP806: power key2: %x\n", value);
	}

	return value;
}

// int do_poweroff(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
// {
// 	pmic_bus_write(AXP806_SHUTDOWN, AXP806_POWEROFF);

// 	/* infinite loop during shutdown */
// 	while (1) {}

// 	/* not reached */
// 	return 0;
// }
