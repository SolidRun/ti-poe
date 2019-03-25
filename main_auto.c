#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <tps238x.h>

#include "linux_i2c.h"

uint8_t tps2388x_i2cAddList[NUM_OF_TPS2388x * NUM_OF_QUARD] = {0x34,0x35};

static void print_resistance(uint8_t port_num)
{
	uint8_t detectR;

	tps_GetPortDetectResistance(port_num, &detectR);
	printf("   Detection Resistance: %luOhm",
	       ((unsigned long)detectR * 1953125) / 10000);
}


int main(int argc, char *argv[])
{
	uint8_t sysPortNum, sysPortNum1, sysPortNum2, sysPortNum3, sysPortNum4,
		sysPortNum5, sysPortNum6, sysPortNum7, sysPortNum8;
	uint16_t voltage, current;
	unsigned long outNum;
	uint8_t devNum;
	uint8_t temperature;
	TPS238x_Detection_Status_t detectStatus;
	TPS238x_Classification_Status_t classStatus;
	uint8_t detectR;
	uint8_t connectcheck;
	uint8_t supplyEvents;
	TPS238x_Ports_t powerEnablePortEvents, powerGoodPortEvents,
		detectionPortEvents, classificationPortEvents,
		pcutPortEvents, disconnectPortEvents, inrushPortEvents,
		ilimPortEvents;

	linux_i2c_open_dev("/dev/i2c-0");

	sysPortNum1 = tps_RegisterPort (tps2388x_i2cAddList[0], TPS238X_PORT_1);
	sysPortNum2 = tps_RegisterPort (tps2388x_i2cAddList[0], TPS238X_PORT_2);
	sysPortNum3 = tps_RegisterPort (tps2388x_i2cAddList[0], TPS238X_PORT_3);
	sysPortNum4 = tps_RegisterPort (tps2388x_i2cAddList[0], TPS238X_PORT_4);

	sysPortNum5 = tps_RegisterPort (tps2388x_i2cAddList[1], TPS238X_PORT_1);
	sysPortNum6 = tps_RegisterPort (tps2388x_i2cAddList[1], TPS238X_PORT_2);
	sysPortNum7 = tps_RegisterPort (tps2388x_i2cAddList[1], TPS238X_PORT_3);
	sysPortNum8 = tps_RegisterPort (tps2388x_i2cAddList[1], TPS238X_PORT_4);

	PSE_TPS238X_init(OPERATING_MODE_AUTO);

	while(1)
	{
		sleep(10);

		tps_GetDeviceInputVoltage (tps2388x_i2cAddList[0], &voltage);
		outNum = ((unsigned long)voltage * 3662) / 1000;
		printf("Input Voltage: %lumV\n", outNum);

		tps_GetDeviceTemperature (tps2388x_i2cAddList[0], &temperature);
		outNum = CONVERT_TEMP((unsigned long)temperature);
		printf("Device Temperature: %lu degrees C\n", outNum);

		tps_GetDeviceFirmwareRev(tps2388x_i2cAddList[0],&firmwareversion);
		printf("Firmware Revision: 0x%02x\n", firmwareversion);

		for(sysPortNum = sysPortNum1; sysPortNum <= sysPortNum8; sysPortNum++)
		{

			if (tps_GetPortPowerGoodStatus(sysPortNum) == TPS_ON)
			{
				printf("Channel %u: ON\n", sysPortNum + 1);
				tps_GetPortMeasurements (sysPortNum, &voltage, &current);
				printf("   Voltage: %lumV Current %lumA\n",
				       ((unsigned long)voltage * 3662) / 1000,
				       ((unsigned long)current * 70190) / 1000000);
			}
			else
			{
				printf("Channel %u: OFF\n", sysPortNum + 1);
			}

			tps_GetPortDetectionStatus(sysPortNum, &detectStatus);

			switch(detectStatus)
			{
			case DETECT_UNKNOWN: uart_puts ("   Detection Status: UNKNOWN");
				break;
			case DETECT_SHORT_CIRCUIT: uart_puts ("   Detection Status: SHORT CIRCUIT");
				break;
			case DETECT_RESIST_LOW: uart_puts ("   Detection Status: RESISTANCE TOO LOW");
				print_resistance(sysPortNum);
				break;
			case DETECT_RESIST_VALID: uart_puts ("   Detection Status: RESISTANCE VALID");
				print_resistance(sysPortNum);
				break;
			case DETECT_RESIST_HIGH: uart_puts ("   Detection Status: RESISTANCE TOO HIGH");
				print_resistance(sysPortNum);
				break;
			case DETECT_OPEN_CIRCUIT: uart_puts ("   Detection Status: OPEN CIRCUIT");
				break;
			case DETECT_MOSFET_FAULT: uart_puts ("   Detection Status: MOSFET FAULT");
				break;
			default:
				break;
			}

			tps_GetPortRequestedClassificationStatus(sysPortNum,&classStatus);
			switch(classStatus)
			{
			case 0x00: uart_puts ("\r\n   Classification Status: Unknown \r\n");
				break;
			case 0x01: uart_puts ("\r\n   Classification Status: Class 1 \r\n");
				break;
			case 0x02: uart_puts ("\r\n   Classification Status: Class 2 \r\n");
				break;
			case 0x03: uart_puts ("\r\n   Classification Status: Class 3 \r\n");
				break;
			case 0x04: uart_puts ("\r\n   Classification Status: Class 4 \r\n");
				break;
			case 0x05: uart_puts ("\r\n   Classification Status: Class 0 \r\n");
				break;
			case 0x06: uart_puts ("\r\n   Classification Status: Class 0 \r\n");
				break;
			case 0x07: uart_puts ("\r\n   Classification Status: Over current \r\n");
				break;
			case 0x08: uart_puts ("\r\n   Classification Status: Class 5, 4 Pair Single Signature \r\n");
				break;
			case 0x09: uart_puts ("\r\n   Classification Status: Class 6, 4 Pair Single Signature \r\n");
				break;
			case 0x0A: uart_puts ("\r\n   Classification Status: Class 7, 4 Pair Single Signature \r\n");
				break;
			case 0x0B: uart_puts ("\r\n   Classification Status: Class 8, 4 Pair Single Signature \r\n");
				break;
			case 0x0C: uart_puts ("\r\n   Classification Status: Class 4+, Type 1 Limited \r\n");
				break;
			case 0x0D: uart_puts ("\r\n   Classification Status: Class 5, 4 Pair Dual Signature \r\n");
				break;
			case 0x0E: uart_puts ("\r\n   Classification Status: UNKNOWN \r\n");
				break;
			case 0x0F: uart_puts ("\r\n   Classification Status: Class Mismatch \r\n");
				break;
			default:
				break;
			}

			if(sysPortNum > sysPortNum4)
			{
				tps_GetPortConnectionCheckStatus(sysPortNum,&connectcheck);

				switch(connectcheck)
				{
				case 0x00: uart_puts("   Connection Check Status: Unknown \r\n");
					break;
				case 0x01: uart_puts("   Connection Check Status: 4-Pair Single Signature PD \r\n");
					break;
				case 0x02: uart_puts("   Connection Check Status: 4-Pair Dual Signature PD \r\n");
					break;
				case 0x03: uart_puts("   Connection Check Status: Invalid Signature Detection \r\n");
					break;
				default:
					break;
				}
			}
		}


		for (devNum = 0; devNum < (NUM_OF_TPS2388x*NUM_OF_QUARD); devNum++)
		{


			// read current value of all event registers (Do not clear)
			tps_GetDeviceAllInterruptEvents (tps2388x_i2cAddList[devNum], TPS_OFF, &powerEnablePortEvents, &powerGoodPortEvents, &detectionPortEvents,
							 &classificationPortEvents, &pcutPortEvents, &disconnectPortEvents,
							 &inrushPortEvents, &ilimPortEvents, &supplyEvents);

			printf("\n---- Event Registers -----Dev : %u -----\n",
			       devNum);
			printf("  0x%02x  0x%02x  0x%02x  0x%02x  0x%02x\n",
			       (powerGoodPortEvents<<4) | powerEnablePortEvents,
			       (classificationPortEvents << 4) | detectionPortEvents,
			       (disconnectPortEvents << 4) | pcutPortEvents,
			       (ilimPortEvents << 4) | inrushPortEvents,
			       supplyEvents << 4);

			uart_puts ("---- Port Status -----\r\n");
			sysPortNum = tps_GetSystemPortNumber (tps2388x_i2cAddList[devNum], TPS238X_PORT_1);
			tps_GetPortDetectRequestedClassStatus (sysPortNum, &detectStatus, &classStatus);
			printf("0x%02x", (classStatus<<4) | detectStatus);
			sysPortNum = tps_GetSystemPortNumber (tps2388x_i2cAddList[devNum], TPS238X_PORT_2);
			tps_GetPortDetectRequestedClassStatus (sysPortNum, &detectStatus, &classStatus);
			printf("0x%02x", (classStatus<<4) | detectStatus);
			sysPortNum = tps_GetSystemPortNumber (tps2388x_i2cAddList[devNum], TPS238X_PORT_3);
			tps_GetPortDetectRequestedClassStatus (sysPortNum, &detectStatus, &classStatus);
			printf("0x%02x", (classStatus<<4) | detectStatus);
			sysPortNum = tps_GetSystemPortNumber (tps2388x_i2cAddList[devNum], TPS238X_PORT_4);
			tps_GetPortDetectRequestedClassStatus (sysPortNum, &detectStatus, &classStatus);
			printf("0x%02x\n\n", (classStatus<<4) | detectStatus);
			uart_puts ("---- Power Status -----\r\n");
			tps_GetDevicePowerStatus (tps2388x_i2cAddList[devNum], &powerEnablePortEvents, &powerGoodPortEvents);
			printf("0x%02x\n\n", (powerGoodPortEvents<<4) | powerEnablePortEvents);

			uart_puts ("---------------------------------------\r\n\n\n");
		}
	}

	return 0;
}
