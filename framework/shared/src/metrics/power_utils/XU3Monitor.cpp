/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#include <metrics/power_utils/XU3Monitor.h>


bool XU3Monitor::odroid_init () {

	powerA7   = NULL;
	powerA15  = NULL;
	powerGPU  = NULL;
	powerDRAM = NULL;

	if (enableSensor(SENSOR_A7) == 0) {
		enableSensor(SENSOR_A15);
		enableSensor(SENSOR_GPU);
		enableSensor(SENSOR_DRAM);
		return true;
	} else {
		return false;
	}

}


bool  XU3Monitor::odroid_start () {
	return true;
}


bool  XU3Monitor::odroid_sample () {

	if (powerA7) this->vpowerA7 = getPower(SENSOR_A7);
	if (powerA15) this->vpowerA15 = getPower(SENSOR_A15);
	if (powerGPU) this->vpowerGPU = getPower(SENSOR_GPU);
	if (powerDRAM) this->vpowerDRAM = getPower(SENSOR_DRAM);

	return true;
}

bool  XU3Monitor::odroid_finish() {

	if (powerA7)
		fclose(powerA7);
	if (powerA15)
		fclose(powerA15);
	if (powerGPU)
		fclose(powerGPU);
	if (powerDRAM)
		fclose(powerDRAM);
	return true;
}

float XU3Monitor::getPower(XU3Sensor sensor) {
	FILE *tmp = NULL;
	float power;

		switch (sensor) {
		case SENSOR_A7:
			tmp = powerA7;
			break;
		case SENSOR_A15:
			tmp = powerA15;
			break;
		case SENSOR_GPU:
			tmp = powerGPU;
			break;
		case SENSOR_DRAM:
			tmp = powerDRAM;
			break;
		}
		if (tmp) {
			rewind(tmp);
			fscanf(tmp, "%f\n", &power);
			return (power);
		}

	return 0;
}

bool XU3Monitor::enableSensor(XU3Sensor sensor) {
	char enableFile[256];
	FILE *tmp;

		for (int dn = 1; dn < 5; dn++) {
			sprintf(enableFile, "/sys/bus/i2c/drivers/INA231/%d-00%d/enable", dn,
				sensor);

			if ((tmp = fopen(enableFile, "a"))!= 0) {
				fprintf(tmp, "1\n");
				fclose(tmp);

				sprintf(enableFile, "/sys/bus/i2c/drivers/INA231/%d-00%d/sensor_W",
					dn, sensor);
				if ((tmp = fopen(enableFile, "r")) != 0) {
					switch (sensor) {
					case SENSOR_A7:
						powerA7 = tmp;
						break;
					case SENSOR_A15:
						powerA15 = tmp;
						break;
					case SENSOR_GPU:
						powerGPU = tmp;
						break;
					case SENSOR_DRAM:
						powerDRAM = tmp;
						break;
					}
					return true;
				}
			}
	}
	return false;
}


