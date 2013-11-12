#include "config.h"

#define _forever while(1)


int main(void)
{
	startInit();

	run();

	_forever{/*do nothing*/}

	return 0;
}

int startInit()
{
	initTiming();
	initInterrupts();
	initTimers();
	initPWM();
	initInterfaces();
	initDataOutput();

	addUpdateHandler(STATE_UPDATE_TIMER, updatePID);
	addUpdateHandler(STATE_UPDATE_TIMER, updateMotors);
	addUpdateHandler(STATE_UPDATE_TIMER, printDebugInfo);

	addUpdateHandler(SENSOR_UPDATE_TIMER, readSensorData);

	initMotors();
}

int initMotors(void)
{
	// set maximum power
	setMotorPower1f(1.0f);

	// wait til start and upper limit set
	sleep(MOTOR_START_DELAY);

	// set bottom limit
	setMotorPower1f(0.0f);

	// wait til set
	sleep(MOTOR_SET_LIMIT_DELAY);

	return 1;
}

int initDataOutput(void)
{
	// init console port
	initUSART(CONSOLE_INTERFACE);

	printf("Welcome to the TOP SECRET debug interface\n");
	printf("current state:\n");
}

void printDebugInfo(void *copter_state)
{
	// output sensor state, motor power
	// sattelite-based location, cellular signal power
	// current battery power, rate time to empty
	// speed, distance crossed
}

volatile pwmInputHandler(int source)
{
	static size_t counter[ROTOR_COUNT];
	static size_t full_interval;

	if(source->up)
	{
		counter[source->id] = getCurrentTime();
	}
	else
	{
		copter_state.motor[source->id] = (float)(counter[source->id] - getCurrentTime())/full_interval;

		counter[source->id] = getCurrentTime();
	}
}

void run(void)
{
	runTimer(SENSOR_UPDATE_TIMER);
	runTimer(STATE_UPDATE_TIMER);
}

void readSensorData(void)
{
	static RawSensorState sensor_data;

	sensor_data = getRawSensorsData();

	copter_state.linear_boost = getAccelData(sensor_data);
	copter_state.angle_state = getAngleState(sensor_data);
	copter_state.angle_boost = getAngleBoost(sensor_data);
	copter_state.magnetic_field = getMagneticFlowDirection(sensor_data);

	copter_state.altitude = getAltitude(sensor_data);
	copter_state.temperature = getTemperature(sensor_data);

	copter_state.location = getGlobalLocation(sensor_data);
}