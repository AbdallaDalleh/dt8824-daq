#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>

#include <vector>
#include <string>
#include <iostream>
#include <numeric>

#include <epicsExport.h>
#include <asynPortDriver.h>
#include <asynOctetSyncIO.h>
#include <iocsh.h>
#include <epicsThread.h>

using std::vector;
using std::string;
using std::cout;
using std::endl;
using std::accumulate;

#define ADMIN_LOGIN		":SYST:PASS:CEN admin\r\n"
#define CHANNEL_ENABLE	":AD:ENAB ON, (@1,2,3,4)\r\n"
#define FREQUENCY_SET	":AD:CLOC:FREQ %d\r\n"
#define FREQUENCY_GET	":AD:CLOCK:FREQ?\r\n"
#define ACQ_STOP		":AD:ABOR\r\n"
#define ACQ_ARM			":AD:ARM\r\n"
#define ACQ_INIT		":AD:INIT\r\n"
#define ACQ_LAST_INDEX	":AD:STAT:SCAN?\r\n"
#define ACQ_FETCH		":AD:FETCH? %d,%d\r\n"
#define SYSTEM_ERROR	":SYSTEM:ERROR?\r\n"

#define P_VOLTAGE_1		"voltage_ch0"
#define P_VOLTAGE_2		"voltage_ch1"
#define P_VOLTAGE_3		"voltage_ch2"
#define P_VOLTAGE_4		"voltage_ch3"
#define P_VOLTAGE_AVG_1	"avg_voltage_ch0"
#define P_VOLTAGE_AVG_2	"avg_voltage_ch1"
#define P_VOLTAGE_AVG_3	"avg_voltage_ch2"
#define P_VOLTAGE_AVG_4	"avg_voltage_ch3"
#define P_FREQUENCY		"frequency"
#define P_AVERAGE_TIME	"average_time"
#define P_ERROR			"error"

#define NUMBER_OF_CHANNELS	4
#define MIN_RX_BYTES		45
#define MIN_DATA_LENGTH		36
#define EMPTY_SCAN_LENGTH	25

class DT8824 : public asynPortDriver
{
public:
	DT8824(const char* port_name, const char* name, int frequency, int buffer_size);
	virtual asynStatus readFloat64(asynUser* pasynUser, epicsFloat64* value);
	virtual asynStatus writeFloat64(asynUser* pasynUser, epicsFloat64 value);
	virtual asynStatus writeInt32(asynUser* pasynUser, epicsInt32 value);
	virtual asynStatus readOctet(asynUser *pasynUser, char *value, size_t maxChars, size_t *nActual, int *eomReason);

	void sendCommand(string cmd, int* value);
	void readCommand(string cmd, char* buffer);
	void performDAQ();
	void performAveraging();
	int  bytes_to_int(char* buffer);

protected:
	int index_voltage_1;
	int index_voltage_2;
	int index_voltage_3;
	int index_voltage_4;

	int index_avg_voltage_1;
	int index_avg_voltage_2;
	int index_avg_voltage_3;
	int index_avg_voltage_4;

	int index_frequency;
	int index_average_time;
	int index_error;

private:
	asynUser* asyn_user;

	vector<double> channels[4];
	double voltages[4];
	double avg_voltages[4];
	pthread_t daq_thread;
	pthread_t averaging_thread;
	pthread_mutex_t avg_mutex;
	size_t max_buffer_size;
	bool frequency_changed;
	double average_time;
};

static void* start_daq_thread(void* pvt)
{
	DT8824* dt = (DT8824*) pvt;
	dt->performDAQ();
	return NULL;
}

static void* start_averaging_thread(void* pvt)
{
	DT8824* dt = (DT8824*) pvt;
	dt->performAveraging();
	return NULL;
}

DT8824::DT8824(const char* port_name, const char* name, int frequency, int buffer_size)
	: asynPortDriver(port_name,
					1,
					asynFloat64Mask | asynInt32Mask | asynOctetMask | asynDrvUserMask,
					asynFloat64Mask | asynInt32Mask | asynOctetMask,
					ASYN_MULTIDEVICE | ASYN_CANBLOCK,
					1, 0, 0)
{
	int status;
	status = pasynOctetSyncIO->connect(name, 1, &this->asyn_user, NULL);

	if(status != asynSuccess)
	{
		// TODO: Use asynPrint
		printf("Could not connect to port %s\n", port_name);
		return;
	}

	createParam(P_VOLTAGE_1, asynParamFloat64, &index_voltage_1);
	createParam(P_VOLTAGE_2, asynParamFloat64, &index_voltage_2);
	createParam(P_VOLTAGE_3, asynParamFloat64, &index_voltage_3);
	createParam(P_VOLTAGE_4, asynParamFloat64, &index_voltage_4);
	createParam(P_VOLTAGE_AVG_1, asynParamFloat64, &index_avg_voltage_1);
	createParam(P_VOLTAGE_AVG_2, asynParamFloat64, &index_avg_voltage_2);
	createParam(P_VOLTAGE_AVG_3, asynParamFloat64, &index_avg_voltage_3);
	createParam(P_VOLTAGE_AVG_4, asynParamFloat64, &index_avg_voltage_4);
	createParam(P_FREQUENCY,     asynParamInt32, &index_frequency);
	createParam(P_AVERAGE_TIME, asynParamFloat64, &index_average_time);
	createParam(P_ERROR, asynParamOctet, &index_error);

	sendCommand(ADMIN_LOGIN, NULL);
	sendCommand(CHANNEL_ENABLE, NULL);
	sendCommand(ACQ_STOP, NULL);
	sendCommand(FREQUENCY_SET, &frequency);

	if(pthread_create(&daq_thread, 0, start_daq_thread, this) != 0)
	{
		printf("Could not create DAQ thread for port %s\n", port_name);
		perror("pthread_create");
	}
	if(pthread_create(&averaging_thread, 0, start_averaging_thread, this) != 0)
	{
		printf("Could not create Averaging thread for port %s\n", port_name);
		perror("pthread_create");
	}
	this->max_buffer_size = buffer_size;
	this->frequency_changed = false;
	this->average_time = 1.0;

	pthread_mutex_init(&avg_mutex, NULL);
}

asynStatus DT8824::readFloat64(asynUser* pasynUser, epicsFloat64* value)
{
	char buffer[100];
	int function = pasynUser->reason;
	
	if(function >= index_voltage_1 && function <= index_voltage_4)
		*value = voltages[function];
	else if(function >= index_avg_voltage_1 && function <= index_avg_voltage_4)
		*value = avg_voltages[function - index_avg_voltage_1];
	else if(function == index_frequency)
	{
		readCommand(FREQUENCY_GET, buffer);
		if(buffer == NULL)
		{
			cout << "Could not read frequency" << endl;
			return asynError;
		}

		*value = atof(buffer);
	}
	else
	{
		cout << "Unknown function: " << function << endl;
		return asynError;
	}
	return asynSuccess;
}

asynStatus DT8824::writeInt32(asynUser* pasynUser, epicsInt32 value)
{
	int function = pasynUser->reason;
	if(function == index_frequency)
	{
		sendCommand(ACQ_STOP, NULL);
		int v = value;
		sendCommand(FREQUENCY_SET, &v);
		frequency_changed = true;
	}
	return asynSuccess;
}

asynStatus DT8824::writeFloat64(asynUser* pasynUser, epicsFloat64 value)
{
	int function = pasynUser->reason;
	if(function == index_average_time)
	{
		channels[0].clear();
		channels[1].clear();
		channels[2].clear();
		channels[3].clear();
		this->average_time = value;
	}
	return asynSuccess;
}

asynStatus DT8824::readOctet(asynUser *pasynUser, char *value, size_t maxChars, size_t *nActual, int *eomReason)
{
	char buffer[100];
	int function = pasynUser->reason;
	if(function == index_error)
	{
		memset(buffer, 0, sizeof(buffer));
		readCommand(SYSTEM_ERROR, buffer);
		if(buffer == NULL)
			return asynError;
		
		*nActual = strlen(buffer);
		*eomReason = ASYN_EOM_EOS;
		size_t i = 0;
		for(i = 0; i < strlen(buffer); i++)
		{
			if(buffer[i] == '\"')
				buffer[i] = ' ';
		}
		buffer[*nActual - 2] = '\0';
		memcpy(value, buffer, *nActual);
	}
	return asynSuccess;
}

void DT8824::sendCommand(string cmd, int* value)
{
	int status;
	char command[50];
	size_t bytes;

	memset(command, 0, sizeof(command));
	if(value != NULL)
		snprintf(command, sizeof(command), cmd.c_str(), *value);
	else
		snprintf(command, sizeof(command), cmd.c_str());
	status = pasynOctetSyncIO->write(this->asyn_user, command, strlen(command), 1, &bytes);
	if(status != asynSuccess || bytes != strlen(command))
	{
		printf("ERROR: Could not send command %s\n", cmd.c_str());
		return;
	}
}

void DT8824::readCommand(string cmd, char* buffer)
{
	size_t bytes_rx;
	size_t bytes_tx;
	int reason;
	int status;
	char command[50];
	char read_buffer[100];

	memset(command, 0, sizeof(command));
	snprintf(command, sizeof(command), cmd.c_str());
	status = pasynOctetSyncIO->writeRead(this->asyn_user, command, strlen(command), read_buffer, sizeof(read_buffer), 1, &bytes_tx, &bytes_rx, &reason);
	if(status != asynSuccess || bytes_tx != strlen(command) || bytes_rx != strlen(read_buffer))
	{
		buffer = NULL;
		return;
	}

	read_buffer[bytes_rx] = '\0';
	memcpy(buffer, read_buffer, bytes_rx);
}

int DT8824::bytes_to_int(char* buffer)
{
	return ((buffer[0] << 24) & 0xff000000) + ((buffer[1] << 16) & 0xff0000) + ((buffer[2] << 8) & 0xff00) + ((buffer[3]) & 0xff);
}

void DT8824::performAveraging()
{
	while(true)
	{
		usleep(this->average_time * 1000000);
//		pthread_mutex_lock(&avg_mutex);
//
//		for(int i = 0; i < NUMBER_OF_CHANNELS; i++)
//		{
//			if(!channels[i].empty())
//			{
//				avg_voltages[i] = std::accumulate(channels[i].begin(), channels[i].end(), 0.0f) / channels[i].size();
//			}
//		}
//
//		channels[0].clear();
//		channels[1].clear();
//		channels[2].clear();
//		channels[3].clear();
//
//		pthread_mutex_unlock(&avg_mutex);
		// epicsThreadSleep(0.1);
	}	
}

void DT8824::performDAQ()
{
	size_t bytes_rx;
	size_t bytes_tx;
	int n = 5;
	int c = 0;
	int size;
	int reason;
	int status;
	int nbytes;
	int length;
	int bytes;
	char raw_data[2000];
	char command[50];
	char buffer[10];
	char* channel_data;
	double sample;
	while(true)
	{
		lock();
		sendCommand(ACQ_STOP, NULL);
		sendCommand(ACQ_ARM, NULL);
		sendCommand(ACQ_INIT, NULL);
		unlock();

		usleep(this->average_time * 1000000);
		// epicsThreadSleep(1);

		if (frequency_changed)
		{
			frequency_changed = false;
			continue;
		}

		lock();
		memset(raw_data, 0, sizeof(raw_data));
		memset(command, 0, sizeof(command));
		snprintf(command, sizeof(command), ACQ_FETCH, 0, (int) n * this->average_time);
		status = pasynOctetSyncIO->writeRead(this->asyn_user, command, strlen(command), raw_data, sizeof(raw_data), 1, &bytes_tx, &bytes_rx, &reason);
		if(status != asynSuccess || bytes_tx != strlen(command) || (bytes_rx < MIN_RX_BYTES && bytes_rx != EMPTY_SCAN_LENGTH))
		{
			printf("ERROR: Could not write command %s\n", command);
			unlock();
			continue;
		}

		if(bytes_rx == EMPTY_SCAN_LENGTH)
		{
			cout << "Empty scan record received." << endl;
			unlock();
			continue;
		}

		raw_data[bytes_rx] = '\0';
		if(isdigit(raw_data[1]))
			nbytes = raw_data[1] - 0x30;
		else
		{
			printf("ERROR: Invalid bytes count\n");
			unlock();
			continue;
		}

		memset(buffer, 0, sizeof(buffer));
		strncpy(buffer, raw_data + 2, nbytes);
		buffer[nbytes] = '\0';
		length = atoi(buffer);
		if(length < MIN_DATA_LENGTH)
		{
			printf("ERROR: Invalid data length\n");
			unlock();
			continue;
		}

		if(raw_data[bytes_rx - 1] != '\n')
		{
			cout << "EOS Error" << endl;
			unlock();
			continue;
		}		

		c = 0;
		channel_data = raw_data + 28;
		size = 4 * 4 * ((int) n*this->average_time);
		for(int i = 0; i < size && i < bytes_rx - 29; i += 4)
		{
			bytes = bytes_to_int(channel_data + i);
			if(bytes == 0)
				continue;
			sample = 0.000001192 * bytes - 10;
			pthread_mutex_lock(&avg_mutex);
			if(channels[c % 4].size() == this->max_buffer_size)
				channels[c % 4].erase(channels[c % 4].begin());
			channels[c % 4].push_back(sample);
			pthread_mutex_unlock(&avg_mutex);
			c++;
		}
		struct timespec tt;
		clock_gettime(CLOCK_REALTIME, &tt);
		// cout << tt.tv_sec << "." << tt.tv_nsec << endl;

		for(int i = 0; i < NUMBER_OF_CHANNELS; i++)
		{
			if(!channels[i].empty())
			{
				voltages[i] = channels[i][channels[i].size() - 1];
				avg_voltages[i] = std::accumulate(channels[i].begin(), channels[i].end(), 0.0f) / channels[i].size();
				setDoubleParam(index_avg_voltage_1 + i, avg_voltages[i]);
			}
		}

		callParamCallbacks();
		unlock();
		// epicsThreadSleep(0.1);
	}
}

extern "C" int DT8824Configure(const char* port_name, const char* name, int frequency, int buffer_size)
{
	new DT8824(port_name, name, frequency, buffer_size);
	return asynSuccess;
}

static const iocshArg configArg0 = { "Port name", iocshArgString };
static const iocshArg configArg1 = { "Asyn name", iocshArgString };
static const iocshArg configArg2 = { "Frequency", iocshArgInt };
static const iocshArg configArg3 = { "Buffer Size", iocshArgInt };
static const iocshArg* const configArgs[] = {&configArg0, &configArg1, &configArg2, &configArg3};
static const iocshFuncDef configFuncDef = {"DT8824Configure",4,configArgs};

static void configCallFunc(const iocshArgBuf *args)
{
    DT8824Configure(args[0].sval, args[1].sval, args[2].ival, args[3].ival);
}

void drvDT8824Register(void)
{
    iocshRegister(&configFuncDef,configCallFunc);
}

extern "C" {
epicsExportRegistrar(drvDT8824Register);
}

