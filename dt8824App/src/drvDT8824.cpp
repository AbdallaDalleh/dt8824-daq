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
#define SET_FREQUENCY	":AD:CLOC:FREQ %d\r\n"
#define ACQ_STOP		":AD:ABOR\r\n"
#define ACQ_ARM			":AD:ARM\r\n"
#define ACQ_INIT		":AD:INIT\r\n"
#define ACQ_LAST_INDEX	":AD:STAT:SCAN?\r\n"
#define ACQ_FETCH		":AD:FETCH? %d,%d\r\n"

#define P_VOLTAGE_1		"voltage_ch1"

class DT8824 : public asynPortDriver
{
public:
	DT8824(const char* port_name, const char* name, int frequency, int buffer_size);
	virtual asynStatus readFloat64(asynUser* pasynUser, epicsFloat64* value);
	
	void sendCommand(string cmd);
	void sendCommand(string cmd, int value);
	void readCommand(string cmd);
	int bytes_to_int(char* buffer);
	void performDAQ();
	pthread_t daq_thread;

protected:
	int index_voltage;

private:
	asynUser* asyn_user;
	vector<double> channels[4];
	double v1;
};

static void* start_daq_thread(void* pvt)
{
	DT8824* dt = (DT8824*) pvt;
	dt->performDAQ();
	return NULL;
}

static void pollerThread(void* pvt)
{
	DT8824* dt = (DT8824*) pvt;
	dt->performDAQ();
}

DT8824::DT8824(const char* port_name, const char* name, int frequency, int buffer_size)
	: asynPortDriver(port_name,
					1,
					asynFloat64Mask,
					asynFloat64Mask,
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

	createParam(P_VOLTAGE_1, asynParamFloat64, &index_voltage);
	setDoubleParam(index_voltage, 74.66);

	sendCommand(ADMIN_LOGIN);
	sendCommand(CHANNEL_ENABLE);
	sendCommand(ACQ_STOP);
	sendCommand(SET_FREQUENCY, frequency);

	// if(pthread_create(&daq_thread, 0, start_daq_thread, this) != 0)
	// {
	// 	printf("Could not create DAQ thread for port %s\n", port_name);
	// 	perror("pthread_create");
	// }

	epicsThreadCreate("DT8824_DAQ",
                    epicsThreadPriorityLow,
                    epicsThreadGetStackSize(epicsThreadStackBig),
                    (EPICSTHREADFUNC)pollerThread,
                    this);

}

asynStatus DT8824::readFloat64(asynUser* pasynUser, epicsFloat64* value)
{
	*value = v1;
	return asynSuccess;
}

void DT8824::sendCommand(string cmd)
{
	int status;
	size_t bytes;
	status = pasynOctetSyncIO->write(this->asyn_user, cmd.c_str(), cmd.length(), 1, &bytes);
	if(status != asynSuccess || bytes != cmd.length())
	{
		printf("ERROR: Could not send command %s\n", cmd.c_str());
		return;
	}
}

void DT8824::sendCommand(string cmd, int value)
{
	int status;
	char command[50];
	size_t bytes;

	memset(command, 0, sizeof(command));
	snprintf(command, sizeof(command), cmd.c_str(), value);
	status = pasynOctetSyncIO->write(this->asyn_user, command, strlen(command), 1, &bytes);
	if(status != asynSuccess || bytes != strlen(command))
	{
		printf("ERROR: Could not send command %s\n", cmd.c_str());
		return;
	}
}

void DT8824::readCommand(string cmd)
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
		printf("ERROR: Could not write command %s\n", cmd.c_str());
		return;
	}
}

int DT8824::bytes_to_int(char* buffer)
{
	return ((buffer[0] << 24) & 0xff000000) + ((buffer[1] << 16) & 0xff0000) + ((buffer[2] << 8) & 0xff00) + ((buffer[3]) & 0xff);
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
	char raw_data[2000];
	char command[50];
	char buffer[10];
	// int header;
	// unsigned fsindex, nscans, spscan, tstamp;
	int bytes;
	double sample;
	while(true)
	{
		lock();
		sendCommand(ACQ_STOP);
		sendCommand(ACQ_ARM);
		sendCommand(ACQ_INIT);

		epicsThreadSleep(1);

		readCommand(ACQ_LAST_INDEX);

		memset(raw_data, 0, sizeof(raw_data));
		memset(command, 0, sizeof(command));
		snprintf(command, sizeof(command), ACQ_FETCH, 0, n);
		status = pasynOctetSyncIO->writeRead(this->asyn_user, command, strlen(command), raw_data, sizeof(raw_data), 1, &bytes_tx, &bytes_rx, &reason);
		if(status != asynSuccess || bytes_tx != strlen(command))
		{
			printf("ERROR: Could not write command %s\n", command);
			unlock();
			continue;
		}
		raw_data[bytes_rx] = '\0';

		if(raw_data[1] >= 0x30 && raw_data[1] <= 0x39)
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
		if(length <= 0)
		{
			printf("ERROR: Invalid data length\n");
			unlock();
			continue;
		}

		// header = nbytes + 2;
		// fsindex = bytes_to_int(raw_data + header);
		// nscans  = bytes_to_int(raw_data + header + 4);
		// spscan  = bytes_to_int(raw_data + header + 8);
		// tstamp  = bytes_to_int(raw_data + header + 12);

		if(raw_data[bytes_rx - 1] != '\n')
		{
			cout << "EOS Error" << endl;
			unlock();
			continue;
		}		

		c = 0;
		char* channel_data = raw_data + 28;
		size = 4 * 4 * n;
		for(int i = 0; i < size; i += 4)
		{
			bytes = bytes_to_int(channel_data + i);
			if(bytes == 0)
				continue;
			sample = 0.000001192 * bytes - 10;
			channels[c++ % 4].push_back(sample);
		}

		// cout << "Channel 1: " << channels[0][channels[0].size() - 1] << " - " << std::accumulate(channels[0].begin(), channels[0].end(), 0) / channels[0].size() << endl;
		// cout << "Channel 2: " << channels[1][channels[1].size() - 1] << " - " << std::accumulate(channels[1].begin(), channels[1].end(), 0) / channels[1].size() << endl;
		// cout << "Channel 3: " << channels[2][channels[2].size() - 1] << " - " << std::accumulate(channels[2].begin(), channels[2].end(), 0) / channels[2].size() << endl;
		// cout << "Channel 4: " << channels[3][channels[3].size() - 1] << " - " << std::accumulate(channels[3].begin(), channels[3].end(), 0) / channels[3].size() << endl;

		if(!channels[0].empty())
			v1 = channels[0][channels[0].size() - 1];

		unlock();
		epicsThreadSleep(0.1);
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

