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
#define CHANNEL_ENABLE	":AD:ENAB ON, (@%d)\r\n"
#define SET_FREQUENCY	":AD:CLOC:FREQ %d\r\n"
#define ACQ_STOP		":AD:ABOR\r\n"
#define ACQ_ARM			":AD:ARM\r\n"
#define ACQ_INIT		":AD:INIT\r\n"
#define ACQ_LAST_INDEX	":AD:STAT:SCA?\r\n"
#define ACQ_FETCH		":AD:FETCH? %d,%d\r\n"

#define P_VOLTAGE_1		"voltage_ch1"

class DT8824 : public asynPortDriver
{
public:
	DT8824(const char* port_name, const char* name, int frequency, int buffer_size);
	
	void sendCommand(string cmd);
	void sendCommand(string cmd, int value);
	void readCommand(string cmd);
	unsigned bytes_to_int(char* buffer);
	void performDAQ();
	pthread_t daq_thread;

protected:
	int index_voltage;

private:
	asynUser* asyn_user;
	size_t bytes;
	char command[50];

	char read_buffer[1000000];
	vector<double> channel_1;
	vector<double> channel_2;
	vector<double> channel_3;
	vector<double> channel_4;
	vector<double> raw;
	vector<double> channels[4];
};

static void* start_daq_thread(void* pvt)
{
	((DT8824*) pvt)->performDAQ();
	return NULL;
}

DT8824::DT8824(const char* port_name, const char* name, int frequency, int buffer_size)
	: asynPortDriver(port_name,
					1,
					asynFloat64Mask,
					0,
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

	sendCommand(ADMIN_LOGIN);

	for(int i = 1; i <= 4; i++)
	{
		sendCommand(CHANNEL_ENABLE, i);
	}

	sendCommand(ACQ_STOP);
	sendCommand(SET_FREQUENCY, frequency);

	channel_1.resize(buffer_size);
	channel_2.resize(buffer_size);
	channel_3.resize(buffer_size);
	channel_4.resize(buffer_size);
	if(pthread_create(&daq_thread, 0, start_daq_thread, this) != 0)
	{
		printf("Could not create DAQ thread for port %s\n", port_name);
		perror("pthread_create");
	}
}

void DT8824::sendCommand(string cmd)
{
	int status;
	status = pasynOctetSyncIO->write(this->asyn_user, cmd.c_str(), cmd.length(), 1, &this->bytes);
	if(status != asynSuccess || this->bytes != cmd.length())
	{
		printf("ERROR: Could not send command %s\n", cmd.c_str());
		return;
	}
}

void DT8824::sendCommand(string cmd, int value)
{
	int status;
	char command[50];
	memset(command, 0, sizeof(command));
	snprintf(command, sizeof(command), cmd.c_str(), value);
	status = pasynOctetSyncIO->write(this->asyn_user, command, strlen(command), 1, &this->bytes);
	if(status != asynSuccess || this->bytes != strlen(command))
	{
		printf("ERROR: Could not send command %s\n", cmd.c_str());
		return;
	}
}

void DT8824::readCommand(string cmd)
{
	size_t bytes_rx;
	int reason;
	int status;
	char command[50];
	memset(command, 0, sizeof(command));
	snprintf(command, sizeof(command), cmd.c_str());
	status = pasynOctetSyncIO->write(this->asyn_user, command, strlen(command), 0, &this->bytes);
	if(status != asynSuccess || this->bytes != strlen(command))
	{
		printf("ERROR: Could not write command %s\n", cmd.c_str());
		return;
	}

	status = pasynOctetSyncIO->read(this->asyn_user, read_buffer, sizeof(read_buffer), 1, &bytes_rx, &reason);
	if(status != asynSuccess || bytes_rx != strlen(read_buffer))
	{
		printf("ERROR: Could not read command %s\n", cmd.c_str());
		return;
	}
}

unsigned DT8824::bytes_to_int(char* buffer)
{
	return (buffer[0] << 24) + (buffer[1] << 16) + (buffer[2] << 8) + (buffer[3]);
}

void DT8824::performDAQ()
{
	int n = 10;
	int size;
	size_t bytes_rx;
	size_t bytes_tx;
	int reason;
	int status;
	char raw_data[2000000];
	char command[50];
	string cmd;
	while(true)
	{
		lock();
		sendCommand(ACQ_STOP);
		sendCommand(ACQ_ARM);
		sendCommand(ACQ_INIT);

		epicsThreadSleep(1);

		readCommand(ACQ_LAST_INDEX);

		cmd = ACQ_FETCH;
		memset(raw_data, 0, sizeof(raw_data));
		memset(command, 0, sizeof(command));
		snprintf(command, sizeof(command), cmd.c_str(), 0, 5);
		status = pasynOctetSyncIO->writeRead(this->asyn_user, command, strlen(command), raw_data, sizeof(raw_data), 1, &bytes_tx, &bytes_rx, &reason);
		if(status != asynSuccess || bytes_tx != strlen(command))
		{
			printf("ERROR: Could not write command %s\n", command);
			unlock();
			continue;
		}

		int header;
		int nbytes;
		if(raw_data[1] >= 0x30 && raw_data[1] <= 0x39)
			nbytes = raw_data[1] - 0x30;
		else
		{
			printf("ERROR: Invalid bytes count\n");
			unlock();
			continue;
		}

		int length;
		char buffer[10];
		header = nbytes + 2;
		strncpy(buffer, raw_data + 2, header - 2);
		buffer[nbytes] = '\0';
		length = atoi(buffer);
		if(length <= 0)
		{
			printf("ERROR: Invalid data length\n");
			unlock();
			continue;
		}

		unsigned fsindex = bytes_to_int(raw_data + header);
		unsigned nscans  = bytes_to_int(raw_data + header + 4);
		unsigned spscan  = bytes_to_int(raw_data + header + 8);
		unsigned tstamp  = bytes_to_int(raw_data + header + 12);

		cout << "length: " << length << endl;
		cout << "nbytes: " << nbytes << endl;
		cout << "index: " << fsindex << endl;
		cout << "nscan: " << nscans << endl;
		cout << "sscan: " << spscan << endl;
		cout << "stamp: " << tstamp << endl;

		if(raw_data[bytes_rx - 1] == '\n')
			cout << "EOS Success!!!!" << endl;
		else
			cout << "EOS Failed!!!!" << endl;

		channel_1.clear();
		channel_2.clear();
		channel_3.clear();
		channel_4.clear();

		int c = 0;
		char* channel_data = raw_data + 28;
		size = 4 * 4 * n;
		for(int i = 0; i < size; i += 4)
		{
			// raw.push_back( 0.000001192 * bytes_to_int(channel_data + i) - 10 );
			double xx = 0.000001192 * bytes_to_int(channel_data + i) - 10;
			// cout << "X: " << xx << endl;
			channels[c++ % 4].push_back(xx);
		}
		// cout << "Sample size: " << c << endl;
		// cout << "Channel 1: " << channels[0][channels[0].size() - 1] << " - " << std::accumulate(channels[0].begin(), channels[0].end(), 0) / channels[0].size() << endl;
		// cout << "Channel 2: " << channels[1][channels[1].size() - 1] << " - " << std::accumulate(channels[1].begin(), channels[1].end(), 0) / channels[1].size() << endl;
		// cout << "Channel 3: " << channels[2][channels[2].size() - 1] << " - " << std::accumulate(channels[2].begin(), channels[2].end(), 0) / channels[2].size() << endl;
		// cout << "Channel 4: " << channels[3][channels[3].size() - 1] << " - " << std::accumulate(channels[3].begin(), channels[3].end(), 0) / channels[3].size() << endl;

		// A test to check setting values for PVs.
		// setDoubleParam(index_voltage, 50);

		unlock();
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

