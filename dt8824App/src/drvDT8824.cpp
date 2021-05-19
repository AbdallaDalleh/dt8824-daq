#include <string.h>
#include <stdio.h>
#include <pthread.h>
#include <unistd.h>

#include <vector>
#include <string>

#include <epicsExport.h>
#include <asynPortDriver.h>
#include <asynOctetSyncIO.h>
#include <iocsh.h>
#include <epicsThread.h>

using std::vector;
using std::string;

#define ADMIN_LOGIN		":SYST:PASS:CEN admin\r\n"
#define CHANNEL_ENABLE	":AD:ENABLE ON, (@%d)\r\n"
#define SET_FREQUENCY	":AD:CLOC:FREQ %d\r\n"
#define ACQ_STOP		":AD:ABOR\r\n"
#define ACQ_ARM			":AD:ARM\r\n"
#define ACQ_INIT		":AD:INIT\r\n"
#define ACQ_LAST_INDEX	":AD:STAT:SCA?\r\n"
#define ACQ_FETCH		":AD:FETCH? %d,%d"

#define P_VOLTAGE_1		"voltage_ch1"

class DT8824 : public asynPortDriver
{
public:
	DT8824(const char* port_name, const char* name, int frequency, int buffer_size);
	
	void sendCommand(string cmd);
	void sendCommand(string cmd, int value);
	void performDAQ();
	pthread_t daq_thread;

protected:
	int index_voltage;

private:
	asynUser* asyn_user;
	size_t bytes;
	char command[50];

	char read_buffer[1000000];
	vector<int> channel_1;
	vector<int> channel_2;
	vector<int> channel_3;
	vector<int> channel_4;
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

void DT8824::performDAQ()
{
	while(true)
	{
		lock();
		sendCommand(ACQ_STOP);
		sendCommand(ACQ_ARM);
		sendCommand(ACQ_INIT);

		epicsThreadSleep(1);

		int x;
		size_t bytes_rx;
		memset(command, 0, sizeof(command));
		snprintf(command, strlen(ACQ_LAST_INDEX), "%s", ACQ_LAST_INDEX);
		printf("%s\n", command);
		pasynOctetSyncIO->writeRead(this->asyn_user, command, strlen(command), this->read_buffer, sizeof(this->read_buffer), 0.1, &bytes, &bytes_rx, &x);
		printf("(%zu - %zu) %s", bytes, bytes_rx, read_buffer);
		printf("Hello 2\n");

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



