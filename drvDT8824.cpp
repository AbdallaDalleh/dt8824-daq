#include <string.h>
#include <stdio.h>
#include <pthread.h>
#include <unistd.h>

#include <vector>

#include <epicsExport.h>
#include <asynPortDriver.h>
#include <asynOctetSyncIO.h>
#include <iocsh.h>

using std::vector;

#define ADMIN_LOGIN		":SYST:PASS:CEN admin\r\n"
#define CHANNEL_ENABLE	":AD:ENABLE ON, (@%d)\r\n"
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
	int status = pasynOctetSyncIO->connect(name, 1, &this->asyn_user, NULL);

	if(status != asynSuccess)
	{
		// TODO: Use asynPrint
		printf("Could not connect to port %s\n", port_name);
		return;
	}

	createParam(P_VOLTAGE_1, asynParamFloat64, &index_voltage);

	snprintf(command, strlen(ADMIN_LOGIN), "%s", ADMIN_LOGIN);
	status = pasynOctetSyncIO->write(this->asyn_user, command, strlen(command), 1, &this->bytes);
	if(status != asynSuccess || this->bytes != strlen(command))
	{
		printf("ERROR: Could not enable protected commands to port %s\n", port_name);
		return;
	}

	for(int i = 1; i <= 4; i++)
	{
		snprintf(command, strlen(CHANNEL_ENABLE), CHANNEL_ENABLE, i);
		status = pasynOctetSyncIO->write(this->asyn_user, command, strlen(command), 1, &this->bytes);
		if(status != asynSuccess || this->bytes != strlen(command))
		{
			printf("ERROR: Could not enable channel %d on port %s\n", i, port_name);
			return;
		}
	}

	snprintf(command, strlen(ACQ_STOP), "%s", ACQ_STOP);
	pasynOctetSyncIO->write(this->asyn_user, command, strlen(command), 1, &this->bytes);

	snprintf(command, strlen(SET_FREQUENCY)+4, SET_FREQUENCY, frequency);
	pasynOctetSyncIO->write(this->asyn_user, command, strlen(command), 1, &this->bytes);

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

void DT8824::performDAQ()
{
	while(true)
	{
		lock();
		snprintf(command, strlen(ACQ_STOP), "%s", ACQ_STOP);
		pasynOctetSyncIO->write(this->asyn_user, command, strlen(command), 1, &this->bytes);
		
		snprintf(command, strlen(ACQ_ARM), "%s", ACQ_ARM);
		pasynOctetSyncIO->write(this->asyn_user, command, strlen(command), 1, &this->bytes);

		snprintf(command, strlen(ACQ_INIT), "%s", ACQ_INIT);
		pasynOctetSyncIO->write(this->asyn_user, command, strlen(command), 1, &this->bytes);

		sleep(1);

		int x;
		snprintf(command, strlen(ACQ_LAST_INDEX), "%s", ACQ_LAST_INDEX);
		pasynOctetSyncIO->writeRead(this->asyn_user, command, strlen(command), this->read_buffer, sizeof(this->read_buffer), 1, &bytes, &bytes, &x);
		printf("%s\n", read_buffer);

		// A test to check setting values for PVs.
		setDoubleParam(index_voltage, 50);

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



