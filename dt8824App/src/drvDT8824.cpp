#include "drvDT8824.h"

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

DT8824::DT8824(const char* port_name, const char* name, double frequency, int buffer_size)
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
	createParam(P_FREQUENCY,     asynParamFloat64, &index_frequency);
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
	//if(pthread_create(&averaging_thread, 0, start_averaging_thread, this) != 0)
	//{
	//	printf("Could not create Averaging thread for port %s\n", port_name);
	//	perror("pthread_create");
	//}
	this->max_buffer_size = static_cast<int>(frequency / 2.5);
	this->frequency_changed = false;
	this->average_time = 0.5;
	this->frequency = frequency;

	cout << "Frequency: " << frequency << endl;
	cout << "Buffer size: " << this->max_buffer_size << endl;

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
		this->frequency = *value;
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
	// int function = pasynUser->reason;
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
		cout << "Set average_time to " << this->average_time << " seconds" << endl;
	}
	else if(function == index_frequency)
	{
		sendCommand(ACQ_STOP, NULL);
		double v = value;
		cout << "Frequency Set: " << value << endl;
		sendCommand(FREQUENCY_SET, &v);
		frequency_changed = true;
		this->max_buffer_size = static_cast<int>(value / 2.5);
		cout << "N samples: " << this->max_buffer_size << endl;
	}
	else
	{
		cout << "Unknown float64 function." << endl;
		return asynError;
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

void DT8824::sendCommand(string cmd, double* value)
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
		long sleep_time = static_cast<long>(this->average_time * 1000000);  // Convert seconds to microseconds
		// cout << "caclulated sleep time: "<< sleep_time<<endl;
        usleep(sleep_time);
		//usleep(this->average_time * 1000000);
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
	int n = this->max_buffer_size;
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
		printf("Acquiring | f = %.1f | N = %d | t = %.1f\n", this->frequency, this->max_buffer_size, this->average_time);
		lock();
		sendCommand(ACQ_STOP, NULL);
		sendCommand(ACQ_ARM, NULL);
		sendCommand(ACQ_INIT, NULL);
		unlock();
		long sleep_time = static_cast<long>(this->average_time * 1000000);  // Convert to microseconds
        usleep(sleep_time);
		// usleep(this->average_time * 1000000);
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
		size = 4 * 4 * ((int) n * this->average_time);
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

		channels[0].clear();
		channels[1].clear();
		channels[2].clear();
		channels[3].clear();

		callParamCallbacks();
		unlock();
		// epicsThreadSleep(0.1);
	}
}

extern "C" int DT8824Configure(const char* port_name, const char* name, double frequency, int buffer_size)
{
	new DT8824(port_name, name, frequency, buffer_size);
	return asynSuccess;
}

static const iocshArg configArg0 = { "Port name", iocshArgString };
static const iocshArg configArg1 = { "Asyn name", iocshArgString };
static const iocshArg configArg2 = { "Frequency", iocshArgDouble };
static const iocshArg configArg3 = { "Buffer Size", iocshArgInt };
static const iocshArg* const configArgs[] = {&configArg0, &configArg1, &configArg2, &configArg3};
static const iocshFuncDef configFuncDef = {"DT8824Configure",4,configArgs};

static void configCallFunc(const iocshArgBuf *args)
{
    DT8824Configure(args[0].sval, args[1].sval, args[2].dval, args[3].ival);
}

void drvDT8824Register(void)
{
    iocshRegister(&configFuncDef,configCallFunc);
}

extern "C" {
epicsExportRegistrar(drvDT8824Register);
}

