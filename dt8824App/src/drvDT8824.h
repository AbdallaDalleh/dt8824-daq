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
