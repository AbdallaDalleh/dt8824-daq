#!../../bin/linux-x86_64/dt8824

epicsEnvSet("TOP","/home/dev.control/top/dt8824")
epicsEnvSet("ENGINEER","Abdalla Al-Dalleh")
epicsEnvSet("LOCATION","Machine")
epicsEnvSet("IOC_PREFIX","IOC-DT8824")

dbLoadDatabase "dbd/dt8824.dbd"
dt8824_registerRecordDeviceDriver pdbbase

dbLoadRecords("db/dt8824.db")
dbLoadRecords("db/iocAdminSoft.db", "IOC=$(IOC_PREFIX)")

drvAsynIPPortConfigure("DT8824", "10.3.1.87:5025", 0, 0, 1);
DT8824Configure("DT", "DT8824", 12, 2000)

iocInit

