#!../../bin/linux-x86_64/dt8824

dbLoadDatabase "dbd/dt8824.dbd"
dt8824_registerRecordDeviceDriver pdbbase

dbLoadRecords("db/test.db")

drvAsynIPPortConfigure("DT8824", "10.3.1.87:5025", 0, 0, 0);
DT8824Configure("DT", "DT8824", 15, 2000)

iocInit

