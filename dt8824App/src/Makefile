TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#=============================
# Build the IOC application

PROD_IOC = dt8824
# dt8824.dbd will be created and installed
DBD += dt8824.dbd

# dt8824.dbd will be made up from these files:
dt8824_DBD += base.dbd
dt8824_DBD += asyn.dbd
dt8824_DBD += drvAsynIPPort.dbd
dt8824_DBD += iocAdmin.dbd

# Include dbd files from all support applications:
dt8824_DBD += DT8824.dbd
dt8824_DBD += current.dbd

# Add all the support libraries needed by this IOC
dt8824_LIBS += asyn devIocStats seq pv

# dt8824_registerRecordDeviceDriver.cpp derives from dt8824.dbd
dt8824_SRCS += dt8824_registerRecordDeviceDriver.cpp drvDT8824.cpp
dt8824_SRCS += current.st

# Build the main IOC entry point on workstation OSs.
dt8824_SRCS_DEFAULT += dt8824Main.cpp
dt8824_SRCS_vxWorks += -nil-

# Add support from base/src/vxWorks if needed
#dt8824_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# Finally link to the EPICS Base libraries
dt8824_LIBS += $(EPICS_BASE_IOC_LIBS)

USR_SYS_LIBS += pthread
#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

