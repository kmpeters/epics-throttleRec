TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

#=============================
# Build the IOC support library

LIBRARY_IOC += throttle

# xxxRecord.h will be created from xxxRecord.dbd
DBDINC += throttleRecord
# Install devXxxSoft.dbd into <top>/dbd
DBD += throttleSupport.dbd

# Compile and add the code to the support library
throttle_SRCS += throttleRecord.c

throttle_LIBS += $(EPICS_BASE_IOC_LIBS)

#=============================
# Build the IOC application

PROD_IOC = throttleEx
# throttle.dbd will be created and installed
DBD += throttleEx.dbd

# throttle.dbd will be made up from these files:
throttleEx_DBD += base.dbd
throttleEx_DBD += throttleSupport.dbd

# throttleEx_registerRecordDeviceDriver.cpp derives from throttle.dbd
throttleEx_SRCS += throttleEx_registerRecordDeviceDriver.cpp

# Build the main IOC entry point on workstation OSs.
throttleEx_SRCS_DEFAULT += throttleExMain.cpp
throttleEx_SRCS_vxWorks += -nil-

# Add support from base/src/vxWorks if needed
#throttleEx_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# Link in the code from the support library
throttleEx_LIBS += throttle

# NOTE: To build SNL programs, SNCSEQ must be defined
# in the <top>/configure/RELEASE file

# Finally link to the EPICS Base libraries
throttleEx_LIBS += $(EPICS_BASE_IOC_LIBS)

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

