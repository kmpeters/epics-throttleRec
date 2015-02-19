#!../../bin/linux-x86_64/dsh

< envPaths

cd ${TOP}

## Register all support components
dbLoadDatabase "dbd/throttleEx.dbd"
throttleEx_registerRecordDeviceDriver pdbbase

## Load record instances
dbLoadRecords "db/throttle.db", "P=throttleEx:,THR=thr"


## Run this to trace the stages of iocInit
#traceIocInit
cd ${TOP}/iocBoot/${IOC}
iocInit

