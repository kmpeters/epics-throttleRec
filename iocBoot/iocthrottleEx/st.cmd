#!../../bin/linux-x86_64/dsh

## You may have to change dsh to something else
## everywhere it appears in this file

< envPaths

cd ${TOP}

## Register all support components
dbLoadDatabase "dbd/throttleEx.dbd"
dsh_registerRecordDeviceDriver pdbbase

## Load record instances
dbLoadRecords "db/throttle.db", "P=throttleEx:"


## Run this to trace the stages of iocInit
#traceIocInit
cd ${TOP}/iocBoot/${IOC}
iocInit

