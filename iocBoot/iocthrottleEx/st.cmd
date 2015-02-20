#!../../bin/linux-x86_64/dsh

< envPaths

cd ${TOP}

## Register all support components
dbLoadDatabase "dbd/throttleEx.dbd"
throttleEx_registerRecordDeviceDriver pdbbase

## Load record instances
#dbLoadRecords "db/throttle.db", "P=throttleEx:,THR=thr,OUT=throttleEx:ao1 PP"
dbLoadRecords "db/throttle.db", "P=throttleEx:,THR=thr"

# give some dummy records
dbLoadRecords "throttleApp/Db/test.db", "P=throttleEx:"

## Run this to trace the stages of iocInit
#traceIocInit
cd ${TOP}/iocBoot/${IOC}
iocInit

