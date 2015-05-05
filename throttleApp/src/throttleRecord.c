/* throttleRecord.c */
  
#ifdef vxWorks
#include <stddef.h>
#include <stdarg.h>
#endif
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <epicsMath.h>
#include <alarm.h>
#include <dbAccess.h>
#include <recGbl.h>
#include <dbEvent.h>
//#include <dbDefs.h>
//#include <dbAccess.h>
#include <errMdef.h>
#include <recSup.h>
#include <special.h>
#include <callback.h>
#define GEN_SIZE_OFFSET
#include "throttleRecord.h"
#undef  GEN_SIZE_OFFSET
#include "epicsExport.h"


#include <epicsVersion.h>
#ifndef EPICS_VERSION_INT
#define VERSION_INT(V,R,M,P) ( ((V)<<24) | ((R)<<16) | ((M)<<8) | (P))
#define EPICS_VERSION_INT VERSION_INT(EPICS_VERSION, EPICS_REVISION, EPICS_MODIFICATION, EPICS_PATCH_LEVEL)
#endif
#define LT_EPICSBASE(V,R,M,P) (EPICS_VERSION_INT < VERSION_INT((V),(R),(M),(P)))


#define VERSION "0-1-0"


/* Create RSET - Record Support Entry Table */
#define report NULL
#define initialize NULL
static long init_record();
static long process();
static long special();
#define get_value NULL
#define cvt_dbaddr NULL
#define get_array_info NULL
#define put_array_info NULL
#define get_units NULL
static long get_precision();
#define get_enum_str NULL
#define get_enum_strs NULL
#define put_enum_str NULL
#define get_graphic_double NULL
#define get_control_double NULL
#define get_alarm_double NULL
 
rset throttleRSET =
  {
    RSETNUMBER,
    report,
    initialize,
    init_record,
    process,
    special,
    get_value,
    cvt_dbaddr,
    get_array_info,
    put_array_info,
    get_units,
    get_precision,
    get_enum_str,
    get_enum_strs,
    put_enum_str,
    get_graphic_double,
    get_control_double,
    get_alarm_double
  };
epicsExportAddress(rset,throttleRSET);


static void checkAlarms(throttleRecord *prec);
//static void monitor(throttleRecord *prec);

static void enterValue( throttleRecord *prec);
static void delayFuncCallback();
static void valuePut( throttleRecord *prec);

static void checkLinkCallback();
static void checkLink();

enum { NO_CA_LINKS, CA_LINKS_ALL_OK, CA_LINKS_NOT_OK };
typedef struct rpvtStruct 
{
  double oval;
  double delay;

  int delay_flag;
  int wait_flag;

  int limit_flag;
  double limit_high;
  double limit_low;

  CALLBACK delayFuncCb;

  CALLBACK checkLinkCb;
  short    pending_checkLinkCB;
  short    caLinkStat; /* NO_CA_LINKS,CA_LINKS_ALL_OK,CA_LINKS_NOT_OK */
} rpvtStruct;


static long init_record(void *precord,int pass)
{
  throttleRecord *prec = (throttleRecord *)precord;
  rpvtStruct  *prpvt;

  struct link         *poutlink;
  unsigned short      *pOutLinkValid;
  struct dbAddr        dbAddr;

  if( pass == 0) 
    {
      strcpy(prec->ver, VERSION);
      prec->rpvt = calloc(1, sizeof(struct rpvtStruct));

      return 0;
    }

  
  prec->sts = throttleSTS_UNK;
  prec->val = 0;

  prpvt = prec->rpvt;
  prpvt->delay = prec->dly;

  prpvt->limit_high = prec->drvlh;
  prpvt->limit_low = prec->drvll;
  if( prec->drvlh > prec->drvll)
    prpvt->limit_flag = 1;
  else
    prpvt->limit_flag = 0;


  /* start link management */

  poutlink = &prec->out;
  pOutLinkValid = &prec->ov;

  prpvt->caLinkStat = NO_CA_LINKS; // as far as I know

  /* check output links */
  if (poutlink->type == CONSTANT) 
    {
      *pOutLinkValid = throttleOV_CON;
    }
  else if (!dbNameToAddr(poutlink->value.pv_link.pvname, &dbAddr)) 
    {
      *pOutLinkValid = throttleOV_LOC;
    }
  else 
    {
      *pOutLinkValid = throttleOV_EXT_NC;
      prpvt->caLinkStat = CA_LINKS_NOT_OK;
    }
  db_post_events(prec,pOutLinkValid,DBE_VALUE|DBE_LOG);

  callbackSetCallback(delayFuncCallback, &prpvt->delayFuncCb);
  callbackSetPriority(prec->prio, &prpvt->delayFuncCb);
  callbackSetUser(prec, &prpvt->delayFuncCb);
  prpvt->delay_flag = 0;
  prpvt->wait_flag = 0;

  callbackSetCallback(checkLinkCallback, &prpvt->checkLinkCb);
  callbackSetPriority(prec->prio, &prpvt->checkLinkCb);
  callbackSetUser(prec, &prpvt->checkLinkCb);
  prpvt->pending_checkLinkCB = 0;
  
  if (prpvt->caLinkStat == CA_LINKS_NOT_OK) 
    {
      prpvt->pending_checkLinkCB = 1;
      callbackRequestDelayed(&prpvt->checkLinkCb, 1.0);
    }

  /* end link management */


  return 0;
}

static long process(throttleRecord *prec)
{
  rpvtStruct *prpvt = prec->rpvt;
  unsigned short  monitor_mask;

  prec->pact = TRUE;
  prec->udf = FALSE;

  prec->wait = TRUE;
  db_post_events(prec,&prec->wait,DBE_VALUE);

  if( prpvt->limit_flag)
    {
      int new_st;

      if( prec->val < prpvt->limit_low)
        {
          prec->val = prpvt->limit_low;
          db_post_events(prec,&prec->val,DBE_VALUE|DBE_LOG);
          new_st = throttleDRVLS_LOW;
        }
      else if( prec->val > prpvt->limit_high)
        {
          prec->val = prpvt->limit_high;
          db_post_events(prec,&prec->val,DBE_VALUE|DBE_LOG);
          new_st = throttleDRVLS_HIGH;
        }
      else // no clipping
        new_st = throttleDRVLS_NORM;

      // if status changed, propagate it
      if( prec->drvls != new_st)
        {
          prec->drvls = new_st;
          db_post_events(prec,&prec->drvls,DBE_VALUE);
        }
    }

  /* if some links are CA, check connections */
  if (prpvt->caLinkStat != NO_CA_LINKS) 
    {
      checkLink(prec);
    }
  enterValue( prec);

  monitor_mask = recGblResetAlarms(prec);
  if(prec->oval != prec->val) 
    {
      monitor_mask |= DBE_VALUE|DBE_LOG;
      prec->oval = prec->val;
    }
  if(monitor_mask)
    db_post_events(prec,&prec->val,monitor_mask);

  /* process the forward scan link record */
  /* recGblFwdLink(prec); */

  prec->pact=FALSE;
  return 0;
}


static long special(DBADDR *paddr, int after)
{
  struct throttleRecord *prec = (throttleRecord *)paddr->precord;
  struct rpvtStruct   *prpvt = prec->rpvt;
  int    special_type = paddr->special;

  int   fieldIndex = dbGetFieldIndex(paddr);
  int   lnkIndex;

  struct link *plink;
  unsigned short    *plinkValid;
  struct dbAddr        dbAddr;

  int new_st;

  
  if( !after) 
    return 0;

  if( special_type != SPC_MOD)
    {
      recGblDbaddrError(S_db_badChoice, paddr, "throttle: special");
      return (S_db_badChoice);
    }

  switch(fieldIndex) 
    {
    case(throttleRecordOUT):
      lnkIndex = throttleRecordOUT;

      plink   = &prec->out;
      plinkValid = &prec->ov;
              
      if (plink->type == CONSTANT) 
        {
          *plinkValid = throttleOV_CON;
        }
      /* see if the PV resides on this ioc */
      else if (!dbNameToAddr(plink->value.pv_link.pvname, &dbAddr)) 
        {
          *plinkValid = throttleOV_LOC;
        }
      /* pv is not on this ioc. Callback later for connection stat */
      else 
        {
          *plinkValid = throttleOV_EXT_NC;
          if (!prpvt->pending_checkLinkCB) 
            {
              prpvt->pending_checkLinkCB = 1;
              callbackRequestDelayed(&prpvt->checkLinkCb, 0.5);
              prpvt->caLinkStat = CA_LINKS_NOT_OK;
            }
        }
      db_post_events(prec,plinkValid,DBE_VALUE|DBE_LOG);

      break;

    case(throttleRecordDLY):
      if( prec->dly < 0.0)
        {
          prpvt->delay = 0.0;
          prec->dly = 0.0;
          db_post_events(prec,&prec->dly,DBE_VALUE);
        }
      else
        prpvt->delay = prec->dly;
      
      if(prpvt->delay_flag == 1)
        {
          // in case the delay was set crazy big, 
          // this kills it and restarts it with new value
          callbackCancelDelayed(&prpvt->delayFuncCb);
          callbackRequestDelayed(&prpvt->delayFuncCb, prpvt->delay);
        }
      break;

    case(throttleRecordDRVLH):
    case(throttleRecordDRVLL):

      prpvt->limit_high = prec->drvlh;
      prpvt->limit_low = prec->drvll;
      if( prec->drvlh <= prec->drvll)
        {
          prpvt->limit_flag = 0;

          new_st = throttleDRVLS_NORM;
        }
      else
        {
          prpvt->limit_flag = 1;
        
          if(prec->val < prpvt->limit_low) 
            new_st = throttleDRVLS_LOW;
          else if(prec->val > prpvt->limit_high)
            new_st = throttleDRVLS_HIGH;
          else
            new_st = throttleDRVLS_NORM;
        }

      // set status flag 
      if( prec->drvls != new_st)
        {
          prec->drvls = new_st;
          db_post_events(prec,&prec->drvls,DBE_VALUE);
        }
      break;

    default:
      recGblDbaddrError(S_db_badChoice, paddr, "throttle: special");
      return(S_db_badChoice);
    }
  
  return 0;
}


static long get_precision(dbAddr *paddr, long *precision)
{
  throttleRecord *prec = (throttleRecord *)paddr->precord;
  int fieldIndex = dbGetFieldIndex(paddr);

  if( fieldIndex == throttleRecordDLY)
    *precision = prec->dprec;
  else
    *precision = prec->prec;

  recGblGetPrec(paddr,precision);

  return 0;
}

static void checkAlarms(throttleRecord *prec)
{
  if (prec->udf == TRUE) 
    {
#if LT_EPICSBASE(3,15,0,2)
      recGblSetSevr(prec,UDF_ALARM,INVALID_ALARM);
#else
      recGblSetSevr(prec,UDF_ALARM,prec->udfs);
#endif
      return;
    }
  return;
}


/* static void monitor(throttleRecord *prec) */
/* { */
/*     unsigned short  monitor_mask; */


/*     monitor_mask = recGblResetAlarms(prec); */
/*     if(prec->oval != prec->val)  */
/*       { */
/* 	monitor_mask |= DBE_VALUE|DBE_LOG; */
/* 	prec->oval = prec->val; */
/*       } */
/*     if(monitor_mask) */
/* 	db_post_events(prec,&prec->val,monitor_mask); */

/*     if(prec->osent != prec->sent)  */
/*       { */
/* 	monitor_mask |= DBE_VALUE|DBE_LOG; */
/* 	prec->osent = prec->sent; */
/*       } */
/*     if(monitor_mask) */
/* 	db_post_events(prec,&prec->sent,monitor_mask); */

/*     return; */
/* } */


static void enterValue( throttleRecord *prec)
{
  rpvtStruct *prpvt = prec->rpvt;

  //  printf("enterValue()\n");

  prpvt->wait_flag = 1; // trigger send
  if( !prpvt->delay_flag)
    valuePut( prec);
  // else it will be set at next callback
}

static void delayFuncCallback(CALLBACK *pcallback)
{
  struct throttleRecord *prec;

  //  printf("delayFuncCallback()\n");

  callbackGetUser(prec, pcallback);
  valuePut( prec);
}

static void valuePut( throttleRecord *prec)
{
  rpvtStruct *prpvt = prec->rpvt;
  unsigned short  monitor_mask;

  struct link *plink;

  long status;

  //  printf("valuePut(), wf%d, df%d\n", prpvt->wait_flag, prpvt->delay_flag);

  if( prpvt->wait_flag)
    {
      /* Process output link. */
      plink = &(prec->out);
      if (plink->type != CONSTANT)
        {
          prpvt->oval = prec->val;
          
          status = dbPutLink(plink, DBR_DOUBLE, &prpvt->oval, 1);
          if( RTN_SUCCESS( status) )
            {
              prec->sts = throttleSTS_SUC;
              prec->sent = prpvt->oval;
              db_post_events(prec,&prec->sent,DBE_VALUE);
            }
          else
            prec->sts = throttleSTS_ERR;

          prec->wait = FALSE;
          db_post_events(prec,&prec->wait,DBE_VALUE);

          // NOW process forward link!
          recGblFwdLink(prec);
        }
      else
        prec->sts = throttleSTS_ERR;

      prpvt->wait_flag = 0;
      prpvt->delay_flag = 1;
      callbackRequestDelayed(&prpvt->delayFuncCb, prpvt->delay);
    }
  else
    {
      prpvt->delay_flag = 0;
    }

  recGblGetTimeStamp(prec);
  /* check for alarms */
  checkAlarms(prec);

  /* check event list */
  monitor_mask = recGblResetAlarms(prec);
  if(prec->osent != prec->sent) 
    {
      monitor_mask |= DBE_VALUE|DBE_LOG;
      prec->osent = prec->sent;
    }
  if(monitor_mask)
    db_post_events(prec,&prec->sent,monitor_mask);
}


static void checkLinkCallback(CALLBACK *pcallback)
{
  struct throttleRecord *prec;
  struct rpvtStruct    *prpvt;

  callbackGetUser(prec, pcallback);
  prpvt = (struct rpvtStruct *)prec->rpvt;

  if (!interruptAccept) 
    {
      /* Can't call dbScanLock yet.  Schedule another CALLBACK */
      prpvt->pending_checkLinkCB = 1;  /* make sure */
      callbackRequestDelayed(&prpvt->checkLinkCb, 0.5);
    } 
  else 
    {
      dbScanLock((struct dbCommon *)prec);
      prpvt->pending_checkLinkCB = 0;
      checkLink(prec);
      dbScanUnlock((struct dbCommon *)prec);
    }
}


static void checkLink(struct throttleRecord *prec)
{
  struct link *plink;
  struct rpvtStruct   *prpvt = (struct rpvtStruct *)prec->rpvt;
  int stat;
  int caLink   = 0;
  int caLinkNc = 0;
  unsigned short *plinkValid;

  plink   = &prec->out;
  plinkValid = &prec->ov;

  if (plink->type == CA_LINK) 
    {
      caLink = 1;
      stat = dbCaIsLinkConnected(plink);
      if (!stat && (*plinkValid == throttleOV_EXT_NC)) 
        {
          caLinkNc = 1;
        }
      else if (!stat && (*plinkValid == throttleOV_EXT)) 
        {
          *plinkValid = throttleOV_EXT_NC;
          db_post_events(prec,plinkValid,DBE_VALUE|DBE_LOG);
          caLinkNc = 1;
        } 
      else if (stat && (*plinkValid == throttleOV_EXT_NC)) 
        {
          *plinkValid = throttleOV_EXT;
          db_post_events(prec,plinkValid,DBE_VALUE|DBE_LOG);
        } 
    }
    
  
  if (caLinkNc)
    prpvt->caLinkStat = CA_LINKS_NOT_OK;
  else if (caLink)
    prpvt->caLinkStat = CA_LINKS_ALL_OK;
  else
    prpvt->caLinkStat = NO_CA_LINKS;

  if (!prpvt->pending_checkLinkCB && caLinkNc) 
    {
      /* Schedule another CALLBACK */
      prpvt->pending_checkLinkCB = 1;
      callbackRequestDelayed(&prpvt->checkLinkCb, 0.5);
    }
}

