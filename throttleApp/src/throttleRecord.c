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


#define VERSION "0-0-1"


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
static void monitor(throttleRecord *prec);

static void delayFuncCallback();

static void checkLinksCallback();
static void checkLinks();

enum { NO_CA_LINKS, CA_LINKS_ALL_OK, CA_LINKS_NOT_OK };
typedef struct rpvtStruct 
{
  double ival, oval;
  double delay;

  int delay_flag;
  int wait_flag;
  double proposed;

  CALLBACK delayFuncCb;

  CALLBACK checkLinkCb;
  short    pending_checkLinkCB;
  short    caLinkStat; /* NO_CA_LINKS,CA_LINKS_ALL_OK,CA_LINKS_NOT_OK */
} rpvtStruct;


static long init_record(void *precord,int pass)
{
  throttleRecord *prec = (throttleRecord *)precord;
  rpvtStruct  *prpvt;

  struct link         *pinlink,      *poutlink;
  unsigned short      *pInLinkValid, *pOutLinkValid;
  struct dbAddr        dbAddr;

  if( pass == 0) 
    {
      strcpy(prec->vers, VERSION);
      prec->rpvt = calloc(1, sizeof(struct rpvtStruct));

      return 0;
    }

  
  prec->sts = throttleSTS_UNK;
  prec->val = 0;

  prpvt = prec->rpvt;

  /* start link management */

  pinlink = &prec->inp;
  poutlink = &prec->out;
  pInLinkValid = &prec->iv;
  pOutLinkValid = &prec->ov;

  prpvt->caLinkStat = NO_CA_LINKS; // as far as I know

  /* check output links */
  if (poutlink->type == CONSTANT) 
    {
      *pOutLinkValid = throttleIV_CON;
    }
  else if (!dbNameToAddr(poutlink->value.pv_link.pvname, &dbAddr)) 
    {
      *pOutLinkValid = throttleIV_LOC;
    }
  else 
    {
      *pOutLinkValid = throttleIV_EXT_NC;
      prpvt->caLinkStat = CA_LINKS_NOT_OK;
    }
  db_post_events(prec,pOutLinkValid,DBE_VALUE|DBE_LOG);

  /* check input links  */
  if (pinlink->type == CONSTANT) 
    {
      //          recGblInitConstantLink(pinlink,DBF_DOUBLE,pvalue);
      //db_post_events(ptran, pvalue, DBE_VALUE|DBE_LOG);
      *pInLinkValid = throttleIV_CON;
    }
  /* see if the PV resides on this ioc */
  else if (!dbNameToAddr(pinlink->value.pv_link.pvname, &dbAddr)) 
    {
      *pInLinkValid = throttleIV_LOC;
    }
  /* pv is not on this ioc. Callback later for connection stat */
  else 
    {
      *pInLinkValid = throttleIV_EXT_NC;
      prpvt->caLinkStat = CA_LINKS_NOT_OK;
    }
  db_post_events(prec,pInLinkValid,DBE_VALUE|DBE_LOG);

  callbackSetCallback(delayFuncCallback, &prpvt->delayFuncCb);
  callbackSetPriority(prec->prio, &prpvt->checkLinkCb);
  callbackSetUser(prec, &prpvt->checkLinkCb);
  prpvt->delay_flag = 0;
  prpvt->wait_flag = 0;

  callbackSetCallback(checkLinksCallback, &prpvt->checkLinkCb);
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

  struct link *plink;

  long status;

  prec->pact = TRUE;
  prec->udf = FALSE;

  /* if some links are CA, check connections */
  if (prpvt->caLinkStat != NO_CA_LINKS) 
    {
      checkLinks(prec);
    }

 /* Process output link. */
  plink = &(prec->out);
  if (plink->type != CONSTANT)
    {
      prpvt->oval = prec->val;

      status = dbPutLink(plink, DBR_DOUBLE, &prpvt->oval, 1);
      if( RTN_SUCCESS( status) )
        prec->sts = throttleSTS_SUC;
      else
        prec->sts = throttleSTS_ERR;
    }
  else
    prec->sts = throttleSTS_ERR;


  if( prec->sts == throttleSTS_SUC)
    {
      /* Process input link. */
      plink = &(prec->inp);
      if( plink->type != CONSTANT) 
        {
          status = dbGetLink(plink, DBR_DOUBLE, &prpvt->ival, NULL, NULL);
          if (!RTN_SUCCESS(status)) 
            {
              prpvt->ival = 0.0;
            }
          prec->rdbk = prpvt->ival;
        }
    }


  recGblGetTimeStamp(prec);
  /* check for alarms */
  checkAlarms(prec);
  /* check event list */
  monitor(prec);
  /* process the forward scan link record */
  recGblFwdLink(prec);

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

  
  if( !after) 
    return 0;

  if( special_type != SPC_MOD)
    {
      recGblDbaddrError(S_db_badChoice, paddr, "throttle: special");
      return (S_db_badChoice);
    }

  switch(fieldIndex) 
    {
    case(throttleRecordINP):
      lnkIndex = throttleRecordINP;

      plink   = &prec->inp;
      //          pvalue  = &ptran->a    + lnkIndex;
      plinkValid = &prec->iv;
              
      if (plink->type == CONSTANT) 
        {
          *plinkValid = throttleIV_CON;
        }
      /* see if the PV resides on this ioc */
      else if (!dbNameToAddr(plink->value.pv_link.pvname, &dbAddr)) 
        {
          *plinkValid = throttleIV_LOC;
        }
      /* pv is not on this ioc. Callback later for connection stat */
      else 
        {
          *plinkValid = throttleIV_EXT_NC;
          if (!prpvt->pending_checkLinkCB) 
            {
              prpvt->pending_checkLinkCB = 1;
              callbackRequestDelayed(&prpvt->checkLinkCb, 0.5);
              prpvt->caLinkStat = CA_LINKS_NOT_OK;
            }
        }
      db_post_events(prec,plinkValid,DBE_VALUE|DBE_LOG);
      break;

    case(throttleRecordOUT):
      lnkIndex = throttleRecordOUT;

      plink   = &prec->out;
      //          pvalue  = &ptran->a    + lnkIndex;
      plinkValid = &prec->ov;
              
      if (plink->type == CONSTANT) 
        {
          *plinkValid = throttleIV_CON;
        }
      /* see if the PV resides on this ioc */
      else if (!dbNameToAddr(plink->value.pv_link.pvname, &dbAddr)) 
        {
          *plinkValid = throttleIV_LOC;
        }
      /* pv is not on this ioc. Callback later for connection stat */
      else 
        {
          *plinkValid = throttleIV_EXT_NC;
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
      prpvt->delay = prec->dly;

      printf("%g\n", prec->dly);
      // this will do something else
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
  //  int fieldIndex = dbGetFieldIndex(paddr);

  *precision = prec->prec;

  /* if( fieldIndex != throttleRecordVAL)  */
  /*   recGblGetPrec(paddr,precision); */

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


static void monitor(throttleRecord *prec)
{
    unsigned short  monitor_mask;


    monitor_mask = recGblResetAlarms(prec);
    if(prec->oval != prec->val) 
      {
	monitor_mask |= DBE_VALUE|DBE_LOG;
	prec->oval = prec->val;
      }
    if(monitor_mask)
	db_post_events(prec,&prec->val,monitor_mask);

    if(prec->ordbk != prec->rdbk) 
      {
	monitor_mask |= DBE_VALUE|DBE_LOG;
	prec->ordbk = prec->rdbk;
      }
    if(monitor_mask)
	db_post_events(prec,&prec->rdbk,monitor_mask);

    return;
}


static void delayFuncCallback(CALLBACK *pcallback)
{

}

static void checkLinksCallback(CALLBACK *pcallback)
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
      checkLinks(prec);
      dbScanUnlock((struct dbCommon *)prec);
    }
}


static void checkLinks(struct throttleRecord *prec)
{
  struct link *plink;
  struct rpvtStruct   *prpvt = (struct rpvtStruct *)prec->rpvt;
  int stat;
  int caLink   = 0;
  int caLinkNc = 0;
  unsigned short *plinkValid;

  plink   = &prec->inp;
  plinkValid = &prec->iv;

  if (plink->type == CA_LINK) 
    {
      caLink = 1;
      stat = dbCaIsLinkConnected(plink);
      if (!stat && (*plinkValid == throttleIV_EXT_NC)) 
        {
          caLinkNc = 1;
        }
      else if (!stat && (*plinkValid == throttleIV_EXT)) 
        {
          *plinkValid = throttleIV_EXT_NC;
          db_post_events(prec,plinkValid,DBE_VALUE|DBE_LOG);
          caLinkNc = 1;
        } 
      else if (stat && (*plinkValid == throttleIV_EXT_NC)) 
        {
          *plinkValid = throttleIV_EXT;
          db_post_events(prec,plinkValid,DBE_VALUE|DBE_LOG);
        } 
    }
    

  plink   = &prec->out;
  plinkValid = &prec->ov;

  if (plink->type == CA_LINK) 
    {
      caLink = 1;
      stat = dbCaIsLinkConnected(plink);
      if (!stat && (*plinkValid == throttleIV_EXT_NC)) 
        {
          caLinkNc = 1;
        }
      else if (!stat && (*plinkValid == throttleIV_EXT)) 
        {
          *plinkValid = throttleIV_EXT_NC;
          db_post_events(prec,plinkValid,DBE_VALUE|DBE_LOG);
          caLinkNc = 1;
        } 
      else if (stat && (*plinkValid == throttleIV_EXT_NC)) 
        {
          *plinkValid = throttleIV_EXT;
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

