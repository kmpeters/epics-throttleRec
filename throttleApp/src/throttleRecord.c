/* cscriptRecord.c */
/* Example record support module */
  
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
#include "cscriptRecord.h"
#undef  GEN_SIZE_OFFSET
#include "epicsExport.h"

#include "cengine.h"
#include "cengine_func.h"
#include "cengine_type.h"

#include <epicsVersion.h>
#ifndef EPICS_VERSION_INT
#define VERSION_INT(V,R,M,P) ( ((V)<<24) | ((R)<<16) | ((M)<<8) | (P))
#define EPICS_VERSION_INT VERSION_INT(EPICS_VERSION, EPICS_REVISION, EPICS_MODIFICATION, EPICS_PATCH_LEVEL)
#endif
#define LT_EPICSBASE(V,R,M,P) (EPICS_VERSION_INT < VERSION_INT((V),(R),(M),(P)))


#define VERSION 1.0

#define CODE_STRING_LENGTH (80)
#define CODE_STRING_NUMBER (10)

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
 
rset cscriptRSET =
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
epicsExportAddress(rset,cscriptRSET);


static void checkAlarms(cscriptRecord *prec);
static void monitor(cscriptRecord *prec);

static void checkLinksCallback();
static void checkLinks();

static void checkCode(cscriptRecord *prec);

#define MAX_INP_FIELDS 4 

enum { NO_CA_LINKS, CA_LINKS_ALL_OK, CA_LINKS_NOT_OK };
typedef struct rpvtStruct 
{
  struct cengine_t *cengine;
  CEFloat ivals[MAX_INP_FIELDS*2];

  CALLBACK checkLinkCb;
  short    pending_checkLinkCB;
  short    caLinkStat; /* NO_CA_LINKS,CA_LINKS_ALL_OK,CA_LINKS_NOT_OK */
  //  short    firstCalcPosted;
} rpvtStruct;


static long init_record(void *precord,int pass)
{
  cscriptRecord *prec = (cscriptRecord *)precord;
  rpvtStruct  *prpvt;
  char *string;
  //  long status;

  struct link         *pinlink,      *poutlink;
  unsigned short      *pInLinkValid, *pOutLinkValid;
  struct dbAddr        dbAddr;

  struct cengine_t *cengine;

  int i;


  if( pass == 0) 
    {
      prec->vers = VERSION;
      prec->rpvt = calloc(1, sizeof(struct rpvtStruct));

      return 0;
    }

  prec->val = 0;

  prpvt = prec->rpvt;


  /* start link management */

  pinlink = &prec->inpa;
  poutlink = &prec->outa;
  pInLinkValid = &prec->iav;
  pOutLinkValid = &prec->oav;

  prpvt->caLinkStat = NO_CA_LINKS; // as far as I know
  for (i = 0; i < MAX_INP_FIELDS; i++) 
    {
      /* check input links  */
      if (pinlink->type == CONSTANT) 
        {
          //          recGblInitConstantLink(pinlink,DBF_DOUBLE,pvalue);
          //db_post_events(ptran, pvalue, DBE_VALUE|DBE_LOG);
          *pInLinkValid = cscriptIAV_CON;
        }
      /* see if the PV resides on this ioc */
      else if (!dbNameToAddr(pinlink->value.pv_link.pvname, &dbAddr)) 
        {
          *pInLinkValid = cscriptIAV_LOC;
        }
      /* pv is not on this ioc. Callback later for connection stat */
      else 
        {
          *pInLinkValid = cscriptIAV_EXT_NC;
          prpvt->caLinkStat = CA_LINKS_NOT_OK;
        }
      db_post_events(prec,pInLinkValid,DBE_VALUE|DBE_LOG);

     /* check output links */
      if (poutlink->type == CONSTANT) 
        {
          *pOutLinkValid = cscriptIAV_CON;
        }
      else if (!dbNameToAddr(poutlink->value.pv_link.pvname, &dbAddr)) 
        {
          *pOutLinkValid = cscriptIAV_LOC;
        }
      else 
        {
          *pOutLinkValid = cscriptIAV_EXT_NC;
          prpvt->caLinkStat = CA_LINKS_NOT_OK;
        }
      db_post_events(prec,pOutLinkValid,DBE_VALUE|DBE_LOG);

      pinlink++;
      poutlink++;
      pInLinkValid++;
      pOutLinkValid++; 
    }

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


  cengine = cengine_init();
  prpvt->cengine = cengine;

  // need helper function for this
  string = malloc( CODE_STRING_LENGTH * CODE_STRING_NUMBER * sizeof( char) );
  string[0] = '\0';
  cengine_string_set( cengine, string);

  cengine_db_add_val( cengine, FLOAT_TYPE, "W", &prec->w);
  cengine_db_add_val( cengine, FLOAT_TYPE, "X", &prec->x);
  cengine_db_add_val( cengine, FLOAT_TYPE, "Y", &prec->y);
  cengine_db_add_val( cengine, FLOAT_TYPE, "Z", &prec->z);

  cengine_db_add_val( cengine, INT_TYPE, "I", &prec->i);
  cengine_db_add_val( cengine, INT_TYPE, "J", &prec->j);
  cengine_db_add_val( cengine, INT_TYPE, "K", &prec->k);
  cengine_db_add_val( cengine, INT_TYPE, "L", &prec->l);


  cengine_db_add_val( cengine, FLOAT_TYPE, "A", &prpvt->ivals[0]);
  cengine_db_add_val( cengine, FLOAT_TYPE, "B", &prpvt->ivals[1]);
  cengine_db_add_val( cengine, FLOAT_TYPE, "C", &prpvt->ivals[2]);
  cengine_db_add_val( cengine, FLOAT_TYPE, "D", &prpvt->ivals[3]);
  cengine_db_add_val( cengine, FLOAT_TYPE, "AA", &prpvt->ivals[4]);
  cengine_db_add_val( cengine, FLOAT_TYPE, "BB", &prpvt->ivals[5]);
  cengine_db_add_val( cengine, FLOAT_TYPE, "CC", &prpvt->ivals[6]);
  cengine_db_add_val( cengine, FLOAT_TYPE, "DD", &prpvt->ivals[7]);

  return 0;
}

static long process(cscriptRecord *prec)
{
  rpvtStruct *prpvt = prec->rpvt;
  struct cengine_t *cengine = prpvt->cengine;

  struct link *plink;

  CEInt v;

  long status;
  int  i;


  if( prec->sts == cscriptSTS_UNC)
    checkCode( prec);
  // second case SHOULDN'T happen
  if( (prec->sts == cscriptSTS_ERR) || ( prec->sts == cscriptSTS_UNC) )
    return 0;

  prec->pact = TRUE;
  prec->udf = FALSE;

  /* if some links are CA, check connections */
  if (prpvt->caLinkStat != NO_CA_LINKS) 
    {
      checkLinks(prec);
    }

  /* Process input links. */
  plink = &prec->inpa;
  for (i = 0; i < MAX_INP_FIELDS; i++, plink++) 
    {
      if( plink->type == CONSTANT) 
        recGblInitConstantLink(plink,DBF_DOUBLE, &prpvt->ivals[i]);
      else
        {
          //          Debug(15, "process: field %s has an input link.\n", Fldnames[i]);
          status = dbGetLink(plink, DBR_DOUBLE, &prpvt->ivals[i], NULL, NULL);
          if (!RTN_SUCCESS(status)) 
            {
              //              Debug(15, "process: dbGetLink() failed for field %s.\n", 
              //                    Fldnames[i]);
              prpvt->ivals[i] = 0.0;
            }
          /* if (DEBUG_LEVEL >= 15)  */
          /*   { */
          /*     printf("transform(%s.%s):process: Val = %f, NSTA=%d, NSEV=%d\n", */
          /*            ptran->name, Fldnames[i], *pval, ptran->nsta, ptran->nsev); */
          /*   } */
        }
    }


  cengine_execute( cengine, &v);
  prec->val = v;




 /* Process output links. */
  plink = &(prec->outa);
  for (i = 0; i < MAX_INP_FIELDS; i++, plink++)
    {
      if (plink->type != CONSTANT)
        {
          //          Debug(15, "process: field %s has an output link.\n", Fldnames[i]);
          status = dbPutLink(plink, DBR_DOUBLE, 
                             &prpvt->ivals[MAX_INP_FIELDS + i], 1);
          /* if (!RTN_SUCCESS(status)) */
          /*   { */
          /*     Debug(15, "process: ERROR %ld PUTTING TO OUTPUT LINK.\n", status); */
          /*   } */
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
  struct cscriptRecord *prec = (cscriptRecord *)paddr->precord;
  struct rpvtStruct   *prpvt = prec->rpvt;
  int    special_type = paddr->special;

  int   fieldIndex = dbGetFieldIndex(paddr);
  int   lnkIndex;
  char  (*pvalue)[CODE_STRING_LENGTH];

  struct link *plink;
  unsigned short    *plinkValid;
  struct dbAddr        dbAddr;

  
  if( !after) 
    return 0;

  if( special_type != SPC_MOD)
    {
      recGblDbaddrError(S_db_badChoice, paddr, "cscript: special");
      return (S_db_badChoice);
    }

  switch(fieldIndex) 
    {
    case(cscriptRecordINPA):
    case(cscriptRecordINPB):
    case(cscriptRecordOUTA):
    case(cscriptRecordOUTB):

      /* If user has changed a link, check it */
      lnkIndex = fieldIndex - cscriptRecordINPA;
      if ((lnkIndex >= 0) && (lnkIndex < 2*MAX_INP_FIELDS)) 
        {
          //          Debug(15, "special: checking link, i=%d\n", lnkIndex);
          plink   = &prec->inpa + lnkIndex;
          //          pvalue  = &ptran->a    + lnkIndex;
          plinkValid = &prec->iav + lnkIndex;
              
          if (plink->type == CONSTANT) 
            {
              /* /\* get initial value if this is an input link *\/ */
              /* if (fieldIndex < cscriptRecordOUTA)  */
              /*   { */
              /*     recGblInitConstantLink(plink,DBF_DOUBLE,pvalue); */
              /*     db_post_events(ptran,pvalue,DBE_VALUE|DBE_LOG); */
              /*   } */
              //            Debug(15, "special: ...constant link, i=%d\n", lnkIndex);
              *plinkValid = cscriptIAV_CON;
            }
          /* see if the PV resides on this ioc */
          else if (!dbNameToAddr(plink->value.pv_link.pvname, &dbAddr)) 
            {
              *plinkValid = cscriptIAV_LOC;
              //             Debug(15, "special: ...local link, i=%d\n", lnkIndex);
            }
          /* pv is not on this ioc. Callback later for connection stat */
          else 
            {
              *plinkValid = cscriptIAV_EXT_NC;
              /* DO_CALLBACK, if not already scheduled */
              //             Debug(15, "special: ...CA link, pending_checkLinkCB=%d\n", 
              //                    prpvt->pending_checkLinkCB);
              if (!prpvt->pending_checkLinkCB) 
                {
                  prpvt->pending_checkLinkCB = 1;
                  callbackRequestDelayed(&prpvt->checkLinkCb, 0.5);
                  prpvt->caLinkStat = CA_LINKS_NOT_OK;
                  //                  Debug(15, "special: ...CA link, i=%d, req. callback\n", 
                  //                        lnkIndex);
                }
            }
          db_post_events(prec,plinkValid,DBE_VALUE|DBE_LOG);
        }
      
      break;

    case(cscriptRecordSTR0):
    case(cscriptRecordSTR1):
    case(cscriptRecordSTR2):
    case(cscriptRecordSTR3):
    case(cscriptRecordSTR4):
    case(cscriptRecordSTR5):
    case(cscriptRecordSTR6):
    case(cscriptRecordSTR7):
    case(cscriptRecordSTR8):
    case(cscriptRecordSTR9):
      lnkIndex = fieldIndex - cscriptRecordSTR0;
      pvalue  = &prec->str0 + lnkIndex;
      db_post_events(prec,pvalue,DBE_VALUE);

      prec->udf = TRUE;

      prec->sts = cscriptSTS_UNC;
      db_post_events(prec,&prec->sts,DBE_VALUE);

      if( prec->err[0] != '\0')
        {
          prec->err[0] = '\0';
          db_post_events(prec,&prec->err,DBE_VALUE);
        }
      break;

    case(cscriptRecordCHK):

      // can only check if unchecked, error cleared by changing something
      if( prec->sts != cscriptSTS_UNC)
        return 0;

      checkCode( prec);
      break;

      default:
        recGblDbaddrError(S_db_badChoice, paddr, "calc: special");
        return(S_db_badChoice);
    }
  
  return 0;
}


static long get_precision(dbAddr *paddr, long *precision)
{
  cscriptRecord *prec = (cscriptRecord *)paddr->precord;
  //  int fieldIndex = dbGetFieldIndex(paddr);

  *precision = prec->prec;

  /* if( fieldIndex != cscriptRecordVAL)  */
  /*   recGblGetPrec(paddr,precision); */

  recGblGetPrec(paddr,precision);

  return 0;
}

static void checkAlarms(cscriptRecord *prec)
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


static void monitor(cscriptRecord *prec)
{
    unsigned short  monitor_mask;

    double *pdnew, *pdprev;
    int32_t *plnew, *plprev;

    int i;

    monitor_mask = recGblResetAlarms(prec);
    if(prec->oval != prec->val) 
      {
	monitor_mask |= DBE_VALUE|DBE_LOG;
	prec->oval = prec->val;
      }
    if(monitor_mask)
	db_post_events(prec,&prec->val,monitor_mask);

    for(i = 0, pdnew = &prec->w, pdprev = &prec->lw; i<4; 
        i++, pdnew++, pdprev++) 
      {
        if((*pdnew != *pdprev) || (monitor_mask&DBE_ALARM)) 
          {
            db_post_events(prec, pdnew, monitor_mask|DBE_VALUE|DBE_LOG);
            *pdprev = *pdnew;
          }
      }

    for(i = 0, plnew = &prec->i, plprev = &prec->li; i<4; 
        i++, plnew++, plprev++) 
      {
        if((*plnew != *plprev) || (monitor_mask&DBE_ALARM)) 
          {
            db_post_events(prec, plnew, monitor_mask|DBE_VALUE|DBE_LOG);
            *plprev = *plnew;
          }
      }

    return;
}


static void checkLinksCallback(CALLBACK *pcallback)
{
  struct cscriptRecord *prec;
  struct rpvtStruct    *prpvt;

  callbackGetUser(prec, pcallback);
  prpvt = (struct rpvtStruct *)prec->rpvt;

  //  Debug(15, "checkLinksCallback() for %s\n", ptran->name);

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


static void checkLinks(struct cscriptRecord *prec)
{
  struct link *plink;
  struct rpvtStruct   *prpvt = (struct rpvtStruct *)prec->rpvt;
  int i;
  int stat;
  int caLink   = 0;
  int caLinkNc = 0;
  unsigned short *plinkValid;

  //  Debug(15, "checkLinks() for %p\n", prec);

  plink   = &prec->inpa;
  plinkValid = &prec->iav;

  // 2*MAX_FIELDS due to INP and OUT are back to back
  for (i=0; i<2*MAX_INP_FIELDS; i++, plink++, plinkValid++) 
    {
      if (plink->type == CA_LINK) 
        {
          caLink = 1;
          stat = dbCaIsLinkConnected(plink);
          if (!stat && (*plinkValid == cscriptIAV_EXT_NC)) 
            {
              caLinkNc = 1;
            }
          else if (!stat && (*plinkValid == cscriptIAV_EXT)) 
            {
              *plinkValid = cscriptIAV_EXT_NC;
              db_post_events(prec,plinkValid,DBE_VALUE|DBE_LOG);
              caLinkNc = 1;
            } 
          else if (stat && (*plinkValid == cscriptIAV_EXT_NC)) 
            {
              *plinkValid = cscriptIAV_EXT;
              db_post_events(prec,plinkValid,DBE_VALUE|DBE_LOG);
            } 
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


static void checkCode(cscriptRecord *prec)
{
  struct rpvtStruct   *prpvt = prec->rpvt;
  struct cengine_t *cengine = prpvt->cengine;

  char *string;
  int i;

  string = cengine_string_get( cengine);
  strcpy( string, prec->str0);
  for( i = 1; i < CODE_STRING_NUMBER; i++)
    {
      strcat( string, " ");
      strcat( string, *(&prec->str0 + i) );
    }

  if( cengine_parse(cengine, 1) )
    {
      prec->sts = cscriptSTS_ERR;
      strcpy( prec->err, cengine_parse_error( cengine) );
      db_post_events(prec,&prec->err,DBE_VALUE);
      prec->udf = TRUE;
    }
  else
    {
      if( !cengine_check( cengine) )
        {
          if( prec->err[0] != '\0')
            {
              prec->err[0] = '\0';
              db_post_events(prec,&prec->err,DBE_VALUE);
            }

          prec->sts = cscriptSTS_RDY;

          // everything is a go if here
          prec->udf = FALSE;
        }
      else
        {
          prec->sts = cscriptSTS_ERR;
          strcpy( prec->err, "Logic error (fix me)" );
          db_post_events(prec,&prec->err,DBE_VALUE);
          prec->udf = TRUE;
        }
    }
  db_post_events(prec,&prec->sts,DBE_VALUE);

}
