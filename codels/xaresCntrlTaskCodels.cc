/**
 ** xaresCntrlTaskCodels.cc
 **
 ** Codels used by the control task xaresCntrlTask
 **
 ** Author: 
 ** Date: 
 **
 **/

#include <portLib.h>

#include "server/xaresHeader.h"


/*------------------------------------------------------------------------
 * xaresSetParametersCtrl  -  control codel of CONTROL request SetParameters
 *
 * Description:    
 * Report: OK
 *
 * Returns:    OK or ERROR
 */

STATUS
xaresSetParametersCtrl(xaresInternalParams *internalParams, int *report)
{
  /* nothing to do here */
  //TODO check consistency
  return OK;
}

/*------------------------------------------------------------------------
 * xaresEnableDumpCtrl  -  control codel of CONTROL request EnableDump
 *
 * Description:    
 * Report: OK
 *
 * Returns:    OK or ERROR
 */

STATUS
xaresEnableDumpCtrl(int *report)
{
  SDI_F->dump = GEN_TRUE;
  return OK;
}

/*------------------------------------------------------------------------
 * xaresDisableDumpCtrl  -  control codel of CONTROL request DisableDump
 *
 * Description:    
 * Report: OK
 *
 * Returns:    OK or ERROR
 */

STATUS
xaresDisableDumpCtrl(int *report)
{
  SDI_F->dump = GEN_FALSE;
  return OK;
}




