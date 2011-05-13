/* --COPYRIGHT--,BSD
 * Copyright (c) 2010, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/

#include <stdio.h>
#include <stdlib.h>

#include <xdc/std.h>

#include <std.h>
#include <tsk.h>

#ifdef Dmai_Device_dm6467
#include <ti/sdo/fc/rman/rman.h>
#include <ti/sdo/fc/ires/edma3chan/iresman_edma3Chan.h>
#include <ti/sdo/fc/ires/hdvicp/iresman_hdvicp.h>
#endif

#include <ti/sdo/utils/trace/gt.h>

#include <ti/sdo/dmai/Dmai.h>

#include "../appMain.h"

extern far Args passedArgs;
extern Void parseArgs(Args *argsp);

#ifdef Dmai_Device_dm6467
static IRESMAN_HdVicpParams hdvicpConfig;

static IRESMAN_Edma3ChanParams edma3Config;
#endif 

/******************************************************************************
 * main
 ******************************************************************************/
Void main()
{
    TSK_Attrs attrs = TSK_ATTRS;
    Int status = 0;
         
    /* Validate the arguments given to the app */
    parseArgs(&passedArgs);    
    
    /* init trace */
    GT_init();
    
    /* Set printf function for GT */
    GT_setprintf( (GT_PrintFxn)printf );

    /* Configure resource managers if DM6467 */
#ifdef Dmai_Device_dm6467
    RMAN_init();
    
    edma3Config.baseConfig.allocFxn = RMAN_PARAMS.allocFxn;
    edma3Config.baseConfig.freeFxn = RMAN_PARAMS.freeFxn;
    edma3Config.baseConfig.size = sizeof(IRESMAN_Edma3ChanParams);
    status = RMAN_register(&IRESMAN_EDMA3CHAN, 
        (IRESMAN_Params *)&edma3Config);
    if (status != IRES_OK) {
        exit(EXIT_FAILURE);
    }
    
    hdvicpConfig.numResources = 2;
    hdvicpConfig.baseConfig.allocFxn = RMAN_PARAMS.allocFxn;
    hdvicpConfig.baseConfig.freeFxn = RMAN_PARAMS.freeFxn;
    status = RMAN_register(&IRESMAN_HDVICP, 
        (IRESMAN_Params *)&hdvicpConfig);
    if (status != IRES_OK) {
        exit(EXIT_FAILURE);
    }
#endif
            
    attrs.stacksize = 0x4000;
    if (TSK_create((Fxn)appMain, &attrs, (Arg)&passedArgs) == NULL) {
        exit(EXIT_FAILURE);
    }
    
    return;
}
