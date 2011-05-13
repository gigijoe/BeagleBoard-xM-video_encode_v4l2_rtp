/* --COPYRIGHT--,BSD
 * Copyright (c) 2009, Texas Instruments Incorporated
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

#ifndef appMain_h_
#define appMain_h_

#include <xdc/std.h>

#define MAX_CODEC_NAME_SIZE     30
#define MAX_ENGINE_NAME_SIZE    30
#define MAX_FILE_NAME_SIZE      100

/* Arguments for app */
typedef struct Args {
    Int   numFrames;
    Int   width;
    Int   height;
    Int   bitRate;
    Int   benchmark;
    Bool  cache;
    Bool  writeReconFrames;
    Char  codecName[MAX_CODEC_NAME_SIZE];
    Char  videoDevice[MAX_FILE_NAME_SIZE];
    Char  outIp[MAX_FILE_NAME_SIZE];
    unsigned short outPort;
    Char  reconFile[MAX_FILE_NAME_SIZE];
    Char  engineName[MAX_ENGINE_NAME_SIZE];
} Args;

#if defined (__cplusplus)
extern "C" {
#endif

extern Void appMain(Args * args);

#if defined (__cplusplus)
}
#endif

#endif // appMain_h_
