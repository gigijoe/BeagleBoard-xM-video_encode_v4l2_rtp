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
/*
 * dm355: The --maintain_cache option is not relevant to DM355.
 *
 * dm6446:  The --recon_file option has not been tested with DM6446 codecs.
 *
 * dm6467:  The --recon_file option has not been tested with DM6467 codecs.
 */

#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <string.h>

#include <xdc/std.h>

#include "../appMain.h"

#define DEFAULT_ENGINE_NAME     "encode"

/* Default arguments for app */
#define DEFAULT_ARGS { 100, 0, 0, -1, FALSE, FALSE, FALSE }

/*
 * Argument IDs for long options. They must not conflict with ASCII values,
 * so start them at 256.
 */
typedef enum
{
   ArgID_BENCHMARK = 256,
   ArgID_BITRATE,
   ArgID_CODEC,
   ArgID_ENGINE,
   ArgID_HELP,
   ArgID_INPUT_FILE,
   ArgID_CACHE,
   ArgID_NUMFRAMES,
   ArgID_OUTPUT_FILE,
   ArgID_RECON_FILE,
   ArgID_RESOLUTION,
   ArgID_NUMARGS
} ArgID;


/******************************************************************************
 * usage
 ******************************************************************************/
static Void usage(void)
{
    fprintf(stderr, "Usage: video_encode_io1_<platform> [options]\n\n"
        "Options:\n"
        "     --benchmark      Print benchmarking information\n"
        "-b | --bitrate        Bitrate used to process video stream [variable]\n"
        "-c | --codec          Name of codec to use\n"
        "-e | --engine         Codec engine containing specified codec\n"
        "-h | --help           Print usage information (this message)\n"
        "-i | --input_file     Name of input file containing raw YUV\n"
        "   | --cache          Cache codecs input/output buffers and perform\n"
        "                      cache maintenance. Useful for local codecs\n"
        "-n | --numframes      Number of frames to process [Default: 100]\n"
        "-o | --output_file    Name of output file for encoded video\n"
        "     --recon_file     Name of output file for reconstructed frames\n"
        "-r | --resolution     Video resolution ('width'x'height')\n"
        "\n"
        "At a minimum the codec name, the resolution and the file names "
        "*must* be given\n\n");
}

/******************************************************************************
 * parseArgs
 ******************************************************************************/
static Void parseArgs(Int argc, Char *argv[], Args *argsp)
{
    const char shortOptions[] = "b:c:e:hi:n:o:r:";

    const struct option longOptions[] = {
        {"benchmark",       no_argument,       NULL, ArgID_BENCHMARK   },
        {"bitrate",         required_argument, NULL, ArgID_BITRATE     },
        {"codec",           required_argument, NULL, ArgID_CODEC       },
        {"engine",          required_argument, NULL, ArgID_ENGINE      },
        {"help",            no_argument,       NULL, ArgID_HELP        },
        {"input_file",      required_argument, NULL, ArgID_INPUT_FILE  },
        {"cache",           no_argument,       NULL, ArgID_CACHE       },
        {"numframes",       required_argument, NULL, ArgID_NUMFRAMES   },
        {"output_file",     required_argument, NULL, ArgID_OUTPUT_FILE },
        {"recon_file",      required_argument, NULL, ArgID_RECON_FILE  },
        {"resolution",      required_argument, NULL, ArgID_RESOLUTION  },
        {0, 0, 0, 0}
    };

    Int  codec   = FALSE;
    Int  inFile  = FALSE;
    Int  outFile = FALSE;
    Int  index;
    Int  argID;

    strncpy(argsp->engineName, DEFAULT_ENGINE_NAME, MAX_ENGINE_NAME_SIZE);

    for (;;) {
        argID = getopt_long(argc, argv, shortOptions, longOptions, &index);

        if (argID == -1) {
            break;
        }

        switch (argID) {

            case ArgID_BENCHMARK:
                argsp->benchmark = TRUE;
                break;

            case ArgID_BITRATE:
            case 'b':
                argsp->bitRate = atoi(optarg);
                break;

            case ArgID_CODEC:
            case 'c':
                strncpy(argsp->codecName, optarg, MAX_CODEC_NAME_SIZE);
                codec = TRUE;
                break;

            case ArgID_ENGINE:
            case 'e':
                strncpy(argsp->engineName, optarg, MAX_ENGINE_NAME_SIZE);
                break;

            case ArgID_HELP:
            case 'h':
                usage();
                exit(EXIT_SUCCESS);

            case ArgID_INPUT_FILE:
            case 'i':
                strncpy(argsp->inFile, optarg, MAX_FILE_NAME_SIZE);
                inFile = TRUE;
                break;

            case ArgID_CACHE:
                argsp->cache = TRUE;
                break;
                
            case ArgID_NUMFRAMES:
            case 'n':
                argsp->numFrames = atoi(optarg);
                break;

            case ArgID_OUTPUT_FILE:
            case 'o':
                strncpy(argsp->outFile, optarg, MAX_FILE_NAME_SIZE);
                outFile = TRUE;
                break;

            case ArgID_RECON_FILE:
                strncpy(argsp->reconFile, optarg, MAX_FILE_NAME_SIZE);
                argsp->writeReconFrames = TRUE;
                break;

            case ArgID_RESOLUTION:
            case 'r':
                if (sscanf(optarg, "%dx%d", &argsp->width,
                                            &argsp->height) != 2) {
                    fprintf(stderr, "Invalid resolution supplied (%s)\n",
                            optarg);
                    usage();
                    exit(EXIT_FAILURE);
                }

                if (argsp->width % 8 || argsp->height % 8) {
                    fprintf(stderr, "Width and height must be multiple of 8\n");                }
                break;

            default:
                usage();
                exit(EXIT_FAILURE);
        }
    }

    if (optind < argc) {
        usage();
        exit(EXIT_FAILURE);
    }

    if (!codec || !inFile || !outFile || !argsp->width || !argsp->height) {
        usage();
        exit(EXIT_FAILURE);
    }
}

/******************************************************************************
 * main
 ******************************************************************************/
Int main(Int argc, Char *argv[])
{
    Args args = DEFAULT_ARGS;
    
    /* Parse the arguments given to the app */
    parseArgs(argc, argv, &args);    

    appMain(&args);
    
    return 0;
}
