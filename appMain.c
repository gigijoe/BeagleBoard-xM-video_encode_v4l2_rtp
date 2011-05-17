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
/*
 * This application encodes a raw yuv file using a specified codec to an
 * elementary stream video file. The format of the yuv file depends on the
 * device, for dm6467 it's 420 planar while on dm355 it's 422 interleaved.
 */

#include <stdio.h>

#include <xdc/std.h>

#include <ti/sdo/ce/Engine.h>
#include <ti/sdo/ce/CERuntime.h>

#include <ti/sdo/dmai/Dmai.h>
#include <ti/sdo/dmai/Ccv.h>
#include <ti/sdo/dmai/Cpu.h>
#include <ti/sdo/dmai/Time.h>
#include <ti/sdo/dmai/BufTab.h>
#include <ti/sdo/dmai/Capture.h>
#include <ti/sdo/dmai/Framecopy.h>
#include <ti/sdo/dmai/BufferGfx.h>
#include <ti/sdo/dmai/ce/Venc1.h>

#include "appMain.h"

/* Align buffers to this cache line size (in bytes)*/
#define BUFSIZEALIGN            128

/* vbuf size that has been selected based on size/performance tradeoff */
#define VBUFSIZE                20480

#include <string.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <linux/videodev.h>
#include <sys/mman.h>

#define VIDEO_BUFFER_COUNT  8
#define CLEAR(x) memset (&(x), 0, sizeof (x))

#define USERPTR 1

typedef struct _VBuffer {
  void *start;
  size_t length;
  unsigned char mark;
} VBuffer;

static VBuffer vbuf[VIDEO_BUFFER_COUNT];

int xioctl(int fd, int request, void* arg)
{
  int r;

  do r = ioctl (fd, request, arg);
  while (-1 == r && EINTR == errno);

  return r;
}

#include <signal.h>

static int bShutdown = 0;

void sig_handler(int s)
{
  switch(s) {
    case SIGINT:  bShutdown = 1;
      printf("Abort ...\n");
      break;
  }
}

#include <pthread.h>

int cameraFd   = -1;

static pthread_t cameraThreadId;
static pthread_mutex_t cameraMutex = PTHREAD_MUTEX_INITIALIZER;

Buffer_Handle hCameraBuf[VIDEO_BUFFER_COUNT];

int CameraThread_Init(char *device, int width, int height, int bufSize, BufferGfx_Attrs *pAttr)
{
    struct sigaction        actions;

    memset(&actions, 0, sizeof(actions));
    sigemptyset(&actions.sa_mask);
    actions.sa_flags = 0;
    actions.sa_handler = sig_handler;

    sigaction(SIGINT, &actions, NULL);
    
    memset(&vbuf[0], 0, sizeof(VBuffer) * VIDEO_BUFFER_COUNT);

//    cameraFd = open(args->videoDevice, O_RDWR | O_NONBLOCK, 0);  //Don't open with no-block, it could be problem
    cameraFd = open(device, O_RDWR, 0);
    if (cameraFd == -1)  {
        printf("Failed to open video device %s\n", device);
        return -1;
    }

    struct v4l2_cropcap cropcap;

    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if(-1 == xioctl(cameraFd, VIDIOC_CROPCAP, &cropcap))
      printf("V4L2 : VIDIOC_CROPCAP fail !!!");  /* Errors ignored. */

    struct v4l2_crop crop;

    crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    crop.c = cropcap.defrect; /* reset to default */

    if(-1 == xioctl(cameraFd, VIDIOC_S_CROP, &crop)) {
      switch (errno) {
        case EINVAL:  printf("Cropping not supported. : %d\n", __LINE__);
        /* Cropping not supported. */
          break;
        default:  printf("Errors ignored. : %d\n", __LINE__);
        /* Errors ignored. */
          break;
      }
    }

    struct v4l2_format fmt;
    CLEAR (fmt);

    fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width       = width;
    fmt.fmt.pix.height      = height;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
    fmt.fmt.pix.field       = V4L2_FIELD_ANY;

    if(-1 == xioctl(cameraFd, VIDIOC_S_FMT, &fmt))  {
      printf("V4L2 : VIDIOC_S_FMT fail\n");
      goto close_camera;
    }

    if(fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_UYVY)  {
      printf("V4L2 : No matching format\n");
      goto close_camera;
    }

    BufTab_Handle hBufTab = NULL;
    
    /* Create a table of buffers to use with the capture and display drivers */
    hBufTab = BufTab_create(VIDEO_BUFFER_COUNT, bufSize,
                            BufferGfx_getBufferAttrs(pAttr));

    if (hBufTab == NULL) {
        printf("Failed to allocate contiguous buffers\n");
        goto close_camera;
    }

    struct v4l2_requestbuffers req;
    CLEAR (req);

    req.count               = VIDEO_BUFFER_COUNT;
    req.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
#ifdef USERPTR
    req.memory              = V4L2_MEMORY_USERPTR;
#else
    req.memory              = V4L2_MEMORY_MMAP;
#endif

    if(-1 == xioctl (cameraFd, VIDIOC_REQBUFS, &req)) {
      printf("V4L2 : VIDIOC_REQBUFS fail !!!\n");
      goto close_camera;
    }

    int i;
    struct v4l2_buffer buf;
    for(i=0;i<VIDEO_BUFFER_COUNT;i++) {
      CLEAR (buf);
      buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
#ifdef USERPTR
      buf.memory = V4L2_MEMORY_USERPTR;
      buf.length = fmt.fmt.pix.sizeimage;
#else
      buf.memory = V4L2_MEMORY_MMAP;
#endif
      buf.index  = i;
      
#ifdef USERPTR
      unsigned int page_size = getpagesize();
      buf.length = (buf.length + page_size - 1) & ~(page_size - 1);
      
      vbuf[i].length = buf.length;
      
      hCameraBuf[i] = BufTab_getBuf(hBufTab, i);
      vbuf[i].start = Buffer_getUserPtr(hCameraBuf[i]);
      //vbuf[i].start = memalign(page_size, buf.length);
      
      if(!vbuf[i].start)  {
        printf("Out of memory\n");
        goto close_camera;
      }
      printf("<%d> buffer length: %d\n", i, vbuf[i].length);
      printf("<%d> buffer address : 0x%08x\n", i, (unsigned int)vbuf[i].start);
#else      
      if(-1 == xioctl(cameraFd, VIDIOC_QUERYBUF, &buf)) {
        printf("V4L2 : VIDIOC_QUERYBUF fail !!!\n");
        goto close_camera;
      }
      
      printf("<%d> buffer length: %d\n", i, buf.length);
      printf("<%d> buffer offset: %d\n", i, buf.m.offset);

      vbuf[i].length = buf.length;
      vbuf[i].start = mmap(NULL,
                    buf.length,
                    PROT_READ | PROT_WRITE,
                    MAP_SHARED,
                    cameraFd,
                    buf.m.offset);
                    
      if(MAP_FAILED == vbuf[i].start)  {
        printf("V4L2 : Mmap fail !!!\n");
        goto close_camera;
      }
      
      printf("<%d> buffer start: 0x%08x\n", i, (unsigned int)vbuf[i].start);

      hCameraBuf[i] = BufTab_getBuf(hBufTab, i);
printf("hCameraBuf[%d] = 0x%x\n", i, (int)hCameraBuf[i]);
      Buffer_setNumBytesUsed(hCameraBuf[i], buf.length);
      Buffer_setUseMask(hCameraBuf[i], pAttr->bAttrs.useMask);
      Buffer_setUserPtr(hCameraBuf[i], vbuf[i].start);
#endif
    }

    /* Free buffers for Display driver */
    BufTab_freeAll(hBufTab);

    return 0;
    
close_camera:
    close(cameraFd);
    return -1;
}

void *CameraThread_Process(void *args)
{  
  struct v4l2_buffer buf;
  int i;
  
  while(!bShutdown) {
    for(i=0;i<VIDEO_BUFFER_COUNT;i++) {
      if(vbuf[i].mark == 0)
        break;
    }
    
    if(i == VIDEO_BUFFER_COUNT) { /*  All buffers are filled  */
      usleep(1000); /*  Wait 1ms  */
      continue;
    }
      
    pthread_mutex_lock(&cameraMutex);

    CLEAR (buf);
    buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
#ifdef USERPTR
    buf.memory = V4L2_MEMORY_USERPTR;
#else
    buf.memory = V4L2_MEMORY_MMAP;
#endif
    
    if(-1 == xioctl (cameraFd, VIDIOC_DQBUF, &buf)) {
      perror("V4L2 : VIDIOC_DQBUF fail !!!");
      break;
    }
    
    for(i=0;i<VIDEO_BUFFER_COUNT;i++)  {
      if(buf.m.userptr == (unsigned long) vbuf[i].start && buf.length == vbuf[i].length)
        break;
    }

#ifdef USERPTR
    vbuf[i].mark++;
#else
    vbuf[buf.index].mark++;
#endif    
    if(-1 == xioctl (cameraFd, VIDIOC_QBUF, &buf))
      printf("V4L2 : VIDIOC_QBUF fail !!!\n");
    
    pthread_mutex_unlock(&cameraMutex);
    
    usleep(33000);  /*  33ms which is 30 fps  */
  }
  
  return 0;
}

void CameraThread_Run()
{
  struct v4l2_buffer buf;
  int i;
  for(i=0;i<VIDEO_BUFFER_COUNT;i++) {
    CLEAR (buf);

    buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
#ifdef USERPTR
    buf.memory = V4L2_MEMORY_USERPTR;
    buf.m.userptr = (unsigned long)vbuf[i].start;
    buf.length = vbuf[i].length;
    buf.index  = i;
#else
    buf.memory = V4L2_MEMORY_MMAP;
#endif

    if(-1 == xioctl (cameraFd, VIDIOC_QBUF, &buf))  {
      printf("V4L2 : VIDIOC_QBUF fail !!!\n");
    }
  }

  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if(-1 == xioctl(cameraFd, VIDIOC_STREAMON, &type))
    printf("V4L2 : VIDIOC_STREAMON fail !!!\n");


  pthread_create(&cameraThreadId, NULL, CameraThread_Process, NULL);
}    

void CameraThread_Deinit()
{
  pthread_kill(cameraThreadId, SIGINT);
  
  pthread_join(cameraThreadId, 0); /*Wait for timer thread ended*/

  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if(-1 == xioctl(cameraFd, VIDIOC_STREAMOFF, &type))
    printf("V4L2 : VIDIOC_STREAMOFF fail !!!\n");

  int i;
  for(i=0;i<VIDEO_BUFFER_COUNT;i++) {
    if(vbuf[i].start && vbuf[i].length)
#ifdef USERPTR
            Buffer_delete(hCameraBuf[i]);
#else
            munmap(vbuf[i].start, vbuf[i].length);
#endif
  }

  close(cameraFd);
}

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/uio.h>
#include "rtpdataheader.h"

static unsigned short sequence = 0;
static unsigned int timestamp = 0;

#define DEFAULT_MTU                     1400
#define IS_ACCESS_UNIT(x) (((x) > 0x00) && ((x) < 0x06))

static Char vbufferRecon[VBUFSIZE];

/******************************************************************************
 * readFrameV4l2
 ******************************************************************************/
//Int readFrameV4l2(Buffer_Handle hBuf, int fd)
Buffer_Handle readFrameV4l2(int benchmark)
{
  Buffer_Handle r = NULL;
  static int buffer_index = 0;
#if 1
  if(vbuf[buffer_index].mark > 0)  {
    
    if(vbuf[buffer_index].mark > 1 && benchmark)
      printf("Buffer[%d] overflow, count %d !!!\n", buffer_index, vbuf[buffer_index].mark);
    
    pthread_mutex_lock(&cameraMutex);

    Buffer_setNumBytesUsed(hCameraBuf[buffer_index], vbuf[buffer_index].length);
    r = hCameraBuf[buffer_index];
    
    vbuf[buffer_index].mark = 0;
  
    pthread_mutex_unlock(&cameraMutex);
    
    if(++buffer_index >= VIDEO_BUFFER_COUNT)
      buffer_index = 0;
  }
  
  if(r == NULL && benchmark)
    printf("Fail to read frame[%d]\n", buffer_index);
#endif
#if 0  
  int i;
  for(i=0;i<VIDEO_BUFFER_COUNT;i++)  {
    if(vbuf[buffer_index].mark > 0)  {
      if(vbuf[buffer_index].mark > 1 && benchmark)
        printf("Buffer[%d] overflow, count %d !!!\n", buffer_index, vbuf[buffer_index].mark);
      
      pthread_mutex_lock(&cameraMutex);

      Buffer_setNumBytesUsed(hCameraBuf[buffer_index], vbuf[buffer_index].length);
      r = hCameraBuf[buffer_index];
      
      vbuf[buffer_index].mark = 0;
    
      pthread_mutex_unlock(&cameraMutex);

      if(++buffer_index >= VIDEO_BUFFER_COUNT)
        buffer_index = 0;
      
      break;
    }
    
    if(++buffer_index >= VIDEO_BUFFER_COUNT)
      buffer_index = 0;
  }

  if(r == NULL && benchmark)
    printf("No frame ready !!!\n");
#endif
  return r;
}

/******************************************************************************
 * processReconData
 *    Transform reconstructed buffer data into a UYVY frame.  The format of
 *    the reconstructed buffer data is expected to conform to the specification
 *    found in "MPEG4 Simple Profile Encoder Codec on DM355 User's Guide"
 *    (SPRUFE4C), and may not work for other codecs.
 ******************************************************************************/
Void processReconData(IVIDEO1_BufDesc* reconBufs, Buffer_Handle hSrcBuf,
                      Buffer_Handle hDstBuf)
{
    Int16                 mbSizeY;
    Int16                 mbSizeX;
    Uint32                lumaColLength;
    Uint32                chromaColSize;
    Uint32                UYVYRowLength;
    Uint32                UYVYMbSize;
    UInt8                *lum_buf;
    UInt8                *chr_buf;
    UInt8                *curr_mb;
    UInt8                *curr_lum_mb;
    UInt8                *curr_chroma_mb;
    Int16                 i, j, k, l;
    BufferGfx_Dimensions  dim;

    /*
     * A single Master Block is 16x16 pixels.  Get our master block dimensions
     * by divding the pixel height and width by 16.
     */
    BufferGfx_getDimensions(hSrcBuf, &dim);

    mbSizeY = dim.height >> 4;
    mbSizeX = dim.width  >> 4;

    /*
     * Our luma buffer is a series of 16x16 byte blocks, and our chroma buffer
     * is a series of 16x8 byte blocks.  Each block corresponds to information
     * for one master block.  The first block in each buffer contains header
     * information.  Set lum_buf and chr_buf to the first block after the
     * header.
     */
    lum_buf = (UInt8*) (reconBufs->bufDesc[0].buf + 16 * 16);
    chr_buf = (UInt8*) (reconBufs->bufDesc[1].buf + 16 * 8);

    /*
     * The luma and chroma buffers are constructed in column-major order.
     * The blocks for a single column are followed by two padding blocks
     * before the next column starts.  Set lumaColLength and chromaColSize
     * to the number of bytes that must be skipped over to get to the next
     * column in the corresponding buffer.
     */
    lumaColLength = (16*16) * (mbSizeY + 2);
    chromaColSize = (16*8)  * (mbSizeY + 2);

    /*
     * Calculate the number of bytes that must be skipped over to go to the
     * next row in the reconstructed UYVY frame.  Also calculate how many
     * bytes in the UYVY file are needed to represent a single master block.
     */
    UYVYRowLength = 32 * mbSizeX;
    UYVYMbSize    = 32 * 16;

    /*
     * Copy the reconstructed buffer information into a UYVY frame.
     */
    for (i = 0; i < mbSizeX; i++) {
        for (j = 0; j < mbSizeY; j++) {

            /* Calculate input and output buffer offsets for the current */
            /* master block                                              */
            curr_lum_mb    = lum_buf + (lumaColLength * i) + (256 * j);
            curr_chroma_mb = chr_buf + (chromaColSize * i) + (128 * j);
            curr_mb        = (UInt8 *) Buffer_getUserPtr(hDstBuf) +
                                 (j * (UYVYMbSize * mbSizeX)) + (i * 32);

            /* Copy Luma information */
            for (k = 0; k < 16; k++) {
                for (l = 0; l < 16; l++) {
                    curr_mb[(k * UYVYRowLength) + (l * 2) + 1] =
                        curr_lum_mb[k * 16 + l];
                }
            }

            /* Copy Chroma information */
            for (k = 0; k < 8; k++) {
                for (l = 0; l < 16; l++) {
                    curr_mb[((k * 2) * UYVYRowLength) + (l * 2)] =
                        curr_chroma_mb[k * 16 + l];
                    curr_mb[((k * 2 + 1) * UYVYRowLength) + (l * 2)] =
                        curr_chroma_mb[k * 16 + l];
                }
            }
        }
    }

    Buffer_setNumBytesUsed(hDstBuf, dim.width * dim.height * 2);
}

/******************************************************************************
 * appMain
 ******************************************************************************/
Void appMain(Args * args)
{
    VIDENC1_Params         params    = Venc1_Params_DEFAULT;
    VIDENC1_DynamicParams  dynParams = Venc1_DynamicParams_DEFAULT;
    BufferGfx_Attrs        gfxAttrs  = BufferGfx_Attrs_DEFAULT;
    Buffer_Attrs           bAttrs    = Buffer_Attrs_DEFAULT;
    Time_Attrs             tAttrs    = Time_Attrs_DEFAULT;
    Venc1_Handle           hVe1      = NULL;
    Int                    sfd       = -1;
    FILE                  *reconFile = NULL;
    Engine_Handle          hEngine   = NULL;
    Time_Handle            hTime     = NULL;
    Time_Handle            hTimestamp= NULL;
    Time_Handle            hTimeFps  = NULL;
    Buffer_Handle          hOutBuf   = NULL;
    Buffer_Handle          hInBuf    = NULL;
    Buffer_Handle          hReconBuf = NULL;
    Int                    numFrame  = 0;
    Int                    averageEncodeTime = 0;
    Int                    inBufSize, outBufSize;
    Cpu_Device             device;
    ColorSpace_Type        colorSpace;
    UInt32                 time, duration = 0;

    printf("Starting application...\n");

    /* Initialize the codec engine run time */
    CERuntime_init();

    /* Initialize DMAI */
    Dmai_init();

    /* Determine which device the application is running on */
    if (Cpu_getDevice(NULL, &device) < 0) {
        printf("Failed to determine target board\n");
        goto cleanup;
    }

    if (args->benchmark) {
        hTime = Time_create(&tAttrs);

        if (hTime == NULL) {
            printf("Failed to create Time object\n");
            goto cleanup;
        }
    }

    hTimestamp = Time_create(&tAttrs);
    if(hTimestamp == NULL)  {
      printf("Failed to create Time object\n");
      goto cleanup;
    }

    hTimeFps = Time_create(&tAttrs);
    if(hTimeFps == NULL)  {
      printf("Failed to create Time object\n");
      goto cleanup;
    }

    sfd = socket(AF_INET, SOCK_DGRAM, 0);
    if(sfd <= 0)  {
      printf("Failed to open output socket\n");
      goto cleanup;
    }

    printf("Trying %s:%u\n", args->outIp, args->outPort);

    struct sockaddr_in addr;

    memset(&addr, 0, sizeof(struct sockaddr_in));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(args->outPort);
    addr.sin_addr.s_addr = inet_addr(args->outIp);

    if(connect(sfd, (struct sockaddr *)&addr, sizeof(addr)) == -1)
      printf("Connect socket fail !!!\n");

    /* Open the output file where to put reconstructed frames */
    if (args->writeReconFrames) {
        reconFile = fopen(args->reconFile, "wb");

        if (reconFile == NULL) {
            printf("Failed to open output file %s\n", args->reconFile);
            goto cleanup;
        }

        /* Using a larger vbuf to enhance performance of file i/o */
        if (setvbuf(reconFile, vbufferRecon, _IOFBF,
                    sizeof(vbufferRecon)) != 0) {
            printf("Failed to setvbuf on output file descriptor\n");
            goto cleanup;
        }
    }

    /* Open the codec engine */
    hEngine = Engine_open(args->engineName, NULL, NULL);

    if (hEngine == NULL) {
        printf("Failed to open codec engine: %s\n", args->engineName);
        goto cleanup;
    }

    /* Set up codec parameters depending on bit rate */
    if (args->bitRate < 0) {
        /* Variable bit rate */
        params.rateControlPreset = IVIDEO_NONE;

        /*
         * If variable bit rate use a bogus bit rate value (> 0)
         * since it will be ignored.
         */
        params.maxBitRate        = 2000000;
    }
    else {
        /* Constant bit rate */
        params.rateControlPreset = IVIDEO_LOW_DELAY;
        params.maxBitRate        = args->bitRate;
    }

    /* Set up codec parameters depending on device */
    if (device != Cpu_Device_DM6467) {
        if (device == Cpu_Device_DM355) {
            params.inputChromaFormat = XDM_YUV_422ILE;
            params.reconChromaFormat = XDM_YUV_420P;
        }
        else {
             params.inputChromaFormat = XDM_YUV_422ILE;
        }
    }

    params.maxWidth              = args->width;
    params.maxHeight             = args->height;

    dynParams.targetBitRate      = params.maxBitRate;
    dynParams.inputWidth         = params.maxWidth;
    dynParams.inputHeight        = params.maxHeight;

    /* Create the video encoder */
    hVe1 = Venc1_create(hEngine, args->codecName, &params, &dynParams);

    if (hVe1 == NULL) {
        printf("Failed to create video encoder: %s\n", args->codecName);
        goto cleanup;
    }

    /* Ask the codec how much in put data it needs */
    inBufSize = Venc1_getInBufSize(hVe1);

    if (inBufSize < 0) {
        printf("Failed to calculate buffer attributes\n");
        goto cleanup;
    }

    /* Ask the codec how much space it needs for output data */
    outBufSize = Venc1_getOutBufSize(hVe1);

    /* Which color space to use in the graphics buffers depends on the device */
    colorSpace = device == Cpu_Device_DM6467 ? ColorSpace_YUV420PSEMI :
                                               ColorSpace_UYVY;

    /* Align buffers to cache line boundary */
    gfxAttrs.bAttrs.memParams.align = bAttrs.memParams.align = BUFSIZEALIGN;

    /* Use cached buffers if requested */
    if (args->cache) {
        gfxAttrs.bAttrs.memParams.flags = bAttrs.memParams.flags
            = Memory_CACHED;
    }

    gfxAttrs.dim.width      = args->width;
    gfxAttrs.dim.height     = args->height;
    gfxAttrs.dim.lineLength = BufferGfx_calcLineLength(args->width, colorSpace);
    gfxAttrs.colorSpace     = colorSpace;
#ifdef USERPTR
    gfxAttrs.bAttrs.reference      = FALSE;
#else    
    gfxAttrs.bAttrs.reference      = TRUE;
#endif

    CameraThread_Init(args->videoDevice, args->width, args->height, inBufSize, &gfxAttrs);
    
    /* Create the reconstructed frame buffer for raw yuv data */
    if (args->writeReconFrames) {
        hReconBuf =
            Buffer_create(Dmai_roundUp(inBufSize, BUFSIZEALIGN),
                BufferGfx_getBufferAttrs(&gfxAttrs));

        if (hReconBuf == NULL) {
            printf("Failed to allocate contiguous buffer\n");
            goto cleanup;
        }
    }

    /* Create the output buffer for encoded video data */
    hOutBuf = Buffer_create(Dmai_roundUp(outBufSize, BUFSIZEALIGN), &bAttrs);

    if (hOutBuf == NULL) {
        printf("Failed to create contiguous buffer\n");
        goto cleanup;
    }

    signal(SIGINT, sig_handler);

    CameraThread_Run();
        
    if (args->benchmark) {
      if (Time_reset(hTime) < 0)
        printf("Failed to reset timer\n");
    
      if (Time_reset(hTimeFps) < 0)
        printf("Failed to reset timer\n");
    }
    
    while (!bShutdown)  {
        if(args->numFrames) {
          if(numFrame < args->numFrames)
            break;
        }

        hInBuf = readFrameV4l2(args->benchmark);
        if(hInBuf == NULL) {
          usleep(10000); /*  10ms */
          continue;
        }
            
        numFrame++;

        if (args->cache) {
            /*
             *  To meet xDAIS DMA Rule 7, when input buffers are cached, we
             *  must writeback the cache into physical memory.  Also, per DMA
             *  Rule 7, we must invalidate the output buffer from
             *  cache before providing it to any xDAIS algorithm.
             */
            Memory_cacheWbInv(Buffer_getUserPtr(hInBuf), Buffer_getSize(hInBuf));

            /* Per DMA Rule 7, our output buffer cache lines must be cleaned */
            Memory_cacheInv(Buffer_getUserPtr(hOutBuf), Buffer_getSize(hOutBuf));

            if (args->benchmark) {
                if (Time_delta(hTime, &time) < 0) {
                    printf("Failed to get timer delta\n");
                    goto cleanup;
                }

                printf("Pre-process cache maintenance: %uus \n", (Uns) time);
            }
        }

        /* Make sure the whole buffer is used for input */
        BufferGfx_resetDimensions(hInBuf);

        if(numFrame % 30 == 0)  { /*  To generate IDR frame every 30 frames */
          int32_t status = 0;
          VIDENC1_Status         encStatus;
          encStatus.data.buf = NULL;
          encStatus.size = sizeof(VIDENC1_Status);

          //printf("Lets create IDR SPS/PPS headers");

          dynParams.generateHeader = XDM_GENERATE_HEADER;
          dynParams.forceFrame = IVIDEO_NA_FRAME;
          status = VIDENC1_control(Venc1_getVisaHandle(hVe1), XDM_SETPARAMS, &dynParams, &encStatus);
          if(status != VIDENC1_EOK)
            printf("Failed to set encoder control SPS/PPS headers");
                    
          /* Encode the video buffer */
          if (Venc1_process(hVe1, hInBuf, hOutBuf) < 0) {
              printf("Failed to encode video buffer\n");
              goto cleanup;
          }
          
          dynParams.generateHeader = XDM_ENCODE_AU;
//          dynParams.forceFrame = IVIDEO_IDR_FRAME;
          dynParams.forceFrame = 0;
          status = VIDENC1_control(Venc1_getVisaHandle(hVe1), XDM_SETPARAMS, &dynParams, &encStatus);
          if(status != VIDENC1_EOK)
            printf("Failed to set encoder control force IDR frame");

          /* Encode the video buffer */
          if (Venc1_process(hVe1, hInBuf, hOutBuf) < 0) {
              printf("Failed to encode video buffer\n");
              goto cleanup;
          }
          
          dynParams.generateHeader = XDM_ENCODE_AU;
          dynParams.forceFrame = IVIDEO_NA_FRAME;
          status = VIDENC1_control(Venc1_getVisaHandle(hVe1), XDM_SETPARAMS, &dynParams, &encStatus);
          if(status != VIDENC1_EOK)
            printf("Failed to set encoder control");
        } else  {
          /* Encode the video buffer */
          if (Venc1_process(hVe1, hInBuf, hOutBuf) < 0) {
              printf("Failed to encode video buffer\n");
              goto cleanup;
          }
        }
        
        if (args->benchmark) {
          if (Time_delta(hTime, &time) < 0) 
            printf("Failed to get encode time\n");
          else
            printf("[%d] Encode: %uus\n", numFrame, (Uns)time);
          
          if(numFrame == 1)
            averageEncodeTime = (Uns)time;
          else
            averageEncodeTime = (averageEncodeTime + (Uns)time) / 2;
        }

        if (args->cache) {
            /* Writeback the outBuf. */
            Memory_cacheWb(Buffer_getUserPtr(hOutBuf), Buffer_getSize(hOutBuf));

            if (args->benchmark) {
                if (Time_delta(hTime, &time) < 0) {
                    printf("Failed to get timer delta\n");
                    goto cleanup;
                }

                printf("Post-process cache write back: %uus \n", (Uns) time);
            }
        }

        /* Write the encoded frame to the network */
        if (Buffer_getNumBytesUsed(hOutBuf)) {
          struct iovec data[3];
          rtp_hdr_t rtp;
          rtp.version = 2;
          rtp.p = 0;
          rtp.x = 0;
          rtp.cc = 0;
          rtp.m = 0;
          rtp.pt = 96;

          rtp.seq = htons( sequence );
          rtp.ts = htonl( timestamp );

          rtp.ssrc = 10;

          data[0].iov_base = &rtp;
          data[0].iov_len = sizeof(rtp_hdr_t);

          unsigned char *ptr = (unsigned char *)Buffer_getUserPtr(hOutBuf);
          size_t len = Buffer_getNumBytesUsed(hOutBuf);
          size_t mtu = DEFAULT_MTU;

          /* Skip NAL Start Code 00 00 00 01 */
          unsigned char nalType = ptr[4] & 0x1f;
          //printf("Processing Buffer with NAL TYPE=%d\n", nalType);

          if(len < (mtu - sizeof(rtp_hdr_t))) {
            /* Remove NAL Start Code 00 00 00 01 */
            data[1].iov_base = (void *)ptr + 4;
            data[1].iov_len = len - 4;

            //printf("NAL Unit fit in one packet size=%d\n", len);
            /* only set the marker bit on packets containing access units */
            if (IS_ACCESS_UNIT (nalType))
              rtp.m = 1;

            writev(sfd, data, 2);

            sequence++;
          } else  {
            int start = 1, end = 0;
            /* We keep 2 bytes for FU indicator and FU Header */
            unsigned payload_len = mtu - sizeof(rtp_hdr_t) - 2;
            unsigned char nalHeader = ptr[4];

            /* Remove NAL Start Code 00 00 00 01 and NAL Type*/
            ptr += 5;
            len -= 5;

            //printf("Using FU-A fragmentation for data size=%d\n", len);

            while(end == 0)  {
              unsigned char fu[2];
              payload_len = len < payload_len ? len : payload_len;
              if (payload_len == len)
                end = 1;

              if (IS_ACCESS_UNIT (nalType))
                rtp.m = end;

              /* FU indicator */
              fu[0] = (nalHeader & 0x60) | 28;
              /* FU Header */
              fu[1] = (start << 7) | (end << 6) | (nalHeader & 0x1f);

              rtp.seq = htons( sequence );

              data[1].iov_base = fu;
              data[1].iov_len = 2;

              data[2].iov_base = (void *)ptr;
              data[2].iov_len = payload_len;

              writev(sfd, data, 3);
              start = 0;

              ptr += payload_len;
              len -= payload_len;

              sequence++;
            }
          }
          
          if (Time_delta(hTimestamp, &time) < 0) {
              printf("Failed to get timestamp !!!\n");
              goto cleanup;
          }

          //timestamp += (unsigned int)(0.09 * time);  /*15 fps : 90000 / 15*/
          timestamp += (unsigned int)((90000 * time) / 1000000);          
        }

        /* Write the reconstructed frame to the file system */
        if (args->writeReconFrames) {
            processReconData(Venc1_getReconBufs(hVe1), hInBuf, hReconBuf);

            if (Buffer_getNumBytesUsed(hReconBuf)) {
                if (fwrite(Buffer_getUserPtr(hReconBuf),
                           Buffer_getNumBytesUsed(hReconBuf), 1,
                           reconFile) != 1) {
                    printf("Failed to write reconstructed frame to file\n");
                    goto cleanup;
                }
            }
        }

        if (args->benchmark) {
            if (Time_delta(hTime, &time) < 0) {
                printf("Failed to get timer delta\n");
                goto cleanup;
            }

            printf("[%d] Write : %uus\n", numFrame, (Uns)time);

            if (Time_reset(hTime) < 0)
              printf("Failed to reset timer\n");
        }
        
        if (Time_delta(hTimeFps, &time) < 0)
          printf("Failed to get timer delta\n");
        
        duration += (Uns)time;
        
        if(numFrame % 30 == 0)  {
          double fps = (double)1000000 / (duration / 30);
          duration = 0;
          printf("[%d] FPS : %f\n", numFrame, fps);
        }
        
        if (Time_reset(hTimeFps) < 0)
          printf("Failed to reset timer\n");
    }


    if (args->benchmark)
      printf("Average encode time : %dus\n", averageEncodeTime);
    
    CameraThread_Deinit();

cleanup:
    /* Clean up the application */
    if (hOutBuf) {
        Buffer_delete(hOutBuf);
    }

    if (hReconBuf) {
        Buffer_delete(hReconBuf);
    }

    if (hVe1) {
        Venc1_delete(hVe1);
    }

    if (hEngine) {
        Engine_close(hEngine);
    }

    if (sfd)
        close(sfd);

    if (reconFile) {
        fclose(reconFile);
    }

    if (hTime) {
        Time_delete(hTime);
    }

    if (hTime) {
        Time_delete(hTimestamp);
    }

    printf("End of application.\n");

    return;
}
