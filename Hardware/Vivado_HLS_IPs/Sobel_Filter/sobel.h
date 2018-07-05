/*******************************************************************************
* Filename:   sobel.h
* Authors:    Othon Tomoutzoglou <otto_sta@hotmail.com>
*             Dimitrios Bakoyiannis <d.bakoyiannis@gmail.com>
* License:
*
* MIT License
*
* Copyright (c) [2018] [Othon Tomoutzoglou, Dimitrios Bakoyiannis]
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
* 
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
* 
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
********************************************************************************/

#ifndef _SOBEL_H_
#define _SOBEL_H_
#include "ap_bmp.h"
#include "ap_axi_sdata2.h"
#include "ap_int.h"
#include "ap_utils.h"
#include "ap_video.h"

#define MAX_WIDTH  1920
#define MAX_HEIGHT 1080

#define SECTORS    16

#define ABSDIFF(x,y)	((x>y)? x - y : y - x)
#define ABS(x)          ((x>0)? x : -x)
#define RGB(r,g,b) ((((word)r)<<16)|(((word)g)<<8)|((word)b))

typedef ap_rgb <8, 8, 8> RGB;
typedef ap_axiu2 <32, 1, 1, 1> AXI_PIXEL;

typedef ap_linebuffer <unsigned char, 4, (MAX_WIDTH/SECTORS)+2> LINE4_SECTOR_BUFFER;
typedef ap_linebuffer <RGB, 1, (MAX_WIDTH/SECTORS)+1> LINE1_SECTOR_BUFFER;

int sobel_filter(AXI_PIXEL STREAM_IN[MAX_WIDTH],
                 AXI_PIXEL STREAM_OUT[MAX_WIDTH],
                 int rows,
                 int cols,
                 int packet_mode_en,
                 int packet_size);

#endif
