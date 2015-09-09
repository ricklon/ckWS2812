/************************************************************************/
/* 
*
* Copyright (c) 2014, Digilent <www.digilentinc.com>
* Contact Digilent for the latest version.
*
* This program is free software; distributed under the terms of 
* BSD 3-clause license ("Revised BSD License", "New BSD License", or "Modified BSD License")
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*
* 1.    Redistributions of source code must retain the above copyright notice, this
*        list of conditions and the following disclaimer.
* 2.    Redistributions in binary form must reproduce the above copyright notice,
*        this list of conditions and the following disclaimer in the documentation
*        and/or other materials provided with the distribution.
* 3.    Neither the name(s) of the above-listed copyright holder(s) nor the names
*        of its contributors may be used to endorse or promote products derived
*        from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
* INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
* DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
* LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
* OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
* OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/************************************************************************/
/*  Revision History:                                                   */
/*                                                                      */
/*    5/1/2014(KeithV): Created                                         */
/*    9/8/2015 (BPS): Updated for more accurate timing and less RAM     */
/************************************************************************/
/************************************************************************/
/*                                                                      */
/*  Supports the WS2812 signalling to drive up to 1000 devices          */
/*                                                                      */
/************************************************************************/
/************************************************************************/
/*                       Supported hardware:                            */
/*                                                                      */
/*  chipKIT WF32  Dout pin 11; unusable pins 12, 13-LED1                */
/*  chipKIT Max32 Dout pin 43 or (51 with JP4 in master);               */
/*          unusable pins 29,50,52                                      */
/*                                                                      */
/*  WARNING: currently this code assumes SPI2. Of the chipKIT boards    */
/*  this works on (WF32/Max32), the standard Arduino SPI just happens   */
/*  to always be SPI2, but this is NOT generalized code!                */
/*                                                                      */
/*  The spec says that Dout Vih = .7Vdd and Vdd = 6v-7v However...      */
/*  it seems to work with Vdd == 4.5v -> 5v and Vih == 3.3              */
/*  But this is "out of spec" operation. If you must be in              */
/*  spec you will need to put a level shifter on Dout to bring 3.3v     */
/*  up to .7Vdd. If your level shifter also inverts the data signal     */
/*  you can specify fInvert=true on begin() to invert the 3.3v signal   */
/*                                                                      */
/************************************************************************/
#include <WProgram.h>

/* CPUs with _DMAC defined have DMA. */
#if !defined(_DMAC)
  #if defined(__PIC32MZ__)
    #error PIC32MZ based chipKIT boards are not yet supported by this library.
  #else
    #error Board does not support needed DMA or SPI resources
  #endif
#endif

/*
 * Measured times (as per https://cpldcpu.wordpress.com/2014/01/14/light_ws2812-library-v2-0-part-i-understanding-the-ws2812/)
 * are 1.25uS to 9uS for a single cycle (1 or 0)
 * A '0' should be high from between 62.5nS to 500nS
 * A '1' should be high from between 625nS to almost 1.25uS (entire bit cycle time) 
 *
 * Ideal times (as per LED spec are)
 * WS2812/WS2812S
 *   0 high = 350nS +/- 150nS
 *   1 high = 800nS +/- 150nS
 *   0 low = 800nS  +/- 150nS
 *   1 low = 600nS  +/- 150nS
 * Total bit time = 1250nS +/- 600nS
 *
 * WS2812B
 *   0 high = 350nS +/- 150nS
 *   1 high = 900nS +/- 150nS
 *   0 low = 900nS  +/- 150nS
 *   1 low = 350nS  +/- 150nS
 * Total bit time = 1250nS +/- 150nS
 */

/* This is the number of SPI clocks per 1 or 0 being sent out to the LED. You MUST
 * change this value if you change WS2812_SPI_CLOCK_RATE in CoreTimer.c to maintain
 * the necessary timing per 1 or 0. With a 3MHz SPI clock, we use a 1 high time of 
 * 1 SPI clock, a 0 high time of 2 SPI clocks, and a total bit time of 4 SPI clocks.
 *
 * You must also change this value if you increase the WS2812_DEFAULT_BIT_WIDTH_CLKS
 * value below, or if you pass in a new cBitWidth value to begin() that's bigger.
 *
 * Normally this value should be set to the same number as WS2812_DEFAULT_BIT_WIDTH_CLKS.
 */
#define WS2812_MAX_SPI_CLOCKS_PER_LED_BIT (4)
/* The number of bytes needed in the SPI DMA buffer to represent one LED worth of 
 * brightness values.*/
#define WS2812_MAX_SPI_BYTES_PER_LED      (WS2812_MAX_SPI_CLOCKS_PER_LED_BIT * 3)    
/* A macro to help the user in their sketch define the size of the SPI DMA buffer */
#define CBWS2812PATBUF(__cDevices)        (WS2812_MAX_SPI_BYTES_PER_LED * __cDevices)
/* Default total, 1 high and 0 high clock counts. Can be over-ridden on begin() */
#define WS2812_DEFAULT_BIT_WIDTH_CLKS      4  // 1332nS  
#define WS2812_DEFAULT_BIT_1_HIGH_CLKS     1  //  333nS
#define WS2812_DEFAULT_BIT_0_HIGH_CLKS     2  //  666nS

class WS2812 {
   
public:

    typedef struct _GRB
    {
        uint8_t green;
        uint8_t red;
        uint8_t blue;
    } GRB;

    WS2812();
    ~WS2812();

    bool begin(
        uint32_t cDevices, 
        uint8_t * pPatternBuffer, 
        uint32_t cbPatternBuffer, 
        bool fInvert = false,
        uint16_t cBitWidth = WS2812_DEFAULT_BIT_WIDTH_CLKS, 
        uint16_t cBit1High = WS2812_DEFAULT_BIT_1_HIGH_CLKS, 
        uint16_t cBit0High = WS2812_DEFAULT_BIT_0_HIGH_CLKS);
    bool updateLEDs(GRB rgGRB[], uint32_t cPass = 5);
    void abortUpdate(void);
    void end(void);

private:

    typedef enum
    {
        INIT,
        WAITUPD,
        CONVGRB,
        INVERT,
        ENDUPD
    } UST;

    bool            _fInit;
    bool            _fInvert;
    uint32_t        _cDevices;
    uint32_t        _iNextDevice;
    uint8_t *       _pPatternBuffer;
    uint32_t        _cbPatternBuffer;
    uint32_t        _iByte;
    uint32_t        _iBit;
    GRB *           _pGRB;
    UST             _updateState;
    uint8_t         _iBitSPIClocks;         // Total number of SPI clocks for a 1 or a 0 bit
    uint8_t         _iBit1SPIClocksHigh;    // Number of SPI clocks for a 1 bit high period
    uint8_t         _iBit0SPIClocksHigh;    // Number of SPI clocks for a 0 bit high period

    void init(void);
    void applyGRB(GRB& grb);
    void applyColor(uint8_t color);
    void applyBit(uint32_t fOne);
};

