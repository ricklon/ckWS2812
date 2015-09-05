/* Simple test for WS2812 library
 * Circuit: A string of WS2812 LEDs on pin 29 (SDO)
 * Use Fubarino Mini. Press PRG button to trigger
 * the frames to 'play back' on the LEDs
 * 
  */
#include <WS2812.h>

// Number of LEDs in the chain
#define CDEVICES 10

WS2812::GRB rgGRB[CDEVICES] =
{
  {0x00, 0x00, 0x00},
  {0x00, 0x00, 0x00},
  {0x00, 0x00, 0x00},
  {0x00, 0x00, 0x00},
  {0x00, 0x00, 0x00},
  {0x00, 0x00, 0x00},
  {0x00, 0x00, 0x00},
  {0x00, 0x00, 0x00},
  {0x00, 0x00, 0x00},
  {0x00, 0x00, 0x00},
};

// Number of 'frames' in the rgGRB array
#define FRAMES 42

WS2812::GRB rgGRB_frames[FRAMES] =
{
    {0x00, 0x00, 0x00},
    {0x08, 0x08, 0x08},
    {0x10, 0x10, 0x10},
    {0x18, 0x28, 0x18},
    {0x20, 0x20, 0x20},
    {0x28, 0x28, 0x28},
    {0x30, 0x30, 0x30},
    {0x38, 0x38, 0x38},
    {0x40, 0x40, 0x40},
    {0x48, 0x48, 0x48},
    {0x50, 0x50, 0x50},
    {0x58, 0x58, 0x58},
    {0x60, 0x60, 0x60},
    {0x68, 0x68, 0x68},
    {0x70, 0x70, 0x70},
    {0x78, 0x78, 0x78},
    {0x80, 0x80, 0x80},
    {0x88, 0x88, 0x88},
    {0x90, 0x90, 0x90},
    {0x98, 0x98, 0x98},
    {0xA0, 0xA0, 0xA0},
    {0xA8, 0xA8, 0xA8},
    {0xB0, 0xB0, 0xB0},
    {0xB8, 0xB8, 0xB8},
    {0xC0, 0xC0, 0xC0},
    {0xC8, 0xC8, 0xC8},
    {0xD0, 0xD0, 0xD0},
    {0xD8, 0xD8, 0xD8},
    {0xE0, 0xE0, 0xE0},
    {0xE8, 0xE8, 0xE8},
    {0xF0, 0xF0, 0xF0},
    {0xF8, 0xF8, 0xF8},
    {0xFF, 0xFF, 0xFF},
    {0x00, 0x00, 0x00},
    {0xFF, 0xFF, 0xFF},
    {0x00, 0x00, 0x00},
    {0xFF, 0xFF, 0xFF},
    {0x00, 0x00, 0x00},
    {0xFF, 0xFF, 0xFF},
    {0x00, 0x00, 0x00},
    {0xFF, 0xFF, 0xFF},
    {0x00, 0x00, 0x00},
};

uint32_t    tStart  = 0;
uint32_t    led     = HIGH;
uint32_t    frame_index = 0;

WS2812      ws2812;
uint8_t     rgbPatternBuffer[CBWS2812PATBUF(CDEVICES)];

typedef enum {
    FILLFRAME,
    LOADFRAME,
    WAIT,
    SPIN
} STATE;

STATE state = SPIN;
uint32_t tWaitShift = 0;
#define MSSHIFT 100

#define TRIGGER  16  // PRG button on Fubarino Mini

void setup() 
{                
    pinMode(PIN_LED1, OUTPUT); 
    digitalWrite(PIN_LED1, led);
    
    tStart = millis();

    pinMode(TRIGGER, INPUT_PULLUP);

    ws2812.begin(CDEVICES, rgbPatternBuffer, sizeof(rgbPatternBuffer), false);
    tWaitShift = millis();
    mapPps(29, PPS_OUT_SDO2);

    state = SPIN;

    for (int i = 0; i < CDEVICES; i++) {
        rgGRB[i] = rgGRB_frames[0];
    }
    digitalWrite(PIN_LED1, LOW);
}

void loop() 
{
    if (digitalRead(TRIGGER) == 0 && state == SPIN) {
        state = FILLFRAME;
        frame_index = 0;
    } 

    switch(state)
    {
        case FILLFRAME:
            // Take a frame of data, and fill the whole LED array with that one frame (all LEDs get the same values)
            for (int i = 0; i < CDEVICES; i++) {
                rgGRB[i] = rgGRB_frames[frame_index];
            }
            // Move to the next state - load this array into LEDs
            state = LOADFRAME;
            break;

        case LOADFRAME:
           if (ws2812.updateLEDs(rgGRB)) {
               state = WAIT;
           }
           break;
            
        case WAIT:
            if(millis() - tWaitShift >= MSSHIFT)
            {
                tWaitShift = millis();
                frame_index++;
                if (frame_index < FRAMES) {
                    state = FILLFRAME;
                }
                else {
                    state = SPIN;
                    frame_index = 0;
                }
            }
            break;

        case SPIN:
        default:
            break;

    }
    if(millis() - tStart > 500)
    {
        led ^= HIGH;
        digitalWrite(PIN_LED1, led);
        tStart = millis();
    }
}
