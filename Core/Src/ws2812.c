/*
 * ws2812.c
 *
 *  Created on: 2019. 6. 4.
 *      Author: HanCheol Cho
 */

#include "ws2812.h"
#include "main.h"

#define BIT_PERIOD      (212)
#define BIT_HIGH        (135)
#define BIT_LOW         (67)

bool is_init = false;


typedef struct
{
  uint16_t led_cnt;
} ws2812_t;

static uint8_t led_buf[50 + 24*64];


ws2812_t ws2812;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim17;



bool ws2812Init(void)
{
  memset(led_buf, 0, sizeof(led_buf));
  is_init = true;

  return true;
}

void ws2812Begin(uint32_t led_cnt)
{
  ws2812.led_cnt = led_cnt;


  HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t *)led_buf, (50 + 24 *  ws2812.led_cnt) * 1);
  HAL_TIM_PWM_Start_DMA(&htim17, TIM_CHANNEL_1, (uint32_t *)led_buf, (50 + 24 *  ws2812.led_cnt) * 1);
}

void ws2812SetColor(uint32_t index, uint8_t red, uint8_t green, uint8_t blue)
{
  uint8_t r_bit[8];
  uint8_t g_bit[8];
  uint8_t b_bit[8];

  uint32_t offset;


  for (int i=0; i<8; i++)
  {
    if (red & (1<<7))
    {
      r_bit[i] = BIT_HIGH;
    }
    else
    {
      r_bit[i] = BIT_LOW;
    }
    red <<= 1;

    if (green & (1<<7))
    {
      g_bit[i] = BIT_HIGH;
    }
    else
    {
      g_bit[i] = BIT_LOW;
    }
    green <<= 1;

    if (blue & (1<<7))
    {
      b_bit[i] = BIT_HIGH;
    }
    else
    {
      b_bit[i] = BIT_LOW;
    }
    blue <<= 1;
  }

  offset = 50;

  memcpy(&led_buf[offset + index*24 + 8*0], g_bit, 8*1);
  memcpy(&led_buf[offset + index*24 + 8*1], r_bit, 8*1);
  memcpy(&led_buf[offset + index*24 + 8*2], b_bit, 8*1);
}


void setBrightness(uint8_t b) {
  // Stored brightness value is different than what's passed.
  // This simplifies the actual scaling math later, allowing a fast
  // 8x8-bit multiply and taking the MSB. 'brightness' is a uint8_t,
  // adding 1 here may (intentionally) roll over...so 0 = max brightness
  // (color values are interpreted literally; no scaling), 1 = min
  // brightness (off), 255 = just below max brightness.
  uint8_t newBrightness = b + 1;
  if(newBrightness != brightness) { // Compare against prior value
    // Brightness has changed -- re-scale existing data in RAM,
    // This process is potentially "lossy," especially when increasing
    // brightness. The tight timing in the WS2811/WS2812 code means there
    // aren't enough free cycles to perform this scaling on the fly as data
    // is issued. So we make a pass through the existing color data in RAM
    // and scale it (subsequent graphics commands also work at this
    // brightness level). If there's a significant step up in brightness,
    // the limited number of steps (quantization) in the old data will be
    // quite visible in the re-scaled version. For a non-destructive
    // change, you'll need to re-render the full strip data. C'est la vie.
    uint8_t  c,
            *ptr           = pixels,
             oldBrightness = brightness - 1; // De-wrap old brightness value
    uint16_t scale;
    if(oldBrightness == 0) scale = 0; // Avoid /0
    else if(b == 255) scale = 65535 / oldBrightness;
    else scale = (((uint16_t)newBrightness << 8) - 1) / oldBrightness;
    for(uint16_t i=0; i<numBytes; i++) {
      c      = *ptr;
      *ptr++ = (c * scale) >> 8;
    }
    brightness = newBrightness;
  }
}


/*!
  @brief   Set a pixel's color using a 32-bit 'packed' RGB or RGBW value.
  @param   n  Pixel index, starting from 0.
  @param   c  32-bit color value. Most significant byte is white (for RGBW
              pixels) or ignored (for RGB pixels), next is red, then green,
              and least significant byte is blue.
*/
void setPixelColor(uint16_t n, uint32_t c) {
  if(n < ws2812.led_cnt) {
    uint8_t *p,
      r = (uint8_t)(c >> 16),
      g = (uint8_t)(c >>  8),
      b = (uint8_t)c;
    if(brightness) { // See notes in setBrightness()
      r = (r * brightness) >> 8;
      g = (g * brightness) >> 8;
      b = (b * brightness) >> 8;
    }
    if(wOffset == rOffset) {
      p = &pixels[n * 3];
    } else {
      p = &pixels[n * 4];
      uint8_t w = (uint8_t)(c >> 24);
      p[wOffset] = brightness ? ((w * brightness) >> 8) : w;
    }
    p[rOffset] = r;
    p[gOffset] = g;
    p[bOffset] = b;
    ws2812SetColor(n, r, g , b);
  }

}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(int8_t WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

uint32_t Color(uint8_t r, uint8_t g, uint8_t b) {
  return ((uint32_t)r << 16) | ((uint32_t)g <<  8) | b;
}

// A 32-bit variant of gamma8() that applies the same function
// to all components of a packed RGB or WRGB value.
uint32_t gamma32(uint32_t x) {
  uint8_t *y = (uint8_t *)&x;
  for(uint8_t i=0; i<4; i++) y[i] = gamma8(y[i]);
  return x; // Packed 32-bit return
}

uint32_t ColorHSV(uint16_t hue, uint8_t sat, uint8_t val) {

  uint8_t r, g, b;
  hue = (hue * 1530L + 32768) / 65536;

  // Convert hue to R,G,B (nested ifs faster than divide+mod+switch):
  if(hue < 510) {         // Red to Green-1
    b = 0;
    if(hue < 255) {       //   Red to Yellow-1
      r = 255;
      g = hue;            //     g = 0 to 254
    } else {              //   Yellow to Green-1
      r = 510 - hue;      //     r = 255 to 1
      g = 255;
    }
  } else if(hue < 1020) { // Green to Blue-1
    r = 0;
    if(hue <  765) {      //   Green to Cyan-1
      g = 255;
      b = hue - 510;      //     b = 0 to 254
    } else {              //   Cyan to Blue-1
      g = 1020 - hue;     //     g = 255 to 1
      b = 255;
    }
  } else if(hue < 1530) { // Blue to Red-1
    g = 0;
    if(hue < 1275) {      //   Blue to Magenta-1
      r = hue - 1020;     //     r = 0 to 254
      b = 255;
    } else {              //   Magenta to Red-1
      r = 255;
      b = 1530 - hue;     //     b = 255 to 1
    }
  } else {                // Last 0.5 Red (quicker than % operator)
    r = 255;
    g = b = 0;
  }

  // Apply saturation and value to R,G,B, pack into 32-bit result:
  uint32_t v1 =   1 + val; // 1 to 256; allows >>8 instead of /255
  uint16_t s1 =   1 + sat; // 1 to 256; same reason
  uint8_t  s2 = 255 - sat; // 255 to 0
  return ((((((r * s1) >> 8) + s2) * v1) & 0xff00) << 8) |
          (((((g * s1) >> 8) + s2) * v1) & 0xff00)       |
         ( ((((b * s1) >> 8) + s2) * v1)           >> 8);
}

uint8_t gamma8(uint8_t x) {
  return _NeoPixelGammaTable[x]; // 0-255 in, 0-255 out
}
