// lib5110
// Regis LeClerc 2012
//
// collection of graphical primitives for the Nokia5110 LCD screen.
// It's written from scratch from the data sheet to be as fast and compact as possible.
// However, it requires an Atmega328 at least, there's not enough memory on the ATMega8 due to the charmap (768 bytes) and the VRAM (504 bytes)
// This program reads the status of a Wii Nunchuck paddle on the I2C bus  and displays it graphically on the Nokia5110 screen
// A "Wake" button is tied to pin 2
// An array of 10 LEDs make a binary chronometer driven by a 4017BE attached to pins 9, 10 and 11 and updated by interrupt

#define RST   8
#define CE    7
#define DC    6
#define DIN   5
#define CLK   4
#define LIGHT 3
#define BUTTON 2
#define BLINK 13

// functions for TIMER1 driven interrupts
#include <TimerOne.h>


// 4017 LED control pins
#define pinPWR 9
#define pinRST 10
#define pinCLK 11

#include "font_6x8.h"
#include <Wire.h>


#define WIDTH 84
#define HEIGHT 48
#define CHAR_WIDTH 6
#define CHAR_HEIGHT 8
#define NB_COL (WIDTH / CHAR_WIDTH)
#define NB_LN  (HEIGHT / CHAR_HEIGHT)
#define VRAM_SIZE (WIDTH * HEIGHT >> 3)

// minimum time in millis between screen redraw
// 80ms => 12.5 redraws per second, reducing this value doesn't seem to improve screen reactivity
#define REFRESH_RATE 80

#define VID_NORMAL  0x00
#define VID_REVERSE 0xFF

#define PIXEL_SET 0
#define PIXEL_CLR 1
#define PIXEL_XOR 2


#define LIGHT_ON  LOW
#define LIGHT_OFF HIGH
#define LIGHT_TIMEOUT 5000
#define setLight(s) LightMode = s
#define setLightOn  setLight(LIGHT_ON)
#define setLightOff setLight(LIGHT_OFF)

// Constants for the I2C communication with the Nunchuck paddle
#define I2C_NUNCHUCK 0x52
#define NUNCHUCK_BUFFER_SIZE 6
#define NUNCHUCK_JOY_X 0
#define NUNCHUCK_JOY_Y 1


unsigned char VRAM[VRAM_SIZE]; // VideoRAM
unsigned int  VPtr = 0;

unsigned char VideoMode = VID_REVERSE; // Video mode for writeVRAM
unsigned char PixelMode = PIXEL_SET;   // Pixel mode for setPixel

unsigned char LightMode = LIGHT_ON;

static uint8_t NunchuckBuffer[NUNCHUCK_BUFFER_SIZE];   // nunchuck data,

/*
 * ======================================================== LCD Screen Hardware control
 */
void LcdWriteCmd(byte cmd)
{
  digitalWrite(DC, LOW); //DC pin is low for commands
  digitalWrite(CE, LOW);
  shiftOut(DIN, CLK, MSBFIRST, cmd); //transmit serial data
  digitalWrite(CE, HIGH);
}


void LcdWriteData(byte dat)
{
  digitalWrite(CE, LOW);
  digitalWrite(DC, HIGH); //DC pin is high for data
  shiftOut(DIN, CLK, MSBFIRST, dat); //transmit serial data
  digitalWrite(CE, HIGH);
}

/*
 * ================================================================== Video RAM control
 */
void writeVRAM(unsigned char c) {
  VRAM[VPtr] = c ^ VideoMode;
  if(++VPtr >= VRAM_SIZE) VPtr = 0;
}

void updateScreen() {
  for(int i = 0 ; i < VRAM_SIZE ; i++) {
    LcdWriteData(VRAM[i]);
  }
}

/*
 * ================================================================== Character functions
 */
 
void writeChar(char c) {
  c -= 0x20;
  int i = 0;
  do
    writeVRAM((charset[(c * CHAR_WIDTH) + i]));
  while(++i < CHAR_WIDTH);
}

void writeString(char*s) {
  while(*s) writeChar(*s++);
}


void setXY(unsigned char x,unsigned char y) {
  VPtr = y * WIDTH + x;
}

const char HexStr[] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

void writeCharHex(char v) {
  writeChar(HexStr[(v >> 4) & 0x0F]); // shr doesn't clean upper bits
  writeChar(HexStr[ v       & 0x0F]);
}

void writeIntHex(int v) {
  writeCharHex(v >> 8);
  writeCharHex(v & 0x0f);
}

void writeLongHex(long v) {
  writeIntHex(v >> 16);
  writeIntHex(v & 0xffff);
}

/*
 * ================================================================== Scrolling functions
 */
void scrollDown() {
  for(int i = 0; i< 84 ; i++) {
    for(int j = 5; j > 0 ; j--) {
      VRAM[j*WIDTH+i] = VRAM[(j-1)*WIDTH+i];
    }
    VRAM[i] = 0;
  }
}

void scrollUp() {
    for(int i = 0; i< WIDTH ; i++) {
    for(int j = 0; j < NB_LN ; j++) {
      VRAM[(j*84)+i] = VRAM[(j+1)*84+i];
    }
    VRAM[i+(5*WIDTH)] = 0;
  }
}

/*
 * ================================================================== Graphical functions
 * x is in [0-83], y is in [0-47]
 */
 
void setPixel(unsigned x,unsigned y) {
  unsigned char t = 1 << (y & 0x07);
  int PixelAddr = ((y >> 3) * WIDTH) + x;
  switch(PixelMode) {
    case PIXEL_CLR : VRAM[PixelAddr] &= ~t; break;
    case PIXEL_XOR : VRAM[PixelAddr] ^=  t; break;
           default : VRAM[PixelAddr] |=  t;
  }
}

void drawLine(int x0, int y0, int x1, int y1) {
 
  int
    dx = abs(x1 - x0), 
    dy = abs(y1 - y0),
    sx = x0 < x1 ? 1 : -1,
    sy = y0 < y1 ? 1 : -1,
    err = ((dx > dy) ? dx : -dy)/2,
    e2;
 
  for(;;) {
    setPixel(x0 , y0);
    if (x0 == x1 && y0 == y1) break;
    e2 = err;
    if (e2 > -dx) { err -= dy; x0 += sx; }
    if (e2 <  dy) { err += dx; y0 += sy; }
  }
}

static int getNunchuck() {
   int cnt = 0;
   Wire.requestFrom (I2C_NUNCHUCK,NUNCHUCK_BUFFER_SIZE); // request data
   while (Wire.available ()) NunchuckBuffer[cnt++] = (Wire.read());
   Wire.beginTransmission(I2C_NUNCHUCK);// transmit to device 0x52
   Wire.write(0x00);// sends one byte
   Wire.endTransmission();// stop transmitting
   return (cnt >= (NUNCHUCK_BUFFER_SIZE - 1));
}

char
    old_jx = 0,old_jy = 0,
    old_ax = 0,old_ay = 0,old_az = 0;

void drawBar(char x,char val) {
  int p = x;
  for(int i = 0 ; i < 3 ; i++) {
    drawLine(p,33 - val,p,33);
    p++;
  }
}

void printNunchuck() {
  VideoMode = VID_NORMAL;
  unsigned int tv;
  unsigned char jx = 0, jy = 0, ax,ay,az;
  setXY(72 + old_jx, 1 + old_jy); VideoMode=VID_NORMAL; writeChar(' ');
  if(NunchuckBuffer[NUNCHUCK_JOY_X] > 0x80) jx = CHAR_WIDTH; else if(NunchuckBuffer[NUNCHUCK_JOY_X] < 0x80) jx = -CHAR_WIDTH;
  if(NunchuckBuffer[NUNCHUCK_JOY_Y] > 0x80) jy = -1; else if(NunchuckBuffer[NUNCHUCK_JOY_Y] < 0x80) jy = +1;
  setXY(72 + jx, 1 + jy); VideoMode=VID_REVERSE; writeChar(' ');
  
  drawLine(66,0,66,23);
  drawLine(83,0,83,23);
  drawLine(66,0,83,0);
  drawLine(66,23,83,23);
  old_jx = jx; old_jy = jy;
  
  VideoMode = VID_NORMAL;
  for(int i = 0 ; i < 4 ; i++) {
    setXY(48,i);
    writeString("  ");
  }

  ax = NunchuckBuffer[2] >> 3;
  ay = NunchuckBuffer[3] >> 3;
  az = NunchuckBuffer[4] >> 3;

  PixelMode = PIXEL_SET;
   drawBar(48,ax);
   drawBar(52,ay);
   drawBar(56,az);
   
  setXY(18,3); writeCharHex(NunchuckBuffer[NUNCHUCK_JOY_X]); // XJoy
  setXY(18,4); writeCharHex(NunchuckBuffer[NUNCHUCK_JOY_Y]); // YJoy  

  tv = NunchuckBuffer[2] << 2 | ((NunchuckBuffer[5] && 0x0c) >> 2);
  //setXY(18,0); writeIntHex(tv);
  
  tv = NunchuckBuffer[3] << 2 | ((NunchuckBuffer[5] && 0x30) >> 4);
  setXY(18,1); writeIntHex(tv); // YAcc

  tv = NunchuckBuffer[4] << 2 | ((NunchuckBuffer[5] && 0xc0) >> 6);
  setXY(18,2); writeIntHex(tv); // ZAcc
  
  VideoMode = (NunchuckBuffer[5] & 2)?VID_NORMAL:VID_REVERSE;
  setXY(66,3); writeChar('C');
  VideoMode = (NunchuckBuffer[5] & 1)?VID_NORMAL:VID_REVERSE;
  setXY(78,3); writeChar('Z');
}

unsigned long BitField4017 = 0;
bool step_4017 = true;
unsigned char loop_4017 = 0;

void stepBitField() {
  if(step_4017) {
   if(loop_4017 == 0) {
    digitalWrite(pinRST,HIGH);
    digitalWrite(pinRST,LOW);
   }
   digitalWrite(pinPWR,BitField4017 & (1 << loop_4017)?HIGH:LOW);
   if(++loop_4017 > 9) loop_4017 = 0;
  } else {
   digitalWrite(pinPWR,LOW);
   digitalWrite(pinCLK,HIGH);   
   digitalWrite(pinCLK,LOW); 
  }
  step_4017 = !step_4017;
}

// 4017 display
// must be split in two and called by timer interrupt to avoid visible blink
void drawBitField() {
  unsigned char b;
  unsigned long v = BitField4017;
  
  digitalWrite(pinRST,HIGH);
  digitalWrite(pinRST,LOW);
  
  for(b=0 ; b<10 ; b++) {
   digitalWrite(pinPWR,v & (1<<b)?HIGH:LOW);
   //delay(1);
   digitalWrite(pinPWR,LOW);
   digitalWrite(pinCLK,HIGH);   
   digitalWrite(pinCLK,LOW); 
  }
}

/*
 * ================================================================== setup()
 */
void setup() {
  
// 4017 LED setup
  pinMode(pinPWR,OUTPUT);
  pinMode(pinRST,OUTPUT);
  pinMode(pinCLK,OUTPUT);
  digitalWrite(pinPWR,HIGH);

// LED on pin 13
  pinMode(BLINK,OUTPUT);
  
// Button
  pinMode(BUTTON,INPUT);
  
// LCD 5110 setuo
  pinMode(RST  ,OUTPUT);
  pinMode(CE   ,OUTPUT);
  pinMode(DC   ,OUTPUT);
  pinMode(DIN  ,OUTPUT);
  pinMode(CLK  ,OUTPUT);
  pinMode(LIGHT,OUTPUT);

// a short LOW pulse on RST
  digitalWrite(RST, LOW);
  delay(20);
  digitalWrite(RST, HIGH);
  delay(20);

  setLight(LIGHT_ON);
  
  LcdWriteCmd(0x21); // LCD extended command
  LcdWriteCmd(0xB1); // set LCD Vop (contrast)
  LcdWriteCmd(0x04); // set temp coefficent
  LcdWriteCmd(0x14); // LCD bias mode 1:40
  LcdWriteCmd(0x20); // LCD basic commands
  LcdWriteCmd(0x0C); // LCD normal video
  for(int i = 0 ; i < VRAM_SIZE ; i++) VRAM[i] = 0;  
  PixelMode = PIXEL_SET;
  VideoMode = VID_NORMAL;
  setXY(0,0);
  writeString("AX:0000       AY:0000       AZ:0000       JX:00      C ZJY:00");
  
  VideoMode = VID_REVERSE;
  setXY(0,5);
  writeString(">             ");

  //Attach interrupt for 4017 update
  Timer1.initialize(1000);
  Timer1.attachInterrupt(stepBitField);
  
  // I2C initialisation
  Wire.begin();
  Wire.beginTransmission(I2C_NUNCHUCK);
  Wire.write(0x40); // white Nunchuck (for black Nunchuck, use 0x55
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.endTransmission();
}

/*
 * ================================================================== loop()
 */
static unsigned long
  last_t       = 0, // to manage time looping around MAX_LONG_INT milliseconds (1.36 years...)
  last_refresh = 0,
  last_blink   = 0,
  last_light   = 0,
  last_scroll  = 0,
  last_nc      = 0;

#define BLINK_DELAY 1000
char blink_mode = HIGH;

void loop() {
  unsigned long t = millis();
  if(last_t > t) {
    // happens only when millis() loops
    last_refresh -= last_t;
    last_blink   -= last_t;
    last_light   -= last_t;
    last_scroll  -= last_t;
    last_nc      -= last_t;
  }
  
// follow the clock
  if(t/BLINK_DELAY >= last_blink) {
    digitalWrite(BLINK,blink_mode);
    blink_mode = !blink_mode;
    if(++BitField4017 == 1024) BitField4017 = 0;
    last_blink++;
  }

// using delay since last update (not accurate timewise)    
  if(t - last_scroll > 40) {
    VideoMode = VID_REVERSE;
    setXY(6,5);
    writeLongHex(t);
    last_scroll = t;
    if(getNunchuck()) printNunchuck();
  }

// Updating the screen comes last
  if(t - last_refresh > REFRESH_RATE) {
    char BtnStatus = 'L';
    
    if(digitalRead(BUTTON) == HIGH) {
      setLightOn;
      last_light = t;
      BtnStatus = 'H';
    }
    
    VideoMode = VID_REVERSE; setXY(78,5);
    writeChar(BtnStatus);
    
    // backlight timeout
    if((LightMode == LIGHT_ON)) {
        if(t - last_light > LIGHT_TIMEOUT) LightMode = LIGHT_OFF;
        digitalWrite(LIGHT,LightMode);
    }
  
    updateScreen();
    
    last_refresh = t;
  }
  last_t = t;
}
