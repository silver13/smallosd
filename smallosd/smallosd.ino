// LICENCE HERE
/*
The MIT License (MIT)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:


THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/
// 24.07.2015


#define MAX7456_SELECT 6
#define MAX7456_VSYNC 2

#include<arduino.h>
#include <SPI.h>

#include"max7456.h"


// voltages to display (max 2) ( YES /NO or 1 / 0)
#define VOLT1ENABLE YES
#define VOLT2ENABLE YES

// inputs for voltahe displays
// choice of BATT1 , BATT2 , RSSI and CURR , VCC and for now FIRTEST
// the BATT1-2 inputs have large dividers and are not as good for 1s voltages
#define VOLT1 CURR
#define VOLT2 VCC

//show status of an led onscreen
// YES / NO
#define LED1ENABLE NO

// input for the led
// choice of BATT1 , BATT2 , RSSI and CURR
#define LED1 BATT1
#define LED1INVERT NO

// positions for various osd parts
#define VOLT1XPOS 2
#define VOLT1YPOS 14

#define VOLT2XPOS 23
#define VOLT2YPOS 14

#define LED1XPOS 2
#define LED1YPOS 8
// ntsc/pal position
#define SYSTEMXPOS 1
#define SYSTEMYPOS 1
// exclamation mark 
// indicates power dropouts
#define FLAGXPOS 23
#define FLAGYPOS 2


// for testing 40 tap FIR filter input
#define FIRINPUT CURR

// voltage correction for the internal voltage reference
// affects all voltages
#define VCCCORR 1.017

// do not change defines below this line
#define PAL 1
#define NTSC 0
#define NONE 2

#define YES 1
#define NO 0

#define VM0_R 0x80
#define OSDBL_R 0xEC
#define OSDBL_W 0x6C

#define BATT1 0   //A2
#define BATT2 1   //A0
#define RSSI 2    // A3
#define CURR 3    // A1
#define VCC 4
#define FIRTEST 7

// osd video system ( PAL /NTSC) at startup if no video input is present
// after input is present the last detected system will be used.
byte osdsystem = PAL;


byte maxrows = 16;
const float  divider[4] = { 16.00 , 16.00 , 1.0 , 1.0 }; // BATT1 , BATT2 , RSSI, CURR
const int  analogpins[4] = { A2 , A0 , A3 , A1 };  
const byte enableserial = 0;

uint8_t analog_reference2 = DEFAULT;

// time the main loop runs at in uS 
const long LOOPTIME = 16000; // loop time in microseconds

float vcc;
byte lastvm0 = B01010101;
byte flag = 0;

byte lastsystem = 99;
unsigned long timesystem;
byte systemclear = 0;

// lpf calculated by tool at
// http://www.schwietering.com/jayduino/filtuino/
//
//Low pass bessel filter order=1 alpha1=0.003 
class filter
{
	public:
		filter()
		{
			v[0]=0.0;
		}
	private:
		float v[2];
	public:
		float step(float x) //class II 
		{
			v[0] = v[1];
			v[1] = (9.337054753656e-3 * x)
				 + (  0.9813258905 * v[0]);
			return 
				 (v[0] + v[1]);
		}
};


filter lpf1 , lpf2, vcclpf;

//filter lpf[4];

//Low pass bessel filter order=4 alpha1=0.01 
class filter2
{
	public:
		filter2()
		{
			for(int i=0; i <= 4; i++)
				v[i]=0.0;
		}
	private:
		float v[5];
	public:
		float step(float x) //class II 
		{
			v[0] = v[1];
			v[1] = v[2];
			v[2] = v[3];
			v[3] = v[4];
			v[4] = (4.425272595240e-6 * x)
				 + ( -0.7428578687 * v[0])
				 + (  3.1954036268 * v[1])
				 + ( -5.1599230905 * v[2])
				 + (  3.7073065280 * v[3]);
			return 
				 (v[0] + v[4])
				+4 * (v[1] + v[3])
				+6 * v[2];
		}
};

filter2 lpf[4];

void setup()
{
  
  if (enableserial) Serial.begin(57600);
  if (enableserial) Serial.print(F("OSD starting...\n"));
  
 pinMode(MAX7456_SELECT,  OUTPUT); // OSD CS
 digitalWrite(MAX7456_SELECT, HIGH);
 if (  LED1 == RSSI ) 
   {
     pinMode(analogpins[RSSI] , INPUT_PULLUP);
   }
 if (  LED1 == CURR ) 
   {
     pinMode(analogpins[CURR] , INPUT_PULLUP);
   }
 

// filtinit();

// pinMode(analogpins[BATT1] , INPUT_PULLUP);
 
 SPI.begin();

 max7456init();
 
 for (int i = 0 ; i < 128; i++)
 {
 if ( i < 4 || i > 80) vcc = getvcc();
 vcclpf.step(vcc);
 }
 
 vcc = vcclpf.step(vcc);
  
 if (enableserial) Serial.println( "VCC: " );
 if (enableserial) Serial.print( vcc);
 if (enableserial) Serial.println( " " );
 
 osd_intro();

 
 delay(1000);
 osd_clear(); 
}

void loop()
{

  byte len;
  char buffer[11]="          ";
   
  unsigned long timeloop = micros();
  
  
  if ( VOLT1ENABLE ) readvolts(VOLT1 , VOLT1XPOS , VOLT1YPOS );
  
  
  if ( VOLT2ENABLE ) readvolts(VOLT2 , VOLT2XPOS , VOLT2YPOS );
  
  
  if ( LED1ENABLE ) readled( LED1 , LED1XPOS , LED1YPOS);
  
 
  vcc = getvcc();
  
  strcpy( buffer , (const char *) "          ");
//  len = floattochar(buffer , vcc); 
//  osd_print( buffer , len + 1 , 2 , 2 );  
  
  if ( vcc < 4.4 )
  {
   // warning for vcc undervoltage 
  }
  vcc = vcclpf.step(vcc);
  
//  strcpy( buffer , (const char *) "          ");
//  len = floattochar(buffer , vcc); 
//  osd_print( buffer , len + 1 , 2 , 3 );  
  
//  analogMux (analogpins[VOLT1]);
//  Serial.print( vcc);
//  Serial.print( " " );
  
  // draw exclamation mark on error
  if ( flag)
  {
  osd_print(  "!" , 1 , FLAGXPOS , FLAGYPOS );
  if (enableserial) Serial.print( " ! " );
  }
  else
  {
  osd_print( " " , 1 , FLAGXPOS , FLAGYPOS ); // 3 spaces
  if (enableserial) Serial.print( "   " );
  }
  
 // PAL / NTSC / no input check 
  checksystem();

// check to see if the max chip is still ok 
// and reinitialize if not  
  check_osd();
  

// strange cs_off here
cs_off();
// needed so we can use the led for flashing(it's on the spi pins)
SPI.end();

led_flasher();
  
// a loop delay smaller then both PAL and NTSC frame time
// so full FPS are used
  if ( micros()- timeloop > LOOPTIME - 1000) 
  {
  if (enableserial) Serial.print( "<<<<<<<<<<<<<<<LOOP TIME" ) ; 
  }  
  while ( micros()- timeloop < LOOPTIME) 
  {   
  }

SPI.begin();

if (enableserial) Serial.println( "" );
}


void osd_clear()
{
  for ( byte y = 0 ; y < maxrows ; y++)
  {
   osd_print( "          " , 10 ,  0 , y ); 
   osd_print( "          " , 10 ,  10 , y ); 
   osd_print( "          " , 10 ,  20 , y ); 
   }
}

byte floattochar(char * buffer, float val)
{// this did not work the simpler way (with %f ) 
  unsigned int volta;
  unsigned int voltb;// decimal digits
  volta = trunc ( val);
  voltb = round ((float)( val - volta)*100 );
  volta = constrain( volta , 0 , 99);
  voltb = constrain( voltb , 0 , 99); 
  byte cx;
  cx = snprintf ( buffer, 10, "%d.%02d",(int) volta, voltb );

 return cx;
}


void osd_print( char *buffer , int len,  byte x , byte y)
{
    if( lastsystem == NTSC)
    {
      //NTSC adjustment 3 lines up if after line 8
      //
      if ( y > 7 ) y = y - 3; 
      
    }
  if ( y > 16 ) y = 16;
  
  unsigned int pos = x + y*30;
  
  cs_on();
   // 16 bit mode, auto increment mode 
  spi_write(DMM, B00000001 );
  // memory address
  spi_write(DMAH, B00000001 & (pos >> 8) );
  spi_write(DMAL, (byte) pos  );
  
  for ( int i = 0; i < len; i++ ) {
  spi_write( DMDI, buffer[i] );
  }
  // off autoincrement mode
  spi_write( DMDI, 0xFF );
  cs_off(); 
}

void max7456init()
{
byte x;
  
cs_on();
spi_write( VM0, B00000010 ); // soft reset
cs_off();

delay(1);

x = spi_read(OSDBL_R); // mine set to 31 (factory) 
cs_on();
spi_write( OSDBL_W , x|B00010000 );
cs_off();

cs_on();
if ( osdsystem == PAL )
  {   
  spi_write( VM0, B01001000 ); // Set pal mode ( ntsc by default) and enable display
  lastvm0 = B01001000;
  }
else 
  {
  spi_write( VM0, B00001000); 
  lastvm0 = B00001000;
  }

 spi_write( VM1, B00001110);
 
 spi_write( 0x0C, B00101101); // osd mux rise/fall ( lowest sharpness)
 
 cs_off();

}



void  checksystem()
{
  if ( !systemclear && millis() - timesystem > 3000)
  {
  // delete the PAL /NTSC /NONE message after a timeout
  osd_print( "    " , 4 , SYSTEMXPOS , SYSTEMYPOS ); 
  systemclear = 1;     
  }
  
  // check detected video system
  byte x = spi_read(STAT);
  if (x & B00000001) 
  { //PAL
  //Serial.println("PAL");
  if ( lastsystem != PAL )
    {
     osd_print( "PAL " , 4 , SYSTEMXPOS , SYSTEMYPOS );
     timesystem = millis(); 
     systemclear = 0;
     lastsystem = PAL;
     if ( osdsystem != PAL ) 
       {
         osd_clear();
         osd_setsystem(PAL);
       }
    }
  } 
  if (x & B00000010) 
  { //NTSC
  //Serial.println("NTSC");
   if ( lastsystem != NTSC )
    {
    osd_print( "NTSC" , 4 , SYSTEMXPOS , SYSTEMYPOS );
    timesystem = millis(); 
    systemclear = 0;
    lastsystem = NTSC;
    if ( osdsystem != NTSC ) 
      {
       osd_clear();
       osd_setsystem(NTSC);
      }
    }
  }  
  
   if ( ! ( x|B00000011 ) )
  {//No signal
   //Serial.println("NONE");
   if ( lastsystem != 2 )
    {
    osd_print( "NONE" , 4 , SYSTEMXPOS , SYSTEMYPOS );
    timesystem = millis(); 
    systemclear = 0;
    lastsystem = 2;
    }
    
  }  
 return;
}


// set the video output system PAL /NTSC
void osd_setsystem( byte sys)
{
byte x = spi_read(VM0_R); 

cs_on();
if ( sys == PAL )
    {
     lastvm0 = x| B01000000; 
     spi_write( VM0, x|( B01000000 ) );
    }
else
    {
     lastvm0 =  x& B10111111;
     spi_write( VM0, x&( B10111111 ) );
    }
cs_off();
 
}

// read vcc by measuring the internal reference against it
float getvcc() 
{
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delayMicroseconds(2000); 
  ADCSRA |= _BV(ADSC); 
  while (bit_is_set(ADCSRA,ADSC));
  byte low  = ADCL;
  byte high = ADCH; 
  return (float) 1125.3* VCCCORR / ((high*256) + low);
}
    
void osd_intro()
{
 osd_print( "smallOSD" , 8 ,  5 , 5 );  // row , col
 osd_print( "v0.8" , 4 ,  5 , 6 );  // row , col   
}

// check for osd "accidental" reset
// the MAX7456 and atmega328 have very different voltage ranges
// MAX resets somewhere between 4.2V and 4.6V
// atmega328 works to below 3.3V (depending on brownout fuses)
void check_osd()
{
  byte x = spi_read(VM0_R);  
 
  if ( x != lastvm0)
  {// the register is not what it's supposed to be

   cs_on();
   spi_write( VM0, B00000010 ); // soft reset
   cs_off();
   delay(1);
   // only set minimum number of registers for functionality
   cs_on();    
   if ( osdsystem == PAL )
    {
    spi_write( VM0, B01001000 ); // Set pal mode ( ntsc by default) and enable display
    lastvm0 = B01001000;
    }
   else
    {
    spi_write( VM0, B00001000); // set NTSC
    lastvm0 = B00001000;
    }
   cs_off();
   flag = 1;
  } 
}


float readvolts( byte input , byte xpos , byte ypos)
{ 
  byte len;
  float volt1 = 0;
  char buffer[11] ="          "; 
  
  if ( input <= 3 )
  {
    for ( byte i = 0; i < 4; i++)
    {
     int x = analogRead(analogpins[input]);
     volt1 = volt1 +  x; 
    }
    volt1 = volt1 / 4;
    
    volt1 = volt1 * vcc / 1023.0 ; 
    volt1 = volt1*divider[input]; 
    volt1 = lpf[input].step(volt1);
  }
  if (input == VCC)
  {
   volt1 = vcc; 
  }
  if (input == FIRTEST)
  {
     for ( byte i = 0; i < 8; i++)
    {
     int x = analogRead(analogpins[FIRINPUT]);
     volt1 = volt1 +  x; 
    }
    volt1 = volt1 / 8;
    
    volt1 = volt1 * vcc / 1023.0 ; 
    volt1 = volt1*divider[FIRINPUT]; 
    
 //   if (enableserial) Serial.print( volt1 );
 //   if (enableserial) Serial.print( "X " );
    //1200 uS for 40 taps
    volt1 = filt ( volt1 );
    
  }
  //analogMux (analogpins[VOLT2]);
   


  len = floattochar(buffer , volt1); 
  osd_print( buffer , len + 1 , xpos , ypos );  
//  strcpy( buffer , (const char *) "          ");
  if (enableserial) Serial.print( " " );
  if (enableserial) Serial.print( buffer );
  if (enableserial) Serial.print( "V " );
 // delay(1);
 return volt1;
 }
 
void readled( byte input , byte xpos , byte ypos )
{
  
  static byte ledon[4] = { 0 , 0 , 0 , 0};
  static unsigned int ledmin[4] = { 1023 , 1023 , 1023 , 1023 };
  static unsigned int ledmax[4] = { 0 , 0 , 0 , 0 };
  
  if ( input > 3 ) input = 3;

  //analog_reference2 = INTERNAL;
  //analogReference(INTERNAL);
  
//  analogMux (analogpins[LED1]);
//  delay(1); // for ADC
//  led = analogRead(analogpins[LED1]);
  float led = 0;

  for ( byte i = 0; i < 4; i++)
  {
   led = led +  analogRead(analogpins[input]); 
  }
  led = led / 4;
  
  if ( led > ledmax[input]) ledmax[input] = led;
  if ( led < ledmin[input]) ledmin[input] = led;
  
  float ledcenter = (ledmax[input] + ledmin[input]) / 2;
  float hyst = (float) (ledmax[input] - ledmin[input]) * 0.25 ; //  hysteresys 
 
  if ( ledmax[input] - ledmin[input] > 20 )
  {
   if ( led > ledcenter + hyst )
     {
      ledon[input] = 1;
     }
   if ( led <  ledcenter - hyst )
     {
      ledon[input] = 0;
     }   
  }
  
  if ( ledon[input] == LED1INVERT)
  {
  osd_print( (const char *) "LED" , 3 , xpos , ypos );
  if (enableserial) Serial.print( " LED " );
  if (enableserial) Serial.print( input );
  if (enableserial) Serial.print( " ON " );
  }
  else
  {
  osd_print( (const char *) "   " , 3 , xpos , ypos ); // 3 spaces
  if (enableserial) Serial.print( " LED " );
  if (enableserial) Serial.print( input );
  if (enableserial) Serial.print( " OFF " );
  } 

}

void led_flasher()
{
 
  static int loopcount = 0;
  int loopsize = 3* (1000/ ( LOOPTIME /1000)  );

  
  if ( loopcount == loopsize * 98 / 100 )
  {// LED on
   digitalWrite(13 , 1); 
  }
  
    if (loopcount > loopsize )
 { //LED off  
   loopcount = 0;
   digitalWrite(13 , 0);
 }
 loopcount++;
  
}

const int m = 39; // even
float filtcoeff[m + 1];
const float fc = 0.005;



void filtinit()
{
for ( int i = 0 ; i <= m ; i++)
  {
   if ( i != m / 2 )
   {
   filtcoeff[i] = sin ( 2 * PI * fc * ( i - m/2) ) / ( i - m/2);
   filtcoeff[i] *= ( 0.42 - 0.5 * cos ( 2 * PI * i / m)  + 0.08 * cos ( 4 * PI * i / m ) ); 
   } 
   else
   filtcoeff[i] = 2 * PI * fc;
   if (enableserial) Serial.print( filtcoeff[i] );
   if (enableserial) Serial.print( " " );
  }
  
  float sum = 0;
  
  for ( int i = 0 ; i <= m ; i++)
   {
    sum = sum + filtcoeff[i];
   }
 if (enableserial) Serial.print( sum );
 if (enableserial) Serial.print( " SUM " );
  for ( int i = 0 ; i <= m ; i++)
   {
   filtcoeff[i] = filtcoeff[i] / sum;
   }
  
}


float coeff1[40] = {
 7.457903316065390700E-6,
 31.13716943841369300E-6,
 89.23170114677552080E-6,
 214.2798924630925510E-6,
 447.5339189571918150E-6,
 847.5164582722063640E-6,
 0.001485713677029906,
 0.002442587441763934,
 0.003801865788582108,
 0.005642690854830525,
 0.008030233939449767,
 0.011005933004448519,
 0.014578676021637900,
 0.018718147209510015,
 0.023351266531667088,
 0.028362267753080005,
 0.033596544654466369,
 0.038868001575518174,
 0.043969311467230827,
 0.048684236459274402,
 0.052801014640400483,
 0.056125763873228984,
 0.058494892154249800,
 0.059785621119290734,
 0.059923907401387942,
 0.058889266016983466,
 0.056716240590297166,
 0.053492507748460441,
 0.049353830242489211,
 0.044476271024526615,
 0.039066237883344353,
 0.033349038300143456,
 0.027556683674744346,
 0.021915691250393589,
 0.016635594313900125,
 0.011898792527096001,
 0.007852262471504431,
 0.004601512865121058,
 0.002207019264664379,
 683.2192156898995560E-6


};

float amplif = 1.00;



float inputdata[m + 1];
float outputdata[1];

const byte useavg = 0;

float filt( float data ) 
{
 for ( int i = 0 ; i < m ; i++)
 {
  inputdata[i] = inputdata[i + 1];
//  if (enableserial) Serial.print( inputdata[i] );
//  if (enableserial) Serial.print( " i " ); 
 }
 
 inputdata[m] = data; 
 
 if ( useavg)
 {
 float sum = 0;
 
  for ( int i = 0 ; i <= m ; i++)
 {
  sum = sum + inputdata[i];
 }
 return ( sum / (m + 1) );
 }
 

outputdata[0] = 0;
 for ( int i = 0 ; i <= m ; i++)
 {
//if (enableserial) Serial.print( inputdata[i] );
// if (enableserial) Serial.print( " i " ); 
  outputdata[0] = outputdata[0] + inputdata[m - i] * coeff1[i];
  
  //if (enableserial) Serial.print( outputdata[i] );
  //if (enableserial) Serial.print( " P " );
 }
 
// if (enableserial) Serial.print( outputdata[0] );
// if (enableserial) Serial.print( " P " );
 return outputdata[0] * amplif;
}















