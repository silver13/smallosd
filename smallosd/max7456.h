

#define DMM 0x04 
#define DMAH 0x05 
#define DMAL 0x06 
#define DMDI 0x07 
//#define CMM 0x08
//#define CMAH 0x09
//#define CMAL 0x0A
//#define CMDI 0x0B
#define VM0 0x00 
#define VM1 0x01 
#define RB0 0x10 
#define STAT 0xA2 //Status register read address
#define CMDO 0xC0 
//#define DMDO 0XB0 


void cs_on()
{
  digitalWrite(MAX7456_SELECT,LOW);
}

void cs_off()
{
 digitalWrite(MAX7456_SELECT,HIGH); 
}

void spi_write( byte reg, byte val )
{
  SPI.transfer( reg );
  SPI.transfer( val );
}




byte spi_read( byte reg )
{ 
  cs_on();
  SPI.transfer( reg );
  return SPI.transfer( 255 );
  cs_off();
}
