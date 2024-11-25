#define FSMAGIC 0x0250FF0F //quieter than ? 0x0405FF08;
//try ff50ff08
//0xFF056408 from code example
 //timekeep-1 startwait5 standbywait100 rstbwait8
// 2 16 FF 08
#define CLKDIVMAGIC ((2)<<9) //7 and 8
#define BCKMAGIC 4<<6 //6 7
#define CLKMAGIC 4

#include "sketch.h"

#define delaysiz (1<<16)
uint8_t *delaybuffa;
uint8_t * delaybuffb;
uint8_t *delptr=delaybuffa; 
uint8_t adc_histogram[256];

uint8_t add_histogram[256];

static int delayptr;
static int delayskp;
static int lastskp;
uint8_t dell;

uint32_t adc_read;
uint32_t akkuval;
bool butt;
bool buttest;
int buttflip;

void IRAM_ATTR pigHandler() {
 REG(GPIO_STATUS_W1TC_REG)[0]=0xFFFFFFFF;
 REG(GPIO_STATUS1_W1TC_REG)[0]=0xFFFFFFFF;
 //REG(I2S_CONF_REG)[0] &= ~(BIT(5)); 
 volatile uint32_t *rr = REG(I2S_FIFO_RD_REG);
 adc_read = rr[0];//>>16;
// akkuval *= 3;
 int divali;
 uint8_t rat;
 divali=(int)akkuval-((uint8_t)(adc_read>>4));
 if ((divali<4)&&(divali>-4)) {
  rat=(adc_read>>4);
 } else rat=akkuval;
   akkuval += (uint8_t)(adc_read>>4);
  akkuval = akkuval >> 1;
  akkuval = (uint8_t)(adc_read>>20);
  
   akkuval = (uint8_t)(adc_read>>4);
 dell = delptr[delayptr&0xFFFF];
 delptr[delayptr&0xFFFF]=(uint8_t)(akkuval);//(adc_read>>4); 
 REG(ESP32_RTCIO_PAD_DAC2)[0] =  BIT(10) | BIT(17) | BIT(18) |  ((dell&0xFF)<<19);
 //  REG(ESP32_RTCIO_PAD_DAC2)[0] =  BIT(10) | BIT(17) | BIT(18) |  (0x80<<19);
 //  REG(ESP32_RTCIO_PAD_DAC2)[0] =  BIT(10) | BIT(17) | BIT(18) |  ((uint8_t)akkuval<<19);
 
 
 
 //REG(ESP32_RTCIO_PAD_DAC1)[0] = BIT(10) | BIT(17) | BIT(18) |  ((REG(RNG_REG)[0]&0xFF)<<19);
 REG(I2S_INT_CLR_REG)[0]=0xFFFFFFFF;
 REG(I2S_CONF_REG)[0] |= (BIT(5)); //start rx
 //akkuval *= 255;

 //uint8_t adiff=((adc_read>>20)&0xFF)-((adc_read>>4)&0xFF);
 //adc_histogram[(adiff-96)&0x3F]++;
 adc_histogram[(((adc_read>>20)&0xFF)-96)&0x3F]++;
 add_histogram[(((adc_read>>4)&0xFF)-96)&0x3F]++;

  if (GPIO_IN1_REG[0]&0x8) delayptr++;
  else delayptr--; 
  delayptr=delayptr&0x1FFFF;

  if (GPIO_IN1_REG[0]&0x10)  {
   if (lastskp==0) delayskp = delayptr;
   lastskp = 1;
  } else {
   if (lastskp) delayptr=delayskp;
   lastskp = 0;
  } 
  if (delayptr&0x10000) delptr = delaybuffa;
  else delptr = delaybuffb;
  int buttnow = (GPIO_IN1_REG[0]&0x1);
  if (buttflip^buttnow)
   if (buttnow) butt = !butt;
  buttflip =  buttnow;//(GPIO_IN1_REG[0]&0x1);
  
  if (butt) GPIO_OUT_REG[0]=(1<<5)|(uint32_t)(delayptr<<12);
  else GPIO_OUT_REG[0]=(uint32_t)(delayptr<<12);
  //if (~GPIO_IN1_REG[0]&0x8)  
}


void setup() { 
 delaybuffa=(uint8_t*)malloc(delaysiz);
 delptr=delaybuffa;
 delayptr=0;
 delaybuffb=(uint8_t*)malloc(delaysiz);
 printf("yodel %08x,%08x\n",delaybuffa,delaybuffb);
 sset();
 attachInterrupt(2,pigHandler,FALLING);
}


 //start rx
    // REG(I2S_CONF_REG)[0] |= (BIT(1)); //start r
    //  REG(I2S_CONF_REG)[0] &= ~(BIT(1)); //start rx
  //REG(I2S_CONF_REG)[0] |= (BIT(5)); //start rx
   //REG(I2S_CONF_REG)[0] |= (BIT(1)); //start rx
   //REG(I2S_CONF_REG)[0] &= ~(BIT(1)); //start rx


void loop() {
  int ryo;
  printf("LOOOOOOOOOOOP\n");
  for (;;) {
    ryo++;
    if (ryo>1000000) ryo = 0;  
    if (ryo==0) {
     for (int i=0;i<64;i++) {
      printf("%03d ",adc_histogram[i]);
      adc_histogram[i]=0;
     }
     printf("\n-----------------------------------\n");
     for (int i=0;i<64;i++) {
      printf("%03d ",add_histogram[i]);
      add_histogram[i]=0;
     }
     
     printf("\n-----%d-------%d\n",akkuval,(uint8_t)(adc_read>>20));
     //printf("\nSNENSAR;MEAS2%x\n",REG(SENS_SAR_MEAS_START2_REG)[0]);
     
     printf("\nADCREAD%x\n",adc_read);
      //printf("\n");
    }
 } 
}


  
