

#define FSMAGIC 0x0250FF0F //quieter than ? 0x0405FF08;
//try ff50ff08
//0xFF056408 from code example
 //timekeep-1 startwait5 standbywait100 rstbwait8
 //2 8 FF 8
// 2 16 FF 08
#define CLKDIVMAGIC ((8)<<9) //7 and 8 2<<7
#define BCKMAGIC 6<<6 //6 7
#define CLKMAGIC 6 //4
#define ADC1_PATT (0x6C<<24)
#define ADC2_PATT (0x0E<<24)


#include "sketch.h"

#define delaysiz (1<<16)
uint8_t *delaybuffa;
uint8_t * delaybuffb;

uint8_t * delaybuffd;
uint8_t * delaybuffc;

uint8_t *delptr=delaybuffa; 
uint8_t adc_histogram[256];


static int delayptr;
static int delayskp;
static int lastskp;
uint8_t dell;

uint32_t adc_read;
int akkuval;
bool butt;
bool buttest;
int buttflip;
  int readr;



void IRAM_ATTR pigHandler() {
 //REG(GPIO_STATUS_W1TC_REG)[0]=0xFFFFFFFF;
 //REG(GPIO_STATUS1_W1TC_REG)[0]=0xFFFFFFFF;
 
 REG(SPI3_W0_REG)[0]=REG(RNG_REG)[0]&0xFFFF;
 REG(SPI3_CMD_REG)[0]=BIT(18);
 
 adc_read=3;
 //REG(I2S_CONF_REG)[0] &= ~(BIT(5)); 
 readr= (0xFFF&(adc_read>>16));
 dell = delptr[delayptr&0xFFFF];
 if (!butt)
 delptr[delayptr&0xFFFF]=(uint8_t)(255-(readr>>4));//(adc_read>>4); 
 REG(ESP32_RTCIO_PAD_DAC2)[0] =  BIT(10) | BIT(17) | BIT(18) |  ((dell&0xFF)<<19);
 //  REG(ESP32_RTCIO_PAD_DAC2)[0] =  BIT(10) | BIT(17) | BIT(18) |  (0x80<<19);
 //  REG(ESP32_RTCIO_PAD_DAC2)[0] =  BIT(10) | BIT(17) | BIT(18) |  ((uint8_t)akkuval<<19); 
 //REG(ESP32_RTCIO_PAD_DAC1)[0] = BIT(10) | BIT(17) | BIT(18) |  ((REG(RNG_REG)[0]&0xFF)<<19);
 adc_histogram[(readr>>4)&0xFF]++;

  if (GPIO_IN1_REG[0]&0x8) delayptr++;
  else delayptr--; 
  delayptr=delayptr&0x1FFFF;

  if (GPIO_IN1_REG[0]&0x2)  {
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

 delaybuffd=(uint8_t*)malloc(delaysiz>>2);
 
 delaybuffc=(uint8_t*)malloc(delaysiz>>2);

// delaybuffc=(uint8_t*)malloc(delaysiz);
 printf("yodel %08x,%08x,%08x,%08x\n",delaybuffa,delaybuffb,delaybuffc,delaybuffd);
 //esp_task_wdt_init(30, false);
  //to be fixed
  REG(ESP32_SENS_SAR_DAC_CTRL1)[0] = 0x0; 
  REG(ESP32_SENS_SAR_DAC_CTRL2)[0] = 0x0; 
  //initiate DIG
      
  //function 2 on the 12 block
  REG(IO_MUX_GPIO12ISH_REG)[0]=BIT(13); //sdi2 q MISO
  REG(IO_MUX_GPIO12ISH_REG)[1]=BIT(13); //d MOSI
  REG(IO_MUX_GPIO12ISH_REG)[2]=BIT(13); //clk
  REG(IO_MUX_GPIO12ISH_REG)[3]=BIT(13); //cs0

    CHANGOR(DPORT_PERIP_CLK_EN_REG,BIT(16))
  CHANGNOR(DPORT_PERIP_RST_EN_REG,BIT(16))

  REG(IO_MUX_GPIO5_REG)[0]=BIT(12); //sdi3 cs0
  REG(IO_MUX_GPIO18_REG)[0]=BIT(12); //sdi3 clk
  REG(IO_MUX_GPIO19_REG)[0]=BIT(12)|BIT(9); //sdi3 q MISO
  REG(IO_MUX_GPIO23_REG)[0]=BIT(12); //sdi3 d MOSI
  REG(SPI3_MOSI_DLEN_REG)[0]=15;
  REG(SPI3_MISO_DLEN_REG)[0]=15;
    REG(SPI3_USER_REG)[0]=BIT(24)|BIT(0)|BIT(27);
    //USR_MOSI, MISO_HIGHPART, and DOUTDIN
   //REG(SPI3_CLOCK_REG)[0]=(0<<18)|(3<<12)|(1<<6)|3;



  //straight out
  //LED//  GPIO_FUNC_OUT_SEL_CFG_REG[5]=256;
  GPIO_FUNC_OUT_SEL_CFG_REG[12]=256;
  GPIO_FUNC_OUT_SEL_CFG_REG[13]=256;
  GPIO_FUNC_OUT_SEL_CFG_REG[14]=256;
  GPIO_FUNC_OUT_SEL_CFG_REG[15]=256;
  GPIO_FUNC_OUT_SEL_CFG_REG[16]=256;
  GPIO_FUNC_OUT_SEL_CFG_REG[17]=256;
  //GPIO_FUNC_OUT_SEL_CFG_REG[18]=256;
  //GPIO_FUNC_OUT_SEL_CFG_REG[19]=256;
  GPIO_FUNC_OUT_SEL_CFG_REG[21]=256;
  GPIO_FUNC_OUT_SEL_CFG_REG[22]=256;
  //GPIO_FUNC_OUT_SEL_CFG_REG[23]=256;
  GPIO_FUNC_OUT_SEL_CFG_REG[27]=256;
  REG(GPIO_ENABLE_REG)[0]=BIT(12)|BIT(13)
  |BIT(14)|BIT(15)|BIT(16)|BIT(17)|BIT(18)
  |BIT(19)|BIT(21)|BIT(22)|BIT(23)|BIT(27);
  //36 and 39  
  REG(GPIO_ENABLE_REG)[3]=0;
    REG(IO_MUX_GPIO32_REG)[0]=BIT(9)|BIT(8); //input enable
    REG(IO_MUX_GPIO32_REG)[1]=BIT(9)|BIT(8); //input enable

 // REG(IO_MUX_GPIO36_REG)[0]=BIT(9)|BIT(8); //input enable
 // REG(IO_MUX_GPIO36_REG)[3]=BIT(9)|BIT(8); //input enable
  REG(IO_MUX_GPIO34_REG)[1]=BIT(9)|BIT(8); //input enable
  REG(IO_MUX_GPIO34_REG)[0]=0;

 REG(IO_MUX_GPIO2_REG)[0]=BIT(9)|BIT(8); //input enable
 attachInterrupt(2,pigHandler,FALLING);
}

void loop() {
  int ryo;
  return;
  printf("yo");
  for (;;) {
    ryo++;
    if (ryo>1000000) ryo = 0;  
    if (ryo==0) { 
     //printf("\n-----%d-------%d\n",(int)REG(SPI2_USER_REG)[0],REG(SPI2_MOSI_DLEN_REG)[0]); 
     printf("\n-----%x-------%d\n",REG(SPI3_W8_REG)[0],REG(RNG_REG)[0]&0xFFFF); 
     
    }
 } 
}


  
