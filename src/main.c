#include "misc.h"
#include "main.h"

// Annoying secret word that needs to go at the
// end of the binary so the default bootloader doesn't
// try to delete it.
// Stored at 0x08033800 by the linker.
char boot_secret[]  __attribute__((section(".mySection"))) =
{'A','D','4','A','_','R','i','c','h','a','r','d', 0x1 , 0x0 , 0x2};

void delay(int millis) {
    while (millis-- > 0) {
        volatile int x = 5971;
        while (x-- > 0) {
            __asm("nop");
        }
    }
}

typedef union{
  struct LCDPinLayout{
  unsigned int rs : 1;
  unsigned int rw : 1;
  unsigned int d7 : 1;
  unsigned int d6 : 1;
  unsigned int d5 : 1;
  unsigned int d4 : 1;
  unsigned int d3 : 1;
  unsigned int d2 : 1;
  unsigned int d1 : 1;
  unsigned int d0 : 1;
  } field;
  unsigned short whole;
} LCD;

/*
 Convert an LCD struct to the correct
 wiring in the MPK 249.
*/
void lcdpulse(){
  GPIOB->BSRR  = 0x1;
  delay(40);
  GPIOB->BSRR  = 0x10000;
}

void LCDToWires(LCD l){
  // B1 and B2 do the control stuff.
  const unsigned short controlbits = l.field.rs << 2 | l.field.rw << 1;

  GPIOB->BSRR = controlbits | (~controlbits << 16);

  // Port C0-7 does the data stuff.
  const unsigned short databits = l.field.d7 << 7 | l.field.d6 << 6 |
        l.field.d5 << 5 | l.field.d4 << 4 | l.field.d3 << 3 |
        l.field.d2 << 2 | l.field.d1 << 1 | l.field.d0;

  GPIOC->BSRR = databits | (~databits << 16);
  // Send the clock pulse to make the LCD process the instruction.
  lcdpulse();
}

// Initialize LCD display
void initDisplay(void){
  LCD l1 = {{0, 0, 0, 0, 1, 1, 1, 1, 1, 1}};
  delay(50);
  LCDToWires(l1);
  delay(10);
  LCDToWires(l1);
  delay(10);
  LCDToWires(l1);
  delay(10);
  LCDToWires(l1);

  LCD displayOFF = {{0, 0, 0, 0 ,0, 0, 1, 0, 0, 0}};
  LCDToWires(displayOFF);
  LCD clear = {{0, 0, 0, 0, 0, 0, 0, 0, 0, 1}};
  LCDToWires(clear);
  LCD entryMode = {{0, 0, 0, 0, 0, 0, 0, 1, 1, 1}};
  LCDToWires(entryMode);


  LCD blink = {{0, 0, 0, 0, 0, 0, 1, 1, 1, 1}};
  LCDToWires(blink);

}

void copyPatterns(){
  // Configure ports B and C
  GPIOB->CRL = 0x44b84222;
  GPIOC->CRL = 0x33333333;
}

int main(void) {
    // Push it forward
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x6000);

    // Try this
    //GPIOC->BSRR = GPIO_BSRR_BR13;
    // Is this the blink?
    //#GPIOE->BSRR = 0x4000;
    // GPIO structure for port initialization
    GPIO_InitTypeDef GPIO_InitStructure;

    // enable clock on APB2
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,  ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,  ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,  ENABLE);

    // configure port A1 for driving an LED
    // GPIO_Pin_All
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    // output push-pull mode
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   // highest speed
    GPIO_Init(GPIOE, &GPIO_InitStructure) ;             // initialize PORT

    //Turn on LCD backlight
    GPIO_SetBits(GPIOE, 0x1000);

    copyPatterns();


    // LCD logic
    initDisplay();
    while(1){};


    // main loop
    while(1) {
        GPIO_SetBits(GPIOE, 0x4000);    // turn the LED on
        delay(300);

        GPIO_ResetBits(GPIOE, 0x4000);  // turn the LED off
        delay(300);
    }
}
