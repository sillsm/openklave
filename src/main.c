#include "misc.h"
#include "main.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>


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

#define BITFIELD10(x0,x1,x2,x3,x4,x5,x6,x7,x8,x9)\
((bitfield){10,0b##x0##x1##x2##x3##x4##x5##x6##x7##x8##x9})

typedef struct BetterBitfield{
    int size;
    uint32_t field;
} bitfield;

// Bit bang an enable pulse to the LCD.
void lcdpulse(){
  GPIOB->BSRR  = 0x1;
  delay(2);
  GPIOB->BSRR  = 0x10000;
}

/*
 Convert a bitfield to the correct
 wiring in the MPK 249.
*/
void LCDToWires2(bitfield b){
  // TODO -- static assert the bitfield has size 10

  // B1 and B2 do the control stuff.
  uint32_t controlbits = (b.field >> 8) << 1;

  // Twiddle only the first 8 bits. So send ~ of the 8
  // to the upper register of BSRR.
  uint32_t controlmask = 0b00000000000000000000000000000011;
  GPIOB->BSRR = controlbits | (~controlbits & controlmask) << 16;

  // Port C0-7 does the data stuff.
  uint32_t databits = b.field&0b0011111111;

  uint32_t datamask = 0b00000000000000000000000011111111;
//  x | ((~x & datamask) << 16)
  GPIOC->BSRR = databits | (~databits & datamask) << 16;
  // Send the clock pulse to make the LCD process the instruction.
  lcdpulse();
}

// Initialize LCD display
void initDisplay(void){
  bitfield init       = BITFIELD10(0,0,0,0,1,1,1,1,1,1);
  bitfield displayOff = BITFIELD10(0,0,0,0,0,0,1,0,0,0);
  bitfield clear      = BITFIELD10(0,0,0,0,0,0,0,0,0,1);
  bitfield entryMode  = BITFIELD10(0,0,0,0,0,0,0,1,1,0);
  bitfield blink      = BITFIELD10(0,0,0,0,0,0,1,1,1,1);

  delay(50);
  LCDToWires2(init);
  delay(10);
  delay(10);
  LCDToWires2(init);
  delay(10);
  LCDToWires2(init);
  LCDToWires2(displayOff);
  LCDToWires2(clear);
  LCDToWires2(entryMode);
  LCDToWires2(blink);
}

void copyPatterns(){
  // Configure ports B and C
  GPIOB->CRL = 0x44b84222;
  GPIOC->CRL = 0x33333333;
}

void writeChar (char c){
  bitfield b = BITFIELD10(1, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  b.field = b.field | c;
  LCDToWires2(b);
}

void contrast(){
  // Everything required to set TIM3 to pulse
  // via PORTB5 to the LCD's potentiometer.

  TIM3 -> CR1 = 0x81;
  TIM3 -> SR  = 0x1f;
  TIM3 -> CCMR1 = 0x00006800;
  TIM3 -> CCER = 0x10;
  TIM3 -> ARR  = 0x000001df;
  TIM3 -> CCR2 = 0x00000155; // This is the contrast value.
  TIM3 -> DMAR = 0x00000081;

}

void writeString(char* s){
  int i;
  for (i = 0; s[i]!='\0';i++) {
    writeChar(s[i]);
  }
}

typedef struct BTable{
    uint32_t add_tx;
    uint32_t count_tx;
    uint32_t add_rx;
    uint32_t count_rx;
} btable;

void usbReset(void){
    // Set 3rd bit of GPIOD, which we're pretty sure
    // triggers a pull up resistor on D+.
    GPIOD->CRL = 0x88844244;
    GPIOD->BSRR = 4;

    USB->DADDR  |= 0x80;

    // Tell interrupt plz process reset.
    USB->CNTR = 0b1000010000000000;
    // Reset interrupts.
    USB->ISTR = 0;
    // Restore first part of PMA
    btable * b = (btable *) 0x40006000;
    b->add_tx = 0x00000028;
    b->count_tx= 0;
    b->add_rx = 0x20;
    b->count_rx = 0x00001000;
    //USB->CNTR = USB_CNTR_PDWN;
    //Enable the USB interrupt
    // set ep0r start_rx to valid, endpoint type to control
    //USB->EP0R = 0b0011001000000000;
    USB->EP0R = 0b0011001000110000;
}

int amIInASetAddressTransaction = 0;
void usb(){
  // Joking around with ISTR handlers.
  uint32_t * val = (uint32_t *)0x40006040;

  // Check if I need to reset
  if ((USB->ISTR &0b10000000000) > 0 ){
    usbReset();
  }

  // Check if I need to set device address.
  if (amIInASetAddressTransaction){
    USB -> DADDR |= *(val + 1);
    // Clear interrupts.
    USB->ISTR = 0;
    // Get ready to listen for OUT token immediately after.
    USB->EP0R = 0b0001001000000000;
    return;
  }

  // If it was a setup packet.
  if ((USB->EP0R &0b100000000000) > 0 ) {
    USB->ISTR = 0;

    if ((*val &0x500 ) > 0 ){
      amIInASetAddressTransaction = 1;
    //  USB -> DADDR |= *(val + 1);

    // done setting address
    // Clear ctr_rx
       USB->ISTR = 0;
      // set ep0r start_rx to valid, endpoint type to control
      //USB->EP0R = 0b0011001000000000;
       //USB->EP0R = 0b0010001000110000;
       // Get ready to listen for IN token immediately after.
       USB->EP0R = 0b0000001000010000;

      return;
    }
  }
}

uint32_t * record = (uint32_t *) 0x20005000;

void USB_LP_CAN1_RX0_IRQHandler(void){
  //if ((USB->ISTR & 0b0000010000000000) > 0){
     //char res[] = "reset";
     // writeString(res);

  //}
  uint32_t * val = (uint32_t *)0x40006040;
  // This logs all the values every time the
  // interrupt was called, starting with
  // initial value 0xdeadbeef at
  // Don't log an event if it's the same as the last one.

  uint32_t EP0StatusBefore = USB -> EP0R;
  usb();
  uint32_t EP0StatusAfter  = USB -> EP0R;

  *record = *val;
  *(record+1) = *(val + 1);
  *(record+2) = EP0StatusBefore;
  *(record+3) = EP0StatusAfter ;
  *(record+4)= 0xdeadbeef;
  record = record + 4;

  return;
}

int main(void) {
    // Push it forward
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x6000);

    GPIO_InitTypeDef GPIO_InitStructure;

    //NVIC structure to set up NVIC controller
    //NVIC_InitTypeDef NVIC_InitStructure;
    //NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn ;

    // enable clock on APB2
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,  ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,  ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,  ENABLE);

   // The three things you need to do to hook up Tim3_ch2
   // PortB5 output. Where the LCD potentiometer is connected.
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    // output push-pull mode
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   // highest speed
    GPIO_Init(GPIOE, &GPIO_InitStructure) ;             // initialize PORT

    //Turn on LCD backlight
    GPIO_SetBits(GPIOE, 0x1000);

    copyPatterns();

    // LCD logic
    initDisplay();
    contrast();
    writeString("  OPEN KLAVE  ");

    // usb init
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD,  ENABLE);
    NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn );

    usbReset();

    while(1){};
    //char* str = malloc( 5 + 1 );
    //writeString(str);

}
