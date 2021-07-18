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

int amIInASetAddressTransaction = 0;

void usbReset(void){
    amIInASetAddressTransaction = 0;
    // Set 3rd bit of GPIOD, which we're pretty sure
    // triggers a pull up resistor on D+.
    GPIOD->CRL = 0x88844244;
    GPIOD->BSRR = 4;

    USB->DADDR  = 0x80;

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
    USB->EP0R = 0b0011001000100000;
}

// Each bit can be Read, Read/Write, or
// Toggle (flips value on 1, no change on 0)
enum BitType{R, RW, T};

struct Register{
  const uint32_t toggleMask;
  // How big is this register?
  int length;
};

struct Mask{ uint32_t mask; int shiftLength; };

constexpr uint32_t GetRegister(uint32_t reg, struct Mask m){
  return (reg & m.mask) >> m.shiftLength;
}
//constexpr uint32_t neg(uint32_t val){return val ^ 0b11111111;} // TODO make more generic.

constexpr uint32_t GetRegisterUpdateValue(Register reg, uint32_t currentVal, uint32_t update, uint32_t mask){
  return (currentVal & (update ^ reg.toggleMask ^ ~mask)) |
  (~currentVal & update);

  // Second reduction
  //return (update & ~reg.toggleMask)|
  //(update & ~currentVal) |
  //(currentVal&((~update&reg.toggleMask&mask) | (~reg.toggleMask&~mask)));


  // This was the first eq.
  //return ((update & ~reg.toggleMask)|
  //(update & ~currentVal) |
  //(~update & reg.toggleMask & currentVal & mask) |
  //(~reg.toggleMask & currentVal & ~mask));
}
enum EPCommsStatus {DISABLED, STALL, NAK, VALID};
enum EPType{BULK, CONTROL, ISO, INTERRUPT};

struct USBEndpointState{
  uint32_t currentVal;
  uint32_t isOutOrSetupToken; // CTR_RX
  uint32_t isSetupToken;      // SETUP
  uint32_t isInToken;         // CTR_TX
  EPCommsStatus STAT_RX;
  EPCommsStatus STAT_TX;
  EPType        EP_TYPE;
  int EndpointAddress;
};

constexpr struct Mask EP_OUT     = {0b1000000000000000, 15};
constexpr struct Mask EP_Setup   = {0b0000100000000000, 11};
constexpr struct Mask EP_IN      = {0b0000000010000000, 7};
constexpr struct Mask EP_Type    = {0b0000011000000000, 9};
constexpr struct Mask EP_addr    = {0b0000000000001111, 0};
constexpr struct Mask EP_Stat_RX = {0b0011000000000000, 12};
constexpr struct Mask EP_Stat_TX = {0b0000000000110000,  4};

// Take bytes from RAM and get them in PMA form.
constexpr uint32_t toPMA(const uint32_t * src, int position){
    uint32_t word = src[position/2];
    // We're taking the lower order halfword
    if (position%2 == 0){
        return (word >> 24) | ((word &0xFF0000)>>8);
    }
    return ((word&0xFF) <<8) | (word&0xFF00)>>8;
}

// Clear all transaction bits, set STAT_TX, STAT_RX, EP_TYPE, and Address.
constexpr uint32_t setEndpointState(USBEndpointState s){
  constexpr struct Register usbendpoint =  Register{0b0111000001110000, 16};
  uint32_t mask = EP_Stat_TX.mask | EP_Stat_RX.mask | EP_Type.mask | EP_addr.mask;
  mask |= 0b1000100010000000; // Clear transmission bits.
  uint32_t update  =
  (s.STAT_RX << EP_Stat_RX.shiftLength) |
  (s.STAT_TX << EP_Stat_TX.shiftLength) |
  s.EndpointAddress                     |
  (s.EP_TYPE << EP_Type.shiftLength);
  return GetRegisterUpdateValue(usbendpoint, s.currentVal, update, mask);
}

constexpr USBEndpointState GetEndpointState(uint32_t regValue){
  struct USBEndpointState ret  = {};
  ret.currentVal        = regValue;
  ret.isOutOrSetupToken = regValue & EP_OUT.mask;
  ret.isSetupToken      = regValue & EP_Setup.mask;
  ret.isInToken         = regValue & EP_IN.mask;
  ret.STAT_RX           = (EPCommsStatus)((regValue & EP_Stat_RX.mask) >> EP_Stat_RX.shiftLength);
  ret.STAT_TX           = (EPCommsStatus)((regValue & EP_Stat_TX.mask) >> EP_Stat_TX.shiftLength);
  ret.EndpointAddress   = regValue & EP_addr.mask;
  return ret;
}

constexpr void SetRegister(uint32_t * ptr, int value){
  *ptr = value;
  return;
}

// Get the USB Device Addressed assigned by Host.
int USBDeviceAddress(void){
  uint32_t * USB_DEVICE = (uint32_t *)0x40005c4c;
  return *USB_DEVICE & 0b1111111;
}

int DeviceConfigRequestState = 0;

int DeviceDescriptorState = 0;

void usb(){

  // Check if I need to reset
  if ((USB->ISTR &0b10000000000) > 0 ){
    //writeString((char*)"  Re ");
    usbReset();
    return;
  }
  // Joking around with ISTR handlers.
  uint32_t * val = (uint32_t *)0x40006040;
  uint32_t * USB_EP0R = (uint32_t *)0x40005c00;
  //uint32_t * USB_EP1R = (uint32_t *)0x40005c04;

  USBEndpointState EP0State = GetEndpointState(*USB_EP0R);
  // ACK if just got a clear feature request.
  if ((USBDeviceAddress() != 0) & (*val == 0x0000) & (*(val+1) == 0x0100)){
    USB->ISTR = 0;

   // Could be anything next who knows
   USBEndpointState state = USBEndpointState{*USB_EP0R, 0, 0, 0, VALID, VALID, CONTROL, 0 };
   SetRegister(USB_EP0R, setEndpointState(state));//EP_Stat_RX, 0b11);
   btable * b = (btable *) 0x40006000;
   b->count_tx= 0;
   return;

  }

  // If the device is addressed, and we're in a get configuration request
  if ((USBDeviceAddress() != 0) & (*val == 0x0680) & (*(val+1) == 0x0302)){
    uint32_t * PMAWrite = (uint32_t *)0x40006050;
    // Is it a SETUP token? Then get ready for an IN.
    if (EP0State.isSetupToken){
      USB->ISTR = 0;
      // Get ready for an IN token next.
      //USBEndpointState state = USBEndpointState{*USB_EP0R, 0, 0, 0, NAK, VALID, CONTROL, 0 };
      //SetRegister(USB_EP0R, setEndpointState(state));
      btable * b = (btable *) 0x40006000;
      b->count_tx= 8;
      DeviceDescriptorState = 0;
    }

    const uint32_t msg[] = {0x09020009, 0x01011001, 0x69690000};


    if (DeviceDescriptorState == 1){
      // Set msg length to 18 = 0x12
      btable * b = (btable *) 0x40006000;
      b->count_tx= 0x8;
      for (int i = 0; i< 4; i++){
        *PMAWrite = toPMA((msg+2), i);
        PMAWrite++;
      }
      DeviceDescriptorState = DeviceDescriptorState + 1;
    }

    if (DeviceDescriptorState == 0){
      // Set msg length to 18 = 0x12
      btable * b = (btable *) 0x40006000;
      b->count_tx= 0x8;
      //btable * b = (btable *) 0x40006050;
      //b->add_tx = 0x1201;
      //b->count_tx= 0x0110;
      //b->add_rx = 0;
      //b->count_rx = 0x8;
      for (int i = 0; i< 4; i++){
        *PMAWrite = toPMA(msg, i);
        PMAWrite++;
      }
      DeviceDescriptorState =  DeviceDescriptorState + 1;
    }
    // clear interrupts
    USB->ISTR = 0;

    // next message could be in or out depending on if the config request message
    // is done yet.
    USBEndpointState state = USBEndpointState{*USB_EP0R, 0, 0, 0, VALID, VALID, CONTROL, 0 };
    SetRegister(USB_EP0R, setEndpointState(state));

    return;

  }

  // If the device is addressed, and we're in a device configuration request.
  if ((USBDeviceAddress() != 0) & (*val == 0x0680) & (*(val+1) == 0x0100))
  {
    // Is it a SETUP token? Then get ready for an IN.
    if (EP0State.isSetupToken){
      // Clear interrupts.
      USB->ISTR = 0;
      // Get ready for an IN token next.
      //USBEndpointState state = USBEndpointState{*USB_EP0R, 0, 0, 0, NAK, VALID, CONTROL, 0 };
      //SetRegister(USB_EP0R, setEndpointState(state));
      btable * b = (btable *) 0x40006000;
      b->count_tx= 8;
      DeviceConfigRequestState = 0;
      //return;
    }
    //writeString((char*)"  GOT TO HERE ");
     // test copy some dead beef stuff
    uint32_t * PMAWrite = (uint32_t *)0x40006050;
    /*
    0x40006050:	0x00000112	0x00000110	0x00000000	0x00000800
    0x40006060:	0x0000194c	0x0000d03d	0x0000a2b8	0x0000ae98
    0x40006070:	0x000036a0	0x0000e58a	0x000017bc	0x00007719
    */
    //                       len|bcdUSB
    // this needs to be 18 bytes. 4.5 words.
    /*

    Product ID:	0x0024
Vendor ID:	0x09e8  (AKAI  professional M.I. Corp.)
Version:	1.00
Serial Number:	NO SERIAL NUMBER
Speed:	Up to 12 Mb/s
Manufacturer:	Akai
Location ID:	0x14110000 / 44
Current Available (mA):	500
*/
// second piece
// 0x000009e8	0x00000024	0x00000000	0x00000800
    //const uint32_t msg[] = {0x01121001, 0x00000800, 0xe8092400, 0x00010102, 0x00010000, 0x00010000};
    const uint32_t msg[] = {0x12010110, 0x00000008, 0xe8092400, 0x01100000, 0x00010000, 0x00010000};
    //const uint32_t msg[] = {0x01121001, 0x00000800, 0xe8092400, 0x00010102, 0x00010000, 0x00010000};
    if (DeviceConfigRequestState == 3){
      // Just sign off on status message here.
      btable * b = (btable *) 0x40006000;
      b->count_tx= 0;
      // clear interrupts
      USB->ISTR = 0;

      // next message could be in or out depending on if the config request message
      // is done yet.
      USBEndpointState state = USBEndpointState{*USB_EP0R, 0, 0, 0, VALID, NAK, CONTROL, 0 };
      SetRegister(USB_EP0R, setEndpointState(state));
      return;

    }
    if (DeviceConfigRequestState == 2){
      // Set msg length to 18 = 0x12
      btable * b = (btable *) 0x40006000;
      b->count_tx= 0x2;
      for (int i = 0; i< 4; i++){
        *PMAWrite = toPMA((msg+4), i);
        PMAWrite++;
      }
      DeviceConfigRequestState = DeviceConfigRequestState + 1;
    }

    if (DeviceConfigRequestState == 1){
      // Set msg length to 18 = 0x12
      btable * b = (btable *) 0x40006000;
      b->count_tx= 0x8;
      for (int i = 0; i< 4; i++){
        *PMAWrite = toPMA((msg+2), i);
        PMAWrite++;
      }
      DeviceConfigRequestState = DeviceConfigRequestState + 1;
    }

    if (DeviceConfigRequestState == 0){
      // Set msg length to 18 = 0x12
      btable * b = (btable *) 0x40006000;
      b->count_tx= 0x8;
      //btable * b = (btable *) 0x40006050;
      //b->add_tx = 0x1201;
      //b->count_tx= 0x0110;
      //b->add_rx = 0;
      //b->count_rx = 0x8;
      for (int i = 0; i< 4; i++){
        *PMAWrite = toPMA(msg, i);
        PMAWrite++;
      }
      DeviceConfigRequestState =  DeviceConfigRequestState + 1;
    }


    // clear interrupts
    USB->ISTR = 0;

    // next message could be in or out depending on if the config request message
    // is done yet.
    USBEndpointState state = USBEndpointState{*USB_EP0R, 0, 0, 0, VALID, VALID, CONTROL, 0 };
    SetRegister(USB_EP0R, setEndpointState(state));

    return;
  }

  // Check if I need to set device address.
  if (amIInASetAddressTransaction){
    USB -> DADDR |= *(val + 1);
    // Clear interrupts.
    USB->ISTR = 0;
    // Get ready to listen for OUT token immediately after.
    //USB->EP0R = 0b0001001000000000;
    USBEndpointState state = USBEndpointState{*USB_EP0R, 0, 0, 0, VALID, NAK, CONTROL, 0 };
    SetRegister(USB_EP0R, setEndpointState(state));
    //SetRegister(USB_EP0R, 02, 0b11);//EP_Stat_TX, 0b11);
    return;
  }

  // If it was a setup packet.
  //if ((USB->EP0R &0b100000000000) > 0 ) {
  if (GetRegister(*USB_EP0R, EP_Setup)){
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

       //USB->EP0R = 0b0000001000010000;
      // Get ready to listen for IN token immediately after.
      USBEndpointState state = USBEndpointState{*USB_EP0R, 0, 0, 0, NAK, VALID, CONTROL, 0 };
      SetRegister(USB_EP0R, setEndpointState(state));//EP_Stat_RX, 0b11);
      return;
    }
  }
}

volatile uint32_t * record = (uint32_t *) 0x20005000;

extern "C"{

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

}
/*
volatile uint32_t * record2 = (uint32_t *) 0x20006000;
// Bootloader logging code
extern "C"{
__attribute__((section(".mySection")))
void BootloaderLogger(void){
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

  *record2 = *val;
  *(record2+1) = *(val + 1);
  *(record2+2) = EP0StatusBefore;
  *(record2+3) = EP0StatusAfter ;
  *(record2+4)= 0xdeadbeef;
  record2 = record2 + 4;

  return;
}

}*/

// We put this include right before main.
// So test code can reference the above declared code.
#include "test_main.h"
//

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
    writeString((char*)"  OPEN Klave  ");

    // Test to see if we should execute test suite.
    uint32_t * testSigil = (uint32_t *) 0x20006000;
    if (*testSigil == 0xFeedBeef){
      Test();
      return 1;
    }

    // usb init
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD,  ENABLE);
    NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn );

    usbReset();

    while(1){};
    //char* str = malloc( 5 + 1 );
    //writeString(str);
}
