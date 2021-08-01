#include "misc.h"
#include "main.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

// Multi-byte USB sequences are LOW BYTE - High BYTE
#define USBList(a,b) b, a

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

// Defaults up to 20 bytes in PMA.
struct PMAAllocation{
    uint32_t a[20];
};
//onst uint32_t msg[] = {0x12010110, 0x00000008, 0xe8092400, 0x01100000, 0x00010000, 0x00010000};

static constexpr uint8_t dev_descriptor[] = {
    18, //bLength
    1, //bDescriptorType
    0x10, 0x01, //bcdUSB
    0x00, //bDeviceClass (defined by interfaces)
    0x00, //bDeviceSubClass
    0x00, //bDeviceProtocl
    0x08, //bMaxPacketSize0
    0x09, 0xe8, //idVendor
    0x00, 0x24, //idProduct
    0x10, 0x01, //bcdDevice
    0, //iManufacturer
    0, //iProduct
    0, //iSerialNumber,
    1, //bNumConfigurations
};

static constexpr uint8_t configurationDescriptor[] = {
    9, //bLength
    2, //bDescriptorType
    0x00, 34, // wTotalLength in bytes of this and subsequent interface + endpoint descriptors
    0x01, //bNumInterfaces Number of interfaces.
    0x01, //bConfigurationValue
    0x00, //iConfiguration (unused)
    0x60, //bmAttributes
    0x00, //MaxPower, not applicable
};


static constexpr uint8_t standardACDescriptor[] = {
    9,    //bLength
    4,    //bDescriptorType
    0x00, //Index of this interface
    0x00, //bAlternateSetting
    0x00, //bNumEndpoints
    0x01, //bInterface AUDIO
    0x01, //BinterfaceSubclass AUDIO_CONTROL
    0x00, //Unused
    0x00, //Unused
};

static constexpr uint8_t mouseConfiguration[] = {
    9, //bLength
    2, //bDescriptorType
    USBList(0x00, 34), // wTotalLength in bytes of this and subsequent interface + endpoint descriptors
    0x01, //bNumInterfaces Number of interfaces.
    0x01, //bConfigurationValue
    0x00, //iConfiguration (unused)
    0x60, //bmAttributes
    0x00, //MaxPower, not applicable
    // Mouse HID interface
    9, // bLength
    4, //bDescriptorType Interface
    0, // Zero-based Interface number
    0, // alternate setting
    1, // bNumEndpoints
    3, // HID
    1, // Boot Interface
    2, // Mouse
    0, // Index of String descritpro
    // Generic HID header
    9, // bLength
    0x21, //HID
    USBList(0x01, 0x10), // HID Class Spec
    0, //No country code
    1, // 1 descriptor
    34,// DescriptorType
    USBList(0, 52), //Descriptor length
    // Endpoint for mouse
    7, // Length
    5, // bDescriptorType ENDPOINT
    0x81,// Endpoint 1 -IN
    0x03,// Interrupt data point
    USBList(0, 4), // Max packet size,
    10, // Polling rate, every X ms.
};

/*
static constexpr uint8_t midiInterfaceDescriptor[] = {
    9, //bLength
    2, //bDescriptorType
    0x00, 30, // wTotalLength in bytes of this and subsequent interface + endpoint descriptors
    0x01, //bNumInterfaces Number of interfaces.
    0x01, //bConfigurationValue
    0x00, //iConfiguration (unused)
    0x80, //bmAttributes bus powered
    0x32, //MaxPower, not applicable
};
*/

// Make a Device Configuration into new PMA Allocation.
constexpr PMAAllocation NewDeviceConfiguration(const uint8_t * src){
   PMAAllocation r = {};
   int DeviceConfigurationPermutation[] ={
    0,
    1,
    3,2,
    4,
    5,
    6,
    7,
    9,8,
    11,10,
    13,12,
    14,
    15,
    16,
    17,
    };
   for (int i = 0; i < 9; i++){
       // Permute to make sure multi-byte strings
       // are LOW BYTE to HIGH BYTE
       uint32_t a = src[DeviceConfigurationPermutation[2*i]];
       uint32_t b = src[DeviceConfigurationPermutation[(2*i)+1]];
       // Swap each high and low bytes because M3 is little endian
       r.a[i] = a | (b << 8);
   }
   return r;
}

// Make a ConfigurationDescriptor into new PMA Allocation.
constexpr PMAAllocation NewConfigurationDescriptor(const uint8_t * src){
   PMAAllocation r = {};
   // A Configuration descriptor is 9 bytes, so pad the end.
   for (int i = 0; i < 4; i++){
       // Permute to make sure multi-byte strings
       // are LOW BYTE to HIGH BYTE
       uint32_t a = src[i];
       uint32_t b = src[i+1];
       // Swap each high and low bytes because M3 is little endian
       r.a[i] = a | (b << 8);
   }
   r.a[4]=configurationDescriptor[8]; //odd byte
   // Reswap the multibyte field.
   r.a[1]= configurationDescriptor[3] | (configurationDescriptor[2] << 8);
   return r;
}

constexpr PMAAllocation NewStandardACDescriptor(const uint8_t * src){
  PMAAllocation r = {};
  for (int i = 0; i < 4; i++){
      // Permute to make sure multi-byte strings
      // are LOW BYTE to HIGH BYTE
      uint32_t a = src[i];
      uint32_t b = src[i+1];
      // Swap each high and low bytes because M3 is little endian
      r.a[i] = a | (b << 8);
  }
  r.a[4]=src[8]; //odd byte
  return r;
}

constexpr PMAAllocation mouseConfigToPMA(const uint8_t * src){
       PMAAllocation r = {};
       for (int i = 0; i < 17; i++){
          uint32_t a = src[i*2];
          uint32_t b = src[(i*2)+1];
          // Swap each high and low bytes because M3 is little endian
         r.a[i] = a | (b << 8);
       }

    return r;
}

struct Iterator{
  const int pos;
  const int size;
  const uint32_t currentWord;
  const PMAAllocation message;
};


constexpr Iterator NewIterator(const PMAAllocation msg, const int size){
  return Iterator{-1, size, 0, msg};
}

constexpr Iterator AdvanceIterator(Iterator i){
    //if (i.pos == i.size) return Iterator{i.pos+1, i.size, 69, i.message};
    uint32_t w = i.message.a[i.pos + 1];
    return Iterator{i.pos+1, i.size, w, i.message};
}


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

Iterator * currentMessageIterator = 0;
uint32_t * sigil = (uint32_t *)0x40006130;

void usb(){
  // Compile time allocation and inlining of Device Descriptor.
  constexpr PMAAllocation deviceDescriptor = NewDeviceConfiguration(dev_descriptor);
  constexpr PMAAllocation configDescriptor = NewConfigurationDescriptor(configurationDescriptor);
  constexpr PMAAllocation standardAC = NewStandardACDescriptor(standardACDescriptor);

  constexpr PMAAllocation mouseTry = mouseConfigToPMA(mouseConfiguration);

  // Message tracking variables to exist for lifetime of program.
  static int messagePosition = 0;
  static int messageSize = 0;
  static const PMAAllocation * currentMessage = 0;

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
    amIInASetAddressTransaction = 0; // reset
    return;
  }

  // If it was a setup packet.
  //if ((USB->EP0R &0b100000000000) > 0 ) {
  if (GetRegister(*USB_EP0R, EP_Setup) & (USBDeviceAddress() == 0)){
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

  if (EP0State.isSetupToken){
      // Clear interrupts.
      //USB->ISTR = 0;
      btable * b = (btable *) 0x40006000;
      b->count_tx= 8;
    // If the device is addressed, and we're in a device configuration request.
    if ((USBDeviceAddress() != 0) & (*val == 0x0680) & (*(val+1) == 0x0100))
    {
      currentMessage = &deviceDescriptor;
      messagePosition = 0;
      messageSize = 9;
    }

    if ((USBDeviceAddress() != 0) & (*val == 0x0680) & (*(val+1) == 0x0f00))
    {
      currentMessage = &deviceDescriptor;
      messagePosition = 1;
      messageSize = 0;
      // Send this right to ACK transaction.
    }
    // If the device is addressed, and we're in a configuration descriptor request.
    if ((USBDeviceAddress() != 0) & (*val == 0x0680) & (*(val+1) == 0x0200))
    {
      static int configured = 0;
      //currentMessage = &configDescriptor;
      currentMessage = &mouseTry;
      messagePosition = 0;
      messageSize = 5; // To examine
      if (configured) messageSize = 17;
      configured = 1;
    }
    /*
    if ((USBDeviceAddress() != 0) & (*val == 0x0680) & (*(val+1) == 0x0300))
    {
      currentMessageIterator = &standardACInterfaceIterator;
      uint32_t * sigil2 = (uint32_t *)0x400061d0;
      *sigil2 = 0xeeee;

    }*/
  }
    if (currentMessage != 0){
      // Is it a SETUP token? Then get ready for an IN.


      uint32_t * PMAWrite = (uint32_t *)0x40006050;
      btable * b = (btable *) 0x40006000;
      //b->count_tx= 0x2;?
      b->count_tx= 8;
      // b-> count_tx defaults to 8 bytes.
      // We might hit this a few times to get the whole 18byte messages
      // in 8 bit chunks. So we use an iterator object that remembers where
      // it was in the stream to read the data out to the PMA.
      //
      // Of interest: in our C + constexpr scheme, the exact values to write to
      // PMA as 32 byte numbers are already stored in the iterator at compiletime.


      static uint32_t * logSigil = (uint32_t *)0x40006150;
      for (int i = 0; i< 4; i++){
        if (messagePosition == messageSize + 1){
          // We're in the ACK status transaction.
          // We just got a zero length out token.
          // Ack it.
          b -> count_tx = 0;
          //currentMessage = 0;
          break;
        }
        if (messagePosition == messageSize){
          b -> count_tx = i*2;
          //currentMessage  = 0;
          if (( currentMessage == &mouseTry) & (messageSize == 5) ){
            b -> count_tx = 1;
          }
          *logSigil = 0xFFFF;
          logSigil++;
          *logSigil = b-> count_tx;
          logSigil++;
          messagePosition++; // To indicate status.
          break;
        }
        *PMAWrite = currentMessage -> a[messagePosition];
        *logSigil = *PMAWrite;
        logSigil++;
        messagePosition++;
        PMAWrite++;

      }

      // clear interrupts
      USB->ISTR = 0;

      // next message could be in or out depending on if the config request message
      // is done yet.
      USBEndpointState state = USBEndpointState{*USB_EP0R, 0, 0, 0, VALID, VALID, CONTROL, 0 };
      SetRegister(USB_EP0R, setEndpointState(state));

      return;
    }
    return;

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
