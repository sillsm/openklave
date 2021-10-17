#ifndef __KERNEL_C
#define __KERNEL_C

#include "misc.h"
#include "kernel.h"
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

void delayTicks(uint32_t ticks) {
        volatile uint32_t x = ticks;
        while (x-- > 0) {
            __asm("nop");
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
  uint32_t controlmask = 0b00000000000000000000000000000110;
  //GPIOB->BSRR = controlbits | (~controlbits & controlmask) << 16;
  uint32_t readMaskChange = ((GPIOB -> ODR) & 0b1001) | controlbits;
  GPIOB -> ODR = readMaskChange;

  // Port C0-7 does the data stuff.
  uint32_t databits = b.field&0b0011111111;

  uint32_t datamask = 0b00000000000000000000000011111111;
//  x | ((~x & datamask) << 16)
  //GPIOC->BSRR = databits | (~databits & datamask) << 16;
  GPIOC -> ODR = databits;
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
/*
void _writeScreen(char*s){
  // entire screen state kept as a local variable.
  static char screenState[80]= {0};
  // update screenState here

  // Rewrite screen here
  for (int i = 0; i<80; i++) {

  }
}*/


// This section defines Event and EventStack.
// Eventually, all integrated signals should be pushed to
// a global EventStack.
//
// USB consumers pop it, convert it, and transmit.
//
// Later, user-defined hook events outside the kernel get a shot
// to provide kernel a call back that pops and pushes events to EventStack
// write before USB or others consume it.
//
// For example, a call back could inspect events for midi notes, and
// wait until Middle C is identified. Once identified, Middle C is popped,
// and G E C events are pushed back on the stack, making an automatic chord.
// Even later, events should be timestamped so consumers can delay event
// transmission by a certain amount of ticks.
//
struct Event{
  uint32_t A;
  uint32_t B;
  uint32_t C;
  uint32_t D;
};

struct EventStack{
  int32_t top;
  int32_t capacity;
  uint32_t scratch_2;
  uint32_t scratch_3;
  Event Buffer[10];

};

// Call this if someone tries to allocate past the capacity of the
// GlobalEventStack
void GlobalEventStackOverflow(){
  while (1) {
      __asm("nop");
  }
}

// Do we need to replace volatile keyword here
EventStack * GlobalEventStack = (EventStack * )0x2000b000;

constexpr void PushEvent(EventStack * ES, Event e){
  // If trying to push past capacity, panic
  if (ES-> top >= (ES-> capacity - 1)){
    GlobalEventStackOverflow();
  }
  ES->top = ES-> top + 1;
  ES -> Buffer[ES->top] = e;
}

constexpr Event PopEvent(EventStack * ES){
    if (ES -> top < 0){
      return Event{0,0,0,0};
    }
    // Otherwise
    ES -> top = ES -> top  -1;
    return ES -> Buffer[ES->top+1];
}

constexpr int IsEventStackEmpty (EventStack *ES){
  if (ES -> top < 0){return 1;}
  return 0;
}

Event PeekTopEvent(EventStack * ES, Event e){
  return ES -> Buffer[ES -> top];
}


void clearScreen(){
  bitfield clear      = BITFIELD10(0,0,0,0,0,0,0,0,0,1);
  LCDToWires2(clear);
  delay(2);
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

// Each bit can be Read, Read/Write, or
// Toggle (flips value on 1, no change on 0)

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
    uint32_t a[56];
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

static constexpr uint8_t mouseConfiguration[] = {
    // Configuration Descriptor
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

static constexpr uint8_t mouseHIDDescriptor[] = {
  0x05, 0x01,
  0x09, 0x02,
  0xA1, 0x01,
  0x09, 0x01,
  0xA1, 0x00,
  0x05, 0x09,
  0x19, 0x01,
  0x29, 0x05,
  0x15, 0x00,
  0x25, 0x01,
  0x95, 0x05,
  0x75, 0x01,
  0x81, 0x02,
  0x95, 0x01,
  0x75, 0x03,
  0x81, 0x01,
  0x05, 0x01,
  0x09, 0x30,
  0x09, 0x31,
  0x09, 0x38,
  0x15, 0x81,
  0x25, 0x7F,
  0x75, 0x08,
  0x95, 0x03,
  0x81, 0x06,
  0xC0,
  0xC0,
};

static constexpr uint8_t USBMidiOneOutConfig[] = {
  // USB MIDI 2.0 Specification.
  //
  // Configuration Descriptor
  9, //bLength
  2, //bDescriptorType
  USBList(0x00, 101), // wTotalLength in bytes of this and subsequent interface + endpoint descriptors
  0x02, //bNumInterfaces Number of interfaces.
  0x01, //bConfigurationValue
  0x00, //iConfiguration (unused)
  0x60, //bmAttributes
  0x00, //MaxPower, not applicable
  // Standard AC Interface Descriptor
  9, // bLength
  4, // bDescriptorType Interface Descriptor
  0, // Zero-based Interface number
  0, // alternate setting
  0, // bNumEndpoints
  1, // AUDIO
  1, // AUDIO_CONTROL
  0, // Unused
  0, // Unused
  // Class-specific AC Interface Descriptor
  // B.2.2
  9, // bLength
  0x24, //CS_INTERFACE
  1, //HEADER subtypbe
  USBList(1,0), // bcdADC
  USBList(0,9), // Total size of class specific descriptors
  1, // 1 number of streaming interface
  1, // Designate MIDIStreaming interface
  //
  // MIDIStreaming Interface 2.0
  // B.6.1
  9, // length of descriptor
  4, // interface descriptor
  1, // index of this interface
  0, // Index of alternate setting.
  2, // 2 endpoints
  1, // AUDIO Class
  3, // MIDISTREAMING sub Class
  0, // unused
  0, // unused
  // B.6.2
  7, // length of descriptor.
  0x24,// CS_INTERFACE descriptor
  1, // MIDISTREAMING header subtypbe
  USBList(0x01,0x00), // MS_MIDI_2.0
  USBList(0,65), // dummy field used for conformity with MIDI 1.0
  // B.8.1
  // B.4.3 MIDI IN Jack Descriptor
  		6, // This length
  		36,// This type
  		2, // MIDI_IN_JACK
  		1, // EMBEDDED bJackType
  		1, // Jack ID
  		0, // String Id

  		6, // This length
  		36,// This type
  		2, // MIDI_IN_JACK desc subtype
  		2, // EXTERNAL bJackType
  		2, // Jack ID
  		0, // String ID

  //B.4.4 MIDI OUT Jack Descriptor
  		9, // This length
  		36,// This type
  		3, // MIDI_OUT_JACK
  		1, // EMBEDDED bJackType
  		3, // bJackID
  		1, // No of input pins
  		2, // BaSourceID
  		1, // BaSourcePin
  		0, // string descriptor

  		9, // This length
  		36,// This type
  		3, // MIDI_OUT_JACK
  		2, // EXTERNAL bJackType
  		4, // bJackID
  		1, // bNrInputPins
  		1, // baSourceID (0)
  		1, // baSourcePin (0)
  		0, // string descriptor

  //B.5.1 Interrupt OUT
  // Note this is Midi 1.0, but oddly, interrupt is supported as of Big Sur on
  // MAC. Great latency, but need to see if compatible and/or compliant.
  		9,   // This length
  		5,   // bDescriptorType = endpoint
  		0x1, // bEndpointAddress OUT endpoint number 1
  		3,   //  bmAttributes: 3:Interrupt endpoint 2:Bulk, see note above.
  		8, 0,// wMaxPacketSize  = 8 bytes
  		1,   // bInterval Interrupt polling interval in ms
  		0,   // bRefresh
  		0,   // bSyncAddress

  // B.5.2 Class-specific MS Bulk OUT Endpoint Descriptor
  		5,  // This length
  		37, // This type
  		1,  // This subtype
  		1,  // bNumEmbMIDIJack
  		1,  // baAssocJackID (0)

  //B.6 Bulk IN Endpoint Descriptors
  //B.6.1 Standard Bulk IN Endpoint Descriptor
  		9,    // This length.
  		5,    // endpoint descriptor
  		0x81, // IN address.
  		3,    // bmAttributes: 3: Interrupt endpoint 2: Bulk, see note above.
  		8, 0, // wMaxPacketSize
  		1,    // bInterval in ms
  		0,    // bRefresh
  		0,    // bSyncAddress

  // B.6.2 Class-specific MS Bulk IN Endpoint Descriptor
  		5, // This length
  		37,// Desc type
  		1, // Desc subtype
  		1, // Num Jack
  		3, // baAssocJackID (0)
};

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

constexpr PMAAllocation NewMouseHIDReportToPMA(const uint8_t * src){
       PMAAllocation r = {};
       for (int i = 0; i < 26; i++){
          uint32_t a = src[i*2];
          uint32_t b = src[(i*2)+1];
          // Swap each high and low bytes because M3 is little endian
         r.a[i] = a | (b << 8);
       }
    return r;
}

// length 55
constexpr PMAAllocation NewUSBMIDIToPMA(const uint8_t * src){
  PMAAllocation r = {};
  for (int i = 0; i < 50; i++){
      // Permute to make sure multi-byte strings
      // are LOW BYTE to HIGH BYTE
      uint32_t a = src[(i*2)];
      uint32_t b = src[(i*2)+1];
      // Swap each high and low bytes because M3 is little endian
      r.a[i] = a | (b << 8);
  }
  r.a[50]=src[100]; //odd byte
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
    volatile btable * b = (btable *) 0x40006000;
    b->add_tx = 0x00000028;
    b->count_tx= 0;
    b->add_rx = 0x20;
    b->count_rx = 0x00001000;
    //USB->CNTR = USB_CNTR_PDWN;
    //Enable the USB interrupt
    // set ep0r start_rx to valid, endpoint type to control
    //USB->EP0R = 0b0011001000000000;
    USB->EP0R = 0b0011001000100000;

    // Now do stuff for EP01, our mouse interrupt transfer
    uint32_t * USB_EP1R = (uint32_t *)0x40005c04;
    USBEndpointState state = USBEndpointState{*USB_EP1R, 0, 0, 0, NAK, VALID, INTERRUPT, 1 };
    SetRegister(USB_EP1R, setEndpointState(state));

    b = (btable *) 0x40006010;
    b->add_tx = 0x00000030;
    b->count_tx= 4;
    b->add_rx = 0x35;
    b->count_rx = 0x00001000;
}

void writeNumberToScreen(uint32_t u){
  clearScreen();
  delay(4);
  char a = (u&0x0000000F) >> 0;
  char b = (u&0x000000F0) >> 4;
  char c = (u&0x00000F00) >> 8;
  char d = (u&0x0000F000) >> 12;
  char e = (u&0x000F0000) >> 16;
  char f = (u&0x00F00000) >> 20;
  char g = (u&0x0F000000) >> 24;
  char h = (u&0xF0000000) >> 28;
  char s[8]= {h,g,f,e,d,c,b,a};

  for (int i = 0; i<8; i++){
    if (s[i] < 0xA) {
      writeChar(s[i]+48);
      continue;
    }
    writeChar(s[i]+55);
  }

}

// Struct that holds values for upper part of keyboard
typedef struct buttons{
    uint32_t Pads[16];
    // Modification flag
    uint32_t PadsToModify;
    // Which Pads are "on" bitfield
    uint32_t WhichPadsAreOn;
    // Faders
    uint32_t Fader1;
    uint32_t Fader2;
    uint32_t Fader3;
    uint32_t Fader4;
    uint32_t Fader5;
    uint32_t Fader6;
    uint32_t Fader7;
    uint32_t Fader8;
    //Knobs
    uint32_t Knob1;
    uint32_t Knob2;
    uint32_t Knob3;
    uint32_t Knob4;
    uint32_t Knob5;
    uint32_t Knob6;
    uint32_t Knob7;
    uint32_t Knob8;
    // Wheels
    uint32_t PitchWheel;
    uint32_t ModWheel;
} Buttons;

 // We need to just hope? that 0x20004000 et. seq. has storage space
 // for our keyboard. We should probably do something in the linker
 // to make this explicit.
 Buttons * KeyboardButtons = (Buttons *) 0x20001000;

// Test this.
// This very dangerous function manipulates the global
// KeyboardButtons pointer object.
//
// TODO: figures out how to generate gloabl events write here.
// Further, if its an event generator, we should consider
// getting rid of the globals and just make this a closure.
void CompareAndSetPad(int Pad, uint16_t valToCompare){
  // First, we shift the Pad's current value 16 bits to the left.
  uint32_t lastMeasuredValue= (KeyboardButtons->Pads[Pad])&0xFFFF;
  KeyboardButtons->Pads[Pad] = (lastMeasuredValue<<16) | valToCompare;
  // compute their difference and set a modification bit if the difference
  // is too big. We use XOR as a proxy for Abs(a-b)

  int modify = 0;
  if ((lastMeasuredValue ^ valToCompare) > 2){
    modify = 1;
    //KeyboardButtons -> PadsToModify |= (1<<Pad);
  }
  static int IsThisPadOn[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  // No matter what, note off at 0
  if ((valToCompare == 0) && IsThisPadOn[Pad] ){
    IsThisPadOn[Pad]= 0;
    Event e = {901,0,60+(uint32_t)Pad,0};
    PushEvent(GlobalEventStack, e);
    return;
  }

  if (modify && !IsThisPadOn[Pad]){
    // If the Pad isn't ON, generate a PADON event.
    Event e = {900,97,60+(uint32_t)Pad,0};
    PushEvent(GlobalEventStack, e);
    IsThisPadOn[Pad] = 1;
    return;
  }
  if (modify && IsThisPadOn[Pad]){
    // Pad is already ON, and not 0.
    // So we want aftertouch pressure event.
    uint32_t velocity = (((KeyboardButtons-> Pads[Pad]) & 0x0FF0)>>4 ) + 3;
    Event e = {905,velocity,60+(uint32_t)Pad,0};
    PushEvent(GlobalEventStack, e);

    return;
  }

  return;
}

// Updates PitchWheel status.
void CheckPitchWheel(){
  // Pitch is a little jittery so we need to smooth it out with a
  // window.
  static uint16_t lastPitchValue = 0;
  volatile uint16_t * ADCBase = (uint16_t *) 0x20002c96;
  volatile uint16_t currentVoltage = *(ADCBase +7);

  uint16_t maxVoltage = 0xfd7; //-> 0x3fff
  uint16_t minVoltage = 0x20;  //-> 0
  uint16_t scaleSize  = (maxVoltage - minVoltage);

  // Mapped Scale is MAX 0x3fff MIN 0.
  // (currentVoltage-minVoltage)/scaleSize = x/ mappedScale
  // mappedScale* (currentVoltage-minVoltage)/scaleSize;

  uint16_t currentPitchValue = ((currentVoltage-minVoltage))*100/scaleSize;
  if (currentPitchValue != lastPitchValue){
    // Emit pitchbender event

    // Map the internal PitchValue to a bigger scale.
    uint32_t mappedPitch = (uint32_t)currentPitchValue*81.92*4;
    // We have a number between 0 and 200.
    uint32_t leftVal = ((mappedPitch & 0xFF00)>>8) + 3;
    if (leftVal > 2*0x3f){leftVal = 0x3f*2;}


    //Event e = {200, (mappedPitch & 0xFF),((mappedPitch & 0xFF00))>>8, 0};
    // Remember, each of MSB and LSB caps at 7F.
    //Event e = {200,0x60,0x12, 0};
    Event e = {200, leftVal,((mappedPitch & 0xff)), 0};

    // Take a number 0 to 200. Make it between 0 and 16383. And double that.
    // so multiply it by 2 * (16383/200).
    //Event e = {200,0x21, 0x22, 0};
    PushEvent(GlobalEventStack, e);
    lastPitchValue= currentPitchValue;
    return;
  }
  return;
}

// Gets called after every ADC1 conversion event is complete.
//extern "C"{
 //void ADC1_2_IRQHandler(){
 void PadChecker(){
   int debug = 0;
   static uint32_t * DEBUG  = (uint32_t *)0x20005000;
   if (debug){
     *DEBUG = 11;
     DEBUG++;
   }
   // reset which buttons need modification
   //KeyboardButtons -> PadsToModify = 0;
   uint16_t * ADCBase = (uint16_t *) 0x20002c96;

   uint16_t * pada = ADCBase;
   uint16_t * padb = ADCBase + 1;
   uint16_t * padc = ADCBase + 2;
   uint16_t * padd = ADCBase + 3;
   // we sample voltages via ADC.
   // we demux by selecting other ODR values.
   volatile uint32_t * GPIOE_ODR   = (uint32_t *)0x4001180c;

   static uint32_t wait = 0;
   wait++;
   if (wait == 9){wait=0; return;}

   if (wait == 1){
   *GPIOE_ODR = 0x5157;
 }

   if (wait == 2){

     CompareAndSetPad(12, *pada);
     CompareAndSetPad(13, *padb);
     CompareAndSetPad(14, *padc);
     CompareAndSetPad(15, *padd);
     return;
   }

   if (wait==3){
   // Switch and wait 20 clock cycles.
     *GPIOE_ODR = 0x514b;
     return;
   }

   if (wait==4){

    CompareAndSetPad(8, *pada);
    CompareAndSetPad(9, *padb);
    CompareAndSetPad(10, *padc);
    CompareAndSetPad(11, *padd);
    return;
   }

   if (wait==5){
   // Switch and wait 20 clock cycles.
     *GPIOE_ODR = 0x510d;
     return;
   }

   if (wait == 6){
   // Switch and wait 20 clock cycles.

     CompareAndSetPad(4, *pada);
     CompareAndSetPad(5, *padb);
     CompareAndSetPad(6, *padc);
     CompareAndSetPad(7, *padd);
     return;
   }

   if (wait==7){
   // Switch and wait 20 clock cycles.
     *GPIOE_ODR = 0x511e;
     return;
   }

   // Switch and wait 20 clock cycles.
   if (wait==8){
     CheckPitchWheel();
     CompareAndSetPad(0, *pada);
     CompareAndSetPad(1, *padb);
     CompareAndSetPad(2, *padc);
     CompareAndSetPad(3, *padd);
     return;
 }
  return;
}
//}

static uint32_t * GlobalButtonRamLocation = (uint32_t *)0x20000c00;
// CheckButtonsPushed checks if any binary buttons have been pushed,
// and generates event on GlobalEventStack if so.
void CheckButtonsPushed(){
  // Start with S buttons.
  uint32_t SButtons = *(GlobalButtonRamLocation+1) &0xFF;
  // One is button pressed, Zero is button up.
  // SButtons start not pressed.
  /*
  static uint32_t SButtonStates[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  for (int i = 0; i<8; i++){
      if (((~SButtons) & (1 << i)) && (SButtonStates[i] == 0)) {
        // Random button down event
        SButtonStates[i] = 1;
        Event e = {300,0,(uint32_t)i,0};
        PushEvent(GlobalEventStack, e);
      }
      if (((SButtons) & (1 << i)) && (SButtonStates[i] == 1)) {
        // Random button up event
        SButtonStates[i] = 0;
        Event e = {301,0,(uint32_t)i,0};
        PushEvent(GlobalEventStack, e);
      }
  }*/

  // Hold the 32 button states from the left word in a single uint_32.
  // Actual button maps are accessible in kernel.h as named constants.
  //static uint64_t bigboy= 0;
  static uint64_t buttonState = 0;
  uint64_t firstPiece   = (uint64_t)(~(*GlobalButtonRamLocation));
  uint64_t secondPiece  = (uint64_t)((~(*(GlobalButtonRamLocation+1)))&0xffff);
  uint64_t currentState = firstPiece | (secondPiece << 32);

  // Todo, parcel this out into smaller chunks because its too long
  // or too often for an interrupt based on blinking pad lights.
  for (uint64_t i = 0; i<64; i++){
        uint64_t one = 1; // stupid integer promotion
      // if button is pressed, but we don't think it is, press it.
      if (((one << i) & currentState) && !((one << i) & buttonState)) {
        buttonState |= one << i;
        Event e = {300,0,(uint32_t)i,0};
        PushEvent(GlobalEventStack, e);
      }
      // if button is not pressed, but we think it is, unpress it.

      if (!((one << i) & currentState) && ((one << i) & buttonState)) {
        // Random button down event
        buttonState ^= one << i;
        Event e = {301,0,(uint32_t)i,0};
        PushEvent(GlobalEventStack, e);
      }
  }

  // Special state-machine just to handle the push-to-enter knob twisting
  // left or right. Maybe this should be refactored to a separate routine
  // for clarity.
  enum pastState{p_none = 0xff00, p_a = 0xef00, p_b = 0xe700, p_c=0xf700};
  enum currState{c_none = 0x00ff, c_a = 0x00ef, c_b = 0x00e7, c_c=0x00f7};
  enum direction{none = 0x010000, left= 0x100000, right = 0x110000};

  static pastState  p = p_none;
  static currState  c = c_none;
  static direction  d = none;
  volatile uint32_t * GPIOD_IN = (uint32_t *)0x40011408;
  volatile uint32_t val = ((*GPIOD_IN)) & 0xff;

  // Need this switch because casting blows up binary size for some reason.
  switch (val){
    case (0x00ff):
      c = c_none;
      break;
    case (0x00ef):
      c = c_a;
      break;
    case (0x00e7):
      c = c_b;
      break;
    case (0x00f7):
      c = c_c;
      break;
  }

  // State machine. Product states of past state, current state, direction.
  switch(p | c){
    case (p_none | c_a):
      p = p_a;
      d = right;
      return;
    case (p_none | c_c):
      p = p_c;
      d = left;
      return;
  }
  switch (p | c | d ) {
    case (p_a | c_b | right):
      p = p_b;
      return;
    case (p_b | c_c | right):
      p = p_c;
      return;
    case p_c | c_none | right:{
       //fire a right
      Event e = {300,91,71,0};
      PushEvent(GlobalEventStack, e);
      d = none;
      p = p_none;
      return;
    }
    case (p_c | c_b | left):
      p = p_b;
      return;
    case (p_b | c_a | left):
      p = p_a;
      return;
    case (p_a | c_none | left):{
       //fire left
      Event e = {300,91,70,0};
      PushEvent(GlobalEventStack, e);
      d = none;
      p = p_none;
      return;
    }
    default:
      // nop no changes
      if (p == (c<<8)) {return;}
      // otherwise incomplete action, reset.
      d = none;
      p = p_none;
  }

}

unsigned int decodeMPK249NoteValue(uint32_t u){
  // magic number map. The reported value of the note
  // from the keybed
  // is remapped to the midi value around middle c. 0
  // is the failure mode.
  unsigned int map[] = {0, 0, 0, 0, 0, 0, 0, 0, 56, 57, 58, 59, 60,      //  0   - C
               61, 62, 63, 0, 0, 0, 0, 0, 0, 0, 0, 0,           //  D   - 0x18
                0,  0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 36,          // 0x19 - 0x24
                37,  38, 39, 64, 65, 66, 67, 68, 69, 70, 71, 0, // 0x25 - 0x30
                0,  0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0,           // 0x31 - 0x3C
                0,  0, 0, 40, 41, 42, 43, 44, 45, 46,  47, 72,  // 0x3D - 0x48
                73, 74, 75, 76, 77, 78, 79, 0, 0, 0,  0, 0,     // 0x49 - 0x54
                0, 0,  0  , 0, 0, 0, 0, 0, 0, 0,  0, 48,        // 0x54 - 0x60
                49, 50,51 , 52, 53, 54, 55, 78, 79, 80, 81, 82,// 0x61 - 0x6C
              };
  unsigned int re = map[(u&0x0000FF00)>>8];
  return re;
}

void CheckKeyBed(){
  uint32_t * BaseKeybedEvents = (uint32_t *)0x20001a86;
  // KeyOffset is actually DMA circular buffer offset.
  uint32_t * KeyOffset        = (uint32_t *)0x40020034;
  static uint32_t KeyBufferOffset = 0x100;

  int noNewKeybedData = ((*KeyOffset & 0xff) == (KeyBufferOffset & 0xff));
  int keybedDataNotReady = ((*KeyOffset % 4) != 0);

  if (noNewKeybedData | keybedDataNotReady){return;}
  // otherwise, we try to find at least one keyboard event and add to
  // the gloval event stack.

  KeyBufferOffset= (KeyBufferOffset - 4) % 0x100;
  uint32_t event = *(BaseKeybedEvents + (0x100 - KeyBufferOffset - 4)/4);
  uint32_t val = (decodeMPK249NoteValue(event));

  uint32_t velocity = 100;

  Event e = {0xFF,100,val,0};

  if ((event & 0x000000FF) == 0xFE){
    e = {0xFE,0,val,0};
  }
  PushEvent(GlobalEventStack, e);

}

void CollectAllKeyboardSignals(){
  // Check pad states and generate events,
  // including polyphonic pressure.
  PadChecker();
  // Check if any binary buttons have been pushed, and generates
  // event on GlobalEventStack if so.
  CheckButtonsPushed();
  // Check KeyBed to see if there are any events to register
  CheckKeyBed();
}

// Basic notes from keybed to usbmidi.
void ConsumeNoteEventStackAndTransmitNotesOverUSB(){
  // This is helpful in understanding MIDI 1.0 UMP
  // https://imitone.com/midi2/
  // clear interrupts

  uint32_t * USB_EP1R = (uint32_t *)0x40005c04;
  USBEndpointState state = USBEndpointState{*USB_EP1R, 0, 0, 0, NAK, VALID, INTERRUPT, 1 };

  // TODO: Period collecting of all keyboard signals needs to be
  // refactored to a timer interrut.
  CollectAllKeyboardSignals();

  if (IsEventStackEmpty(GlobalEventStack)){
    state = USBEndpointState{*USB_EP1R, 0, 0, 0, NAK, VALID, INTERRUPT, 1 };
    btable * b = (btable *) 0x40006010;
    b->count_tx= 0; // Transmit nothing.
    // Clear interrupts.
    USB->ISTR = 0;
    SetRegister(USB_EP1R, setEndpointState(state));
    return;
  }
  // So a new note has been played.
  uint32_t event = 0;
  unsigned int val=0;
  int velocity = 99;
  uint32_t noteChannel = 121 | (144 << 8);// Note on, Channel 1, M type 0x2, group 0
  //uint32_t noteChannel = 121 | (0b10100011 << 8);// Note aftertouch, Channel 1, M type 0x2, group 0


    Event e = PopEvent(GlobalEventStack);
    // Careful! the Event you popped may not have note data then you are
    // transmitting spurious info.
    // Really, you should peak up above and see if the event
    // is consumable by USB, and act like nothing is there if
    // it is not.

    // button down
    if (e.A == 300){
      val = 0 + e.C;
      velocity = 88;
    }
    // button up
    if (e.A == 301){
       val = 0 + e.C;
       noteChannel = 121 | (128 << 8);// Note off, Channel 1, M type 0x2, group 0
    }
    // keybed key down
    if ((e.A)== 0xFF){
      velocity = e.B;
      val = e.C;
      noteChannel = 121 | (144 << 8); // Note on.
    }
    // keybed key up
    if ((e.A)== 0xFE){
      velocity = e.B;
      val = e.C;
      noteChannel = 121 | (128 << 8);// Note off, Channel 1, M type 0x2, group 0
    }

    // Pad On
    if ((e.A)== 900){
      velocity = e.B;
      val = e.C;
      noteChannel = 121 | (144 << 8); // Note on.
    }
    //0b1111001
    // Pad Off
    if ((e.A)== 901){
      velocity = e.B;
      val = e.C;
      noteChannel = 121 | (128 << 8);// Note off, Channel 1, M type 0x2, group 0
    }
    // Pad Polyphonic pressure.
    if ((e.A)== 905){
      velocity = e.B;
      val = e.C;
      noteChannel = 121 | (0b10100001 << 8); // note on channel pressure.
    }
    // Pitch wheel
    if ((e.A)== 200){
      velocity = e.B;
      val = e.C;
      noteChannel = 121 | (0b11100000 << 8); // pitchbend chan 1.
    }


  // 144 120 100
  // FE is up
  uint32_t * PMAWrite = (uint32_t *)0x40006060;
  *PMAWrite= noteChannel;
  PMAWrite++;
  *PMAWrite =  val | velocity << 8 ; // Note val, velocity

  btable * b = (btable *) 0x40006010;
  b->count_tx= 4;
  // Clear interrupts.
  USB->ISTR = 0;
  SetRegister(USB_EP1R, setEndpointState(state));
  return;
}

void usb(){
  // Compile time allocation and inlining of Device Descriptor.
  constexpr PMAAllocation deviceDescriptor = NewDeviceConfiguration(dev_descriptor);

  constexpr PMAAllocation mouseTry = mouseConfigToPMA(mouseConfiguration);
  constexpr PMAAllocation mouseHID = NewMouseHIDReportToPMA(mouseHIDDescriptor);

  constexpr PMAAllocation USBMidi  = NewUSBMIDIToPMA(USBMidiOneOutConfig);

  // Message tracking variables to exist for lifetime of program.
  static int messagePosition = 0;
  static int messageSize = 0;
  static const PMAAllocation * currentMessage = 0;

  // Most of this is for enumeration on Endpoint 0. If we have other requests,
  // forward them along.
  if ((USB->ISTR &0b1111) == 1 ){
    ConsumeNoteEventStackAndTransmitNotesOverUSB();
    //uint32_t * USB_EP1R = (uint32_t *)0x40005c04;
    //USBEndpointState state = USBEndpointState{*USB_EP1R, 0, 0, 0, NAK, VALID, INTERRUPT, 1 };
    //SetRegister(USB_EP1R, setEndpointState(state));
    return;
  }

  // Check if I need to reset
  if ((USB->ISTR &0b10000000000) > 0 ){
    //writeString((char*)"  Re ");
    usbReset();
    return;
  }
  // Joking around with ISTR handlers.
  volatile uint32_t * val = (uint32_t *)0x40006040;

  // THIS NEEDS TO BE VOLATILE
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
      currentMessage = &USBMidi;
      messagePosition = 0;
      messageSize = 5; // To examine
      // THIS IS VERY STUPID. We shouldn't need to know messageSize here.
      // A data type should encapsulate all this information.
      if (configured) messageSize = 51;
      configured = 1;
    }

    // Get HID Report Descriptor for a USB Mouse.
    if ((USBDeviceAddress() != 0) & (*val == 0x0681) & (*(val+1) == 0x2200))
    {
      currentMessage = &mouseHID;
      messagePosition = 0;
      messageSize = 26; // To examine
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


      volatile uint32_t * PMAWrite = (uint32_t *)0x40006050;
      volatile btable * b = (btable *) 0x40006000;
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
          // TODO this is a general issue for all configuration requests.
          if (( currentMessage == &USBMidi) & (messageSize == 5) ){
            b -> count_tx = 1;
          } else if (currentMessage == &USBMidi){
            b -> count_tx = 5; // THIS IS SO FRAGILE AND WRONG.
          }
          *logSigil = 0xFFFF;
          logSigil++;
          *logSigil = b-> count_tx;
          logSigil++;
          messagePosition++; // To indicate status.
          break;
        }
        *PMAWrite = currentMessage -> a[messagePosition];
        if ( (messagePosition == 4) & ( currentMessage == &USBMidi) & (messageSize == 5) ){
          *PMAWrite = 0;
        }
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
  int debug = 0;
  volatile uint32_t * val = (uint32_t *)0x40006040;
  // This logs all the values every time the
  // interrupt was called, starting with
  // initial value 0xdeadbeef at
  // Don't log an event if it's the same as the last one.

  uint32_t EP0StatusBefore = USB -> EP0R;
  usb();
  uint32_t EP0StatusAfter  = USB -> EP0R;

  if (debug){
    *record = *val;
    *(record+1) = *(val + 1);
    *(record+2) = EP0StatusBefore;
    *(record+3) = EP0StatusAfter ;
    *(record+4)= 0xdeadbeef;
    record = record + 4;
  }
  return;
}

}

// USART Setup is to prepare USART
// To register black and white key signals from the bottom of the keyboard.
void USARTSetup(){

  // Set up TIM4
  volatile uint32_t * TIM4_CR1 = (uint32_t *) 0x40000800;
  volatile uint32_t * TIM4_SR  = (uint32_t *) 0x40000810;
  volatile uint32_t * TIM4_CCMR_IN1 = (uint32_t *)0x40000818;
  volatile uint32_t * TIM4_CCER    = (uint32_t *)0x40000820;
  volatile uint32_t * TIM4_ARR = (uint32_t *)0x4000042c;

  *TIM4_CR1 = 1;
  *TIM4_SR  = 0x1f;
  *TIM4_CCMR_IN1 = 0x30;
  *TIM4_CCER = 1;
  *TIM4_ARR = 0xb;

// Enable DMA between USART3 and RAM
// So note values will be instantly recorded in circular buffers.
volatile uint32_t * DMA1_IFCR = (uint32_t *)   0x40020004;
volatile uint32_t * DMA1_CCR3 = (uint32_t *)   0x40020030;
volatile uint32_t * DMA1_CPAR = (uint32_t *)   0x40020038;
volatile uint32_t * DMA1_CMAR = (uint32_t *)   0x4002003c;
volatile uint32_t * DMA1_CNDTR = (uint32_t *)  0x40020034;
// Clear all DMA issues.
*DMA1_IFCR = 0xFFFFFFFF;
*DMA1_CPAR = 0x40004804;
*DMA1_CMAR = 0x20001a86; // Some spot in ram is dest.
*DMA1_CNDTR = 0x100;
// Circular buffer, high priority.
*DMA1_CCR3 = 0x000030a1; // Must be inited last.

volatile uint32_t * USART3_CR1 = (uint32_t *) 0x4000480c;
volatile uint32_t * USART3_CR2 = (uint32_t *) 0x40004810;
volatile uint32_t * USART3_CR3 = (uint32_t *) 0x40004814;
volatile uint32_t * USART3_BAUD= (uint32_t *) 0x40004808;
*USART3_CR1  = 0b10000000000000;
delay(1);
*USART3_CR1 |= 0b1000000000000;
*USART3_CR3 |= 0x40;
*USART3_CR2 = 0;
*USART3_BAUD= 0x34;
delay(1);
*USART3_CR1   = 0x340c;
delay(1);
GPIOD -> CRH = 0x422a444b;
GPIOD -> BSRR= 0x20fc;

return;
}

// ADC is used to sample voltages for the top of the Keyboard.
// to retrieve signals, including position of fader, knob, modulation
// and pitch wheel.
//
// The voltages are multiplexed by the value in GPIOE_ODR.
void ADCSetup(){
  // Set Clock CONFIGURATION
  // We expect this is done in USART Setup already.
  // Set DMA1 clock.
  // We expect this is done in USART Setup already.

  // Configure ADC1 (analog to digital converter)
  volatile uint32_t * RCC_APB2ENR = (uint32_t *) 0x40021018;
  *RCC_APB2ENR = 0x427d;

  // Configure DMA1_1 to read off ADC1 values.
  volatile uint32_t * DMA1_CNDTR = (uint32_t *) 0x4002000c;
  volatile uint32_t * DMA1_CPAR = (uint32_t *) 0x40020010;
  volatile uint32_t * DMA1_CMAR = (uint32_t *) 0x40020014;
  volatile uint32_t * DMA_CCR1 = (uint32_t *) 0x40020008;

  *DMA1_CNDTR = 0x10;
  // Copy from ADC1
  *DMA1_CPAR  = 0x4001244c;
  // Copy to RAM
  *DMA1_CMAR  = 0x20002c96;
  // Circular buffer
  *DMA_CCR1= 0x25a1;


  volatile uint32_t * ADC1_SR     = (uint32_t *) 0x40012400;
  volatile uint32_t * ADC1_CR1    = (uint32_t *) 0x40012404;
  volatile uint32_t * ADC1_CR2    = (uint32_t *) 0x40012408;
  volatile uint32_t * ADC1_SMP2   = (uint32_t *) 0x40012410;
  volatile uint32_t * ADC1_WatchH = (uint32_t *) 0x40012424;
  volatile uint32_t * ADC1_SQR1   = (uint32_t *) 0x4001242c;
  volatile uint32_t * ADC1_SQR2   = (uint32_t *) 0x40012430;
  volatile uint32_t * ADC1_SQR3   = (uint32_t *) 0x40012434;

  *ADC1_SR =  0x10;
  //*ADC1_CR1 = 0x100;
  *ADC1_CR1 = 0x120; // Drive an interrupt at the end of each conversion.
  *ADC1_CR2 = 0x1e0103;
  *ADC1_SMP2= 0x006db6db;
  *ADC1_WatchH= 0xfff;
  *ADC1_SQR1 = 0x00700000;
  *ADC1_SQR2 = 0xe6;
  *ADC1_SQR3 = 0x0a418820;

  // Software trigger to start ADC1.
  delay(1);
  *ADC1_CR2 = 0x5E0103;

  // GPIOE to pick initial group.
  volatile uint32_t * GPIOE_CRL   = (uint32_t *)0x40011800;
  volatile uint32_t * GPIOE_CRH   = (uint32_t *)0x40011804;
  volatile uint32_t * GPIOE_ODR   = (uint32_t *)0x4001180c;

  *GPIOE_CRL = 0x22222222;
  *GPIOE_CRH = 0x22224248;
  *GPIOE_ODR = 0x513a;

  // Clear Keyboard state.
  KeyboardButtons -> PadsToModify = 0;
  KeyboardButtons -> WhichPadsAreOn=0;
  uint32_t * clearRam  = (uint32_t *) KeyboardButtons;
  for (int i = 0; i < 150; i++){
    clearRam = 0;
    clearRam++;
  }
  return;
}

// SPI2 Setup
void SPISetup(){
  // Set Clock CONFIGURATION
  // We expect this is done in USART Setup already.
  // Set DMA1 clock.
  // We expect this is done in USART Setup already.
  // Configure GPIOE so pad LEDS are on
  // We expect this is done already.
  volatile uint32_t * GPIOB_CRL   = (uint32_t *)0x40010c00;
  volatile uint32_t * GPIOB_CRH   = (uint32_t *)0x40010c04;
  *GPIOB_CRL = 0x44b84222;
  *GPIOB_CRH = 0xa8a22444;

  volatile uint32_t * SPI2_CR1   = (uint32_t *)0x40003800;
  volatile uint32_t * SPI2_X     = (uint32_t *)0x40003804;

  *SPI2_CR1 = 0x0000037c;
  //*SPI2_X   = 0b111;
  *SPI2_X   = 0b1000111;
}

// Pad color frame stuff.

// Each Frame of colors to send to SPI will take up a word, just to
// make alignment prettier. Really, only 0xa bytes are sent.
static uint32_t RED = 0b001;
static uint32_t B   = 0b100;
static uint32_t G   = 0b010;

// Actual Frame that will be written to RAM and sent to SPI.
struct Frame {
  uint32_t one;
  uint32_t two;
  uint32_t three;
  uint32_t four;
};

// Configuration data to be converted to a packed 4 word thing.
struct FrameData {
  uint32_t Pads[16];
  // We'll eat the lower three bytes from the word here and pack it
  // into the frame.
  uint32_t otherbuttons;
};

constexpr Frame MakeFrame (FrameData fd){
  struct Frame f = Frame{};
  // Start byte packing.
  f.one =
  ((~fd.Pads[0])&0b111) << 8  |
  ((~fd.Pads[1])&0b111) << 11 |
  ((~fd.Pads[2])&0b111) << 14 |
  ((~fd.Pads[3])&0b111) << 17 |
  ((~fd.Pads[4])&0b111) << 20 |
  ((~fd.Pads[5])&0b111) << 23 |
  ((~fd.Pads[6])&0b111) << 26 |
  ((~fd.Pads[7])&0b111) << 29;

  f.two =
  ((~fd.Pads[8])&0b111) <<  0  |
  ((~fd.Pads[9])&0b111) <<  3  |
  ((~fd.Pads[10])&0b111) << 6  |
  ((~fd.Pads[11])&0b111) << 9  |
  ((~fd.Pads[12])&0b111) << 12 |
  ((~fd.Pads[13])&0b111) << 15 |
  ((~fd.Pads[14])&0b111) << 18 |
  ((~fd.Pads[15])&0b111) << 21;

  return f;
}

void SetupPadColorFrames(){
    FrameData fd1 = FrameData{{RED,B,G,RED|B,RED|G,RED,B,G,RED|B,RED|G,RED,B,G, RED|B, RED|G, RED},0};
    struct Frame f1 = MakeFrame(fd1);
    FrameData fd2 = FrameData{{B,  B,G,RED|B,RED|G,RED,B,G,RED|B,RED|G,RED,B,G, RED|B, RED|G, RED},0};
    struct Frame f2 = MakeFrame(fd2);
    FrameData fd3 = FrameData{{B,  B,G,RED|B,RED|G,RED,B,G,RED|B,RED|G,RED,B,G, RED|B, RED|G, RED},0};
    struct Frame f3 = MakeFrame(fd3);


    uint32_t * info = (uint32_t *)0x200019d0;
    info[0] = f1.one;
    info[1] = f1.two;
    info[2] = 0xffffffff;

    info= (uint32_t *)0x200019e0;
    info[0] = f2.one;
    info[1] = f2.two;
    info[2] = 0xffffffff;

    info= (uint32_t *)0x200019f0;
    info[0] = f3.one;
    info[1] = f3.two;
    info[2] = 0xffffffff;
}


// We use TIM2 to circulate pad colors.
// TODO: Make TIM2 a generic "do this later"
// function which takes a fptr and a ticks variable,
// counts down the ticks until it hits 0 and then fires
// fptr.
void TIM2Setup(){
  TIM2 -> CR1 = 0x81;
  TIM2 -> SR  = 0;
  TIM2 -> PSC = 7200-1; // milisecond because STM32f103 is clocked at 72 MHZ
  TIM2 -> CCMR1 = 0x00006800;
  TIM2 -> CCER = 0x10;
  TIM2 -> ARR  = 5; // every 5 miliseconds
  TIM2 -> DIER = 1;
  //TIM3 -> CCR2 = 0x00000155; // This is the contrast value.
  //TIM3 -> DMAR = 0x00000081;

}

// This handles both pad color animation and
// fetching button state from SPI2.
extern "C" void TIM2_IRQHandler(){
  volatile uint32_t * DMA1_4CCR    = (uint32_t *)0x40020044;
  volatile uint32_t * DMA1_4CPAR   = (uint32_t *)0x4002004c;
  volatile uint32_t * DMA1_4CMAR   = (uint32_t *)0x40020050;
  volatile uint32_t * DMA1_4CNDT   = (uint32_t *)0x40020048;

  volatile uint32_t * DMA1_5CCR    = (uint32_t *)0x40020058;
  volatile uint32_t * DMA1_5CPAR   = (uint32_t *)0x40020060;
  volatile uint32_t * DMA1_5CMAR   = (uint32_t *)0x40020064;
  volatile uint32_t * DMA1_5CNDT   = (uint32_t *)0x4002005c;

  volatile uint32_t * GPIOB_ODR    = (uint32_t *)0x40010c0c;

  uint32_t Frame1       = 0x200019d0;
  uint32_t Frame2       = 0x200019e0;
  uint32_t Frame3       = 0x200019f0;

  static int i = -1;
  i++;

  if (i == 0){

// DMA off
  *DMA1_5CCR = 0x3190;
// Hook to SPI2 DR
  *DMA1_5CPAR= 0x4000380c;

// Transmit 0xa bits to SPI2
  *DMA1_5CNDT = 0xa;


  *DMA1_4CCR  = 0x1080;
  *DMA1_4CMAR = 0x20000c00;
  *DMA1_4CPAR = 0x4000380c;
  *DMA1_4CNDT = 0x6;
// Then turn back on 4 and wait for 5
    static int j = 0;
    switch(j) {
    case 0  :
       *DMA1_5CMAR = Frame1;
       break; /* optional */
    case 1  :
       *DMA1_5CMAR = Frame2;
       break;
    case 2  :
       *DMA1_5CMAR = Frame3;
       break;
    default :
    j = -1;
    }
    j++;


  //setGPIOB11and12(0b01);
  *GPIOB_ODR = 0x1010;
    TIM2 -> SR  = 0;
    return;
  }
  if (i == 1){
  *GPIOB_ODR = 0x1810;
    TIM2 -> SR  = 0;
  return;
  }
  //setGPIOB11and12(0b11);

  if (i == 2){
    *DMA1_4CCR  = 0x1081;
    *DMA1_5CCR =  0x3191;
      TIM2 -> SR  = 0;
    return;
  }

  if (i == 3){
    *GPIOB_ODR = 0x810;
      TIM2 -> SR  = 0;
    return;
  }

  if (i == 4){
      *GPIOB_ODR = 0x1810;
        TIM2 -> SR  = 0;
      return;
  }
  if (i == 5){
    i = -1;
      TIM2 -> SR  = 0;
    return;
  }

}

enum AFIOOptions{AFIO_PartialRemap_TIM3 = 0b100000000000, AFIO_Remap_TIM4=0b1000000000000,
AFIO_FullRemap_USART3 = 0b110000 };

void EnableAFIO(AFIOOptions o){
  volatile uint32_t * AFIO_MAPR = (uint32_t *)0x40010004;
  *AFIO_MAPR |= o;
}

enum APB2Options{APB2_AFIO,APB2_nop1,APB2_IOA, APB2_IOB, APB2_IOC, APB2_IOD,
APB2_IOE, APB2_IOF, APB2_IOG, APB2_ADC1, APB2_ADC2, APB2_TIM1, APB2_SPI1,
APB2_TIM8, APB2_USART, APB2_ADC3, APB2_nop2, APB2_nop3, APB2_nop4,
APB2_TIM9, APB2_TIM10, APB2_TIM11};

void EnableAPB2(APB2Options o){
  volatile uint32_t * RCC_APB2ENR = (uint32_t *)0x40021018;
  uint32_t option = 1 << o;
  *RCC_APB2ENR |= option;
}

enum APB1Options{APB1_TIM2,APB1_TIM3,APB1_TIM4,APB1_TIM5,APB1_TIM6,
APB1_TIM7,APB1_TIM12,APB1_TIM13, APB1_TIM14, ABP1_nop1, ABP1_nop2,
APB1_WWD, APB1_nop4, APB1_nop5, APB1_SPI2, APB1_SPI3, APB1_nop6,
APB1_USART2, APB1_USART3, APB1_UART4, APB1_UART5, APB1_I2C1, APB1_I2C2,
APB1_USB, APB1_nop7, APB1_CAN, APB1_nop8, APB1_BKP, APB1_PWR, APB1_DAC};

void EnableAPB1(APB1Options o){
  volatile uint32_t * RCC_APB1ENR = (uint32_t *)0x4002101c;
  uint32_t option = 1 << o;
  *RCC_APB1ENR |= option;
}

int start() {
    // Push it forward
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x6000);


    //NVIC structure to set up NVIC controller
    //NVIC_InitTypeDef NVIC_InitStructure;
    //NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn ;

    // enable clock on APB2
    //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,  ENABLE);
    //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,  ENABLE);
    //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,  ENABLE);
    //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD,  ENABLE);
    EnableAPB2(APB2_IOA);
    EnableAPB2(APB2_IOB);
    EnableAPB2(APB2_IOC);
    EnableAPB2(APB2_IOD);

    //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE,  ENABLE);

   // The three things you need to do to hook up Tim3_ch2
   // PortB5 output. Where the LCD potentiometer is connected.
    //RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    //RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    EnableAPB1(APB1_TIM3);
    EnableAPB1(APB1_TIM4);

    //RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    EnableAPB2(APB2_AFIO);
    // Enable SPI2 for sending colors to top of keyboard.
    //RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
    EnableAPB1(APB1_SPI2);
    //GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);
    EnableAFIO(AFIO_PartialRemap_TIM3);
    //GPIO_PinRemapConfig(GPIO_Remap_TIM4, ENABLE);
    EnableAFIO(AFIO_Remap_TIM4);

    // Turn on USART3 for keybed signals routing.
    //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD,  ENABLE);//?
    //RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    EnableAPB1(APB1_USART3);
    // Enable DMA1 clock.
    RCC->AHBENR |= (1 << 0);


    GPIOB -> CRH = 0xa8a22444;
    //GPIOD -> CRH = 0x422a444b;
    //GPIO_PinRemapConfig(GPIO_FullRemap_USART3, ENABLE);
    EnableAFIO(AFIO_FullRemap_USART3);
    //GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE);
    // Init GlobalEventStack
    GlobalEventStack->top=-1;
    GlobalEventStack->capacity = 100;
    // Set up USART for bottom keybed.
    USARTSetup();
    // Set up Analog to digital voltage converter to read top of keyboard.
    ADCSetup();
    // Finally, set up SPI2 to send rgb and on off color signals to pads
    // and buttons.
    SPISetup();

    copyPatterns();

    // LCD logic
    initDisplay();
    contrast();
    writeString((char*)"  I <3 Gretchen  ");

    // usb init
    //RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
    EnableAPB1(APB1_USB);
    //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD,  ENABLE);
    NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn );


    // Enable ADC interrupt


    usbReset();
    // Allow some time for USB to attempt to connect.
    //delay(50);

     // Init SPI2
    // Kick off SPI2 first time.
  	//DMA1_Channel5_IRQHandler();
    //NVIC_EnableIRQ(	DMA1_Channel5_IRQn);
    SetupPadColorFrames();
    //RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    EnableAPB1(APB1_TIM2);
    //volatile uint32_t * RCC_APB1ENR = (uint32_t *)0x4002101c;
    //*RCC_APB1ENR |=1;
    TIM2Setup();
    NVIC_EnableIRQ(TIM2_IRQn );


    while(1){

    };

}

#endif
