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
  uint32_t controlmask = 0b00000000000000000000000000000110;
  //GPIOB->BSRR = controlbits | (~controlbits & controlmask) << 16;
  GPIOB -> ODR = controlbits;

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
  		5, // bInterval Interrupt polling interval in ms
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
  		10,   // bInterval in ms
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

// Test function to move mouse to the right
void usb_ep1(){
  int notes[]= {60, 64, 67, 59, 62, 67};
  static int note = 0;
  // clear interrupts
  uint32_t * USB_EP1R = (uint32_t *)0x40005c04;
  USBEndpointState state = USBEndpointState{*USB_EP1R, 0, 0, 0, NAK, VALID, INTERRUPT, 1 };
  USB->ISTR = 0;

  static int i = 0;
  i++;
  if ((i%2) != 0){
    state = USBEndpointState{*USB_EP1R, 0, 0, 0, NAK, VALID, INTERRUPT, 1 };
    btable * b = (btable *) 0x40006010;
    b->count_tx= 0; // Transmit nothing.
    SetRegister(USB_EP1R, setEndpointState(state));
    return;
  }

  uint32_t * BaseKeybedEvents = (uint32_t *)0x20001a86;
  // KeyOffset is actually DMA circular buffer offset.
  uint32_t * KeyOffset        = (uint32_t *)0x40020034;
  static uint32_t KeyBufferOffset = 0x100;
  uint32_t event = *(BaseKeybedEvents + (0x100 - (*KeyOffset) - 4)/4);
  //event = *BaseKeybedEvents;

  if (*KeyOffset != KeyBufferOffset){
    writeNumberToScreen(event);
    KeyBufferOffset = *KeyOffset;
  }

  // 144 120 100
  uint32_t * PMAWrite = (uint32_t *)0x40006060;
  *PMAWrite = 121 | (144 << 8); // Chan 1 Note on
  PMAWrite++;
  //*PMAWrite = 120; // note 120
  //PMAWrite++;
  *PMAWrite = (notes[note%6] + (note/6)) | (100 << 8); // note velocity
  btable * b = (btable *) 0x40006010;
  b->count_tx= 3;
  SetRegister(USB_EP1R, setEndpointState(state));
  note++;
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
    usb_ep1();
    return;
  }

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
  uint32_t * val = (uint32_t *)0x40006040;
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

void USARTSetup(){

  // Set up system clocks.
  //uint32_t * RCC_CNFGR = (uint32_t *) 0x40021004;
  //  uint32_t * RCC_CR    = (uint32_t *) 0x40021000;

  // Make sure PLL is disabled before you try to write
  // config values. See p. 102 in manual.
  //*RCC_CR    = 0;
  //*RCC_CNFGR = 0x11440a;
  //*RCC_CR    = 0x3035583;

  // Set up TIM4
  uint32_t * TIM4_CR1 = (uint32_t *) 0x40000800;
  uint32_t * TIM4_SR  = (uint32_t *) 0x40000810;
  uint32_t * TIM4_CCMR_IN1 = (uint32_t *)0x40000818;
  uint32_t * TIM4_CCER    = (uint32_t *)0x40000820;
  uint32_t * TIM4_ARR = (uint32_t *)0x4000042c;

  *TIM4_CR1 = 1;
  *TIM4_SR  = 0x1f;
  *TIM4_CCMR_IN1 = 0x30;
  *TIM4_CCER = 1;
  *TIM4_ARR = 0xb;

  /*
  # SET CLOCKS
  # RCC_CNFGR
  set {int} 0x40021004 = 0x11440a
  # RCC_CR
  set {int} 0x40021000 = 0x3035583

  # Enable DMA1 Clock
  # RCC->AHBENR = 1
  set {int}0x40021014= *0x40021014 | 1
  # Enable USART3 Clock
  # RCC_APB1ENR bit 18
  set {int}0x4002101c= *0x4002101c | (1 << 18)
  # Enable TIM4 Clock
  # RCC_APB1ENR bit 2
  set {int}0x4002101c= *0x4002101c | (1 << 2)
  # Clock AFIO_MAPR, and clock USART1, GPIOs
  set {int}0x40021018 = 0x427d
  # Correct Vals for RCC_CFGR

  # ENABLE DMA1
  # Set CPAR to USART3_DR
  set {int} 0x40020038= 0x40004804
  # Set CMAR to spot in RAM
  set {int} 0x4002003c= 0x20001a86
  # Set CNDR (buffer length)
  set {int}0x40020034=0x100
  # DMA1_CCR3
  # Circular buffer, High priority
  # Set last for DMA
  set {int} 0x40020030= 0x000030a1

  # ENABLE TIM4
  # display/x "TIM4_CR1", *0x40000800
  set {int} 0x40000800 = 1
  # display/x "TIM4_SR ", *0x40000810
  set {int} 0x40000810= 0x1f
  # display/x "CCMR_IN1", *0x40000818
  set {int} 0x40000818 = 0x30
  # Set CCER
  set {int} 0x40000820 = 0x1
  # display/x "TIM4_ARR", *0x4000042c
  set {int} 0x4000042c = 0xb

  # ENABLE USART3
  # FullREMAP USART3 and Full remap TIM4
  set{int} 0x40010004 = 0x1830

  #USART3_CR1 configure but don't start
  #set {int}0x4000480c= 0x140c

  # ENABLE GPIOB
  #Turn on GPIOB clock
  set {int}0x40021018 = *0x40021018 | 0b1000
  #GPIOB_low
  set {int}0x40010c00 = 0x44b84222
  #GPIOB IDR
  set {int}0x40010c08=0xffd0
  #GPIOB ODR
  set {int}0x40010c0c=0x10

  # ENABLE GPIOD
  # Turn on GPIOD clock d
  set {int}0x40021018 = *0x40021018 | 0b100000
  # GPIOD ODR
  #set {int}0x4001140c= 0xe0
  # GPIOD IDR
  #set {int}0x40011408= 0x7ffb
  # GPIOD Config_HIGHBITS
  #set {int}0x40011404= 0x422a444b

  #Enable GPIOA
  # Turn on GPIOA clock
end

  From the STM32 F103 Manual.

1. Enable the USART by writing the UE bit in USART_CR1 register to 1.

0x00000040

40004814 is USART3_cr3
What is UE bit.
2. Program the M bit in USART_CR1 to define the word length.
What is
3. Program the number of stop bits in USART_CR2.
4. Select DMA enable (DMAT) in USART_CR3 if Multi buffer Communication is to take
place. Configure the DMA register as explained in multibuffer communication.
5. Select the desired baud rate using the USART_BRR register.
*/
// 40004800 STATUS REGISTER FOR USART3

/*

#USART3_CR1
set {int}0x4000480c= 0b10000000000000
shell sleep .001
# M bit
set {int}0x4000480c |= 0b1000000000000
#USART3_CR3 DMA enable
set {int}0x40004814= 0x40
shell sleep .001
#USART_BAUD
set{int} 0x40004808 = 0x34
shell sleep .001
#USART enable TE and RE
set {int}0x4000480c |= 0x40c
# GPIOD Config_HIGHBITS
set {int}0x40011404= 0x422a444b
shell sleep .001
# GPIOD ODR
set {int}0x4001140c= 0x20fc
# GPIOD IDR
set {int}0x40011408= 0xbfff

*/

// Enable DMA between USART3 and RAM
// So note values will be instantly recorded in circular buffers.
uint32_t * DMA1_CCR3 = (uint32_t *)   0x40020030;
uint32_t * DMA1_CPAR = (uint32_t *)   0x40020038;
uint32_t * DMA1_CMAR = (uint32_t *)   0x4002003c;
uint32_t * DMA1_CNDTR = (uint32_t *)  0x40020034;
*DMA1_CPAR = 0x40004804;
*DMA1_CMAR = 0x20001a86; // Some spot in ram is dest.
*DMA1_CNDTR = 0x100;
// Circular buffer, high priority.
*DMA1_CCR3 = 0x000030a1; // Must be inited last.

uint32_t * USART3_CR1 = (uint32_t *) 0x4000480c;
uint32_t * USART3_CR2 = (uint32_t *) 0x40004810;
uint32_t * USART3_CR3 = (uint32_t *) 0x40004814;
uint32_t * USART3_BAUD= (uint32_t *) 0x40004808;
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
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD,  ENABLE);

   // The three things you need to do to hook up Tim3_ch2
   // PortB5 output. Where the LCD potentiometer is connected.
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_TIM4, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    // output push-pull mode
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   // highest speed
    GPIO_Init(GPIOE, &GPIO_InitStructure) ;             // initialize PORT

    //Turn on LCD backlight
    GPIO_SetBits(GPIOE, 0x1000);

    // Turn on USART3 for keybed signals routing.
    //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD,  ENABLE);//?
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    // Enable DMA1 clock.
    RCC->AHBENR |= (1 << 0);


    GPIOB -> CRH = 0xa8a22444;
    //GPIOD -> CRH = 0x422a444b;
    GPIO_PinRemapConfig(GPIO_FullRemap_USART3, ENABLE);
    //GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE);
    USARTSetup();

    copyPatterns();

    // LCD logic
    initDisplay();
    contrast();
    writeString((char*)"  OPEN Kave  ");

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
