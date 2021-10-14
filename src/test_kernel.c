#include <assert.h>
#include <string.h>
#include <array>
#include "compile_time_test_library.h"
// The file we're testing.
#include "kernel.c"

template<uint32_t i>
__attribute__((noinline))
__attribute__((error("condition not guaranteed")))
static constexpr void actually_got(void) {abort(); } // must define a side effect, to not be optimized away

#define EXPECT(condition, got){if (condition != 1){actually_got<got>();};}

// Mock USB Endpoint Register for the STM32F103.
//
// Try to set it to given values, and we
// return the documented behavior of the
// endpoint register.
constexpr uint32_t mockUSBEndpoint(uint32_t reg, uint32_t in){
  uint32_t a[][3]=
  {
    //{register state,         input,         mocked_output}
    {0b1110101001100000, 0b0000001000010000, 0b0110101001110000},
    //0x000062a0       ,          ?         // 0x00007220
  // 0b0000101000010000
  // 0b0110001001110000

  // 0b0110101001110000
    {2,3,4},
    {4,5,6}
  };
  int size = sizeof(a)/sizeof(a[0]);
   for (int i=0; i<size; i++){
     if ((reg == a[i][0]) && (in ==a[i][1])) return a[i][2];
   }
   // If you got here, that means we didn't mock your register state +
   // input pair, so should compiler panic.
   actually_got<0>();
   return 0;
}

void TestGetRegisterUpdateValue(){
  // Test GetRegister
  // So say we have this kind of register, with these initial values.
  //rw t  t  t  rw  rw  rw  rw   [reg]
  // currentVal
  //1  1  0  1  0   1   0   1
  constexpr struct Register testR1 =  Register{0b01110000, 8};
  constexpr uint32_t currentVal1 = 0b11010101;
  // So say we want testR1 to read
  // update
  // 0  0  1  1  0   1   1   0 with change
  // mask
  // 0  0  1  1  0   1   1   1.
  //
  // We would expect to need to set this type of register to value
  // 1  1  1  1  0   1   1   0
  // 0  0  1  1  0   0   0   0
  //result?
  // 0  1  1  0   0  1   1   0
  constexpr uint32_t update = 0b00110110;
  constexpr uint32_t mask   = 0b00110111;
  constexpr uint32_t test1 = GetRegisterUpdateValue(testR1, currentVal1, update, mask);
  EXPECT((test1==0b10100110),test1);
  //EXPECT((test1==0b010100110),test1);
              //   010100111
  // Ok now look at the actual USB register.
  constexpr struct Register usbendpoint =  Register{0b0111000001110000, 16};
  // So say you just received a packet and both endpoints are set to NAK.
  constexpr uint32_t currentVal2     = 0b1110101001100000;
  constexpr uint32_t mask2           = 0b1000000000110001;
  // So you want to turn off CTR_RX, set TX to receive, and set addy to 1.
  constexpr uint32_t update2         = 0b0000000000110001;
  //                          So you'd need
  //                              0b00000101000010001
  constexpr uint32_t test2 = GetRegisterUpdateValue(usbendpoint, currentVal2, update2, mask2);
  EXPECT((test2==0b00000101000010001),test2);
}

void TestUSBFunctions(){
  // Let's say an endpoint register started as
  constexpr uint32_t ep = 0x0000ea60;
  auto constexpr status = GetEndpointState(ep);
  EXPECT((status.STAT_RX == NAK), status.STAT_RX);
  EXPECT((status.STAT_TX == NAK), status.STAT_TX);
  EXPECT((status.isSetupToken > 0 ), status.isSetupToken);
  // Cool. Let's form a reply.
  auto constexpr reply = USBEndpointState{ep, 0, 0, 0, NAK, VALID, CONTROL, 0 };
  auto constexpr update= setEndpointState(reply);
  EXPECT((update == 0b1000010000 ), update);
  // Ok now we 'set' the mock, and decode what we got and test its properties.
  auto constexpr result = mockUSBEndpoint(ep, update);
  auto constexpr finalStatus = GetEndpointState(result);
  EXPECT((finalStatus.STAT_RX == NAK), finalStatus.STAT_RX);
  EXPECT((finalStatus.STAT_TX == VALID), finalStatus.STAT_TX);
  EXPECT((finalStatus.isOutOrSetupToken == 0 ), finalStatus.isOutOrSetupToken);
  EXPECT((finalStatus.isInToken == 0 ), finalStatus.isInToken);
}

void TestPMACopy(){
  constexpr const uint32_t ram[] = {0x8006CDAB, 0xDEADBEEF, 0xFEEDBEEF};
  // We expect that it will take 6 PMA slots to consume src RAM.
  constexpr auto filled = TestFill<6>(toPMA, ram);
  constexpr auto got = [](){return filled;};
  constexpr auto want =
    [](){return ToArray({0x0680, 0xABCD, 0xADDE, 0xEFBE, 0xEDFE, 0xEFBE});};
  //  compile time compare arrays
  //  crash verbosely if not equal
  //constexpr int test = compareArrays(got, want);

  // Gotta be static so it's only defined once.
  static int test = compareArrays(got, want);
  // Just for unused variable warning.
  if (test) {};
}

void TestPMAIterators(){
  constexpr PMAAllocation deviceDescriptor = NewDeviceConfiguration(dev_descriptor);
  constexpr Iterator deviceDescriptorIterator = NewIterator(deviceDescriptor, 9);
  constexpr auto v1 = AdvanceIterator(deviceDescriptorIterator);
  EXPECT((v1.currentWord == 0x0112), v1.currentWord);

  constexpr auto v2 = AdvanceIterator(v1);
  EXPECT((v2.currentWord == 0x1001), v2.currentWord);

}

constexpr EventStack StackMock1(){
  EventStack es = {-1, 10, 0, 0, {}};
  Event Event1 =  {44, 55, 66, 77};
  PushEvent(&es, Event1 );
  PushEvent(&es, Event1 );
  return es;
}

constexpr EventStack StackMock2(int toPush, int toPop){
  EventStack es = {-1, 10, 0, 0, {}};
  Event Event1 =  {44, 55, 66, 77};
  for (int j = 0; j < toPush; j++){
    PushEvent(&es, Event1);
  }
  for (int j = 0; j < toPop; j++){
    PopEvent(&es);
  }
  return es;
}

void TestEventStack(){

  constexpr EventStack e1 = StackMock1();
  EXPECT((e1.top==1), e1.top);

  //
  constexpr EventStack e2 = StackMock2(5,0);
  EXPECT((e2.top==4), e2.top);

  constexpr EventStack e3 = StackMock2(5,2);
  EXPECT((e3.top==2), e3.top);


}

int main(){
  TestGetRegisterUpdateValue();
  TestUSBFunctions();
  TestPMACopy();
  TestPMAIterators();
  TestEventStack();
}

void Reset_Handler(void);
