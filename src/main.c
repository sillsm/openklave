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

int main(void) {
    // Push it forward
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x6000);

    // GPIO structure for port initialization
    GPIO_InitTypeDef GPIO_InitStructure;

    // enable clock on APB2
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,  ENABLE);

    // configure port A1 for driving an LED
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    // output push-pull mode
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   // highest speed
    GPIO_Init(GPIOA, &GPIO_InitStructure) ;             // initialize port

    // main loop
    while(1) {
        GPIO_SetBits(GPIOA, GPIO_Pin_1);    // turn the LED on
        delay(DELAY);

        GPIO_ResetBits(GPIOA, GPIO_Pin_1);  // turn the LED off
        delay(DELAY);
    }
}
