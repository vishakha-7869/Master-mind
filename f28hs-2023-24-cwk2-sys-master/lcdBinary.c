/* *************************** */
/* You can use this file to define the low-level hardware control fcts for       */
/* LED, button and LCD devices.                                                  */
/* Note that these need to be implemented in Assembler.                          */
/* You can use inline Assembler code, or use a stand-alone Assembler file.       */
/* Alternatively, you can implement all fcts directly in master-mind.c,          */
/* using inline Assembler code there.                                            */
/* The Makefile assumes you define the functions here.                           */
/* *************************** */

#ifndef TRUE
#define TRUE (1 == 1)
#define FALSE (1 == 2)
#endif

#define PAGE_SIZE (4 * 1024)
#define BLOCK_SIZE (4 * 1024)

#define INPUT 0
#define OUTPUT 1

#define LOW 0
#define HIGH 1

// APP constants   ---------------------------------

// Wiring (see call to lcdInit in main, using BCM numbering)
// NB: this needs to match the wiring as defined in master-mind.c

#define STRB_PIN 24
#define RS_PIN 25
#define DATA0_PIN 23
#define DATA1_PIN 10
#define DATA2_PIN 27
#define DATA3_PIN 22

// -----------------------------------------------------------------------------
// includes
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/types.h>
#include <time.h>

// -----------------------------------------------------------------------------
// prototypes

int failure(int fatal, const char *message, ...);

void digitalWrite(uint32_t *gpio, int pin, int value)
{
    int offVal, result;
    offVal = (value == LOW) ? 10 : 7;
    // inline assembly
    asm volatile(
        // load the gpio base address into r1
        "\tLDR R1, %[gpio]\n"
        // add the offset to the base address
        "\tADD R0, R1, %[offVal]\n"
        // load 2 into r2
        "\tMOV R2, #1\n"
        // load the pin number into r1
        "\tMOV R1, %[pin]\n"
        // shift 31 bits to the right and store in r1
        "\tAND R1, #31\n"
        // shift r2 left by r1 bits and store in r2
        "\tLSL R2, R1\n"
        // store the value in r2 at the address in r0
        "\tSTR R2, [R0, #0]\n"
        // output operands
        "\tMOV %[result], R2\n"
        : [result] "=r"(result)
        : [pin] "r"(pin), [gpio] "m"(gpio), [offVal] "r"(offVal * 4)
        : "r0", "r1", "r2", "cc");
}

/* set the @mode@ of a GPIO @pin@ to INPUT or OUTPUT; @gpio@ is the mmaped GPIO base address */
// fsel is the function select register
void pinMode(uint32_t *gpio, int pin, int mode)
{
    // get the function select register
    int fSel = pin / 10;
    // get the shift amount
    int shift = (pin % 10) * 3;
    // store the result
    int result;

    // output condition
    if (mode == OUTPUT)
    {
        // assembly code
        asm(
            // load the gpio base address into r1
            "\tLDR R1, %[gpio]\n"
            // add the function select register to the base address
            "\tADD R0, R1, %[fSel]\n"
            // load the value at the address in r0 into r1
            "\tLDR R1, [R0, #0]\n"
            // shift r2 left by the shift amount and store in r2
            "\tMOV R2, #0b111\n"
            "\tLSL R2, %[shift]\n"
            "\tBIC R1, R1, R2\n"
            // load 1 into r2
            "\tMOV R2, #1\n"
            // shift r2 left by the shift amount and store in r2
            "\tLSL R2, %[shift]\n"
            "\tORR R1, R2\n"
            // store the value in r1 at the address in r0
            "\tSTR R1, [R0, #0]\n"
            // store the value in r1 into result
            "\tMOV %[result], R1\n"
            : [result] "=r"(result)
            : [pin] "r"(pin), [gpio] "m"(gpio), [fSel] "r"(fSel * 4), [shift] "r"(shift)
            : "r0", "r1", "r2", "cc");
    }

    else if (mode == INPUT)
    {
        asm(
            // load the gpio base address into r1
            "\tLDR R1, %[gpio]\n"
            // translating *(gpio + fSel)
            "\tADD R0, R1, %[fSel]\n"
            "\tLDR R1, [R0, #0]\n"
            // moving binary value of 7 into register
            "\tMOV R2, #0b111\n"

            // translating (7 << shift)
            "\tLSL R2, %[shift]\n"
            "\tBIC R1, R1, R2\n"
            "\tSTR R1, [R0, #0]\n"
            "\tMOV %[result], R1\n"
            : [result] "=r"(result)
            : [pin] "r"(pin), [gpio] "m"(gpio), [fSel] "r"(fSel * 4), [shift] "r"(shift)
            : "r0", "r1", "r2", "cc");
    }

    else
    {
        fprintf(stderr, "Invalid Mode");
    }
}
// TODO
void writeLED(uint32_t *gpio, int led, int value)
{
    int off, result;
    // write value to the led
    if (value == LOW)
    {
        off = 10;
        asm volatile(
            // load the gpio base address into r1
            "\tLDR R1, %[gpio]\n"
            // add the offset to the base address
            "\tADD R0, R1, %[off]\n"
            // load 1 into r2
            "\tMOV R2, #1\n"
            // load the pin number into r1
            "\tMOV R1, %[pin]\n"
            // shift 31 bits to the right and store in r1
            "\tAND R1, #31\n"
            // shift r2 left by r1 bits and store in r2
            "\tLSL R2, R1\n"
            // store the value in r2 at the address in r0
            "\tSTR R2, [R0, #0]\n"
            // store the value in r2 into result
            "\tMOV %[result], R2\n"
            // output operands
            : [result] "=r"(result)
            : [pin] "r"(led), [gpio] "m"(gpio), [off] "r"(off * 4)
            : "r0", "r1", "r2", "cc");
    }
    else
    {
        // set the offset
        off = 7;
        asm volatile(
            "\tLDR R1, %[gpio]\n"
            "\tADD R0, R1, %[off]\n"
            "\tMOV R2, #1\n"
            "\tMOV R1, %[pin]\n"
            "\tAND R1, #31\n"
            "\tLSL R2, R1\n"
            "\tSTR R2, [R0,#0]\n"
            "\tMOV %[result], R2\n"
            : [result] "=r"(result)
            : [pin] "r"(led), [gpio] "m"(gpio), [off] "r"(off * 4)
            : "r0", "r1", "r2", "cc");
    }
}

/* read a @value@ (LOW or HIGH) from pin number @pin@ (a button device); @gpio@ is the mmaped GPIO base address */
// reading the input from the button
int readButton(uint32_t *gpio, int button)
{
    int state = 0;
    asm volatile(
        "MOV R1, %[gpio]\n"                   // Load the gpio base address into R1
        "LDR R2, [R1, #0x34]\n"               // Load the value at the address in R1 into R2 (GPLEV0 register offset for Raspberry Pi)
        "MOV R3, %[pin]\n"                    // Load the pin number into R3
        "MOV R4, #1\n"                        // Load 1 into R4
        "LSL R4, R3\n"                        // Logical shift R4 left by the pin number and store in R4
        "AND R4, R2, R4\n"                    // Logical AND R2 and R4 and store the result in R4
        "MOV %[state], R4\n"                  // Move R4 to state
        : [state] "=r"(state)                 // Output operands
        : [pin] "r"(button), [gpio] "r"(gpio) // Input operands
        : "r1", "r2", "r3", "r4", "cc"        // Clobbered registers
    );

    if (state != 0)
        return 1; // If state is not zero, the button is pressed
    else
        return 0; // If state is zero, the button is not pressed
}

/* wait for a button input on pin number @button@; @gpio@ is the mmaped GPIO base address */
/* can use readButton(), depending on your implementation */
void waitForButton(uint32_t *gpio, int button)
{
    while (!readButton(gpio, button))
    {
    }
}