/*
 * MasterMind implementation: template; see comments below on which parts need to be completed
 * CW spec: https://www.macs.hw.ac.uk/~hwloidl/Courses/F28HS/F28HS_CW2_2022.pdf
 * This repo: https://gitlab-student.macs.hw.ac.uk/f28hs-2021-22/f28hs-2021-22-staff/f28hs-2021-22-cwk2-sys

 * Compile:
 gcc -c -o lcdBinary.o lcdBinary.c
 gcc -c -o master-mind.o master-mind.c
 gcc -o master-mind master-mind.o lcdBinary.o
 * Run:
 sudo ./master-mind
 ./cw2 [-v] [-d] [-u <seq1> <seq2>] [-s <secret sequence>]

 OR use the Makefile to build
 > make all
 and run
 > make run
 and test
 > make test

 *************************
 * The Low-level interface to LED, button, and LCD is based on:
 * wiringPi libraries by
 * Copyright (c) 2012-2013 Gordon Henderson.
 *************************
 * See:
 *	https://projects.drogon.net/raspberry-pi/wiringpi/
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public License
 *    along with wiringPi.  If not, see <http://www.gnu.org/licenses/>.
 *************************
*/

/* ======================================================= */
/* SECTION: includes                                       */
/* ------------------------------------------------------- */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>

#include <unistd.h>
#include <string.h>
#include <time.h>

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <sys/ioctl.h>

/* --------------------------------------------------------------------------- */
/* Config settings */
/* you can use CPP flags to e.g. print extra debugging messages */
/* or switch between different versions of the code e.g. digitalWrite() in Assembler */
#define DEBUG
#undef ASM_CODE

// =======================================================
// Tunables
// PINs (based on BCM numbering)
// For wiring see CW spec: https://www.macs.hw.ac.uk/~hwloidl/Courses/F28HS/F28HS_CW2_2022.pdf
// GPIO pin for green LED
#define LED 13
// GPIO pin for red LED
#define LED2 5
// GPIO pin for button
#define BUTTON 19
// =======================================================
// delay for loop iterations (mainly), in ms
// in mili-seconds: 0.2s
#define DELAY 200
// in micro-seconds: 4s
#define TIMEOUT 4000000
// =======================================================
// APP constants   ---------------------------------
// number of colours and length of the sequence
#define COLS 3
#define SEQL 3
// =======================================================

// generic constants

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

// =======================================================
// Wiring (see inlined initialisation routine)

#define STRB_PIN 24
#define RS_PIN 25
#define DATA0_PIN 23
#define DATA1_PIN 10
#define DATA2_PIN 27
#define DATA3_PIN 22

/* ======================================================= */
/* SECTION: constants and prototypes                       */
/* ------------------------------------------------------- */

// =======================================================
// char data for the CGRAM, i.e. defining new characters for the display

static unsigned char newChar[8] =
    {
        0b11111,
        0b10001,
        0b10001,
        0b10101,
        0b11111,
        0b10001,
        0b10001,
        0b11111,
};

/* Constants */

static const int colors = COLS;
static const int seqlen = SEQL;

static char *color_names[] = {"red", "green", "blue"};

static int *theSeq = NULL;

static int *seq1, *seq2, *cpy1, *cpy2;

/* --------------------------------------------------------------------------- */

// data structure holding data on the representation of the LCD
struct lcdDataStruct
{
  int bits, rows, cols;
  int rsPin, strbPin;
  int dataPins[8];
  int cx, cy;
};

static int lcdControl;

/* *************************** */
/* INLINED fcts from wiringPi/devLib/lcd.c: */
// HD44780U Commands (see Fig 11, p28 of the Hitachi HD44780U datasheet)

#define LCD_CLEAR 0x01
#define LCD_HOME 0x02
#define LCD_ENTRY 0x04
#define LCD_CTRL 0x08
#define LCD_CDSHIFT 0x10
#define LCD_FUNC 0x20
#define LCD_CGRAM 0x40
#define LCD_DGRAM 0x80

// Bits in the entry register

#define LCD_ENTRY_SH 0x01
#define LCD_ENTRY_ID 0x02

// Bits in the control register

#define LCD_BLINK_CTRL 0x01
#define LCD_CURSOR_CTRL 0x02
#define LCD_DISPLAY_CTRL 0x04

// Bits in the function register

#define LCD_FUNC_F 0x04
#define LCD_FUNC_N 0x08
#define LCD_FUNC_DL 0x10

#define LCD_CDSHIFT_RL 0x04

// Mask for the bottom 64 pins which belong to the Raspberry Pi
//	The others are available for the other devices

#define PI_GPIO_MASK (0xFFFFFFC0)

static unsigned int gpiobase;
static uint32_t *gpio;

static int timed_out = 0;

/* ------------------------------------------------------- */
// misc prototypes

int failure(int fatal, const char *message, ...);
void waitForEnter(void);
void waitForButton(uint32_t *gpio, int button);

/* ======================================================= */
/* SECTION: hardware interface (LED, button, LCD display)  */
/* ------------------------------------------------------- */
/* low-level interface to the hardware */

/* ******************** */
/* COMPLETE the code for all of the functions in this SECTION */
/* Either put them in a separate file, lcdBinary.c, and use   */
/* inline Assembler there, or use a standalone Assembler file */
/* You can also directly implement them here (inline Asm).    */
/* ******************** */

/* These are just prototypes; you need to complete the code for each function */

/* send a @value@ (LOW or HIGH) on pin number @pin@; @gpio@ is the mmaped GPIO base address */
void digitalWrite(uint32_t *gpio, int pin, int value);

/* set the @mode@ of a GPIO @pin@ to INPUT or OUTPUT; @gpio@ is the mmaped GPIO base address */
void pinMode(uint32_t *gpio, int pin, int mode);

/* send a @value@ (LOW or HIGH) on pin number @pin@; @gpio@ is the mmaped GPIO base address */
/* can use digitalWrite(), depending on your implementation */
void writeLED(uint32_t *gpio, int led, int value);

/* read a @value@ (LOW or HIGH) from pin number @pin@ (a button device); @gpio@ is the mmaped GPIO base address */
int readButton(uint32_t *gpio, int button);

/* wait for a button input on pin number @button@; @gpio@ is the mmaped GPIO base address */
/* can use readButton(), depending on your implementation */
void waitForButton(uint32_t *gpio, int button);

/* ======================================================= */
/* SECTION: game logic                                     */
/* ------------------------------------------------------- */
/* AUX fcts of the game logic */

/* ******************** */
/* COMPLETE the code for all of the functions in this SECTION */
/* Implement these as C functions in this file                */
/* ******************** */

/* initialise the secret sequence; by default it should be a random sequence */
void initSeq()
{
  theSeq = (int *)malloc(seqlen * sizeof(int)); //allocates dynamic memory for theSeq array using malloc()

  srand(time(0)); //seeds the random number generator using the current time

//loops seqlen times, generating a random number between 1 and 3 (inclusive) for each position in the sequence.
  for (int i = 0; i < seqlen; i++)
  {
    theSeq[i] = (rand() % 3) + 1;
  }
}

/* display the sequence on the terminal window, using the format from the sample run in the spec */
void showSeq(int *seq)
{
  fprintf(stdout, "Secret: ");
  for (int i = 0; i < seqlen; i++) //loops through each element of the seq array, printing it out followed by a space.
  {
    fprintf(stdout, "%d ", seq[i]);// loops the correct number of times to print the full sequence.
  }

  fprintf(stdout, "\n");
}

#define NAN1 8
#define NAN2 9

/* counts how many entries in seq2 match entries in seq1 */
/* returns exact and approximate matches */
/* as a pointer to a pair of values */
int *countMatches(int *seq1, int *seq2)
{
  int *data = (int *)malloc(2 * sizeof(int)); // allocate memory for result

  int r_exact = 0;   // variable to store exact match result
  int r_approx = 0; // variable to store approximate match result
// Inline assembly code to count number of matches
  asm(
      "start:\n" // label for start of code
      "\tMOV R0, #0\n" // initialize R0 to store exact match count  
      "\tMOV R3, #0\n" // initialize R3 to store approximate match count
      "\tMOV R1, %[seq1]\n"// load address of seq1 into R1
      "\tMOV R2, %[seq2]\n"// load address of seq2 into R2
      "\tMOV R4, #0\n" // initialize R4 as temp for approximate match value
      "\tMOV R5, #0\n" // initialize R5 as loop counter for seq1
      "\tMOV R7, #0\n" // initialize R7 as index into seq1
      "\tMOV R6, #0\n"  // initialize R6 as loop counter for seq2 
      "\tMOV R8, #0\n" // initialize R8 as index into seq2
      "\tB main_loop\n" // branch to main match loop

    // Main matching loop
      "main_loop:\n"
      "\tCMP R5, #3\n"// compare R5 to max length
      "\tBEQ exit_routine\n"// if R5=3, done - jump to end

  // Load current values from each sequence
      "\tLDR R9, [R1, R7]\n"
      "\tLDR R10, [R2, R7]\n"
// Compare values 
      "\tCMP R9, R10\n"
      "\tBEQ add_exact\n"// if equal, increment exact match count

      // Reset R6 and R8 for approximate match loop
      "\tMOV R6, #0\n"
      "\tMOV R8, #0\n"
      "\rB approx_loop\n"// branch to loop for approximate match
      "\tB loop_increment1\n"// increment indices and loop

  // Increment indices and loop back  
      "loop_increment1:\n"
      "\tADD R5, R5, #1\n"
      "\tADD R7, R7, #4\n"
      "\tB main_loop\n"

      "add_exact:\n"
      "\tADD R0, R0, #1\n"// Increment R0 for exact match count
      "\tCMP R10, R4\n" // Compare current value to approximate match value 
      "\tBEQ check_approx_size\n"// If equal, check approximate match count
      "\tB loop_increment1\n"// Otherwise continue to next loop iteration


      "check_approx_size:\n"
      "\tCMP R3, #0\n"// Check if any approximate matches so far
      "\tBNE decrement_approx\n"// If yes, decrement count
      "\tB loop_increment1\n"// Otherwise continue loop

      "decrement_approx:\n"
      "\tSUB R3, R3, #1\n"// Decrement approximate match count
      "\tB loop_increment1\n"// Continue to next loop iteration

      "approx_loop:\n"
      "\tCMP R6, #3\n"// Check if reached end of sequence 2
      "\tBEQ loop_increment1\n"// If yes, continue outer loop


      "\tLDR R10, [R2, R8]\n"

      "\tCMP R9, R10\n"
      "\tBEQ index_check\n" // If values match, check indices
      "\tB loop_increment2\n"// Otherwise continue inner loop

      "loop_increment2:\n"
      "\tADD R6, R6, #1\n"// Increment sequence 2 index 
      "\tADD R8, R8, #4\n"// Increment sequence 2 pointer
      "\tB approx_loop\n" // Loop back

      "index_check:\n"
      "\tCMP R5, R6\n"// Compare indices 
      "\tBNE approx_check\n"// If different, check for approximate
      "\tB loop_increment2\n"// Otherwise continue 

      "approx_check:\n"
      "\tCMP R10, R4\n"// If value already matched, skip
      "\tBNE add_approx\n"
      "\tB loop_increment2\n"

      "add_approx:\n"
      "\tMOV R4, R10\n"// Save matched value 
      "\tADD R3, R3, #1\n"// Increment approximate match count
      "\tMOV R6, #2\n"// Reset index
      "\tB loop_increment2\n"

      "exit_routine:\n"
      "\tMOV %[result_exact], R0\n"  //store exact match result
      "\tMOV %[result_approx], R3\n" // store approximate match result
      : [result_exact] "=r"(r_exact), [r_approx] "=r"(r_approx)
      : [seq1] "r"(seq1), [seq2] "r"(seq2), [seqlen] "r"(seqlen)
      : "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7", "r8", "r9", "cc");
  data[0] = r_exact;  // store the result of exact matches in the first element of data array
  data[1] = r_approx; // store the result of approximate matches in the second element of data array

  return data; // return the pointer to the result data
}

/* show the results from calling countMatches on seq1 and seq1 */
void showMatches(int *code, int *seq1, int *seq2, int lcd_format)
{
  code = countMatches(seq1, seq2);              // call countMatches on seq1 and seq2
  fprintf(stdout, "%d exact\n", code[0]);       // print the count of exact matches
  fprintf(stdout, "%d approximate\n", code[1]); // print the count of approximate matches
}

/* parse an integer value as a list of digits, and put them into @seq@ */
/* needed for processing command-line with options -s or -u            */
void readSeq(int *seq, int val)
{
  int len = seqlen; // length of the sequence

  while (len != 0) // while there are still digits to be read
  {
    /* Accessing the integer digit by digit */
    seq[len - 1] = val % 10; // store the last digit of val in the sequence
    val /= 10;               // divide val by 10 to get the next digit
    len--;                   // decrement the length of the sequence
  }
}

/* read a guess sequence fron stdin and store the values in arr */
/* only needed for testing the game logic, without button input */
int readNum(int max)
{
  int sequence[max]; // array to store the input sequence
  int temp;      // temporary variable to store the input
  for (int i = 0; i < max; i++)
  {
    /* Accessing the integer digit by digit */
    scanf("Enter number: %d", &temp); // read the input from stdin
    sequence[i] = temp;               // store the input in the sequence array
  }

  return temp; // return the last value in the sequence array
  }

/* ======================================================= */
/* SECTION: TIMER code                                     */
/* ------------------------------------------------------- */
/* TIMER code */

/* timestamps needed to implement a time-out mechanism */
static uint64_t startT, stopT;

/* ******************** */
/* COMPLETE the code for all of the functions in this SECTION */
/* Implement these as C functions in this file                */
/* ******************** */

/* you may need this function in timer_handler() below  */
/* use the libc fct gettimeofday() to implement it      */
uint64_t timeInMicroseconds()
{
  struct timeval tv;
  uint64_t now;
  gettimeofday(&tv, NULL);
  now = (uint64_t)tv.tv_sec * (uint64_t)1000000 + (uint64_t)tv.tv_usec;
  return (uint64_t)now;
}

/* this should be the callback, triggered via an interval timer, */
/* that is set-up through a call to sigaction() in the main fct. */
void timer_handler(int signum)
{
  static int count = 0;
  stopT = timeInMicroseconds(); // Get the current time
  count++;
  fprintf(stderr, "Timer expired %d times. Time took: %f\n", count, (stopT - startT) / 1000000.0);
  timed_out = 1; // Set the flag that the timer has expired
}

/* initialise time-stamps, setup an interval timer, and install the timer_handler callback */
void initITimer(uint64_t timeout)
{
  struct sigaction sa; // Structure to store the signal action
  struct itimerval timer; //  Structure to store the timer values

  /* set up the signal action for SIGALRM */
  memset(&sa, 0, sizeof(sa));
  sa.sa_handler = &timer_handler;

  sigaction(SIGALRM, &sa, NULL); 

  /* set up the timer values */
  timer.it_value.tv_sec = timeout; // Set the initial timer value in seconds
  timer.it_value.tv_usec = 0;      // Set the initial timer value in microseconds

  timer.it_interval.tv_sec = 0;  // Set the interval for recurring timer (0 for non-recurring)
  timer.it_interval.tv_usec = 0; // Set the interval for recurring timer (0 for non-recurring)

  setitimer(ITIMER_REAL, &timer, NULL); // Start the timer with the specified values

  startT = timeInMicroseconds(); // Initialize the start timestamp
}

/* ======================================================= */
/* SECTION: Aux function                                   */
/* ------------------------------------------------------- */
/* misc aux functions */

int failure(int fatal, const char *message, ...)
{
  va_list argp;
  char buffer[1024];

  if (!fatal) //  && wiringPiReturnCodes)
    return -1;

  va_start(argp, message);
  vsnprintf(buffer, 1023, message, argp);
  va_end(argp);

  fprintf(stderr, "%s", buffer);
  exit(EXIT_FAILURE);

  return 0;
}

/*
 * waitForEnter:
 ***************************
 */

void waitForEnter(void)
{
  printf("Press ENTER to continue: ");
  (void)fgetc(stdin);
}

/*
 * delay:
 *	Wait for some number of milliseconds
 ***************************
 */

void delay(unsigned int howLong)
{
  struct timespec sleeper, dummy;

  sleeper.tv_sec = (time_t)(howLong / 1000);
  sleeper.tv_nsec = (long)(howLong % 1000) * 1000000;

  nanosleep(&sleeper, &dummy);
}

/* From wiringPi code; comment by Gordon Henderson
 * delayMicroseconds:
 *	This is somewhat intersting. It seems that on the Pi, a single call
 *	to nanosleep takes some 80 to 130 microseconds anyway, so while
 *	obeying the standards (may take longer), it's not always what we
 *	want!
 *
 *	So what I'll do now is if the delay is less than 100uS we'll do it
 *	in a hard loop, watching a built-in counter on the ARM chip. This is
 *	somewhat sub-optimal in that it uses 100% CPU, something not an issue
 *	in a microcontroller, but under a multi-tasking, multi-user OS, it's
 *	wastefull, however we've no real choice )-:
 *
 *      Plan B: It seems all might not be well with that plan, so changing it
 *      to use gettimeofday () and poll on that instead...
 ***************************
 */

void delayMicroseconds(unsigned int howLong)
{
  struct timespec sleeper;
  unsigned int uSecs = howLong % 1000000;
  unsigned int wSecs = howLong / 1000000;

  if (howLong == 0)
    return;
#if 0
  else if (howLong  < 100)
    delayMicrosecondsHard (howLong) ;
#endif
  else
  {
    sleeper.tv_sec = wSecs;
    sleeper.tv_nsec = (long)(uSecs * 1000L);
    nanosleep(&sleeper, NULL);
  }
}

/* ======================================================= */
/* SECTION: LCD functions                                  */
/* ------------------------------------------------------- */
/* medium-level interface functions (all in C) */

/* from wiringPi:
 * strobe:
 *	Toggle the strobe (Really the "E") pin to the device.
 *	According to the docs, data is latched on the falling edge.
 ***************************
 */

void strobe(const struct lcdDataStruct *lcd)
{

  // Note timing changes for new version of delayMicroseconds ()
  digitalWrite(gpio, lcd->strbPin, 1);
  delayMicroseconds(50);
  digitalWrite(gpio, lcd->strbPin, 0);
  delayMicroseconds(50);
}

/*
 * sentDataCmd:
 *	Send an data or command byte to the display.
 ***************************
 */

void sendDataCmd(const struct lcdDataStruct *lcd, unsigned char data)
{
  register unsigned char myData = data;
  unsigned char i, d4;

  if (lcd->bits == 4)
  {
    d4 = (myData >> 4) & 0x0F;
    for (i = 0; i < 4; ++i)
    {
      digitalWrite(gpio, lcd->dataPins[i], (d4 & 1));
      d4 >>= 1;
    }
    strobe(lcd);

    d4 = myData & 0x0F;
    for (i = 0; i < 4; ++i)
    {
      digitalWrite(gpio, lcd->dataPins[i], (d4 & 1));
      d4 >>= 1;
    }
  }
  else
  {
    for (i = 0; i < 8; ++i)
    {
      digitalWrite(gpio, lcd->dataPins[i], (myData & 1));
      myData >>= 1;
    }
  }
  strobe(lcd);
}

/*
 * lcdPutCommand:
 *	Send a command byte to the display
 ***************************
 */

void lcdPutCommand(const struct lcdDataStruct *lcd, unsigned char command)
{
#ifdef DEBUG
  fprintf(stderr, "lcdPutCommand: digitalWrite(%d,%d) and sendDataCmd(%d,%d)\n", lcd->rsPin, 0, lcd, command);
#endif
  digitalWrite(gpio, lcd->rsPin, 0);
  sendDataCmd(lcd, command);
  delay(2);
}

void lcdPut4Command(const struct lcdDataStruct *lcd, unsigned char command)
{
  register unsigned char myCommand = command;
  register unsigned char i;

  digitalWrite(gpio, lcd->rsPin, 0);

  for (i = 0; i < 4; ++i)
  {
    digitalWrite(gpio, lcd->dataPins[i], (myCommand & 1));
    myCommand >>= 1;
  }
  strobe(lcd);
}

/*
 * lcdHome: lcdClear:
 *	Home the cursor or clear the screen.
 ***************************
 */

void lcdHome(struct lcdDataStruct *lcd)
{
#ifdef DEBUG
  fprintf(stderr, "lcdHome: lcdPutCommand(%d,%d)\n", lcd, LCD_HOME);
#endif
  lcdPutCommand(lcd, LCD_HOME);
  lcd->cx = lcd->cy = 0;
  delay(5);
}

void lcdClear(struct lcdDataStruct *lcd)
{
#ifdef DEBUG
  fprintf(stderr, "lcdClear: lcdPutCommand(%d,%d) and lcdPutCommand(%d,%d)\n", lcd, LCD_CLEAR, lcd, LCD_HOME);
#endif
  lcdPutCommand(lcd, LCD_CLEAR);
  lcdPutCommand(lcd, LCD_HOME);
  lcd->cx = lcd->cy = 0;
  delay(5);
}

/*
 * lcdPosition:
 *	Update the position of the cursor on the display.
 *	Ignore invalid locations.
 ***************************
 */

void lcdPosition(struct lcdDataStruct *lcd, int x, int y)
{
  // struct lcdDataStruct *lcd = lcds [fd] ;

  if ((x > lcd->cols) || (x < 0))
    return;
  if ((y > lcd->rows) || (y < 0))
    return;

  lcdPutCommand(lcd, x + (LCD_DGRAM | (y > 0 ? 0x40 : 0x00) /* rowOff [y] */));

  lcd->cx = x;
  lcd->cy = y;
}

/*
 * lcdDisplay: lcdCursor: lcdCursorBlink:
 *	Turn the display, cursor, cursor blinking on/off
 ***************************
 */

void lcdDisplay(struct lcdDataStruct *lcd, int state)
{
  if (state)
    lcdControl |= LCD_DISPLAY_CTRL;
  else
    lcdControl &= ~LCD_DISPLAY_CTRL;

  lcdPutCommand(lcd, LCD_CTRL | lcdControl);
}

void lcdCursor(struct lcdDataStruct *lcd, int state)
{
  if (state)
    lcdControl |= LCD_CURSOR_CTRL;
  else
    lcdControl &= ~LCD_CURSOR_CTRL;

  lcdPutCommand(lcd, LCD_CTRL | lcdControl);
}

void lcdCursorBlink(struct lcdDataStruct *lcd, int state)
{
  if (state)
    lcdControl |= LCD_BLINK_CTRL;
  else
    lcdControl &= ~LCD_BLINK_CTRL;

  lcdPutCommand(lcd, LCD_CTRL | lcdControl);
}

/*
 * lcdPutchar:
 *	Send a data byte to be displayed on the display. We implement a very
 *	simple terminal here - with line wrapping, but no scrolling. Yet.
 ***************************
 */

void lcdPutchar(struct lcdDataStruct *lcd, unsigned char data)
{
  digitalWrite(gpio, lcd->rsPin, 1);
  sendDataCmd(lcd, data);

  if (++lcd->cx == lcd->cols)
  {
    lcd->cx = 0;
    if (++lcd->cy == lcd->rows)
      lcd->cy = 0;

    // TODO: inline computation of address and eliminate rowOff
    lcdPutCommand(lcd, lcd->cx + (LCD_DGRAM | (lcd->cy > 0 ? 0x40 : 0x00) /* rowOff [lcd->cy] */));
  }
}

/*
 * lcdPuts:
 *	Send a string to be displayed on the display
 ***************************
 */

void lcdPuts(struct lcdDataStruct *lcd, const char *string)
{
  while (*string)
    lcdPutchar(lcd, *string++);
}

/* ======================================================= */
/* SECTION: aux functions for game logic                   */
/* ------------------------------------------------------- */

/* ******************** */
/* COMPLETE the code for all of the functions in this SECTION */
/* Implement these as C functions in this file                */
/* ******************** */

/* --------------------------------------------------------------------------- */
/* interface on top of the low-level pin I/O code */

/* blink the led on pin @led@, @c@ times */
void blinkN(uint32_t *gpio, int led, int c)
{
  /* *  COMPLETE the code here  *  */
  for (int i = 0; i < c; i++) // blink the led c times
  {
    
    writeLED(gpio, led, HIGH);// turn on the led
    delay(700);
    writeLED(gpio, led, LOW);// turn off the led
    delay(700);
  }

  delay(500);// wait for 500ms
}

/* ======================================================= */
/* SECTION: main fct                                       */
/* ------------------------------------------------------- */

int main(int argc, char *argv[])
{ // this is just a suggestion of some variable that you may want to use

  struct lcdDataStruct *lcd;
  int bits, rows, cols;
  unsigned char func;

  int found = 0, attempts = 0, i, j, code;
  int c, d, buttonPressed, rel, foo;
  int *attSeq;

  int pinLED = LED, pin2LED2 = LED2, pinButton = BUTTON;
  int fSel, shift, pin, clrOff, setOff, off, res;
  int fd;

  int exact, contained;
  char str1[32];
  char str2[32];

  struct timeval t1, t2;
  int t;

  char buf[32];

  // variables for command-line processing
  char str_in[20], str[20] = "some text";
  int verbose = 0, debug = 0, help = 0, opt_m = 0, opt_n = 0, opt_s = 0, unit_test = 0, res_matches = 0;

  // -------------------------------------------------------
  // process command-line arguments

  // see: man 3 getopt for docu and an example of command line parsing
  { // see the CW spec for the intended meaning of these options
    int opt;
    while ((opt = getopt(argc, argv, "hvdus:")) != -1)
    {
      switch (opt)
      {
      case 'v':
        verbose = 1;
        break;
      case 'h':
        help = 1;
        break;
      case 'd':
        debug = 1;
        break;
      case 'u':
        unit_test = 1;
        break;
      case 's':
        opt_s = atoi(optarg);
        break;
      default: /* '?' */
        fprintf(stderr, "Usage: %s [-h] [-v] [-d] [-u <seq1> <seq2>] [-s <secret seq>]  \n", argv[0]);
        exit(EXIT_FAILURE);
      }
    }
  }

  if (help)
  {
    fprintf(stderr, "MasterMind program, running on a Raspberry Pi, with connected LED, button and LCD display\n");
    fprintf(stderr, "Use the button for input of numbers. The LCD display will show the matches with the secret sequence.\n");
    fprintf(stderr, "For full specification of the program see: https://www.macs.hw.ac.uk/~hwloidl/Courses/F28HS/F28HS_CW2_2022.pdf\n");
    fprintf(stderr, "Usage: %s [-h] [-v] [-d] [-u <seq1> <seq2>] [-s <secret seq>]  \n", argv[0]);
    exit(EXIT_SUCCESS);
  }

  if (unit_test && optind >= argc - 1)
  {
    fprintf(stderr, "Expected 2 arguments after option -u\n");
    exit(EXIT_FAILURE);
  }

  if (verbose && unit_test)
  {
    printf("1st argument = %s\n", argv[optind]);
    printf("2nd argument = %s\n", argv[optind + 1]);
  }

  if (verbose)
  {
    fprintf(stdout, "Settings for running the program\n");
    fprintf(stdout, "Verbose is %s\n", (verbose ? "ON" : "OFF"));
    fprintf(stdout, "Debug is %s\n", (debug ? "ON" : "OFF"));
    fprintf(stdout, "Unittest is %s\n", (unit_test ? "ON" : "OFF"));
    if (opt_s)
      fprintf(stdout, "Secret sequence set to %d\n", opt_s);
  }

  seq1 = (int *)malloc(seqlen * sizeof(int));
  seq2 = (int *)malloc(seqlen * sizeof(int));
  cpy1 = (int *)malloc(seqlen * sizeof(int));
  cpy2 = (int *)malloc(seqlen * sizeof(int));

  // check for -u option, and if so run a unit test on the matching function
  if (unit_test && argc > optind + 1)
  { // more arguments to process; only needed with -u
    strcpy(str_in, argv[optind]);
    opt_m = atoi(str_in);
    strcpy(str_in, argv[optind + 1]);
    opt_n = atoi(str_in);
    // CALL a test-matches function; see testm.c for an example implementation
    readSeq(seq1, opt_m); // turn the integer number into a sequence of numbers
    readSeq(seq2, opt_n); // turn the integer number into a sequence of numbers
    if (verbose)
      fprintf(stdout, "Testing matches function with sequences %d and %d\n", opt_m, opt_n);
    int *res_matches = countMatches(seq1, seq2);
    showMatches(res_matches, seq1, seq2, 1);
    exit(EXIT_SUCCESS);
  }
  else
  {
    /* nothing to do here; just continue with the rest of the main fct */
  }

  if (opt_s)
  { // if -s option is given, use the sequence as secret sequence
    if (theSeq == NULL)
      theSeq = (int *)malloc(seqlen * sizeof(int));
    readSeq(theSeq, opt_s);
    if (verbose)
    {
      fprintf(stderr, "Running program with secret sequence:\n");
      showSeq(theSeq);
    }
  }

  // -------------------------------------------------------
  // LCD constants, hard-coded: 16x2 display, using a 4-bit connection
  bits = 4;
  cols = 16;
  rows = 2;
  // -------------------------------------------------------

  printf("Raspberry Pi LCD driver, for a %dx%d display (%d-bit wiring) \n", cols, rows, bits);

  if (geteuid() != 0)
    fprintf(stderr, "setup: Must be root. (Did you forget sudo?)\n");

  // init of guess sequence, and copies (for use in countMatches)
  attSeq = (int *)malloc(seqlen * sizeof(int));
  cpy1 = (int *)malloc(seqlen * sizeof(int));
  cpy2 = (int *)malloc(seqlen * sizeof(int));

  // -----------------------------------------------------------------------------
  // constants for RPi2
  gpiobase = 0x3F200000;

  // -----------------------------------------------------------------------------
  // memory mapping
  // Open the master /dev/memory device

  if ((fd = open("/dev/mem", O_RDWR | O_SYNC | O_CLOEXEC)) < 0)
    return failure(FALSE, "setup: Unable to open /dev/mem: %s\n", strerror(errno));

  // GPIO:
  gpio = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, gpiobase);
  if ((int32_t)gpio == -1)
    return failure(FALSE, "setup: mmap (GPIO) failed: %s\n", strerror(errno));

  // -------------------------------------------------------
  // Configuration of LED and BUTTON
  pinMode(gpio, pinLED, OUTPUT);   // set LED pin to output
  pinMode(gpio, pin2LED2, OUTPUT); // set LED pin to output
  pinMode(gpio, pinButton, INPUT); // set button pin to input

  writeLED(gpio, pinLED, 0);   // turn LED off
  writeLED(gpio, pin2LED2, 0); // turn LED off

  /* *  COMPLETE the code here  *  */

  // -------------------------------------------------------
  // INLINED version of lcdInit (can only deal with one LCD attached to the RPi):
  // you can use this code as-is, but you need to implement digitalWrite() and
  // pinMode() which are called from this code
  // Create a new LCD:
  lcd = (struct lcdDataStruct *)malloc(sizeof(struct lcdDataStruct));
  if (lcd == NULL)
    return -1;

  // hard-wired GPIO pins
  lcd->rsPin = RS_PIN;
  lcd->strbPin = STRB_PIN;
  lcd->bits = 4;
  lcd->rows = rows; // # of rows on the display
  lcd->cols = cols; // # of cols on the display
  lcd->cx = 0;      // x-pos of cursor
  lcd->cy = 0;      // y-pos of curosr

  lcd->dataPins[0] = DATA0_PIN;
  lcd->dataPins[1] = DATA1_PIN;
  lcd->dataPins[2] = DATA2_PIN;
  lcd->dataPins[3] = DATA3_PIN;
  // lcd->dataPins [4] = d4 ;
  // lcd->dataPins [5] = d5 ;
  // lcd->dataPins [6] = d6 ;
  // lcd->dataPins [7] = d7 ;

  // lcds [lcdFd] = lcd ;

  digitalWrite(gpio, lcd->rsPin, 0);
  pinMode(gpio, lcd->rsPin, OUTPUT);
  digitalWrite(gpio, lcd->strbPin, 0);
  pinMode(gpio, lcd->strbPin, OUTPUT);

  for (i = 0; i < bits; ++i)
  {
    digitalWrite(gpio, lcd->dataPins[i], 0);
    pinMode(gpio, lcd->dataPins[i], OUTPUT);
  }
  delay(35); // mS

  // Gordon Henderson's explanation of this part of the init code (from wiringPi):
  // 4-bit mode?
  //	OK. This is a PIG and it's not at all obvious from the documentation I had,
  //	so I guess some others have worked through either with better documentation
  //	or more trial and error... Anyway here goes:
  //
  //	It seems that the controller needs to see the FUNC command at least 3 times
  //	consecutively - in 8-bit mode. If you're only using 8-bit mode, then it appears
  //	that you can get away with one func-set, however I'd not rely on it...
  //
  //	So to set 4-bit mode, you need to send the commands one nibble at a time,
  //	the same three times, but send the command to set it into 8-bit mode those
  //	three times, then send a final 4th command to set it into 4-bit mode, and only
  //	then can you flip the switch for the rest of the library to work in 4-bit
  //	mode which sends the commands as 2 x 4-bit values.

  if (bits == 4)
  {
    func = LCD_FUNC | LCD_FUNC_DL; // Set 8-bit mode 3 times
    lcdPut4Command(lcd, func >> 4);
    delay(35);
    lcdPut4Command(lcd, func >> 4);
    delay(35);
    lcdPut4Command(lcd, func >> 4);
    delay(35);
    func = LCD_FUNC; // 4th set: 4-bit mode
    lcdPut4Command(lcd, func >> 4);
    delay(35);
    lcd->bits = 4;
  }
  else
  {
    failure(TRUE, "setup: only 4-bit connection supported\n");
    func = LCD_FUNC | LCD_FUNC_DL;
    lcdPutCommand(lcd, func);
    delay(35);
    lcdPutCommand(lcd, func);
    delay(35);
    lcdPutCommand(lcd, func);
    delay(35);
  }

  if (lcd->rows > 1)
  {
    func |= LCD_FUNC_N;
    lcdPutCommand(lcd, func);
    delay(35);
  }

  // Rest of the initialisation sequence
  lcdDisplay(lcd, TRUE);
  lcdCursor(lcd, FALSE);
  lcdCursorBlink(lcd, FALSE);
  lcdClear(lcd);

  lcdPutCommand(lcd, LCD_ENTRY | LCD_ENTRY_ID);     // set entry mode to increment address counter after write
  lcdPutCommand(lcd, LCD_CDSHIFT | LCD_CDSHIFT_RL); // set display shift to right-to-left

  // END lcdInit ------
  // -----------------------------------------------------------------------------
  // Start of game
  // Start of game
  fprintf(stderr, " \n");
  fprintf(stderr, "Preparing to display welcome message on the LCD display ...\n");
  fprintf(stderr, " \n");
  lcdPosition(lcd, 0, 0);
  lcdPuts(lcd, "Welcome to the game!");
  lcdPosition(lcd, 0, 1);
  lcdPuts(lcd, "            ");

  // optionally one of these 2 calls:
  // waitForButton (gpio, pinButton) ;
  waitForEnter();
  lcdClear(lcd);
  fprintf(stderr, "\n");

  /* initialise the secret sequence */
  if (!opt_s)
    initSeq();
  if (debug)
    showSeq(theSeq);

  // optionally one of these 2 calls:
  // waitForEnter () ;
  // waitForButton (gpio, pinButton) ;

  // -----------------------------------------------------------------------------
  // +++++ main loop
  fprintf(stderr, "Lets start the game... \n");
  delay(700);


  while (!found)// Loop until we find the correct sequence
  {
    int guess[SEQL]; // Array to hold the guess
    printf("round %d\n", attempts + 1);// Display the round number

    for (int i = 0; i < SEQL; i++)// Loop through the sequence
    {
      int buttonPresses = 0; // Number of button presses
      int buttonState = LOW, lastButtonState = LOW; // State of the button
      unsigned long long endTime = timeInMicroseconds() + TIMEOUT; // Time when the guess is over

      // Display the number
      printf("Enter number %d: Press the button\n", i + 1);
      initITimer(6);// Initialise the ITimer

      while (timeInMicroseconds() < endTime)// Loop until the button is pressed
      {
        buttonState = readButton(gpio, BUTTON);// Read the button
        if (buttonState == HIGH && lastButtonState == LOW)// If the button is pressed
        {
          if (buttonPresses < 3)
          { // If the button has been pressed less than 3 times
            buttonPresses++;// Increment the number of button presses
          }
          delay(250); // Short delay to debounce
        }
        lastButtonState = buttonState; // Update the button state
      }

      fprintf(stderr, "\n");

      printf("Button pressed %d times\n", buttonPresses);// Display the number of button presses

      // Acknowledge the button press input with a red LED blink
      blinkN(gpio, LED2, 1);
      // Echo the input with the green LED
      // printf("Button was pressed %d times\n", buttonPresses);
      guess[i] = buttonPresses;
      blinkN(gpio, LED, buttonPresses); // Blink green LED as per the input

      delay(2000); // Delay for 2 seconds
    }

    // Signal end of input sequence
    blinkN(gpio, LED2, 2);
    fprintf(stderr, "\n");
    fprintf(stderr, "your input sequence was %d %d %d! \n", guess[0], guess[1], guess[2]);

    // Calculate and show matches
    int *matches = countMatches(theSeq, guess);
    int exactMatches = matches[0];
    int approximateMatches = matches[1];

    // Communicate the match results
    blinkN(gpio, LED, exactMatches);       // Green LED for exact matches
    blinkN(gpio, LED2, 1);                 // Red LED as separator
    blinkN(gpio, LED, approximateMatches); // Green LED for approximate matches
    printf("Exact: %d, Approximate: %d\n", exactMatches, approximateMatches);

    // Display on LCD if connected
    if (lcd != NULL)
    {
      lcdClear(lcd);
      lcdPosition(lcd, 0, 0);
      char line1[16];
      snprintf(line1, sizeof(line1), "Exact matches: %d", exactMatches);
      lcdPuts(lcd, line1);

      lcdPosition(lcd, 0, 1);
      char line2[16];
      snprintf(line2, sizeof(line2), "Approx matches: %d", approximateMatches);
      lcdPuts(lcd, line2);
    }

    free(matches);

    if (exactMatches == SEQL)
    {
      found = 1;                  // Sequence guessed correctly
      blinkN(gpio, LED, 3);       // Blink green LED three times for success
      writeLED(gpio, LED2, HIGH); // Turn red LED on for success indication
      // Display success message on LCD
      if (lcd != NULL)
      {
        lcdClear(lcd);
        lcdPosition(lcd, 0, 0);
        lcdPuts(lcd, "SUCCESS");
        lcdPosition(lcd, 0, 1);
        char attemptsStr[16];
        snprintf(attemptsStr, sizeof(attemptsStr), "Attempts: %d", attempts);
        lcdPuts(lcd, attemptsStr);
      }
    }
    else
    {
      // Prepare for the next round
      blinkN(gpio, LED2, 3);
    }

    attempts++;
  }

  // clean up
  *(gpio + 10) = 1 << (LED & 31);
  *(gpio + 10) = 1 << (LED2 & 31);

  return 0;
}