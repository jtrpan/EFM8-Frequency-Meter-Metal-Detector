#include <EFM8LB1.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

#define SYSCLK      72000000L  // SYSCLK frequency in Hz
#define BAUDRATE      115200L  // Baud rate of UART in

#define RA 2000
#define RB 2000

#define LCD_RS P2_6
// #define LCD_RW Px_x // Not used in this code.  Connect to GND
#define LCD_E  P2_5
#define LCD_D4 P2_4
#define LCD_D5 P2_3
#define LCD_D6 P2_2
#define LCD_D7 P2_1

#define LED_GREEN P2_0
#define LED_RED P1_7
#define CAL_BUTTON P1_5
#define DET_BUTTON P1_4
#define SPEAKER P1_3

#define CHARS_PER_LINE 16

unsigned char overflow_count;

char _c51_external_startup (void)
{
    // Disable Watchdog with key sequence
    SFRPAGE = 0x00;
    WDTCN = 0xDE; //First key
    WDTCN = 0xAD; //Second key

    VDM0CN |= 0x80;
    RSTSRC = 0x02;

#if (SYSCLK == 48000000L)
    SFRPAGE = 0x10;
		PFE0CN  = 0x10; // SYSCLK < 50 MHz.
		SFRPAGE = 0x00;
#elif (SYSCLK == 72000000L)
    SFRPAGE = 0x10;
    PFE0CN  = 0x20; // SYSCLK < 75 MHz.
    SFRPAGE = 0x00;
#endif

#if (SYSCLK == 12250000L)
    CLKSEL = 0x10;
		CLKSEL = 0x10;
		while ((CLKSEL & 0x80) == 0);
#elif (SYSCLK == 24500000L)
    CLKSEL = 0x00;
		CLKSEL = 0x00;
		while ((CLKSEL & 0x80) == 0);
#elif (SYSCLK == 48000000L)
    // Before setting clock to 48 MHz, must transition to 24.5 MHz first
		CLKSEL = 0x00;
		CLKSEL = 0x00;
		while ((CLKSEL & 0x80) == 0);
		CLKSEL = 0x07;
		CLKSEL = 0x07;
		while ((CLKSEL & 0x80) == 0);
#elif (SYSCLK == 72000000L)
    // Before setting clock to 72 MHz, must transition to 24.5 MHz first
    CLKSEL = 0x00;
    CLKSEL = 0x00;
    while ((CLKSEL & 0x80) == 0);
    CLKSEL = 0x03;
    CLKSEL = 0x03;
    while ((CLKSEL & 0x80) == 0);
#else
#error SYSCLK must be either 12250000L, 24500000L, 48000000L, or 72000000L
#endif

    P0MDOUT |= 0x10; // Enable UART0 TX as push-pull output
    XBR0     = 0x01; // Enable UART0 on P0.4(TX) and P0.5(RX)
    XBR1     = 0X10; // Enable T0 on P0.0
    XBR2     = 0x40; // Enable crossbar and weak pull-ups

#if (((SYSCLK/BAUDRATE)/(2L*12L))>0xFFL)
#error Timer 0 reload value is incorrect because (SYSCLK/BAUDRATE)/(2L*12L) > 0xFF
#endif
    // Configure Uart 0
    SCON0 = 0x10;
    CKCON0 |= 0b_0000_0000 ; // Timer 1 uses the system clock divided by 12.
    TH1 = 0x100-((SYSCLK/BAUDRATE)/(2L*12L));
    TL1 = TH1;      // Init Timer1
    TMOD &= ~0xf0;  // TMOD: timer 1 in 8-bit auto-reload
    TMOD |=  0x20;
    TR1 = 1; // START Timer1
    TI = 1;  // Indicate TX0 ready

    return 0;
}


void InitADC (void)
{
    SFRPAGE = 0x00;
    ADC0CN1 = 0b_10_000_000; //14-bit,  Right justified no shifting applied, perform and Accumulate 1 conversion.
    ADC0CF0 = 0b_11111_0_00; // SYSCLK/32
    ADC0CF1 = 0b_0_0_011110; // Same as default for now
    ADC0CN0 = 0b_0_0_0_0_0_00_0; // Same as default for now
    ADC0CF2 = 0b_0_01_11111 ; // GND pin, Vref=VDD
    ADC0CN2 = 0b_0_000_0000;  // Same as default for now. ADC0 conversion initiated on write of 1 to ADBUSY.
    ADEN=1; // Enable ADC
}

// Uses Timer3 to delay <us> micro-seconds.
void Timer3us(unsigned char us)
{
    unsigned char i;               // usec counter

    // The input for Timer 3 is selected as SYSCLK by setting T3ML (bit 6) of CKCON0:
    CKCON0|=0b_0100_0000;

    TMR3RL = (-(SYSCLK)/1000000L); // Set Timer3 to overflow in 1us.
    TMR3 = TMR3RL;                 // Initialize Timer3 for first overflow

    TMR3CN0 = 0x04;                 // Sart Timer3 and clear overflow flag
    for (i = 0; i < us; i++)       // Count <us> overflows
    {
        while (!(TMR3CN0 & 0x80));  // Wait for overflow
        TMR3CN0 &= ~(0x80);         // Clear overflow indicator
        if (TF0)
        {
            TF0=0;
            overflow_count++;
        }
    }
    TMR3CN0 = 0 ;                   // Stop Timer3 and clear overflow flag
}

void waitms (unsigned int ms)
{
    unsigned int j;
    for(j=ms; j!=0; j--)
    {
        Timer3us(249);
        Timer3us(249);
        Timer3us(249);
        Timer3us(250);
    }
}

#define VDD 3.3035 // The measured value of VDD in volts

void InitPinADC (unsigned char portno, unsigned char pin_num)
{
    unsigned char mask;

    mask=1<<pin_num;

    SFRPAGE = 0x20;
    switch (portno)
    {
        case 0:
            P0MDIN &= (~mask); // Set pin as analog input
            P0SKIP |= mask; // Skip Crossbar decoding for this pin
            break;
        case 1:
            P1MDIN &= (~mask); // Set pin as analog input
            P1SKIP |= mask; // Skip Crossbar decoding for this pin
            break;
        case 2:
            P2MDIN &= (~mask); // Set pin as analog input
            P2SKIP |= mask; // Skip Crossbar decoding for this pin
            break;
        default:
            break;
    }
    SFRPAGE = 0x00;
}

unsigned int ADC_at_Pin(unsigned char pin)
{
    ADC0MX = pin;   // Select input from pin
    ADBUSY=1;       // Dummy conversion first to select new pin
    while (ADBUSY); // Wait for dummy conversion to finish
    ADBUSY = 1;     // Convert voltage at the pin
    while (ADBUSY); // Wait for conversion to complete
    return (ADC0);
}

float Volts_at_Pin(unsigned char pin)
{
    return ((ADC_at_Pin(pin)*VDD)/16383.0);
}

void LCD_pulse (void)
{
    LCD_E=1;
    Timer3us(40);
    LCD_E=0;
}

void LCD_byte (unsigned char x)
{
    // The accumulator in the C8051Fxxx is bit addressable!
    ACC=x; //Send high nible
    LCD_D7=ACC_7;
    LCD_D6=ACC_6;
    LCD_D5=ACC_5;
    LCD_D4=ACC_4;
    LCD_pulse();
    Timer3us(40);
    ACC=x; //Send low nible
    LCD_D7=ACC_3;
    LCD_D6=ACC_2;
    LCD_D5=ACC_1;
    LCD_D4=ACC_0;
    LCD_pulse();
}

void WriteData (unsigned char x)
{
    LCD_RS=1;
    LCD_byte(x);
    waitms(2);
}

void WriteCommand (unsigned char x)
{
    LCD_RS=0;
    LCD_byte(x);
    waitms(5);
}

void LCD_4BIT (void)
{
    LCD_E=0; // Resting state of LCD's enable is zero
    // LCD_RW=0; // We are only writing to the LCD in this program
    waitms(20);
    // First make sure the LCD is in 8-bit mode and then change to 4-bit mode
    WriteCommand(0x33);
    WriteCommand(0x33);
    WriteCommand(0x32); // Change to 4-bit mode

    // Configure the LCD
    WriteCommand(0x28);
    WriteCommand(0x0c);
    WriteCommand(0x01); // Clear screen command (takes some time)
    waitms(20); // Wait for clear screen command to finsih.
}

void LCDprint(char * string, unsigned char line, bit clear)
{
    int j;
    WriteCommand(line==2?0xc0:0x80);
    waitms(5);
    for(j=0; string[j]!=0; j++)	WriteData(string[j]);// Write the message
    if(clear) for(; j<CHARS_PER_LINE; j++) WriteData(' '); // Clear the rest of the line
}

void TIMER0_Init(void)
{
    TMOD&=0b_1111_0000; // Set the bits of Timer/Counter 0 to zero
    TMOD|=0b_0000_0101; // Timer/Counter 0 used as a 16-bit counter
    TR0=0; // Stop Timer/Counter 0
}

int checkFreq(unsigned long F, unsigned long stableFreq) {
    int retVal = 0;
    if (F > stableFreq) {
        if (F - stableFreq > 12000){
            retVal = -1;
            LED_GREEN = 0;
            LED_RED = 0;
            SPEAKER = 1;
        } else if (F - stableFreq > 3350){
            retVal = 1;
            LED_GREEN = 0;
            LED_RED = 1;
            SPEAKER = 0;
        } else {
            retVal = 0;
            LED_GREEN = 1;
            LED_RED = 1;
            SPEAKER = 1;
        }
    } else {
        if (stableFreq - F > 4000) {
            retVal = -1;
            LED_GREEN = 0;
            LED_RED = 0;
            SPEAKER = 1;
        } else {
            retVal = 0;
            LED_GREEN = 1;
            LED_RED = 1;
            SPEAKER = 1;
        }
    }
    return retVal;
}

void main (void)
{
    unsigned long F;
    unsigned long stableFreq = 0;
    unsigned long sum = 0;
    int isMetal = 0;    // 0 for no metal, 1 for metal, -1 for error
    int mode = 1;       // mode 0 is waiting, mode 1 is calibration, mode 2 is detection
    int loopCount = 0;

    char buff1[17] = ("Metal Detector");
    char buff2[17] = ("Initializing...");
    char fHolder[4];
    char refHolder[4];

    // LED: 0 = on, 1 = off
    LED_RED = 1;
    LED_GREEN = 1;
    SPEAKER = 1;        // SPEAKER: 0 = ON, 1 = OFF
    TIMER0_Init();

    // Configure the LCD
    LCD_4BIT();
    LCDprint(buff1, 1, 1);
    LCDprint(buff2, 2, 1);

    // Display something in the LCD
    printf("\x1b[2J"); // Clear screen using ANSI escape sequence.
    printf("Project 2 Initializing...\n\n");
    waitms(1000); // Give PuTTY a chance to start.
    printf("\x1b[2J"); // Clear screen using ANSI escape sequence.
    InitADC();

    while(1)
    {
        strcpy(buff1, "");
        strcpy(buff2, "");
        if (mode == 0){
            strcpy(buff1, "P2: Detect Metal");
            strcpy(buff2, "Press a button.");
            LED_RED = 1;
            LED_GREEN = 1;
            SPEAKER = 1;

            if (CAL_BUTTON == 0 && DET_BUTTON == 1){
                mode = 1;           // go into calibration mode
                loopCount = 0;
                sum = 0;
                strcpy(buff1, "Calibration");
                strcpy(buff2, "Starting...");
                LCDprint(buff1, 1, 1);
                LCDprint(buff2, 2, 1);
                LED_RED = 0;
                LED_GREEN = 0;
                SPEAKER = 1;
                waitms(300); // Give PuTTY a chance to start.
                LED_RED = 1;
                LED_GREEN = 1;
                SPEAKER = 1;
            }

            if (DET_BUTTON == 0 && CAL_BUTTON == 1){
                mode = 2;           // go into detection mode
                strcpy(buff1, "Detection");
                strcpy(buff2, "Starting...");
                LCDprint(buff1, 1, 1);
                LCDprint(buff2, 2, 1);
                LED_RED = 1;
                LED_GREEN = 1;
                SPEAKER = 1;
                waitms(300); // Give PuTTY a chance to start.
                LED_RED = 0;
                LED_GREEN = 0;
                SPEAKER = 1;
                waitms(300); // Give PuTTY a chance to start.
                LED_RED = 1;
                LED_GREEN = 1;
                SPEAKER = 1;
            }

        } else if (mode == 1){
            TL0=0;
            TH0=0;
            overflow_count=0;
            TF0=0;
            TR0=1; // Start Timer/Counter 0
            waitms(1000);
            TR0=0; // Stop Timer/Counter 0
            F=overflow_count*0x10000L+TH0*0x100L+TL0;

            strcpy(buff1, "Calibrating...");
            strcpy(buff2, "F:");

            loopCount++;
            sum = sum + F;
            stableFreq = sum/loopCount;

            printf("\rf=%luHz", F);
            printf("\x1b[0K"); // ANSI: Clear from cursor to end of line.
            printf("\n\rStable F=%luHz", stableFreq);
            printf("\x1b[0K"); // ANSI: Clear from cursor to end of line.

            sprintf(fHolder, "%lu", F/1000);
            sprintf(refHolder, "%lu", stableFreq/1000);

            strncat(buff2, fHolder, 4);
            strncat(buff2, "kHz ", 5);
            strncat(buff2, "R:", 4);
            strncat(buff2, refHolder, 4);
            strncat(buff2, "kHz", 4);

            if (loopCount > 20){
                mode = 0;
            }

        } else if (mode == 2){
            TL0=0;
            TH0=0;
            overflow_count=0;
            TF0=0;
            TR0=1; // Start Timer/Counter 0
            waitms(1000);
            TR0=0; // Stop Timer/Counter 0
            F=overflow_count*0x10000L+TH0*0x100L+TL0;

            strcpy(buff2, "F:");

            printf("\rf=%luHz", F);
            printf("\x1b[0K"); // ANSI: Clear from cursor to end of line.
            printf("\n\rStable F=%luHz", stableFreq);
            printf("\x1b[0K"); // ANSI: Clear from cursor to end of line.

            isMetal = checkFreq(F, stableFreq);
            if (isMetal == 1){
                strcpy(buff1, "Metal Detected!");
            } else if (isMetal == 0){
                strcpy(buff1, "Searching...");
            } else if (isMetal == -1){
                strcpy(buff1, "Interference!");
            } else {
                strcpy(buff1, "ERROR!");
            }

            sprintf(fHolder, "%lu", F/1000);
            sprintf(refHolder, "%lu", stableFreq/1000);
            
            strncat(buff2, fHolder, 4);
            strncat(buff2, "kHz ", 5);
            strncat(buff2, "R:", 4);
            strncat(buff2, refHolder, 4);
            strncat(buff2, "kHz", 4);

            if (DET_BUTTON == 0 && CAL_BUTTON == 0){
                mode = 0;           // go into main mode
            }

        } else {
            strcpy(buff1, "ERROR!");
            strcpy(buff2, "ERROR!");
        }
        LCDprint(buff1, 1, 1);
        LCDprint(buff2, 2, 1);
    }

}
