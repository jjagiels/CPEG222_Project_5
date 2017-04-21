// CPEG222 - Project 3
// Authors: Justin Jagielski, Joaquin Martinez
// Inputs:  On-board buttons 1 and 2, PMODkypd
// Outputs: Two PmodSSDs
// Program Description: Implements common calculator functions using the Keypad PMOD as input and 2 PMOD SSDs as output.

#include <p32xxxx.h>
#include <plib.h>
#include <math.h>                   //include math library
#include "fftc.h"                   //FFT library
#include "dsplib_dsp.h"             //Digital Signal Processing Library

// Configuration Bit settings (Don't touch them if not necessary)
// SYSCLK = 80 MHz (8MHz Crystal/ FPLLIDIV * FPLLMUL / FPLLODIV)
// PBCLK = 40 MHz
// Primary Osc w/PLL (XT+,HS+,EC+PLL)
//#define SYS_FREQ     80000000L

//#pragma config FPLLMUL = MUL_20, FPLLIDIV = DIV_2, FPLLODIV = DIV_1, FWDTEN = OFF//not changed
//#pragma config POSCMOD = HS, FNOSC = PRIPLL,   FPBDIV = DIV_2// set PBCLK as 40MHZ


#pragma config ICESEL = ICS_PGx1	//configure on board licensed debugger
#pragma config FNOSC = PRIPLL		//configure system clock 96 MHz
#pragma config POSCMOD = EC, FPLLIDIV = DIV_2,FPLLMUL = MUL_24,FPLLODIV = DIV_1
#pragma config FPBDIV = DIV_1		//configure peripheral bus clock to 96 MHz
#define SYS_FREQ    (96000000L)   // 160MHz system clock
#define T1_INTR_RATE 4166
#define T2_INTR_RATE 600
#define SAMPLE_FREQ 20000

//FFT Definitions
#define N 2048 //the number of samples stored (needs to be 2^n for FFT)
#define fftc fft16c2048 //FFT for 1024 samples on 16 bit data

// The speed control
#define DELAY 100
// The refresh rate of the segments
#define REFRESH_RATE 200   // ~1/1000 secs

/****************************************************/
/* Port Definition                           */
/*    You should add your port definition if necessary*/
/****************************************************/

// Buttons
#define Btn1	PORTGbits.RG6
#define Btn2    PORTGbits.RG7
#define Btn3    PORTAbits.RA0

#define LED1    LATGbits.LATG12

// SSD Pmod1 (2 rightmost SSDs)using the bottom rows of JA & JB jumpers
#define SegA1       LATBbits.LATB7
#define SegB1       LATBbits.LATB8
#define SegC1       LATBbits.LATB9
#define SegD1       LATBbits.LATB10
#define SegE1       LATEbits.LATE4
#define SegF1       LATEbits.LATE5
#define SegG1       LATEbits.LATE6
#define DispSel1    LATEbits.LATE7 //Select between the cathodes of the 2 SSDs


// 7 Segment Display pmod for using the bottom row JC & JD jumpers
// Segments
#define SegA 	LATBbits.LATB15
#define SegB	LATDbits.LATD5
#define SegC 	LATDbits.LATD4
#define SegD 	LATBbits.LATB14
#define SegE	LATDbits.LATD1
#define SegF	LATDbits.LATD2
#define SegG	LATDbits.LATD3
// Display selection. 0 = right, 1 = left (Cathode)
#define DispSel PORTDbits.RD12

// LED Pmod attached to MX7 jumper JF
#define PLED1 LATFbits.LATF12
#define PLED2 LATFbits.LATF5
#define PLED3 LATFbits.LATF4
#define PLED4 LATFbits.LATF13
#define PLED5 LATEbits.LATE9
#define PLED6 LATAbits.LATA1
#define PLED7 LATAbits.LATA4
#define PLED8 LATAbits.LATA5


/*    Definition of Modes    */
/*    Init/Left : Left digit display    */
/*   Right:      Right digit display   */
enum setDisplay {Left,Right};

//Subroutine, add yours for the project
void displayDigit(unsigned char, unsigned char, unsigned char); // Display one digit among two digits
void slowDownDisplay(unsigned char, unsigned char, unsigned char); // Frequency control function
void resetDisplay(void);
void performOp(void);
void toArray(int);
void resetAll(void);
int readADC(int ch);
void displaySigLevel(int);
void delay_ms(int);
int computeFFT(int16c*);
void timer1_interrupt_initialize(void);
void timer2_interrupt_initialize(void);

// look-up table for the numbers
unsigned char number[]={
    0x3f,   //0
    0x06,   //1
    0x5B,   //2
    0x4F,   //3
    0x66,   //4
    0x6D,   //5
    0x7D,   //6
    0x07,   //7
    0x7F,   //8
    0x6F,   //9
    0x77,   //A
    0x7C,   //B
    0x39,   //C
    0x5E,   //D
    0x79,   //E
    0x71,   //F
    0x00,    //clear
    0x76,    //H
    0x38,    //L
    0x79,    //E
};

//variable definition
/*----------General variables----------*/
char mode = 1;
char btnLock = 0; //Used for debouncing
int i = 0; //for 'for' loops.
int lastRow = 0;
int lastNum = 0;
char ledActive = 1;
int LEDs = 0;
int amplitude = 0;

/*----------Variables for SSD display----------*/
unsigned short intDisplay = 0; //This is the number to be displayed on the SSDs
signed short total = 0;
unsigned char SSDisplay[4] = {16,16,16,0};
signed char currentDispLoc = 3;
unsigned int display_value = 0; // The initial displayed value
unsigned int display_value1 = 0; //Initial displayed value for the rightmost SSD
enum setDisplay disp=Left; //Initial mode is left

/*----------Variables for reading from microphone----------*/
int micVal = 0;
int sigPeak = 540;
int sigOffset = 410;
char mute = 1; //system starts muted
int tcount = 0;

/*----------Variables for computing FFT----------*/
int log2N = 11; // i.e. 2^11 = N = 2048
int16c sampleBuffer[N];
int16c scratch[N]; //intermediate array
int16c dout[N]; // intermediate array holds computed FFT until transmission
int singleSidedFFT[N]; //intermediate array holds computed single side FFT
short freqVector[N]; //array that stores the corresponding frequency of each point in frequency domain
short freqIndex = 0;
int finalFreq = 0;


main(){
    //initialization
    INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR); //configure system for best performance
    INTEnableSystemMultiVectoredInt(); //enable multi-vector interrupts
    timer1_interrupt_initialize(); //initialize timer 1
    timer2_interrupt_initialize(); //initialize timer 2
    SYSTEMConfig(SYS_FREQ, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);
    
	PORTClearBits (IOPORT_G, BIT_12|BIT_13| BIT_14|BIT_15); //Turn all 4 LEDs OFF
    PORTSetPinsDigitalIn (IOPORT_G, BIT_6); //Set Btn1 as input
    DDPCONbits.JTAGEN = 0; //Shutoff JTAG !!!CRITICAL!!!
    PORTSetPinsDigitalOut (IOPORT_G, BIT_12|BIT_13| BIT_14|BIT_15); //Set LD1 through LD4 as digital output
    
    /*----------Set the 2 PMOD SSDs as output----------*/
    PORTSetPinsDigitalOut (IOPORT_E, BIT_4 |BIT_5|BIT_6|BIT_7); //Need to set these bits as outputs for the SSD, along with port B
    PORTSetPinsDigitalOut (IOPORT_B, BIT_7|BIT_8| BIT_9| BIT_10| BIT_14| BIT_15);
    PORTSetPinsDigitalOut (IOPORT_D, BIT_1|BIT_2| BIT_3|BIT_4 |BIT_5| BIT_12);
    PORTB = 0;      // initialize PORTG to 0
    PORTD = 0;      // initialize PortD to 0
    
    /*----------Set the PMOD LEDs as output----------*/

    PORTSetPinsDigitalOut (IOPORT_F, BIT_12| BIT_5| BIT_4| BIT_13);
    PORTSetPinsDigitalOut (IOPORT_E, BIT_9);
    PORTSetPinsDigitalOut (IOPORT_A, BIT_1| BIT_4| BIT_5);
    PLED1=PLED2=PLED3=PLED4=PLED5=PLED6=PLED7=PLED8=0; //zero out the LEDs
    
    /*----------Configure Analog to Digital Converter----------*/
    AD1PCFGbits.PCFG3 = 0; // AN3 is an adc pin
    AD1CON3bits.ADCS = 2; // ADC clock period is Tad = 2*(ADCS+1)*Tpb = 2*3*12.5ns = 75ns
    AD1CON1bits.ADON = 1; // turn on A/D converter

    for(i=0; i< (N/2);i++){
        freqVector[i] = i*(SAMPLE_FREQ/2) / ((N/2) - 1);
    }
    
    while(1){
        
        if(Btn1 && !btnLock){
            for(i=0;i<800;i++){}
            btnLock = 1;
            if(mute){
                LED1 = 1;
            }
            else{
                LED1 = 0;
            }
            mute = !mute;
        }
        else if(!Btn1 && btnLock){
            btnLock = 0;
        }
        if(tcount == N){
            freqIndex = computeFFT(sampleBuffer);
            for(i = 0; i<N; i++){
                if(sampleBuffer[i].re > amplitude){
                    amplitude = sampleBuffer[i].re;
                }
            }
            LEDs = floor(((amplitude-sigOffset)*8.0)/(sigPeak-sigOffset)+0.5);
            finalFreq = ((freqVector[freqIndex]));
            toArray(finalFreq);
            tcount = 0;
            amplitude = 0;
        }
    }
}

//display digit
void displayDigit (unsigned char display_sel, unsigned char value, unsigned char value1){ //used for displaying numbers on the SSDs
    
    DispSel = display_sel;
    DispSel1 = display_sel;
    SegA    = value & 1;
    SegB    = (value >> 1) & 1;
    SegC    = (value >> 2) & 1;
    SegD    = (value >> 3) & 1;
    SegE    = (value >> 4) & 1;
    SegF    = (value >> 5) & 1;
    SegG    = (value >> 6) & 1;
    
    SegA1    = value1 & 1;
    SegB1    = (value1 >> 1) & 1;
    SegC1    = (value1 >> 2) & 1;
    SegD1    = (value1 >> 3) & 1;
    SegE1    = (value1 >> 4) & 1;
    SegF1    = (value1 >> 5) & 1;
    SegG1    = (value1 >> 6) & 1;
}

// debouncing keys & display
void slowDownDisplay(unsigned char display_sel, unsigned char value, unsigned char value1){
    unsigned int slow = 0;
    while(slow < REFRESH_RATE){
        displayDigit(display_sel,value,value1);
        slow++;
    }
}

void resetDisplay(void){ //zeroes out all relevant displaying variables and arrays
    intDisplay = 0;
    for(i; i<4; i++){
        if(i == 3){
            SSDisplay[i] = 0;
        }
        else{
            SSDisplay[i] = 16;
        }
    }
    i = 0;
    currentDispLoc = 3;
}

void toArray(int number){ //converts an int to seperate numbers to place in an array
    number = abs(number);
        for ( i = 3; i >= 0; i--, number /= 10 )
        {
            SSDisplay[i] = number % 10;
        }
        i = 0;
        
        for(i = 0; i < 3; i++){
            if(SSDisplay[i] != 0){
            break;
            }
        else{
            SSDisplay[i] = 16;
            }

        }
    }
void resetAll(void){ //resets all values

}

int readADC(int ch){
    AD1CHSbits.CH0SA = ch; // 1. select analog input
    AD1CON1bits.SAMP = 1; // 2. start sampling
    T3CON = 0x0; TMR3 = 0; T3CONSET = 0x8000; // 3. wait for sampling time
    while (TMR3 < 10); //
    AD1CON1bits.SAMP = 0; // 4. start the conversion
    while (!AD1CON1bits.DONE); // 5. wait conversion complete
    return ADC1BUF0; // 6. read result
}

void delay_ms(int ms){
    int j = 0;
    int counter = 0;
    for(counter = 0; counter<ms; counter++){
        for(j=0;j<1250;i++){} //software delay 1 millisecond
    }
}

void displaySigLevel(int volume){
    if(volume > 8){
        volume = 8;
    }else if(volume < 0){
        volume = 0;
    }
    switch(volume){
        case 0:PLED1=PLED2=PLED3=PLED4=PLED5=PLED6=PLED7=PLED8=0;break;
        case 1:PLED1=1;PLED2=PLED3=PLED4=PLED5=PLED6=PLED7=PLED8=0;break;
        case 2:PLED1=PLED2=1;PLED3=PLED4=PLED5=PLED6=PLED7=PLED8=0;break;
        case 3:PLED1=PLED2=PLED3=1;PLED4=PLED5=PLED6=PLED7=PLED8=0;break;
        case 4:PLED1=PLED2=PLED3=PLED4=1;PLED5=PLED6=PLED7=PLED8=0;break;
        case 5:PLED1=PLED2=PLED3=PLED4=PLED5=1;PLED6=PLED7=PLED8=0;break;
        case 6:PLED1=PLED2=PLED3=PLED4=PLED5=PLED6=1;PLED7=PLED8=0;break;
        case 7:PLED1=PLED2=PLED3=PLED4=PLED5=PLED6=PLED7=1;PLED8=0;break;
        case 8:PLED1=PLED2=PLED3=PLED4=PLED5=PLED6=PLED7=PLED8=1;break;
        default:break;
    }
}

int computeFFT(int16c *sampleBuffer){
    int dominantFreq = 1;
    
    mips_fft16(dout, sampleBuffer, fftc, scratch, log2N); //compute N point FFT
    
    for(i = 0; i < (N/2); i++){
        singleSidedFFT[i] = 2 * ((dout[i].re * dout[i].re) + (dout[i].im * dout[i].im));
    }
    
    for(i = 1; i < (N/2); i++){
        if(singleSidedFFT[dominantFreq] < singleSidedFFT[i]){
            dominantFreq = i;
        }
    }
    return dominantFreq;
}

void timer1_interrupt_initialize(void){
    OpenTimer1( (T1_ON | T1_SOURCE_INT | T1_PS_1_256), (T1_INTR_RATE - 1) );
    
    mT1SetIntPriority(2);
    mT1SetIntSubPriority(2);
    mT1IntEnable(1);
}

void timer2_interrupt_initialize(void){
    OpenTimer2( (T2_ON | T2_SOURCE_INT | T2_PS_1_8), (T2_INTR_RATE - 1) );
    
    mT2SetIntPriority(2);
    mT2SetIntSubPriority(3);
    mT2IntEnable(1);
}

void __ISR(_TIMER_1_VECTOR, IPL2SOFT) TimerHandler(void){
    
    slowDownDisplay(disp==Left, number[display_value], number[display_value1]);   // debouncing & display digit

    
    if(disp==Left) //display on the right side SSD
    {
        disp=Right;display_value=SSDisplay[1];display_value1=SSDisplay[3];
    }
    else //display on the left side SSD
    {
        disp=Left;display_value=SSDisplay[0];display_value1=SSDisplay[2];
    }
    displaySigLevel(LEDs);
    
    
    mT1ClearIntFlag();  // Clear the interrupt flag

}
void __ISR(_TIMER_2_VECTOR, IPL2SOFT) Timer2Handler(void){
    if((!mute) && (tcount != N)){
        micVal = readADC(3); // sample and convert pin 3
        sampleBuffer[tcount].re = micVal;  
        tcount++;
    }
    
    mT2ClearIntFlag();
}

