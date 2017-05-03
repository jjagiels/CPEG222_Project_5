// CPEG222 - Project 5
// Authors: Justin Jagielski, Joaquin Martinez
// Inputs:  On-board buttons 1 and 2, PMOD MIC, PMOD LSensors
// Outputs: Two PmodSSDs, 8PLEDs, 2 Servos, On-board LEDs
// Program Description: Implements a line-following robot on the PIC32MX7 Hardware.

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
#define SYS_FREQ    (96000000L)   // 96MHz system clock

#define SAMPLE_FREQ 20000
#define T1_INTR_RATE (SYS_FREQ/256/SAMPLE_FREQ)

#define SSD_UPDATE_FREQ 85
#define T2_INTR_RATE (SYS_FREQ/256/SSD_UPDATE_FREQ)

#define TIMER_FREQ 10
#define CORE_TICK_RATE (SYS_FREQ/2/TIMER_FREQ)

#define MOTOR_UPDATE 62.5
#define MOTOR_TICK_RATE (SYS_FREQ/256/MOTOR_UPDATE)

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
#define LED2    LATGbits.LATG13
#define LED3    LATGbits.LATG14
#define LED4    LATGbits.LATG15

// SSD Pmod1 (2 rightmost SSDs)using the top rows of JA & JB jumpers
#define SegA1       LATBbits.LATB2
#define SegB1       LATBbits.LATB3
#define SegC1       LATBbits.LATB4
#define SegD1       LATBbits.LATB6
#define SegE1       LATEbits.LATE0
#define SegF1       LATEbits.LATE1
#define SegG1       LATEbits.LATE2
#define DispSel1    LATEbits.LATE3 //Select between the cathodes of the 2 SSDs


// 7 Segment Display pmod for using the top row JC & JD jumpers
// Segments
#define SegA 	LATCbits.LATC1
#define SegB	LATGbits.LATG0
#define SegC 	LATGbits.LATG1
#define SegD 	LATDbits.LATD7
#define SegE	LATDbits.LATD9
#define SegF	LATDbits.LATD0
#define SegG	LATCbits.LATC4
#define DispSel LATDbits.LATD10 //Select between the cathodes of the 2 SSDs

// LED Pmod attached to MX7 jumper JF
#define PLED1 LATFbits.LATF12
#define PLED2 LATFbits.LATF5
#define PLED3 LATFbits.LATF4
#define PLED4 LATFbits.LATF13
#define PLED5 LATEbits.LATE9
#define PLED6 LATAbits.LATA1
#define PLED7 LATAbits.LATA4
#define PLED8 LATAbits.LATA5

// Sensor Pmod connected to MX7 Jumper JE
#define Sensor1 PORTEbits.RE8
#define Sensor2 PORTAbits.RA7
#define Sensor3 PORTAbits.RA9
#define Sensor4 PORTAbits.RA10

/*    Definition of Modes    */
/*    Init/Left : Left digit display    */
/*   Right:      Right digit display   */
enum setDisplay {Left,Right};

//Subroutine, add yours for the project
void displayDigit(unsigned char, unsigned char, unsigned char); // Display one digit among two digits
void slowDownDisplay(unsigned char, unsigned char, unsigned char); // Frequency control function
void resetDisplay(void);
void performOp(void);
void toArray(short, char);
void resetAll(void);
int readADC(int ch);
void displaySigLevel(int);
void delay_ms(int);
void core_timer_interrupt_initialize(void);
//void timer1_interrupt_initialize(void);
void timer2_interrupt_initialize(void);
void timer3_interrupt_initialize(void);
void output_compare2_initialize(void);
void output_compare3_initialize(void);

// look-up table for the numbers
unsigned char number[]={
    0x3f,   //0   gfedcba
    0x30,   //1   0110000
    0x5B,   //2   1011011
    0x79,   //3   1111001
    0x74,   //4   1110100
    0x6D,   //5   1101101
    0x6F,   //6   1101111
    0x38,   //7   0111000
    0x7F,   //8   1111111
    0x7D,   //9   1111101
    0x77,   //A
    0x7C,   //B
    0x39,   //C
    0x5E,   //D
    0x79,   //E
    0x71,   //F
    0x00,    //clear
    0x76,    //H
    0x38,    //L
    0x4F,    //E
};

//variable definition
/*----------General variables----------*/
char btnLock = 0; //Used for debouncing
int i = 0; //for 'for' loops.
int LEDs = 0;
int amplitude = 0;
enum mode {stop, forward, left, right, reverse};
enum mode movementMode = stop;
char active = 0;

/*----------Variables for SSD display----------*/
unsigned short intDisplay = 0; //This is the number to be displayed on the SSDs
signed short total = 0;
unsigned char SSDisplay[4] = {16,16,16,0};
unsigned int display_value = 0; // The initial displayed value
unsigned int display_value1 = 0; //Initial displayed value for the rightmost SSD
enum setDisplay disp=Left; //Initial mode is left

/*----------Variables for Timer Counting----------*/
char tenthSec = 0;
short sec = 0;

/*----------Variables for reading from microphone----------*/
int micVal = 0;
int sigPeak = 310;
int sigOffset = 255;
int tcount = 0;

/*----------Debug Variables----------*/

int OCAccept = 0;

/*----------Variables for driving Servos----------*/

float DC2 = 0.0; //Duty Cycle for OC2
float DC3 = 0.0; //Duty Cycle for OC3

main(){
    //initialization
    INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR); //configure system for best performance
    INTEnableSystemMultiVectoredInt(); //enable multi-vector interrupts
    core_timer_interrupt_initialize();
    //timer1_interrupt_initialize(); //initialize timer 1
    timer2_interrupt_initialize(); //initialize timer 2
    timer3_interrupt_initialize(); //initialize timer 3
    output_compare2_initialize();
    output_compare3_initialize();
    SYSTEMConfig(SYS_FREQ, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);
    
	PORTClearBits (IOPORT_G, BIT_12|BIT_13| BIT_14|BIT_15); //Turn all 4 LEDs OFF
    PORTSetPinsDigitalIn (IOPORT_G, BIT_6|BIT_7); //Set Btn1 as input
    PORTSetPinsDigitalIn (IOPORT_A, BIT_0);
    DDPCONbits.JTAGEN = 0; //Shutoff JTAG !!!CRITICAL!!!
    PORTSetPinsDigitalOut (IOPORT_G, BIT_12|BIT_13| BIT_14|BIT_15); //Set LD1 through LD4 as digital output
    
    /*----------Set the 2 PMOD SSDs as output----------*/
    PORTSetPinsDigitalOut (IOPORT_B, BIT_2 |BIT_3|BIT_4|BIT_6); //Need to set these bits as outputs for the SSD, along with port B
    PORTSetPinsDigitalOut (IOPORT_E, BIT_0|BIT_1| BIT_2| BIT_3);
    PORTSetPinsDigitalOut (IOPORT_C, BIT_1|BIT_4);
    PORTSetPinsDigitalOut (IOPORT_G, BIT_0|BIT_1);
    PORTSetPinsDigitalOut (IOPORT_D, BIT_7|BIT_9|BIT_0|BIT_10);
    
    /*----------Set the PMOD LEDs as output----------*/

    PORTSetPinsDigitalOut (IOPORT_F, BIT_12| BIT_5| BIT_4| BIT_13);
    PORTSetPinsDigitalOut (IOPORT_E, BIT_9);
    PORTSetPinsDigitalOut (IOPORT_A, BIT_1| BIT_4| BIT_5);
    PLED1=PLED2=PLED3=PLED4=PLED5=PLED6=PLED7=PLED8=0; //zero out the LEDs
    
    /*----------Set the Pmod Sensor as input----------*/
    PORTSetPinsDigitalIn (IOPORT_A, BIT_7|BIT_9|BIT_10);
    PORTSetPinsDigitalIn (IOPORT_E, BIT_8);
    
    /*----------Set the Servo Driver Pmod as output----------*/
    PORTSetPinsDigitalOut (IOPORT_D, BIT_1| BIT_2);
    
    /*----------Set the Mic Pmod as input----------*/
    //PORTSetPinsAnalogIn (IOPORT_B, BIT_8);
    /*----------Configure Analog to Digital Converter----------*/
    /*AD1PCFGbits.PCFG8 = 0; // AN8 is an adc pin
    AD1CON3bits.ADCS = 2; // ADC clock period is Tad = 2*(ADCS+1)*Tpb = 2*3*12.5ns = 75ns
    AD1CON1bits.ADON = 1; // turn on A/D converter*/
    
    LED1=LED2=LED3=LED4=1;

    
    while(1){
        if((Btn1 || Btn2) && !btnLock){
            btnLock = 1;
            for(i; i<3000; i++){}
            i = 0;
            
            if(Btn1){
                if(!active){
                    active = 1;
                }
                switch(movementMode){ /*----------OC2 IS CURRENTLY SET TO BE THE RIGHTMOST SERVO, OC3 IS THE LEFTMOST SERVO----------*/
                    case stop:
                        movementMode = forward;
                        active = 1;
                        LED1=1;
                        LED2=LED3=LED4=0;
                        break;

                    case forward: /*----------!!!If Robot moves oddly, switch the forward and back Duty Cycle configs!!!----------*/
                        movementMode = right;
                        LED1=LED2=1;
                        LED3=LED4=0;
                        break;

                    case left:
                        movementMode = reverse;
                        LED1=LED2=LED3=0;
                        LED4=1;
                        break;

                    case right:
                        movementMode = left;
                        LED1=LED2=0;
                        LED3=LED4=1;
                        break;

                    case reverse: /*----------!!!If Robot moves oddly, switch the forward and back Duty Cycle configs!!!----------*/
                        movementMode = stop;
                        active = 0;
                        LED1=LED2=LED3=LED4=1;
                        break;

                    default:
                        break;
                }
            }
            else if(Btn2){
                resetDisplay();
            }
        }
        else if (!Btn1 && btnLock) { // When both buttons are off, unlock the buttons. 
                btnLock = 0;
        }
        LEDs = floor(((micVal-sigOffset)*8.0)/(sigPeak-sigOffset)+0.5);
        displaySigLevel(LEDs);

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
    tenthSec = 0;
    sec = 0;
    for(i; i<4; i++){
        if(i == 3){
            SSDisplay[i] = 0;
        }
        else{
            SSDisplay[i] = 16;
        }
    }
    i = 0;
}

void toArray(short sec, char tenthSec){ //converts an int to seperate numbers to place in an array
    sec = abs(sec);
    tenthSec = abs(tenthSec);
        for ( i = 2; i >= 0; i--, sec /= 10 )
        {
            SSDisplay[i] = sec % 10;
        }
        i=0;
        SSDisplay[3] = tenthSec;
        
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
    T4CON = 0x0; TMR4 = 0; T4CONSET = 0x8000; // 3. wait for sampling time
    while (TMR4 < 10); //
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

void core_timer_interrupt_initialize(void){ //Timer to control updating of SSDs. Pings at 75Hz
    OpenCoreTimer(CORE_TICK_RATE);
    mConfigIntCoreTimer((CT_INT_ON | CT_INT_PRIOR_6 | CT_INT_SUB_PRIOR_1));
}

/*void timer1_interrupt_initialize(void){ //Timer used to set the microphone sample rate. Pings at 20,000Hz
    OpenTimer1( (T2_ON | T1_SOURCE_INT | T1_PS_1_256), (T1_INTR_RATE) );
    
    mT1SetIntPriority(6);
    mT1SetIntSubPriority(2);
    mT1IntEnable(1);
}
*/
void timer2_interrupt_initialize(void){ //Timer used to control seconds counter. Pings every 100ms or 10Hz
    OpenTimer2( (T2_ON | T2_SOURCE_INT | T2_PS_1_256), (T2_INTR_RATE) );
    
    mT2SetIntPriority(7);
    mT2SetIntSubPriority(0);
    mT2IntEnable(1);
}

void timer3_interrupt_initialize(void){ //Timer used to drive the servos. Pings every 62.5Hz
    OpenTimer3( (T3_ON | T3_SOURCE_INT | T3_PS_1_256), (MOTOR_TICK_RATE - 1) );
    
    mT3SetIntPriority(4);
    mT3SetIntSubPriority(0);
    mT3IntEnable(1);
}

//TIMER 4 IS USED WITH THE ADC, SO IT IS RESERVED

void output_compare2_initialize(void){
    OpenOC2(OC_ON|OC_TIMER_MODE16|OC_TIMER3_SRC|OC_PWM_FAULT_PIN_DISABLE, 0, 0);
}

void output_compare3_initialize(void){
    OpenOC3(OC_ON|OC_TIMER_MODE16|OC_TIMER3_SRC|OC_PWM_FAULT_PIN_DISABLE, 0, 0);
}

void __ISR(_CORE_TIMER_VECTOR, IPL6SOFT) coreTimerHandler(void){ //Displaying on SSDs
    
        if(active){
            if(tenthSec == 9){
                tenthSec = 0;
                sec++;
            }
            else{
                tenthSec++;
            }
        
        if(sec > 999){
            sec = 0;
        }
    }
       
    mCTClearIntFlag();  // Clear the interrupt flag
    UpdateCoreTimer(CORE_TICK_RATE);

}
/*void __ISR(_TIMER_1_VECTOR, IPL6SOFT) Timer1Handler(void){ //Reading from microphone
    micVal = readADC(3); // sample and convert pin 3
    
    if(active){
        CloseTimer1();
    }
    mT1ClearIntFlag();
}
*/
void __ISR(_TIMER_2_VECTOR, IPL7SOFT) Timer2Handler(void){ //Counting Time
    
    slowDownDisplay(disp==Left, number[display_value], number[display_value1]);   // debouncing & display digit

    toArray(sec, tenthSec);
    
    if(disp==Left) //display on the right side SSD
    {
        disp=Right;display_value=SSDisplay[2];display_value1=SSDisplay[0];
    }
    else //display on the left side SSD
    {
        disp=Left;display_value=SSDisplay[3];display_value1=SSDisplay[1];
    }
    
    mT2ClearIntFlag();
}

void __ISR(_TIMER_3_VECTOR, IPL4SOFT) Timer3Handler(void){ //Settings for PWM
    switch(movementMode){ /*----------OC2 IS CURRENTLY SET TO BE THE RIGHTMOST SERVO, OC3 IS THE LEFTMOST SERVO----------*/
        case stop:
            DC2 = 9.375; //Stop
            DC3 = 9.375; //Stop
            SetDCOC2PWM((MOTOR_TICK_RATE + 1) * ((float)DC2 / 100));
            SetDCOC3PWM((MOTOR_TICK_RATE + 1) * ((float)DC3 / 100));

            break;
        
        case forward:
            DC2 = 5.625; //CW
            DC3 = 13.125; //CCW
            SetDCOC2PWM((MOTOR_TICK_RATE + 1) * ((float)DC2 / 100));
            SetDCOC3PWM((MOTOR_TICK_RATE + 1) * ((float)DC3 / 100));

            break;
        
        case left:
            DC2 = 5.625; //CW
            DC3 = 9.375; //Stop
            SetDCOC2PWM((MOTOR_TICK_RATE + 1) * ((float)DC2 / 100));
            SetDCOC3PWM((MOTOR_TICK_RATE + 1) * ((float)DC3 / 100));

            break;
        
        case right:
            DC2 = 9.375; //Stop
            DC3 = 13.125; //CCW
            SetDCOC2PWM((MOTOR_TICK_RATE + 1) * ((float)DC2 / 100));
            SetDCOC3PWM((MOTOR_TICK_RATE + 1) * ((float)DC3 / 100));

            break;
        
        case reverse:
            DC2 = 13.125; //CCW
            DC3 = 5.625; //CW
            SetDCOC2PWM((MOTOR_TICK_RATE + 1) * ((float)DC2 / 100));
            SetDCOC3PWM((MOTOR_TICK_RATE + 1) * ((float)DC3 / 100));

            break;
        
        default:
            break;
    }
    mT3ClearIntFlag();
}

