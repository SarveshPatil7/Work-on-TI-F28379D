//#############################################################################
// FILE:   LABstarter_main.c
//
// TITLE:  Lab Starter
//#############################################################################

// Included Files
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"
#include "F28379dSerial.h"
#include "LEDPatterns.h"
#include "song.h"
#include "dsp.h"
#include "fpu32/fpu_rfft.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
// The Launchpad's CPU Frequency set to 200 you should not change this value
#define LAUNCHPAD_CPU_FREQUENCY 200

// BM: new definitions
#define XKS_SIZE    22
#define XKS_SIZE1   32
#define XKS_SIZE2   101

// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);

// BM: new function declarations
void setDACA(float dacouta0);
void setDACB(float dacouta1);
__interrupt void ADCD_ISR (void);
__interrupt void ADCA_ISR (void);
__interrupt void ADCB_ISR (void);

void setEPWM8A_RCServo(float angle);
void setEPWM8B_RCServo(float angle);

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
extern uint32_t numRXA;
uint16_t UARTPrint = 0;
uint16_t LEDdisplaynum = 0;

// BM: new global variables
int16_t adcd0result;
int16_t adcd1result;
int16_t adca0result;
int16_t adca1result;
int16_t adcb0result;
int16_t adcb1result;
float volt_adcd0;
float volt_adcd1;
float volt_adca0;
float volt_adca1;
int16_t counter = 0;
float xks0[XKS_SIZE] = {0};
float xks1[XKS_SIZE] = {0};
float yk0 = 0;
float yk1 = 0;
float yk3 = 0;

// BM: new global variables
uint16_t count = 0;
uint16_t angle_count = 0;
uint16_t song_count = 0;
float control_CMPA = 10.0;
float control_CMPB = -10.0;
float control_count = 0;
float control_angle0 = 0.0;
float control_angle1 = 0.0;

float thumb = -100.0;
float finger = -100.0;
float thumb_init = 190.0;
float finger_init = 100.0;


void main(void)
{
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xD_SysCtrl.c file.
    InitSysCtrl();

    InitGpio();
	
	// Blue LED on LaunchPad
    GPIO_SetupPinMux(31, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO31 = 1;

	// Red LED on LaunchPad
    GPIO_SetupPinMux(34, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(34, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO34 = 1;

	// LED1 and PWM Pin
    GPIO_SetupPinMux(22, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(22, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;
	
	// LED2
    GPIO_SetupPinMux(94, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(94, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1;

	// LED3
    GPIO_SetupPinMux(95, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(95, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO95 = 1;

	// LED4
    GPIO_SetupPinMux(97, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(97, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO97 = 1;

	// LED5
    GPIO_SetupPinMux(111, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(111, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO111 = 1;

	// LED6
    GPIO_SetupPinMux(130, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(130, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO130 = 1;

	// LED7	
    GPIO_SetupPinMux(131, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(131, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO131 = 1;

	// LED8
    GPIO_SetupPinMux(25, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(25, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;

	// LED9
    GPIO_SetupPinMux(26, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(26, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;

	// LED10
    GPIO_SetupPinMux(27, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(27, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;

	// LED11	
    GPIO_SetupPinMux(60, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(60, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;

	// LED12	
    GPIO_SetupPinMux(61, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(61, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;

	// LED13
    GPIO_SetupPinMux(157, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(157, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO157 = 1;

	// LED14
    GPIO_SetupPinMux(158, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(158, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO158 = 1;
	
	// LED15
    GPIO_SetupPinMux(159, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(159, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO159 = 1;

	// LED16
    GPIO_SetupPinMux(160, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(160, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPFCLEAR.bit.GPIO160 = 1;

    //WIZNET Reset
    GPIO_SetupPinMux(0, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(0, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO0 = 1;

    //ESP8266 Reset
    GPIO_SetupPinMux(1, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(1, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO1 = 1;

	//SPIRAM  CS  Chip Select
    GPIO_SetupPinMux(19, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(19, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;

    //DRV8874 #1 DIR  Direction
    GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO29 = 1;

    //DRV8874 #2 DIR  Direction
    GPIO_SetupPinMux(32, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(32, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO32 = 1;

    //DAN28027  CS  Chip Select
    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO9 = 1;
	
    //MPU9250  CS  Chip Select
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
	
	//WIZNET  CS  Chip Select
    GPIO_SetupPinMux(125, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(125, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDSET.bit.GPIO125 = 1;
	
    //PushButton 1
    GPIO_SetupPinMux(4, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(4, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 2
    GPIO_SetupPinMux(5, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(5, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 3
    GPIO_SetupPinMux(6, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(6, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 4
    GPIO_SetupPinMux(7, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(7, GPIO_INPUT, GPIO_PULLUP);
	
	//Joy Stick Pushbutton
    GPIO_SetupPinMux(8, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(8, GPIO_INPUT, GPIO_PULLUP);

    // BM: setup pin mux for GPIO52
    GPIO_SetupPinMux(52, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(52, GPIO_OUTPUT, GPIO_PUSHPULL);


    // Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    DINT;

    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F2837xD_PieCtrl.c file.
    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2837xD_DefaultIsr.c.
    // This function is found in F2837xD_PieVect.c.
    InitPieVectTable();

    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this project
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    PieVectTable.TIMER1_INT = &cpu_timer1_isr;
    PieVectTable.TIMER2_INT = &cpu_timer2_isr;
    PieVectTable.SCIA_RX_INT = &RXAINT_recv_ready;
    PieVectTable.SCIB_RX_INT = &RXBINT_recv_ready;
    PieVectTable.SCIC_RX_INT = &RXCINT_recv_ready;
    PieVectTable.SCID_RX_INT = &RXDINT_recv_ready;
    PieVectTable.SCIA_TX_INT = &TXAINT_data_sent;
    PieVectTable.SCIB_TX_INT = &TXBINT_data_sent;
    PieVectTable.SCIC_TX_INT = &TXCINT_data_sent;
    PieVectTable.SCID_TX_INT = &TXDINT_data_sent;

    PieVectTable.EMIF_ERROR_INT = &SWI_isr;

    // BM: adding new interrupt functions for ADCs
//    PieVectTable.ADCD1_INT = &ADCD_ISR;
    PieVectTable.ADCA1_INT = &ADCA_ISR;
//    PieVectTable.ADCB1_INT = &ADCB_ISR;
    EDIS;    // This is needed to disable write to EALLOW protected registers


    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every given period:
    // 200MHz CPU Freq,                       Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, LAUNCHPAD_CPU_FREQUENCY, 10000);
    ConfigCpuTimer(&CpuTimer1, LAUNCHPAD_CPU_FREQUENCY, 20000);
    ConfigCpuTimer(&CpuTimer2, LAUNCHPAD_CPU_FREQUENCY, 1000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

	init_serialSCIA(&SerialA,115200);

    // BM: setting EPWM8 REGs
    // BM: setting TBCTL registers
    EPwm8Regs.TBCTL.bit.FREE_SOFT = 2;
    EPwm8Regs.TBCTL.bit.CTRMODE = 0;
    EPwm8Regs.TBCTL.bit.CLKDIV = 4; // BM: set the divider to 16
    EPwm8Regs.TBCTL.bit.PHSEN = 0;
    // BM: set TBCTR registers
    EPwm8Regs.TBCTR = 0;
    // BM: set TBPRD registers, set to 50Hz
    EPwm8Regs.TBPRD = 62500;
    // BM: set CMPA registers
    EPwm8Regs.CMPA.bit.CMPA = 5000;
    // BM: set CMPA registers for EPwm8B
    EPwm8Regs.CMPB.bit.CMPB = 5000;
    // BM: set AQCTLA registers
    EPwm8Regs.AQCTLA.bit.CAU = 1;
    EPwm8Regs.AQCTLA.bit.ZRO = 2;
    // BM: set AQCTLA registers for EPwm8B
    EPwm8Regs.AQCTLB.bit.CBU = 1;
    EPwm8Regs.AQCTLB.bit.ZRO = 2;
    // BM: set TBPHS registers
    EPwm8Regs.TBPHS.bit.TBPHS = 0;
    // BM: set pinmux for the correct GPIO according to the PinMuxTable
    GPIO_SetupPinMux(14, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinMux(15, GPIO_MUX_CPU1, 1);

	// BM: adding new lines according to the lab manual
	EALLOW;
	EPwm5Regs.ETSEL.bit.SOCAEN = 0; // Disable SOC on A group
	EPwm5Regs.TBCTL.bit.CTRMODE = 3; // freeze counter
	EPwm5Regs.ETSEL.bit.SOCASEL = 2; // Select Event when counter equal to PRD
	EPwm5Regs.ETPS.bit.SOCAPRD = 1; // Generate pulse on 1st event (“pulse” is the same as “trigger”)
	EPwm5Regs.TBCTR = 0x0; // Clear counter
	EPwm5Regs.TBPHS.bit.TBPHS = 0x0000; // Phase is 0
	EPwm5Regs.TBCTL.bit.PHSEN = 0; // Disable phase loading
	EPwm5Regs.TBCTL.bit.CLKDIV = 0; // divide by 1 50Mhz Clock
//	EPwm5Regs.TBPRD = 50000; // Set Period to 1ms sample. Input clock is 50MHz.
//	EPwm5Regs.TBPRD = 50000 / 4; // BM: Set Period to 0.25ms sample. Input clock is 50MHz.
	EPwm5Regs.TBPRD = 5000; // BM: Set Period to 0.1ms sample. Input clock is 50MHz.
	// Notice here that we are not setting CMPA or CMPB because we are not using the PWM signal
	EPwm5Regs.ETSEL.bit.SOCAEN = 1; //enable SOCA
	EPwm5Regs.TBCTL.bit.CTRMODE = 0; //unfreeze, and enter up count mode

	GpioCtrlRegs.GPAPUD.bit.GPIO14 = 1; // For EPWM8A
	GpioCtrlRegs.GPAPUD.bit.GPIO15 = 1; // For EPWM8B
	EDIS;

	EALLOW;
	//write configurations for all ADCs ADCA, ADCB, ADCC, ADCD
	AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
	AdcbRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
	AdccRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
	AdcdRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
	AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
	AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
	AdcSetMode(ADC_ADCC, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
	AdcSetMode(ADC_ADCD, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
	//Set pulse positions to late
	AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
	AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;
	AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;
	AdcdRegs.ADCCTL1.bit.INTPULSEPOS = 1;
	//power up the ADCs
	AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
	AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;
	AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;
	AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1;
	//delay for 1ms to allow ADC time to power up
	DELAY_US(1000);
	//Select the channels to convert and end of conversion flag
	//Many statements commented out, To be used when using ADCA or ADCB ME 461 4

	/***************BM: for ADCA***************/
	//ADCA
	// BM: set to ADCINA2 according to the lab manual
	AdcaRegs.ADCSOC0CTL.bit.CHSEL = 2; //SOC0 will convert Channel you choose Does not have to be A0
	AdcaRegs.ADCSOC0CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
	AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 13;// EPWM5 ADCSOCA or another trigger you choose will trigger SOC0
	// BM: set to ADCINA3 according to the lab manual
	AdcaRegs.ADCSOC1CTL.bit.CHSEL = 3; //SOC1 will convert Channel you choose Does not have to be A1
	AdcaRegs.ADCSOC1CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
	AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 13;// EPWM5 ADCSOCA or another trigger you choose will trigger SOC1
	AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 1; //set to last SOC that is converted and it will set INT1 flag ADCA1
	AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1; //enable INT1 flag
	AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared

	/***************BM: for ADCB***************/
	//ADCB
//	AdcbRegs.ADCSOC0CTL.bit.CHSEL = 4; //SOC0 will convert Channel you choose Does not have to be B0
//	AdcbRegs.ADCSOC0CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
//	AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 13; // EPWM5 ADCSOCA or another trigger you choose will trigger SOC0
////	AdcbRegs.ADCSOC1CTL.bit.CHSEL = ???; //SOC1 will convert Channel you choose Does not have to be B1
////	AdcbRegs.ADCSOC1CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
////	AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = ???; // EPWM5 ADCSOCA or another trigger you choose will trigger SOC1
////	AdcbRegs.ADCSOC2CTL.bit.CHSEL = ???; //SOC2 will convert Channel you choose Does not have to be B2
////	AdcbRegs.ADCSOC2CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
////	AdcbRegs.ADCSOC2CTL.bit.TRIGSEL = ???; // EPWM5 ADCSOCA or another trigger you choose will trigger SOC2
////	AdcbRegs.ADCSOC3CTL.bit.CHSEL = ???; //SOC3 will convert Channel you choose Does not have to be B3
////	AdcbRegs.ADCSOC3CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
////	AdcbRegs.ADCSOC3CTL.bit.TRIGSEL = ???; // EPWM5 ADCSOCA or another trigger you choose will trigger SOC3
//	AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 0; //set to last SOC that is converted and it will set INT1 flag ADCB1
//	AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1; //enable INT1 flag
//	AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared

	/***************BM: for ADCD***************/
	//ADCD
//	AdcdRegs.ADCSOC0CTL.bit.CHSEL = 0; // set SOC0 to convert pin D0
//	AdcdRegs.ADCSOC0CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
//	AdcdRegs.ADCSOC0CTL.bit.TRIGSEL = 13; // EPWM5 ADCSOCA will trigger SOC0
//	AdcdRegs.ADCSOC1CTL.bit.CHSEL = 1; //set SOC1 to convert pin D1
//	AdcdRegs.ADCSOC1CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
//	AdcdRegs.ADCSOC1CTL.bit.TRIGSEL = 13; // EPWM5 ADCSOCA will trigger SOC1
//	//AdcdRegs.ADCSOC2CTL.bit.CHSEL = ???; //set SOC2 to convert pin D2
//	//AdcdRegs.ADCSOC2CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
//	//AdcdRegs.ADCSOC2CTL.bit.TRIGSEL = ???; // EPWM5 ADCSOCA will trigger SOC2
//	//AdcdRegs.ADCSOC3CTL.bit.CHSEL = ???; //set SOC3 to convert pin D3
//	//AdcdRegs.ADCSOC3CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
//	//AdcdRegs.ADCSOC3CTL.bit.TRIGSEL = ???; // EPWM5 ADCSOCA will trigger SOC3
//	AdcdRegs.ADCINTSEL1N2.bit.INT1SEL = 1; //set to SOC1, the last converted, and it will set INT1 flag ADCD1
//	AdcdRegs.ADCINTSEL1N2.bit.INT1E = 1; //enable INT1 flag
//	AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
	EDIS;

	// Enable DACA and DACB outputs
	EALLOW;
	DacaRegs.DACOUTEN.bit.DACOUTEN = 1; //enable dacA output-->uses ADCINA0
	DacaRegs.DACCTL.bit.LOADMODE = 0; //load on next sysclk
	DacaRegs.DACCTL.bit.DACREFSEL = 1; //use ADC VREF as reference voltage
	DacbRegs.DACOUTEN.bit.DACOUTEN = 1; //enable dacB output-->uses ADCINA1
	DacbRegs.DACCTL.bit.LOADMODE = 0; //load on next sysclk
	DacbRegs.DACCTL.bit.DACREFSEL = 1; //use ADC VREF as reference voltage
	EDIS;

    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.  
    IER |= M_INT1;
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
	// Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;

    // BM: enable the 6th interrupt source of interrupt 1
    // BM: for ADCD
//    PieCtrlRegs.PIEIER1.bit.INTx6 = 1;
    // BM: for ADCA
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;
    // BM: for ADCB
//    PieCtrlRegs.PIEIER1.bit.INTx2 = 1;
	
	init_serialSCIC(&SerialC,115200);
	init_serialSCID(&SerialD,115200);
    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    
    // IDLE loop. Just sit and loop forever (optional):
    while(1)
    {
        if (UARTPrint == 1 ) {
			serial_printf(&SerialA,"Voltage of yk0: %0.6f, Voltage of yk1: %0.6f\r\n", yk0, yk1);
//			serial_printf(&SerialA,"Voltage 0: %0.6f, Voltage 1: %0.6f\r\n", xks0[0], xks1[0]);
//			serial_printf(&SerialA,"Voltage of yk0: %0.6f\r\n", yk3);/
            UARTPrint = 0;
        }
    }
}


// SWI_isr,  Using this interrupt as a Software started interrupt
__interrupt void SWI_isr(void) {

    // These three lines of code allow SWI_isr, to be interrupted by other interrupt functions
	// making it lower priority than all other Hardware interrupts.  
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
    asm("       NOP");                    // Wait one cycle
    EINT;                                 // Clear INTM to enable interrupts
	
	
	
    // Insert SWI ISR Code here.......
	
	
    numSWIcalls++;
    
    DINT;

}

// cpu_timer0_isr - CPU Timer0 ISR
__interrupt void cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;

    numTimer0calls++;

//    if ((numTimer0calls%50) == 0) {
//        PieCtrlRegs.PIEIFR12.bit.INTx9 = 1;  // Manually cause the interrupt for the SWI
//    }

    if ((numTimer0calls%25) == 0) {
        displayLEDletter(LEDdisplaynum);
        LEDdisplaynum++;
        if (LEDdisplaynum == 0xFFFF) {  // prevent roll over exception
            LEDdisplaynum = 0;
        }
    }

    if ((numTimer0calls%50) == 0) {
		// Blink LaunchPad Red LED
		GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
    }


    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void)
{
		
    CpuTimer1.InterruptCount++;
}

// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void)
{
	// Blink LaunchPad Blue LED
    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;

    CpuTimer2.InterruptCount++;

    // BM: BM: adding new codes to timer2 ISR, for EPWM8A and B
//    if (angle_count == 1) {
//        control_angle -= 0.05;
//    } else {
//        control_angle += 0.05;
//    }

    control_angle0 = finger * yk0 + finger_init;
    // BM: bounding check for control angle
    if (90.0 <= control_angle0) {
        control_angle0 = 90.0;
    }
    if (control_angle0 <= -90.0) {
        control_angle0 = -90.0;
    }

    if ((CpuTimer2.InterruptCount % 50) == 0) {
        UARTPrint = 1;
    }

    control_angle1 = thumb * yk1 + thumb_init;
    // BM: bounding check for control angle
    if (90.0 <= control_angle1) {
        control_angle1 = 90.0;
    }
    if (control_angle1 <= -90.0) {
        control_angle1 = -90.0;
    }

    // BM: setting EPWM8A CMPA and EPWM8B CMPB
    setEPWM8A_RCServo(control_angle0);
    setEPWM8B_RCServo(control_angle1);
	
    // BM: comment this so that the print frequency is completely controlled by the EPWM5
//	if ((CpuTimer2.InterruptCount % 10) == 0) {
//		UARTPrint = 1;
//	}
}


//This function sets DACA to the voltage between 0V and 3V passed to this function.
//If outside 0V to 3V the output is saturated at 0V to 3V
//Example code
//float myu = 2.25;
//setDACA(myu); // DACA will now output 2.25 Volts
void setDACA(float dacouta0) {
    int16_t DACOutInt = 0;
    DACOutInt = 4096.0 * dacouta0 / 3.0; // perform scaling of 0 – almost 3V to 0 - 4095
    if (DACOutInt > 4095) DACOutInt = 4095;
    if (DACOutInt < 0) DACOutInt = 0;
    DacaRegs.DACVALS.bit.DACVALS = DACOutInt;
}

void setDACB(float dacouta1) {
    int16_t DACOutInt = 0;
    DACOutInt = 4096.0 * dacouta1 / 3.0; // perform scaling of 0 – almost 3V to 0 - 4095
    if (DACOutInt > 4095) DACOutInt = 4095;
    if (DACOutInt < 0) DACOutInt = 0;
    DacbRegs.DACVALS.bit.DACVALS = DACOutInt;
}


//xk is the current ADC reading, xk_1 is the ADC reading one millisecond ago, xk_2 two milliseconds ago, etc
float xk = 0;
float xk_1 = 0;
float xk_2 = 0;
float xk_3 = 0;
float xk_4 = 0;
//yk is the filtered value
float yk = 0;
//b is the filter coefficients
//float b[5] = {0.2,0.2,0.2,0.2,0.2}; // 0.2 is 1/5th therefore a 5 point average

// BM: new b generated by matlab with cutoff frequency 50Hz
//float b[5] = {   3.3833240118424500e-02,
//    2.4012702387971543e-01,
//    4.5207947200372001e-01,
//    2.4012702387971543e-01,
//    3.3833240118424500e-02};

// BM: new b generated by matlab with 4th order FIR with cutoff frequency 75Hz
//float b[5] = {  3.1532325664697325e-02,
//                2.3887950588306769e-01,
//                4.5917633690446985e-01,
//                2.3887950588306769e-01,
//                3.1532325664697325e-02};

// BM: new b generated by matlab with 21th order FIR with cutoff frequency 75Hz
float b[XKS_SIZE] = { -2.3890045153263611e-03,
                -3.3150057635348224e-03,
                -4.6136191242627002e-03,
                -4.1659855521681268e-03,
                1.4477422497795286e-03,
                1.5489414225159667e-02,
                3.9247886844071371e-02,
                7.0723964095458614e-02,
                1.0453473887246176e-01,
                1.3325672639406205e-01,
                1.4978314227429904e-01,
                1.4978314227429904e-01,
                1.3325672639406205e-01,
                1.0453473887246176e-01,
                7.0723964095458614e-02,
                3.9247886844071371e-02,
                1.5489414225159667e-02,
                1.4477422497795286e-03,
                -4.1659855521681268e-03,
                -4.6136191242627002e-03,
                -3.3150057635348224e-03,
                -2.3890045153263611e-03};

// BM: xk array
float xks[XKS_SIZE] = {0};

/***************** BM: the following parts are for ADCA ************************/
__interrupt void ADCA_ISR (void) {
    adca0result = AdcaResultRegs.ADCRESULT0;
    adca1result = AdcaResultRegs.ADCRESULT1;
    xks0[0] = adca0result * 3.0 / 4096.0;
    xks1[0] = adca1result * 3.0 / 4096.0;
    yk0 = 0;
    yk1 = 0;
    // BM: loop solution for calculating yk, just adding all multiplied values
    for (uint16_t i = 0; i < XKS_SIZE; i++) {
        yk0 += b[i] * xks0[i];
        yk1 += b[i] * xks1[i];
    }

    /* BM: the for loop solution for the past states update,
           start from the last part of the array,
           to the first one, and check for 0 in case that it overflow */
    for (uint16_t i = XKS_SIZE - 1; i > 0; i--) {
        if (i == 0) {
            break;
        }
        xks0[i] = xks0[i - 1];
        xks1[i] = xks1[i - 1];
    }
//    setDACA(yk0);
//    setDACB(yk1);
//    counter++;
//    if (counter >= 100) {
//        UARTPrint = 1;
//        counter = 0;
//    }
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

void setEPWM8A_RCServo(float angle) {
    // BM: check the bound of the angle first
    if (angle < -90) {
        angle = -90;
    }
    if (angle > 90) {
        angle = 90;
    }
    // BM: set CMPA
    EPwm8Regs.CMPA.bit.CMPA = 2500.0 * angle / 90 + 5000.0;
}
void setEPWM8B_RCServo(float angle) {
    // BM: check the bound of the angle first
    if (angle < -90) {
        angle = -90;
    }
    if (angle > 90) {
        angle = 90;
    }
    // BM: set CMPB
    EPwm8Regs.CMPB.bit.CMPB = 2500.0 * angle / 90 + 5000.0;
}
