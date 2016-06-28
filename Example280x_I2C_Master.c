//###########################################################################
//
// FILE:    Example_i2c_master.c
//
// TITLE:   DSP280x I2C MASTER EXAMPLE
//
// ASSUMPTIONS:
//
//          This program requires the DSP280x header files.  
//          As supplied, this project is configured for "boot to SARAM" operation.
//
//			This program will allow the I2C to act as a master. External
//          connections must be made to the I2C Clock and Data pins.
//      
//          Signal         2808 Pin         eZdsp Pin
//          I2C Clock      GPIO33 (pin 5)    P8-38
//          I2C Data       GPIO32 (pin 100)  P8-36
//          Ground         Vss (pin 2)       P8-40
//
// DESCRIPTION:
//
// This program will communicate to 6 read/write locations in the Slave 2808 RAM. 
// It will allow communications with those registers by the following I2C commands:
//
//  I2C Write (from host)
//
//  S  ADDR  W  A  DATA  A  DATA  A  DATA  A  P
//
//     S = Start bit
//  ADDR = Device Address (7 bits - to this 2808)
//     W = Write
//     A = Acknowledge (from 2808 to host)
//  DATA = location number (0 - 5)
//     A = Acknowledge (from 2808)
//  DATA = High byte of word to be stored
//     A = Ack
//  DATA = Low byte of word to be stored
//     A = Ack
//     P = Stop bit 
//
//
//  I2C Read (from host)
//
//  S   ADDR  W  A  DATA  A  S   ADDR R A  DATA  A  DATA  A  P
//
//     S = Start bit
//  ADDR = Device Address (7 bits - to this 2808)
//     W = Write
//     A = Acknowledge (from 2808 to host)
//  DATA = location number (0 - 5)
//     A = Acknowledge (from 2808)
//
//     S = Repeated Start Bit
//  ADDR = Device Address (7 bits - to this 2808)
//     R = Read
//     A = Acknowlege (from 2808)
//  DATA = High byte of word to be read
//     A = Ack (from host)
//  DATA = Low byte of word to be read
//     A = (N)Ack (from host)
//     P = Stop bit   
//
//###########################################################################
// Author: Todd Anderson
//
// October, 2006
//###########################################################################
// Change Log
//---------------------------------------------------------------------------
//   Date               Change
//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------
//test
//test
#include "DSP280x_Device.h"     // DSP280x Headerfile Include File
#include "DSP280x_Examples.h"   // DSP280x Examples Include File
#include "DSP280x_I2C_defines.h"

// Note: I2C Macros used in this example can be found in the 
// DSP280x_I2C_defines.h file

// Prototype statements for functions found within this file.
void I2CA_Init(void);
void I2CA_Write(int);
void I2CA_Read(int);
void I2CA_Wait(void);
 
interrupt void i2c_int1a_isr(void);


struct FLAGREG_BITS
{
	volatile unsigned int Rsvd:16;		//bits 0-14
};
union FLAG_REG
{
	volatile unsigned int all;
	struct FLAGREG_BITS   bit;
}Flags;


Uint16 Register;
Uint16 Reg[6];
Uint16 ReadReg[6] = {0,0,0,0,0,0};

Uint16 InData[3];
Uint16 OutData[3];
Uint16 I2cIndex;

#define I2C_SLAVE_ADDR        0x2c

void main(void)
{
// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the DSP280x_SysCtrl.c file.
	InitSysCtrl();
	

// Step 2. Initalize GPIO: 
// This example function is found in the DSP280x_Gpio.c file and
// illustrates how to set the GPIO to it's default state.
// InitGpio();    
// Setup only the GP I/O only for I2C functionality
	InitI2CGpio();

// Step 3. Clear all interrupts and initialize PIE vector table:
// Disable CPU interrupts 
	DINT;   

// Initialize PIE control registers to their default state.
// The default state is all PIE interrupts disabled and flags
// are cleared.  
// This function is found in the DSP280x_PieCtrl.c file.
	InitPieCtrl();

// Disable CPU interrupts and clear all CPU interrupt flags:
	IER = 0x0000;
	IFR = 0x0000;

// Initialize the PIE vector table with pointers to the shell Interrupt 
// Service Routines (ISR).  
// This will populate the entire table, even if the interrupt
// is not used in this example.  This is useful for debug purposes.
// The shell ISR routines are found in DSP280x_DefaultIsr.c.
// This function is found in DSP280x_PieVect.c.
	InitPieVectTable();

// Interrupts that are used in this example are re-mapped to
// ISR functions found within this file.  
	EALLOW;	// This is needed to write to EALLOW protected registers
	PieVectTable.I2CINT1A = &i2c_int1a_isr;
	EDIS;   // This is needed to disable write to EALLOW protected registers 

	I2cIndex = 0;

// Step 4. Initialize all the Device Peripherals:
	I2CA_Init();

// Step 5, Master I2C register initialization
//
    Reg[0] = 0x0055;
    Reg[1] = 0x01AA;
    Reg[2] = 0x0234;
    Reg[3] = 0x0321;
    Reg[4] = 0x0468;
    Reg[5] = 0x0531;

// Enable interrupts required for this example
// Enable I2C interrupt 1 in the PIE: Group 8 interrupt 1
	PieCtrlRegs.PIEIER8.bit.INTx1 = 1;

// Enable CPU INT8 which is connected to PIE group 8
	IER |= M_INT8;
	EINT;

   // Application loop
	while(1)
	{
      //////////////////////////////////
      // Write data to slave section  //
      //////////////////////////////////
//     Register        Value
//        0            x0055
//        1            x01AA
//        2            x0234
//        3            x0321
//        4            x0468
//        5            x0531
// 	
//   I2caRegs.I2CSAR = I2C_SLAVE_ADDR;// Set up address written to.

   I2CA_Write(0);                   // Transfer Register 0 contents to slave.
   I2CA_Wait();						// Wait for I2C bus to clear

   I2CA_Write(1);
   I2CA_Wait();						// Wait for I2C bus to clear

   I2CA_Write(2);
   I2CA_Wait();						// Wait for I2C bus to clear

   I2CA_Write(3);
   I2CA_Wait();						// Wait for I2C bus to clear

   I2CA_Write(4);
   I2CA_Wait();						// Wait for I2C bus to clear

   I2CA_Write(5);
   I2CA_Wait();						// Wait for I2C bus to clear

      ///////////////////////////////////
      // Read data from slave section  //
      ///////////////////////////////////
//
//  This needs to be changed to (1) Send out the Register as a write, 
//   followed by (2) Receiving the response.
//
	I2CA_Read(0);
	I2CA_Wait();
    ReadReg[0] = (InData[0]<<8) + InData[1];

	I2CA_Read(1);
	I2CA_Wait();
    ReadReg[1] = (InData[0]<<8) + InData[1];

	I2CA_Read(2);
	I2CA_Wait();
    ReadReg[2] = (InData[0]<<8) + InData[1];

	I2CA_Read(3);
	I2CA_Wait();
    ReadReg[3] = (InData[0]<<8) + InData[1];

	I2CA_Read(4);
	I2CA_Wait();
    ReadReg[4] = (InData[0]<<8) + InData[1];

	I2CA_Read(5);
	I2CA_Wait();
    ReadReg[5] = (InData[0]<<8) + InData[1];

	}
}   // end of main


void I2CA_Init(void)
{
   // Initialize I2C
	I2caRegs.I2CSAR = 0x002C;		// Slave Address.
	I2caRegs.I2COAR = 0x002D;       //  address as Master.
	I2caRegs.I2CPSC.all = 9;		// Prescaler - need 7-12 Mhz on module clk
	I2caRegs.I2CCLKL = 10;			// NOTE: must be non zero
	I2caRegs.I2CCLKH = 5;			// NOTE: must be non zero
    I2caRegs.I2CIER.all = 0x2C;		// Enable SCD & ARDY interrupts

    I2caRegs.I2CMDR.bit.IRS = 1;	// Take I2C out of reset
   									// Stop I2C when suspended

    I2caRegs.I2CFFTX.all = 0x6000;	// Enable FIFO mode and TXFIFO
//    I2caRegs.I2CFFRX.all = 0x2040;	// Enable RXFIFO, clear RXFFINT,
	return;   
}

void I2CA_Write(Register)
{
	int Byte0;
	int Byte1;

    Byte0 = Reg[Register]&0x0FF;    // Get low byte of selected register.
	Byte1 = Reg[Register]>>8;       // Get high byte of selected register.

// Slave Address info gets passed with Start Condition
//    I2caRegs.I2CFFTX.all = 0x6000;	// Enable FIFO mode and TXFIFO
	I2caRegs.I2CCNT = 3; 			// 3 Additional Bytes being tranferred.
	I2caRegs.I2CDXR = Register;     // Send Register to be updated.
	I2caRegs.I2CDXR = Byte1;        // Next is high byte of register.
	I2caRegs.I2CDXR = Byte0;        // Next is low byte of register.

    I2caRegs.I2CMDR.all = 0x6E20;   // Set up the control register:
	                                // bit 14 FREE = 1
									// bit 13 STT = 1  (Start condition)
									
									// bit 11 STP = 1  (Stop condition after 
									//                transfer of bytes.)
									// bit 10 MST = 1  Master
									// bit  9 TRX = 1  Transmit
									
									// bit  5 IRS = 1 to Reset I2C bus.
}

void I2CA_Read(Register)
{
    I2cIndex = 0;                   // Reset value for ISR.
// Slave Address info gets passed with Start Condition
	I2caRegs.I2CCNT = 1; 			// 1 Additional Byte being tranferred.
	I2caRegs.I2CDXR = Register;     // Send Register to be updated.
    I2caRegs.I2CMDR.all = 0x6620;   // Set up the control register:
	                                // bit 14 FREE = 1
									// bit 13 STT = 1  (Start condition)
									
									// bit 11 STP = 0  (Stop condition after 
									//                transfer of bytes.)
									// bit 10 MST = 1  Master
									// bit  9 TRX = 1  Transmit
									
									// bit  5 IRS = 1 to Reset I2C bus.

    DELAY_US(50);                   // Delay 50 usec 

	I2caRegs.I2CCNT = 2;            // Set up receive of 2 bytes.
	I2caRegs.I2CMDR.all = 0x6C20;	// Send "repeated" Start with Read (TRX off)
	                                // and Stop.
//    while (I2caRegs.I2CMDR.bit.STP == 1); // Wait for Stop condition bit to be zero.

//	while (I2caRegs.I2CSTR.bit.BB == 1);  // Wait for Bus Busy to be zero.

}

void I2CA_Wait(void)
{
   // Wait until the STP bit is cleared from any previous master communication.
   // Clearing of this bit by the module is delayed until after the SCD bit is
   // set. If this bit is not checked prior to initiating a new message, the
   // I2C could get confused.

    while (I2caRegs.I2CMDR.bit.STP == 1); // Wait for Stop condition bit to be zero.

	while (I2caRegs.I2CSTR.bit.BB == 1);  // Wait for Bus Busy to be zero.
 
}
interrupt void i2c_int1a_isr(void)     // I2C-A
{
	Uint16 IntSource;

   // Read interrupt source
	IntSource = I2caRegs.I2CISRC.bit.INTCODE & 0x7;
	
	switch(IntSource)
	{
		case I2C_NO_ISRC:   // =0
			break;

		case I2C_ARB_ISRC:  // =1
			break;

		case I2C_NACK_ISRC: // =2
			break;

		case I2C_ARDY_ISRC: // =3
			break;

		case I2C_RX_ISRC:   // =4
			InData[I2cIndex++] = I2caRegs.I2CDRR;
			break;

		case I2C_TX_ISRC:   // =5
			break;

		case I2C_SCD_ISRC:  // =6
			break;

		case I2C_AAS_ISRC:  // =7
			break;
	
		default:
			asm("   ESTOP0"); // Halt on invalid number.
	}

   // Enable future I2C (PIE Group 8) interrupts
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;
}

//===========================================================================
// No more.
//===========================================================================

