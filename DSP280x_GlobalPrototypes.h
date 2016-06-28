// TI File $Revision:: 9    $
//###########################################################################
//
// FILE:   DSP280x_GlobalPrototypes.h
//
// TITLE:  Global prototypes for DSP280x Examples
// 
//###########################################################################
// $TI Release: Internal 006 $
// $Release Date: November 8, 2004 $
//###########################################################################

#ifndef DSP280X_GLOBALPROTOTYPES_H
#define DSP280X_GLOBALPROTOTYPES_H


#ifdef __cplusplus
extern "C" {
#endif

/*---- shared global function prototypes -----------------------------------*/
extern void InitAdc(void);
extern void InitPeripherals(void);
extern void InitECan(void);
extern void InitGpio(void);
extern void InitPieCtrl(void);
extern void InitPieVectTable(void);
extern void InitSci(void);
extern void InitSpi(void);
extern void InitSysCtrl(void);
extern void InitXIntrupt(void);
extern void InitPll(Uint16 val);
extern void InitPeripheralClocks(void);
void EnableInterrupts(void);
extern void DSP28x_usDelay(Uint32 Count);



// Watchdog functions
// DSP28_SysCtrl.c
extern void KickDog(void);
extern void DisableDog(void);

// DSP28_DBGIER.asm
extern void SetDBGIER(Uint16 dbgier);




//                 CAUTION
// This function MUST be executed out of RAM. Executing it
// out of OTP/Flash will yield unpredictable results
extern void InitFlash(void);


void MemCopy(Uint16 *SourceAddr, Uint16* SourceEndAddr, Uint16* DestAddr);


//---------------------------------------------------------------------------
// External symbols created by the linker cmd file
// DSP28 examples will use these to relocate code from one LOAD location 
// in either Flash or XINTF to a different RUN location in internal
// RAM
extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadEnd;
extern Uint16 RamfuncsRunStart;

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif   // - end of DSP280X_GLOBALPROTOTYPES_H

