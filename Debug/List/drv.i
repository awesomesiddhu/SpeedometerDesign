#line 1 "C:\\Siddharth\\Personal\\Embedded C\\IAR_programs\\SpeedometerDesign\\drv.c"
/*************************************************************************/
/*  File name: 	drv.c
*
*  Purpose:	Hardware Driver Layer which defines ports, directly
*               interacts with the Hall Sensor, LED and Timer and
*               defines Interrupt Service Routines.
*
*  Owner:  	Srividya Prasad, Siddharth D Srinivas, Gaurav Sai Palasari
*
*  Department:   ECE, ECE, EEE
*
*  Version History:
*  V4.0  30 July, 2023        Final version created
*/
/******************************************************************************/

#line 1 "C:\\Siddharth\\Personal\\Embedded C\\IAR_programs\\SpeedometerDesign\\drv.h"
/*************************************************************************/
/*  File name: 	drv.h
*
*  Purpose:	Header file for the Hardware Driver.
*
*  Owner:  	Srividya Prasad, Siddharth D Srinivas, Gaurav Sai Palasari
*
*  Department:   ECE, ECE, EEE
*
*  Version History:
*  V4.0  30 July, 2023        Final version created
*/
/******************************************************************************/




#line 1 "C:\\Siddharth\\Personal\\Embedded C\\IAR_programs\\SpeedometerDesign\\datatypes.h"
/*************************************************************************/
/*  File name: 	datatypes.h
 *
 *  Purpose:	Header file to have a definition for dataypes
 *              that clearly shows by name the datatypes including
 *              the size of the variables
 *
 *  Owner:  	Rangan
 *
 *  Department:   EE Engineering
 *
 *  Copyright (C) 2015 PES University
 *  All rights reserved.
 *
 *
 *  Version History:
 *  V1.0  10th Sept, 2019	Rangan 	  Initial version created
 */
/******************************************************************************/





typedef unsigned char BOOL;
typedef unsigned char UCHAR;
//typedef signed char CHAR;

typedef unsigned char UBYTE;
typedef signed char BYTE;

typedef unsigned short int UINT16;
typedef signed short int INT16;

typedef unsigned long int UINT32;
typedef signed long int INT32;

typedef float FLOAT32;

typedef double FLOAT64;

#line 19 "C:\\Siddharth\\Personal\\Embedded C\\IAR_programs\\SpeedometerDesign\\drv.h"
#line 1 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\io430.h"

/********************************************************************
 *
 * This file is a generic include file controlled by 
 * compiler/assembler IDE generated defines
 *
 * Copyright 1996-2021 IAR Systems AB.
 *
 ********************************************************************/






#pragma system_include


/********************************************************************
 *  /CC430x5xx Family/ 
 ********************************************************************/

#line 1 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\io430g2553.h"
/********************************************************************
 *
 * Standard register and bit definitions for the Texas Instruments
 * MSP430 microcontroller.
 *
 * This file supports assembler and C/EC++ development for
 * msp430g2553 devices.
 *
 * Copyright 1996-2021 IAR Systems AB.
 *
 *
 *
 ********************************************************************/






#line 1 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\intrinsics.h"
/**************************************************
 *
 * Intrinsic functions for the IAR Embedded Workbench for MSP430.
 *
 * Copyright 2002-2008 IAR Systems AB.
 *
 * $Revision: 14461 $
 *
 **************************************************/





  #pragma system_include


#pragma language=save
#pragma language=extended


/*
 * Interrupt state, used by "__get_interrupt_state" and
 * "__set_interrupt_state".
 */

typedef unsigned short __istate_t;

/* Deprecated. */
typedef __istate_t istate_t;






  __intrinsic void __no_operation(void);
  __intrinsic void __enable_interrupt(void);
  __intrinsic void __disable_interrupt(void);

  __intrinsic __istate_t __get_interrupt_state(void);
  __intrinsic void       __set_interrupt_state(__istate_t);

  __intrinsic void __op_code(unsigned short);

  __intrinsic unsigned short __swap_bytes(unsigned short);

  /* Deprecated along with ROPI. */
  __intrinsic long __code_distance(void);

  __intrinsic void           __bic_SR_register(unsigned short);
  __intrinsic void           __bis_SR_register(unsigned short);
  __intrinsic unsigned short __get_SR_register(void);

  __intrinsic void           __bic_SR_register_on_exit(unsigned short);
  __intrinsic void           __bis_SR_register_on_exit(unsigned short);
  __intrinsic unsigned short __get_SR_register_on_exit(void);

  /* Binary encoded decimal operations. */
  __intrinsic unsigned short     __bcd_add_short(unsigned short,
                                                 unsigned short);

  __intrinsic unsigned long      __bcd_add_long (unsigned long,
                                                 unsigned long);

  __intrinsic unsigned long long __bcd_add_long_long(unsigned long long,
                                                     unsigned long long);

  /* Saturated operations. */
  __intrinsic signed char __saturated_add_signed_char     (signed char,
                                                           signed char);
  __intrinsic short       __saturated_add_signed_short    (short, short);
  __intrinsic long        __saturated_add_signed_long     (long, long);
  __intrinsic long long   __saturated_add_signed_long_long(long long,
                                                           long long);

  __intrinsic signed char __saturated_sub_signed_char     (signed char,
                                                           signed char);
  __intrinsic short       __saturated_sub_signed_short    (short, short);
  __intrinsic long        __saturated_sub_signed_long     (long, long);
  __intrinsic long long   __saturated_sub_signed_long_long(long long,
                                                           long long);

  __intrinsic unsigned char  __saturated_add_unsigned_char (unsigned char,
                                                            unsigned char);
  __intrinsic unsigned short __saturated_add_unsigned_short(unsigned short,
                                                            unsigned short);
  __intrinsic unsigned long  __saturated_add_unsigned_long (unsigned long,
                                                            unsigned long);
  __intrinsic unsigned long long __saturated_add_unsigned_long_long(
    unsigned long long,
    unsigned long long);

  __intrinsic unsigned char  __saturated_sub_unsigned_char (unsigned char,
                                                            unsigned char);
  __intrinsic unsigned short __saturated_sub_unsigned_short(unsigned short,
                                                            unsigned short);
  __intrinsic unsigned long  __saturated_sub_unsigned_long (unsigned long,
                                                            unsigned long);
  __intrinsic unsigned long long __saturated_sub_unsigned_long_long(
    unsigned long long,
    unsigned long long);


  /*
   * Support for efficient switch:es. E.g. switch(__even_in_range(x, 10))
   *
   * Note that the value must be even and in the range from 0 to
   * __bound, inclusive. No code will be generated that checks this.
   *
   * This is typically used inside interrupt dispatch functions, to
   * switch on special processor registers like TAIV.
   */

  __intrinsic unsigned short __even_in_range(unsigned short __value,
                                             unsigned short __bound);

  /* Insert a delay with a specific number of cycles. */
  __intrinsic void __delay_cycles(unsigned long __cycles);

  /*
   * The following R4/R5 intrinsic functions are only available when
   * the corresponding register is locked.
   */

  __intrinsic unsigned short __get_R4_register(void);
  __intrinsic void           __set_R4_register(unsigned short);

  __intrinsic unsigned short __get_R5_register(void);
  __intrinsic void           __set_R5_register(unsigned short);

  __intrinsic unsigned short __get_SP_register(void);
  __intrinsic void           __set_SP_register(unsigned short);


  /*
   * If the application provides this function, it is called by the
   * startup code before variables are initialized. If the function
   * returns 0 the data segments will not be initialized.
   */
  __intrinsic int __low_level_init(void);


  /* ----------------------------------------
   * MSP430X-specific intrinsic functions.
   */

  /*
   * Intrinsic functions to allow access to the full 1 Mbyte memory
   * range in small data model.
   *
   * The functions are available in medium and large data model
   * aswell, however it is recommended to access memory using normal
   * __data20 variables and/or pointers.
   *
   * Please note that interrupts must be disabled when the following
   * intrinsics are used.
   */

  __intrinsic void __data20_write_char (unsigned long  __addr,
                                        unsigned char  __value);

  __intrinsic void __data20_write_short(unsigned long  __addr,
                                        unsigned short __value);

  __intrinsic void __data20_write_long (unsigned long  __addr,
                                        unsigned long  __value);

  __intrinsic unsigned char  __data20_read_char (unsigned long __addr);
  __intrinsic unsigned short __data20_read_short(unsigned long __addr);
  __intrinsic unsigned long  __data20_read_long (unsigned long __addr);

  /*
   * The following two functions can be used to access 20-bit SFRs in the
   * lower 64kB. They are only available in extended mode (--core=430X).
   */
  __intrinsic void __data16_write_addr (unsigned short __addr,
                                        unsigned long  __value);

  __intrinsic unsigned long  __data16_read_addr (unsigned short __addr);











/*
 * Alias for locations used for global register variables.  For example,
 * "__no_init __regvar int x @ __R4;".
 */




/*
 * Control bits in the processor status register, SR.
 */








/*
 * Restore GIE without affecting other parts of the status register.
 *
 * Intended usage:
 *     __istate_t state = __get_interrupt_state();
 *     __disable_interrupt();
 *     ...
 *     __bis_GIE_interrupt_state(state);
 */




/*
 * Functions for controlling the processor operation modes.
 */


















#line 251 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\intrinsics.h"







#pragma language=restore

#line 21 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\io430g2553.h"

#pragma system_include









#pragma language=save
#pragma language=extended









/*-------------------------------------------------------------------------
 *   Standard Bits
 *-------------------------------------------------------------------------*/

#line 63 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\io430g2553.h"

/*-------------------------------------------------------------------------
 *   Status register bits
 *-------------------------------------------------------------------------*/

#line 77 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\io430g2553.h"

/* Low Power Modes coded with Bits 4-7 in SR */








#line 97 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\io430g2553.h"



/*-------------------------------------------------------------------------
 *   Special Function
 *-------------------------------------------------------------------------*/



__no_init volatile union
{
  unsigned char IE1;   /* Interrupt Enable 1  */

  struct
  {
    unsigned char WDTIE           : 1; /* Watchdog Interrupt Enable  */
    unsigned char OFIE            : 1; /* Osc. Fault  Interrupt Enable  */
    unsigned char                : 2;
    unsigned char NMIIE           : 1; /* NMI Interrupt Enable  */
    unsigned char ACCVIE          : 1; /* Flash Access Violation Interrupt Enable  */
  }IE1_bit;
} @ 0x0000;


enum {
  WDTIE           = 0x0001,
  OFIE            = 0x0002,
  NMIIE           = 0x0010,
  ACCVIE          = 0x0020
};


__no_init volatile union
{
  unsigned char IFG1;   /* Interrupt Flag 1  */

  struct
  {
    unsigned char WDTIFG          : 1; /* Watchdog Interrupt Flag  */
    unsigned char OFIFG           : 1; /* Osc. Fault Interrupt Flag  */
    unsigned char PORIFG          : 1; /* Power On Interrupt Flag  */
    unsigned char RSTIFG          : 1; /* Reset Interrupt Flag  */
    unsigned char NMIIFG          : 1; /* NMI Interrupt Flag  */
  }IFG1_bit;
} @ 0x0002;


enum {
  WDTIFG          = 0x0001,
  OFIFG           = 0x0002,
  PORIFG          = 0x0004,
  RSTIFG          = 0x0008,
  NMIIFG          = 0x0010
};


__no_init volatile union
{
  unsigned char IE2;   /* Interrupt Enable 2  */

  struct
  {
    unsigned char UCA0RXIE        : 1; /*   */
    unsigned char UCA0TXIE        : 1; /*   */
    unsigned char UCB0RXIE        : 1; /*   */
    unsigned char UCB0TXIE        : 1; /*   */
  }IE2_bit;
} @ 0x0001;


enum {
  UCA0RXIE        = 0x0001,
  UCA0TXIE        = 0x0002,
  UCB0RXIE        = 0x0004,
  UCB0TXIE        = 0x0008
};


__no_init volatile union
{
  unsigned char IFG2;   /* Interrupt Flag 2  */

  struct
  {
    unsigned char UCA0RXIFG       : 1; /*   */
    unsigned char UCA0TXIFG       : 1; /*   */
    unsigned char UCB0RXIFG       : 1; /*   */
    unsigned char UCB0TXIFG       : 1; /*   */
  }IFG2_bit;
} @ 0x0003;


enum {
  UCA0RXIFG       = 0x0001,
  UCA0TXIFG       = 0x0002,
  UCB0RXIFG       = 0x0004,
  UCB0TXIFG       = 0x0008
};






/*-------------------------------------------------------------------------
 *   ADC10
 *-------------------------------------------------------------------------*/



__no_init volatile union
{
  unsigned char ADC10DTC0;   /* ADC10 Data Transfer Control 0  */

  struct
  {
    unsigned char ADC10FETCH      : 1; /* This bit should normally be reset  */
    unsigned char ADC10B1         : 1; /* ADC10 block one  */
    unsigned char ADC10CT         : 1; /* ADC10 continuous transfer  */
    unsigned char ADC10TB         : 1; /* ADC10 two-block mode  */
  }ADC10DTC0_bit;
} @ 0x0048;


enum {
  ADC10FETCH      = 0x0001,
  ADC10B1         = 0x0002,
  ADC10CT         = 0x0004,
  ADC10TB         = 0x0008
};


  /* ADC10 Data Transfer Control 1  */
__no_init volatile unsigned char ADC10DTC1 @ 0x0049;



  /* ADC10 Analog Enable 0  */
__no_init volatile unsigned char ADC10AE0 @ 0x004A;



__no_init volatile union
{
  unsigned short ADC10CTL0;   /* ADC10 Control 0  */

  struct
  {
    unsigned short ADC10SC         : 1; /* ADC10 Start Conversion  */
    unsigned short ENC             : 1; /* ADC10 Enable Conversion  */
    unsigned short ADC10IFG        : 1; /* ADC10 Interrupt Flag  */
    unsigned short ADC10IE         : 1; /* ADC10 Interrupt Enalbe  */
    unsigned short ADC10ON         : 1; /* ADC10 On/Enable  */
    unsigned short REFON           : 1; /* ADC10 Reference on  */
    unsigned short REF2_5V         : 1; /*  */
    unsigned short MSC             : 1; /* ADC10 Multiple SampleConversion  */
    unsigned short REFBURST        : 1; /* ADC10 Reference Burst Mode  */
    unsigned short REFOUT          : 1; /* ADC10 Enalbe output of Ref.  */
    unsigned short ADC10SR         : 1; /* ADC10 Sampling Rate 0:200ksps / 1:50ksps  */
    unsigned short ADC10SHT0       : 1; /* ADC10 Sample Hold Select Bit: 0  */
    unsigned short ADC10SHT1       : 1; /* ADC10 Sample Hold Select Bit: 1  */
    unsigned short SREF0           : 1; /* ADC10 Reference Select Bit: 0  */
    unsigned short SREF1           : 1; /* ADC10 Reference Select Bit: 1  */
    unsigned short SREF2           : 1; /* ADC10 Reference Select Bit: 2  */
  }ADC10CTL0_bit;
} @ 0x01B0;


enum {
  ADC10SC         = 0x0001,
  ENC             = 0x0002,
  ADC10IFG        = 0x0004,
  ADC10IE         = 0x0008,
  ADC10ON         = 0x0010,
  REFON           = 0x0020,
  REF2_5V         = 0x0040,
  MSC             = 0x0080,
  REFBURST        = 0x0100,
  REFOUT          = 0x0200,
  ADC10SR         = 0x0400,
  ADC10SHT0       = 0x0800,
  ADC10SHT1       = 0x1000,
  SREF0           = 0x2000,
  SREF1           = 0x4000,
  SREF2           = 0x8000
};


__no_init volatile union
{
  unsigned short ADC10CTL1;   /* ADC10 Control 1  */

  struct
  {
    unsigned short ADC10BUSY       : 1; /* ADC10 BUSY  */
    unsigned short CONSEQ0         : 1; /* ADC10 Conversion Sequence Select 0  */
    unsigned short CONSEQ1         : 1; /* ADC10 Conversion Sequence Select 1  */
    unsigned short ADC10SSEL0      : 1; /* ADC10 Clock Source Select Bit: 0  */
    unsigned short ADC10SSEL1      : 1; /* ADC10 Clock Source Select Bit: 1  */
    unsigned short ADC10DIV0       : 1; /* ADC10 Clock Divider Select Bit: 0  */
    unsigned short ADC10DIV1       : 1; /* ADC10 Clock Divider Select Bit: 1  */
    unsigned short ADC10DIV2       : 1; /* ADC10 Clock Divider Select Bit: 2  */
    unsigned short ISSH            : 1; /* ADC10 Invert Sample Hold Signal  */
    unsigned short ADC10DF         : 1; /* ADC10 Data Format 0:binary 1:2's complement  */
    unsigned short SHS0            : 1; /* ADC10 Sample/Hold Source Bit: 0  */
    unsigned short SHS1            : 1; /* ADC10 Sample/Hold Source Bit: 1  */
    unsigned short INCH0           : 1; /* ADC10 Input Channel Select Bit: 0  */
    unsigned short INCH1           : 1; /* ADC10 Input Channel Select Bit: 1  */
    unsigned short INCH2           : 1; /* ADC10 Input Channel Select Bit: 2  */
    unsigned short INCH3           : 1; /* ADC10 Input Channel Select Bit: 3  */
  }ADC10CTL1_bit;
} @ 0x01B2;


enum {
  ADC10BUSY       = 0x0001,
  CONSEQ0         = 0x0002,
  CONSEQ1         = 0x0004,
  ADC10SSEL0      = 0x0008,
  ADC10SSEL1      = 0x0010,
  ADC10DIV0       = 0x0020,
  ADC10DIV1       = 0x0040,
  ADC10DIV2       = 0x0080,
  ISSH            = 0x0100,
  ADC10DF         = 0x0200,
  SHS0            = 0x0400,
  SHS1            = 0x0800,
  INCH0           = 0x1000,
  INCH1           = 0x2000,
  INCH2           = 0x4000,
  INCH3           = 0x8000
};


  /* ADC10 Memory  */
__no_init volatile unsigned short ADC10MEM @ 0x01B4;



  /* ADC10 Data Transfer Start Address  */
__no_init volatile unsigned short ADC10SA @ 0x01BC;








#line 354 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\io430g2553.h"











#line 373 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\io430g2553.h"






#line 396 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\io430g2553.h"

/*-------------------------------------------------------------------------
 *   System Clock
 *-------------------------------------------------------------------------*/



__no_init volatile union
{
  unsigned char DCOCTL;   /* DCO Clock Frequency Control  */

  struct
  {
    unsigned char MOD0            : 1; /* Modulation Bit 0  */
    unsigned char MOD1            : 1; /* Modulation Bit 1  */
    unsigned char MOD2            : 1; /* Modulation Bit 2  */
    unsigned char MOD3            : 1; /* Modulation Bit 3  */
    unsigned char MOD4            : 1; /* Modulation Bit 4  */
    unsigned char DCO0            : 1; /* DCO Select Bit 0  */
    unsigned char DCO1            : 1; /* DCO Select Bit 1  */
    unsigned char DCO2            : 1; /* DCO Select Bit 2  */
  }DCOCTL_bit;
} @ 0x0056;


enum {
  MOD0            = 0x0001,
  MOD1            = 0x0002,
  MOD2            = 0x0004,
  MOD3            = 0x0008,
  MOD4            = 0x0010,
  DCO0            = 0x0020,
  DCO1            = 0x0040,
  DCO2            = 0x0080
};


__no_init volatile union
{
  unsigned char BCSCTL1;   /* Basic Clock System Control 1  */

  struct
  {
    unsigned char RSEL0           : 1; /* Range Select Bit 0  */
    unsigned char RSEL1           : 1; /* Range Select Bit 1  */
    unsigned char RSEL2           : 1; /* Range Select Bit 2  */
    unsigned char RSEL3           : 1; /* Range Select Bit 3  */
    unsigned char DIVA0           : 1; /* ACLK Divider 0  */
    unsigned char DIVA1           : 1; /* ACLK Divider 1  */
    unsigned char XTS             : 1; /* LFXTCLK 0:Low Freq. / 1: High Freq.  */
    unsigned char XT2OFF          : 1; /* Enable XT2CLK  */
  }BCSCTL1_bit;
} @ 0x0057;


enum {
  RSEL0           = 0x0001,
  RSEL1           = 0x0002,
  RSEL2           = 0x0004,
  RSEL3           = 0x0008,
  DIVA0           = 0x0010,
  DIVA1           = 0x0020,
  XTS             = 0x0040,
  XT2OFF          = 0x0080
};


__no_init volatile union
{
  unsigned char BCSCTL2;   /* Basic Clock System Control 2  */

  struct
  {
    unsigned char                : 1;
    unsigned char DIVS0           : 1; /* SMCLK Divider 0  */
    unsigned char DIVS1           : 1; /* SMCLK Divider 1  */
    unsigned char SELS            : 1; /* SMCLK Source Select 0:DCOCLK / 1:XT2CLK/LFXTCLK  */
    unsigned char DIVM0           : 1; /* MCLK Divider 0  */
    unsigned char DIVM1           : 1; /* MCLK Divider 1  */
    unsigned char SELM0           : 1; /* MCLK Source Select 0  */
    unsigned char SELM1           : 1; /* MCLK Source Select 1  */
  }BCSCTL2_bit;
} @ 0x0058;


enum {
  DIVS0           = 0x0002,
  DIVS1           = 0x0004,
  SELS            = 0x0008,
  DIVM0           = 0x0010,
  DIVM1           = 0x0020,
  SELM0           = 0x0040,
  SELM1           = 0x0080
};


__no_init volatile union
{
  unsigned char BCSCTL3;   /* Basic Clock System Control 3  */

  struct
  {
    unsigned char LFXT1OF         : 1; /* Low/high Frequency Oscillator Fault Flag  */
    unsigned char XT2OF           : 1; /* High frequency oscillator 2 fault flag  */
    unsigned char XCAP0           : 1; /* XIN/XOUT Cap 0  */
    unsigned char XCAP1           : 1; /* XIN/XOUT Cap 1  */
    unsigned char LFXT1S0         : 1; /* Mode 0 for LFXT1 (XTS = 0)  */
    unsigned char LFXT1S1         : 1; /* Mode 1 for LFXT1 (XTS = 0)  */
    unsigned char XT2S0           : 1; /* Mode 0 for XT2  */
    unsigned char XT2S1           : 1; /* Mode 1 for XT2  */
  }BCSCTL3_bit;
} @ 0x0053;


enum {
  LFXT1OF         = 0x0001,
  XT2OF           = 0x0002,
  XCAP0           = 0x0004,
  XCAP1           = 0x0008,
  LFXT1S0         = 0x0010,
  LFXT1S1         = 0x0020,
  XT2S0           = 0x0040,
  XT2S1           = 0x0080
};








































/*-------------------------------------------------------------------------
 *   Comparator A
 *-------------------------------------------------------------------------*/



__no_init volatile union
{
  unsigned char CACTL1;   /* Comparator A Control 1  */

  struct
  {
    unsigned char CAIFG           : 1; /* Comp. A Interrupt Flag  */
    unsigned char CAIE            : 1; /* Comp. A Interrupt Enable  */
    unsigned char CAIES           : 1; /* Comp. A Int. Edge Select: 0:rising / 1:falling  */
    unsigned char CAON            : 1; /* Comp. A enable  */
    unsigned char CAREF0          : 1; /* Comp. A Internal Reference Select 0  */
    unsigned char CAREF1          : 1; /* Comp. A Internal Reference Select 1  */
    unsigned char CARSEL          : 1; /* Comp. A Internal Reference Enable  */
    unsigned char CAEX            : 1; /* Comp. A Exchange Inputs  */
  }CACTL1_bit;
} @ 0x0059;


enum {
  CAIFG           = 0x0001,
  CAIE            = 0x0002,
  CAIES           = 0x0004,
  CAON            = 0x0008,
  CAREF0          = 0x0010,
  CAREF1          = 0x0020,
  CARSEL          = 0x0040,
  CAEX            = 0x0080
};


__no_init volatile union
{
  unsigned char CACTL2;   /* Comparator A Control 2  */

  struct
  {
    unsigned char CAOUT           : 1; /* Comp. A Output  */
    unsigned char CAF             : 1; /* Comp. A Enable Output Filter  */
    unsigned char P2CA0           : 1; /* Comp. A +Terminal Multiplexer  */
    unsigned char P2CA1           : 1; /* Comp. A -Terminal Multiplexer  */
    unsigned char P2CA2           : 1; /* Comp. A -Terminal Multiplexer  */
    unsigned char P2CA3           : 1; /* Comp. A -Terminal Multiplexer  */
    unsigned char P2CA4           : 1; /* Comp. A +Terminal Multiplexer  */
    unsigned char CASHORT         : 1; /* Comp. A Short + and - Terminals  */
  }CACTL2_bit;
} @ 0x005A;


enum {
  CAOUT           = 0x0001,
  CAF             = 0x0002,
  P2CA0           = 0x0004,
  P2CA1           = 0x0008,
  P2CA2           = 0x0010,
  P2CA3           = 0x0020,
  P2CA4           = 0x0040,
  CASHORT         = 0x0080
};


__no_init volatile union
{
  unsigned char CAPD;   /* Comparator A Port Disable  */

  struct
  {
    unsigned char CAPD0           : 1; /* Comp. A Disable Input Buffer of Port Register .0  */
    unsigned char CAPD1           : 1; /* Comp. A Disable Input Buffer of Port Register .1  */
    unsigned char CAPD2           : 1; /* Comp. A Disable Input Buffer of Port Register .2  */
    unsigned char CAPD3           : 1; /* Comp. A Disable Input Buffer of Port Register .3  */
    unsigned char CAPD4           : 1; /* Comp. A Disable Input Buffer of Port Register .4  */
    unsigned char CAPD5           : 1; /* Comp. A Disable Input Buffer of Port Register .5  */
    unsigned char CAPD6           : 1; /* Comp. A Disable Input Buffer of Port Register .6  */
    unsigned char CAPD7           : 1; /* Comp. A Disable Input Buffer of Port Register .7  */
  }CAPD_bit;
} @ 0x005B;


enum {
  CAPD0           = 0x0001,
  CAPD1           = 0x0002,
  CAPD2           = 0x0004,
  CAPD3           = 0x0008,
  CAPD4           = 0x0010,
  CAPD5           = 0x0020,
  CAPD6           = 0x0040,
  CAPD7           = 0x0080
};










/*-------------------------------------------------------------------------
 *   Flash
 *-------------------------------------------------------------------------*/



__no_init volatile union
{
  unsigned short FCTL1;   /* FLASH Control 1  */

  struct
  {
    unsigned short                : 1;
    unsigned short ERASE           : 1; /* Enable bit for Flash segment erase  */
    unsigned short MERAS           : 1; /* Enable bit for Flash mass erase  */
    unsigned short                : 3;
    unsigned short WRT             : 1; /* Enable bit for Flash write  */
    unsigned short BLKWRT          : 1; /* Enable bit for Flash segment write  */
  }FCTL1_bit;
} @ 0x0128;


enum {
  ERASE           = 0x0002,
  MERAS           = 0x0004,
  WRT             = 0x0040,
  BLKWRT          = 0x0080
};


__no_init volatile union
{
  unsigned short FCTL2;   /* FLASH Control 2  */

  struct
  {
    unsigned short FN0             : 1; /* Divide Flash clock by 1 to 64 using FN0 to FN5 according to:  */
    unsigned short FN1             : 1; /* 32*FN5 + 16*FN4 + 8*FN3 + 4*FN2 + 2*FN1 + FN0 + 1  */
    unsigned short FN2             : 1; /*   */
    unsigned short FN3             : 1; /*   */
    unsigned short FN4             : 1; /*   */
    unsigned short FN5             : 1; /*   */
    unsigned short FSSEL0          : 1; /* Flash clock select 0  */
    unsigned short FSSEL1          : 1; /* Flash clock select 1  */
  }FCTL2_bit;
} @ 0x012A;


enum {
  FN0             = 0x0001,
  FN1             = 0x0002,
  FN2             = 0x0004,
  FN3             = 0x0008,
  FN4             = 0x0010,
  FN5             = 0x0020,
  FSSEL0          = 0x0040,
  FSSEL1          = 0x0080
};


__no_init volatile union
{
  unsigned short FCTL3;   /* FLASH Control 3  */

  struct
  {
    unsigned short BUSY            : 1; /* Flash busy: 1  */
    unsigned short KEYV            : 1; /* Flash Key violation flag  */
    unsigned short ACCVIFG         : 1; /* Flash Access violation flag  */
    unsigned short WAIT            : 1; /* Wait flag for segment write  */
    unsigned short LOCK            : 1; /* Lock bit: 1 - Flash is locked (read only)  */
    unsigned short EMEX            : 1; /* Flash Emergency Exit  */
    unsigned short LOCKA           : 1; /* Segment A Lock bit: read = 1 - Segment is locked (read only)  */
    unsigned short FAIL            : 1; /* Last Program or Erase failed  */
  }FCTL3_bit;
} @ 0x012C;


enum {
  BUSY            = 0x0001,
  KEYV            = 0x0002,
  ACCVIFG         = 0x0004,
  WAIT            = 0x0008,
  LOCK            = 0x0010,
  EMEX            = 0x0020,
  LOCKA           = 0x0040,
  FAIL            = 0x0080
};















/*-------------------------------------------------------------------------
 *   Port 1/2
 *-------------------------------------------------------------------------*/



__no_init volatile union
{
  unsigned const char P1IN;   /* Port 1 Input  */

  struct
  {
    unsigned const char P0              : 1; /*  */
    unsigned const char P1              : 1; /*  */
    unsigned const char P2              : 1; /*  */
    unsigned const char P3              : 1; /*  */
    unsigned const char P4              : 1; /*  */
    unsigned const char P5              : 1; /*  */
    unsigned const char P6              : 1; /*  */
    unsigned const char P7              : 1; /*  */
  }P1IN_bit;
} @ 0x0020;


enum {
  P0              = 0x0001,
  P1              = 0x0002,
  P2              = 0x0004,
  P3              = 0x0008,
  P4              = 0x0010,
  P5              = 0x0020,
  P6              = 0x0040,
  P7              = 0x0080
};


__no_init volatile union
{
  unsigned char P1OUT;   /* Port 1 Output  */

  struct
  {
    unsigned char P0              : 1; /*  */
    unsigned char P1              : 1; /*  */
    unsigned char P2              : 1; /*  */
    unsigned char P3              : 1; /*  */
    unsigned char P4              : 1; /*  */
    unsigned char P5              : 1; /*  */
    unsigned char P6              : 1; /*  */
    unsigned char P7              : 1; /*  */
  }P1OUT_bit;
} @ 0x0021;


/*
enum {
  P0              = 0x0001,
  P1              = 0x0002,
  P2              = 0x0004,
  P3              = 0x0008,
  P4              = 0x0010,
  P5              = 0x0020,
  P6              = 0x0040,
  P7              = 0x0080,
};

*/

__no_init volatile union
{
  unsigned char P1DIR;   /* Port 1 Direction  */

  struct
  {
    unsigned char P0              : 1; /*  */
    unsigned char P1              : 1; /*  */
    unsigned char P2              : 1; /*  */
    unsigned char P3              : 1; /*  */
    unsigned char P4              : 1; /*  */
    unsigned char P5              : 1; /*  */
    unsigned char P6              : 1; /*  */
    unsigned char P7              : 1; /*  */
  }P1DIR_bit;
} @ 0x0022;


/*
enum {
  P0              = 0x0001,
  P1              = 0x0002,
  P2              = 0x0004,
  P3              = 0x0008,
  P4              = 0x0010,
  P5              = 0x0020,
  P6              = 0x0040,
  P7              = 0x0080,
};

*/

__no_init volatile union
{
  unsigned char P1IFG;   /* Port 1 Interrupt Flag  */

  struct
  {
    unsigned char P0              : 1; /*  */
    unsigned char P1              : 1; /*  */
    unsigned char P2              : 1; /*  */
    unsigned char P3              : 1; /*  */
    unsigned char P4              : 1; /*  */
    unsigned char P5              : 1; /*  */
    unsigned char P6              : 1; /*  */
    unsigned char P7              : 1; /*  */
  }P1IFG_bit;
} @ 0x0023;


/*
enum {
  P0              = 0x0001,
  P1              = 0x0002,
  P2              = 0x0004,
  P3              = 0x0008,
  P4              = 0x0010,
  P5              = 0x0020,
  P6              = 0x0040,
  P7              = 0x0080,
};

*/

__no_init volatile union
{
  unsigned char P1IES;   /* Port 1 Interrupt Edge Select  */

  struct
  {
    unsigned char P0              : 1; /*  */
    unsigned char P1              : 1; /*  */
    unsigned char P2              : 1; /*  */
    unsigned char P3              : 1; /*  */
    unsigned char P4              : 1; /*  */
    unsigned char P5              : 1; /*  */
    unsigned char P6              : 1; /*  */
    unsigned char P7              : 1; /*  */
  }P1IES_bit;
} @ 0x0024;


/*
enum {
  P0              = 0x0001,
  P1              = 0x0002,
  P2              = 0x0004,
  P3              = 0x0008,
  P4              = 0x0010,
  P5              = 0x0020,
  P6              = 0x0040,
  P7              = 0x0080,
};

*/

__no_init volatile union
{
  unsigned char P1IE;   /* Port 1 Interrupt Enable  */

  struct
  {
    unsigned char P0              : 1; /*  */
    unsigned char P1              : 1; /*  */
    unsigned char P2              : 1; /*  */
    unsigned char P3              : 1; /*  */
    unsigned char P4              : 1; /*  */
    unsigned char P5              : 1; /*  */
    unsigned char P6              : 1; /*  */
    unsigned char P7              : 1; /*  */
  }P1IE_bit;
} @ 0x0025;


/*
enum {
  P0              = 0x0001,
  P1              = 0x0002,
  P2              = 0x0004,
  P3              = 0x0008,
  P4              = 0x0010,
  P5              = 0x0020,
  P6              = 0x0040,
  P7              = 0x0080,
};

*/

__no_init volatile union
{
  unsigned char P1SEL;   /* Port 1 Selection  */

  struct
  {
    unsigned char P0              : 1; /*  */
    unsigned char P1              : 1; /*  */
    unsigned char P2              : 1; /*  */
    unsigned char P3              : 1; /*  */
    unsigned char P4              : 1; /*  */
    unsigned char P5              : 1; /*  */
    unsigned char P6              : 1; /*  */
    unsigned char P7              : 1; /*  */
  }P1SEL_bit;
} @ 0x0026;


/*
enum {
  P0              = 0x0001,
  P1              = 0x0002,
  P2              = 0x0004,
  P3              = 0x0008,
  P4              = 0x0010,
  P5              = 0x0020,
  P6              = 0x0040,
  P7              = 0x0080,
};

*/

__no_init volatile union
{
  unsigned char P1SEL2;   /* Port 1 Selection 2  */

  struct
  {
    unsigned char P0              : 1; /*  */
    unsigned char P1              : 1; /*  */
    unsigned char P2              : 1; /*  */
    unsigned char P3              : 1; /*  */
    unsigned char P4              : 1; /*  */
    unsigned char P5              : 1; /*  */
    unsigned char P6              : 1; /*  */
    unsigned char P7              : 1; /*  */
  }P1SEL2_bit;
} @ 0x0041;


/*
enum {
  P0              = 0x0001,
  P1              = 0x0002,
  P2              = 0x0004,
  P3              = 0x0008,
  P4              = 0x0010,
  P5              = 0x0020,
  P6              = 0x0040,
  P7              = 0x0080,
};

*/

__no_init volatile union
{
  unsigned char P1REN;   /* Port 1 Resistor Enable  */

  struct
  {
    unsigned char P0              : 1; /*  */
    unsigned char P1              : 1; /*  */
    unsigned char P2              : 1; /*  */
    unsigned char P3              : 1; /*  */
    unsigned char P4              : 1; /*  */
    unsigned char P5              : 1; /*  */
    unsigned char P6              : 1; /*  */
    unsigned char P7              : 1; /*  */
  }P1REN_bit;
} @ 0x0027;


/*
enum {
  P0              = 0x0001,
  P1              = 0x0002,
  P2              = 0x0004,
  P3              = 0x0008,
  P4              = 0x0010,
  P5              = 0x0020,
  P6              = 0x0040,
  P7              = 0x0080,
};

*/

__no_init volatile union
{
  unsigned const char P2IN;   /* Port 2 Input  */

  struct
  {
    unsigned const char P0              : 1; /*  */
    unsigned const char P1              : 1; /*  */
    unsigned const char P2              : 1; /*  */
    unsigned const char P3              : 1; /*  */
    unsigned const char P4              : 1; /*  */
    unsigned const char P5              : 1; /*  */
    unsigned const char P6              : 1; /*  */
    unsigned const char P7              : 1; /*  */
  }P2IN_bit;
} @ 0x0028;


/*
enum {
  P0              = 0x0001,
  P1              = 0x0002,
  P2              = 0x0004,
  P3              = 0x0008,
  P4              = 0x0010,
  P5              = 0x0020,
  P6              = 0x0040,
  P7              = 0x0080,
};

*/

__no_init volatile union
{
  unsigned char P2OUT;   /* Port 2 Output  */

  struct
  {
    unsigned char P0              : 1; /*  */
    unsigned char P1              : 1; /*  */
    unsigned char P2              : 1; /*  */
    unsigned char P3              : 1; /*  */
    unsigned char P4              : 1; /*  */
    unsigned char P5              : 1; /*  */
    unsigned char P6              : 1; /*  */
    unsigned char P7              : 1; /*  */
  }P2OUT_bit;
} @ 0x0029;


/*
enum {
  P0              = 0x0001,
  P1              = 0x0002,
  P2              = 0x0004,
  P3              = 0x0008,
  P4              = 0x0010,
  P5              = 0x0020,
  P6              = 0x0040,
  P7              = 0x0080,
};

*/

__no_init volatile union
{
  unsigned char P2DIR;   /* Port 2 Direction  */

  struct
  {
    unsigned char P0              : 1; /*  */
    unsigned char P1              : 1; /*  */
    unsigned char P2              : 1; /*  */
    unsigned char P3              : 1; /*  */
    unsigned char P4              : 1; /*  */
    unsigned char P5              : 1; /*  */
    unsigned char P6              : 1; /*  */
    unsigned char P7              : 1; /*  */
  }P2DIR_bit;
} @ 0x002A;


/*
enum {
  P0              = 0x0001,
  P1              = 0x0002,
  P2              = 0x0004,
  P3              = 0x0008,
  P4              = 0x0010,
  P5              = 0x0020,
  P6              = 0x0040,
  P7              = 0x0080,
};

*/

__no_init volatile union
{
  unsigned char P2IFG;   /* Port 2 Interrupt Flag  */

  struct
  {
    unsigned char P0              : 1; /*  */
    unsigned char P1              : 1; /*  */
    unsigned char P2              : 1; /*  */
    unsigned char P3              : 1; /*  */
    unsigned char P4              : 1; /*  */
    unsigned char P5              : 1; /*  */
    unsigned char P6              : 1; /*  */
    unsigned char P7              : 1; /*  */
  }P2IFG_bit;
} @ 0x002B;


/*
enum {
  P0              = 0x0001,
  P1              = 0x0002,
  P2              = 0x0004,
  P3              = 0x0008,
  P4              = 0x0010,
  P5              = 0x0020,
  P6              = 0x0040,
  P7              = 0x0080,
};

*/

__no_init volatile union
{
  unsigned char P2IES;   /* Port 2 Interrupt Edge Select  */

  struct
  {
    unsigned char P0              : 1; /*  */
    unsigned char P1              : 1; /*  */
    unsigned char P2              : 1; /*  */
    unsigned char P3              : 1; /*  */
    unsigned char P4              : 1; /*  */
    unsigned char P5              : 1; /*  */
    unsigned char P6              : 1; /*  */
    unsigned char P7              : 1; /*  */
  }P2IES_bit;
} @ 0x002C;


/*
enum {
  P0              = 0x0001,
  P1              = 0x0002,
  P2              = 0x0004,
  P3              = 0x0008,
  P4              = 0x0010,
  P5              = 0x0020,
  P6              = 0x0040,
  P7              = 0x0080,
};

*/

__no_init volatile union
{
  unsigned char P2IE;   /* Port 2 Interrupt Enable  */

  struct
  {
    unsigned char P0              : 1; /*  */
    unsigned char P1              : 1; /*  */
    unsigned char P2              : 1; /*  */
    unsigned char P3              : 1; /*  */
    unsigned char P4              : 1; /*  */
    unsigned char P5              : 1; /*  */
    unsigned char P6              : 1; /*  */
    unsigned char P7              : 1; /*  */
  }P2IE_bit;
} @ 0x002D;


/*
enum {
  P0              = 0x0001,
  P1              = 0x0002,
  P2              = 0x0004,
  P3              = 0x0008,
  P4              = 0x0010,
  P5              = 0x0020,
  P6              = 0x0040,
  P7              = 0x0080,
};

*/

__no_init volatile union
{
  unsigned char P2SEL;   /* Port 2 Selection  */

  struct
  {
    unsigned char P0              : 1; /*  */
    unsigned char P1              : 1; /*  */
    unsigned char P2              : 1; /*  */
    unsigned char P3              : 1; /*  */
    unsigned char P4              : 1; /*  */
    unsigned char P5              : 1; /*  */
    unsigned char P6              : 1; /*  */
    unsigned char P7              : 1; /*  */
  }P2SEL_bit;
} @ 0x002E;


/*
enum {
  P0              = 0x0001,
  P1              = 0x0002,
  P2              = 0x0004,
  P3              = 0x0008,
  P4              = 0x0010,
  P5              = 0x0020,
  P6              = 0x0040,
  P7              = 0x0080,
};

*/

__no_init volatile union
{
  unsigned char P2SEL2;   /* Port 2 Selection 2  */

  struct
  {
    unsigned char P0              : 1; /*  */
    unsigned char P1              : 1; /*  */
    unsigned char P2              : 1; /*  */
    unsigned char P3              : 1; /*  */
    unsigned char P4              : 1; /*  */
    unsigned char P5              : 1; /*  */
    unsigned char P6              : 1; /*  */
    unsigned char P7              : 1; /*  */
  }P2SEL2_bit;
} @ 0x0042;


/*
enum {
  P0              = 0x0001,
  P1              = 0x0002,
  P2              = 0x0004,
  P3              = 0x0008,
  P4              = 0x0010,
  P5              = 0x0020,
  P6              = 0x0040,
  P7              = 0x0080,
};

*/

__no_init volatile union
{
  unsigned char P2REN;   /* Port 2 Resistor Enable  */

  struct
  {
    unsigned char P0              : 1; /*  */
    unsigned char P1              : 1; /*  */
    unsigned char P2              : 1; /*  */
    unsigned char P3              : 1; /*  */
    unsigned char P4              : 1; /*  */
    unsigned char P5              : 1; /*  */
    unsigned char P6              : 1; /*  */
    unsigned char P7              : 1; /*  */
  }P2REN_bit;
} @ 0x002F;


/*
enum {
  P0              = 0x0001,
  P1              = 0x0002,
  P2              = 0x0004,
  P3              = 0x0008,
  P4              = 0x0010,
  P5              = 0x0020,
  P6              = 0x0040,
  P7              = 0x0080
};
*/



#line 1354 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\io430g2553.h"

/*-------------------------------------------------------------------------
 *   Port 3/4
 *-------------------------------------------------------------------------*/



__no_init volatile union
{
  unsigned const char P3IN;   /* Port 3 Input  */

  struct
  {
    unsigned const char P0              : 1; /*  */
    unsigned const char P1              : 1; /*  */
    unsigned const char P2              : 1; /*  */
    unsigned const char P3              : 1; /*  */
    unsigned const char P4              : 1; /*  */
    unsigned const char P5              : 1; /*  */
    unsigned const char P6              : 1; /*  */
    unsigned const char P7              : 1; /*  */
  }P3IN_bit;
} @ 0x0018;


/*
enum {
  P0              = 0x0001,
  P1              = 0x0002,
  P2              = 0x0004,
  P3              = 0x0008,
  P4              = 0x0010,
  P5              = 0x0020,
  P6              = 0x0040,
  P7              = 0x0080,
};

*/

__no_init volatile union
{
  unsigned char P3OUT;   /* Port 3 Output  */

  struct
  {
    unsigned char P0              : 1; /*  */
    unsigned char P1              : 1; /*  */
    unsigned char P2              : 1; /*  */
    unsigned char P3              : 1; /*  */
    unsigned char P4              : 1; /*  */
    unsigned char P5              : 1; /*  */
    unsigned char P6              : 1; /*  */
    unsigned char P7              : 1; /*  */
  }P3OUT_bit;
} @ 0x0019;


/*
enum {
  P0              = 0x0001,
  P1              = 0x0002,
  P2              = 0x0004,
  P3              = 0x0008,
  P4              = 0x0010,
  P5              = 0x0020,
  P6              = 0x0040,
  P7              = 0x0080,
};

*/

__no_init volatile union
{
  unsigned char P3DIR;   /* Port 3 Direction  */

  struct
  {
    unsigned char P0              : 1; /*  */
    unsigned char P1              : 1; /*  */
    unsigned char P2              : 1; /*  */
    unsigned char P3              : 1; /*  */
    unsigned char P4              : 1; /*  */
    unsigned char P5              : 1; /*  */
    unsigned char P6              : 1; /*  */
    unsigned char P7              : 1; /*  */
  }P3DIR_bit;
} @ 0x001A;


/*
enum {
  P0              = 0x0001,
  P1              = 0x0002,
  P2              = 0x0004,
  P3              = 0x0008,
  P4              = 0x0010,
  P5              = 0x0020,
  P6              = 0x0040,
  P7              = 0x0080,
};

*/

__no_init volatile union
{
  unsigned char P3SEL;   /* Port 3 Selection  */

  struct
  {
    unsigned char P0              : 1; /*  */
    unsigned char P1              : 1; /*  */
    unsigned char P2              : 1; /*  */
    unsigned char P3              : 1; /*  */
    unsigned char P4              : 1; /*  */
    unsigned char P5              : 1; /*  */
    unsigned char P6              : 1; /*  */
    unsigned char P7              : 1; /*  */
  }P3SEL_bit;
} @ 0x001B;


/*
enum {
  P0              = 0x0001,
  P1              = 0x0002,
  P2              = 0x0004,
  P3              = 0x0008,
  P4              = 0x0010,
  P5              = 0x0020,
  P6              = 0x0040,
  P7              = 0x0080,
};

*/

__no_init volatile union
{
  unsigned char P3SEL2;   /* Port 3 Selection 2  */

  struct
  {
    unsigned char P0              : 1; /*  */
    unsigned char P1              : 1; /*  */
    unsigned char P2              : 1; /*  */
    unsigned char P3              : 1; /*  */
    unsigned char P4              : 1; /*  */
    unsigned char P5              : 1; /*  */
    unsigned char P6              : 1; /*  */
    unsigned char P7              : 1; /*  */
  }P3SEL2_bit;
} @ 0x0043;


/*
enum {
  P0              = 0x0001,
  P1              = 0x0002,
  P2              = 0x0004,
  P3              = 0x0008,
  P4              = 0x0010,
  P5              = 0x0020,
  P6              = 0x0040,
  P7              = 0x0080,
};

*/

__no_init volatile union
{
  unsigned char P3REN;   /* Port 3 Resistor Enable  */

  struct
  {
    unsigned char P0              : 1; /*  */
    unsigned char P1              : 1; /*  */
    unsigned char P2              : 1; /*  */
    unsigned char P3              : 1; /*  */
    unsigned char P4              : 1; /*  */
    unsigned char P5              : 1; /*  */
    unsigned char P6              : 1; /*  */
    unsigned char P7              : 1; /*  */
  }P3REN_bit;
} @ 0x0010;


/*
enum {
  P0              = 0x0001,
  P1              = 0x0002,
  P2              = 0x0004,
  P3              = 0x0008,
  P4              = 0x0010,
  P5              = 0x0020,
  P6              = 0x0040,
  P7              = 0x0080
};
*/







/*-------------------------------------------------------------------------
 *   Timer0_A3
 *-------------------------------------------------------------------------*/



  /* Timer0_A3 Interrupt Vector Word  */
__no_init volatile unsigned const short TA0IV @ 0x012E;



__no_init volatile union
{
  unsigned short TA0CTL;   /* Timer0_A3 Control  */

  struct
  {
    unsigned short TAIFG           : 1; /* Timer A counter interrupt flag  */
    unsigned short TAIE            : 1; /* Timer A counter interrupt enable  */
    unsigned short TACLR           : 1; /* Timer A counter clear  */
    unsigned short                : 1;
    unsigned short MC0             : 1; /* Timer A mode control 0  */
    unsigned short MC1             : 1; /* Timer A mode control 1  */
    unsigned short ID0             : 1; /* Timer A clock input divider 0  */
    unsigned short ID1             : 1; /* Timer A clock input divider 1  */
    unsigned short TASSEL0         : 1; /* Timer A clock source select 0  */
    unsigned short TASSEL1         : 1; /* Timer A clock source select 1  */
  }TA0CTL_bit;
} @ 0x0160;


enum {
  TAIFG           = 0x0001,
  TAIE            = 0x0002,
  TACLR           = 0x0004,
  MC0             = 0x0010,
  MC1             = 0x0020,
  ID0             = 0x0040,
  ID1             = 0x0080,
  TASSEL0         = 0x0100,
  TASSEL1         = 0x0200
};


__no_init volatile union
{
  unsigned short TA0CCTL0;   /* Timer0_A3 Capture/Compare Control 0  */

  struct
  {
    unsigned short CCIFG           : 1; /* Capture/compare interrupt flag  */
    unsigned short COV             : 1; /* Capture/compare overflow flag  */
    unsigned short OUT             : 1; /* PWM Output signal if output mode 0  */
    unsigned short CCI             : 1; /* Capture input signal (read)  */
    unsigned short CCIE            : 1; /* Capture/compare interrupt enable  */
    unsigned short OUTMOD0         : 1; /* Output mode 0  */
    unsigned short OUTMOD1         : 1; /* Output mode 1  */
    unsigned short OUTMOD2         : 1; /* Output mode 2  */
    unsigned short CAP             : 1; /* Capture mode: 1 /Compare mode : 0  */
    unsigned short                : 1;
    unsigned short SCCI            : 1; /* Latched capture signal (read)  */
    unsigned short SCS             : 1; /* Capture sychronize  */
    unsigned short CCIS0           : 1; /* Capture input select 0  */
    unsigned short CCIS1           : 1; /* Capture input select 1  */
    unsigned short CM0             : 1; /* Capture mode 0  */
    unsigned short CM1             : 1; /* Capture mode 1  */
  }TA0CCTL0_bit;
} @ 0x0162;


enum {
  CCIFG           = 0x0001,
  COV             = 0x0002,
  OUT             = 0x0004,
  CCI             = 0x0008,
  CCIE            = 0x0010,
  OUTMOD0         = 0x0020,
  OUTMOD1         = 0x0040,
  OUTMOD2         = 0x0080,
  CAP             = 0x0100,
  SCCI            = 0x0400,
  SCS             = 0x0800,
  CCIS0           = 0x1000,
  CCIS1           = 0x2000,
  CM0             = 0x4000,
  CM1             = 0x8000
};


__no_init volatile union
{
  unsigned short TA0CCTL1;   /* Timer0_A3 Capture/Compare Control 1  */

  struct
  {
    unsigned short CCIFG           : 1; /* Capture/compare interrupt flag  */
    unsigned short COV             : 1; /* Capture/compare overflow flag  */
    unsigned short OUT             : 1; /* PWM Output signal if output mode 0  */
    unsigned short CCI             : 1; /* Capture input signal (read)  */
    unsigned short CCIE            : 1; /* Capture/compare interrupt enable  */
    unsigned short OUTMOD0         : 1; /* Output mode 0  */
    unsigned short OUTMOD1         : 1; /* Output mode 1  */
    unsigned short OUTMOD2         : 1; /* Output mode 2  */
    unsigned short CAP             : 1; /* Capture mode: 1 /Compare mode : 0  */
    unsigned short                : 1;
    unsigned short SCCI            : 1; /* Latched capture signal (read)  */
    unsigned short SCS             : 1; /* Capture sychronize  */
    unsigned short CCIS0           : 1; /* Capture input select 0  */
    unsigned short CCIS1           : 1; /* Capture input select 1  */
    unsigned short CM0             : 1; /* Capture mode 0  */
    unsigned short CM1             : 1; /* Capture mode 1  */
  }TA0CCTL1_bit;
} @ 0x0164;


/*
enum {
  CCIFG           = 0x0001,
  COV             = 0x0002,
  OUT             = 0x0004,
  CCI             = 0x0008,
  CCIE            = 0x0010,
  OUTMOD0         = 0x0020,
  OUTMOD1         = 0x0040,
  OUTMOD2         = 0x0080,
  CAP             = 0x0100,
  SCCI            = 0x0400,
  SCS             = 0x0800,
  CCIS0           = 0x1000,
  CCIS1           = 0x2000,
  CM0             = 0x4000,
  CM1             = 0x8000,
};

*/

__no_init volatile union
{
  unsigned short TA0CCTL2;   /* Timer0_A3 Capture/Compare Control 2  */

  struct
  {
    unsigned short CCIFG           : 1; /* Capture/compare interrupt flag  */
    unsigned short COV             : 1; /* Capture/compare overflow flag  */
    unsigned short OUT             : 1; /* PWM Output signal if output mode 0  */
    unsigned short CCI             : 1; /* Capture input signal (read)  */
    unsigned short CCIE            : 1; /* Capture/compare interrupt enable  */
    unsigned short OUTMOD0         : 1; /* Output mode 0  */
    unsigned short OUTMOD1         : 1; /* Output mode 1  */
    unsigned short OUTMOD2         : 1; /* Output mode 2  */
    unsigned short CAP             : 1; /* Capture mode: 1 /Compare mode : 0  */
    unsigned short                : 1;
    unsigned short SCCI            : 1; /* Latched capture signal (read)  */
    unsigned short SCS             : 1; /* Capture sychronize  */
    unsigned short CCIS0           : 1; /* Capture input select 0  */
    unsigned short CCIS1           : 1; /* Capture input select 1  */
    unsigned short CM0             : 1; /* Capture mode 0  */
    unsigned short CM1             : 1; /* Capture mode 1  */
  }TA0CCTL2_bit;
} @ 0x0166;


/*
enum {
  CCIFG           = 0x0001,
  COV             = 0x0002,
  OUT             = 0x0004,
  CCI             = 0x0008,
  CCIE            = 0x0010,
  OUTMOD0         = 0x0020,
  OUTMOD1         = 0x0040,
  OUTMOD2         = 0x0080,
  CAP             = 0x0100,
  SCCI            = 0x0400,
  SCS             = 0x0800,
  CCIS0           = 0x1000,
  CCIS1           = 0x2000,
  CM0             = 0x4000,
  CM1             = 0x8000,
};

*/

  /* Timer0_A3 Counter Register  */
__no_init volatile unsigned short TA0R @ 0x0170;



  /* Timer0_A3 Capture/Compare 0  */
__no_init volatile unsigned short TA0CCR0 @ 0x0172;



  /* Timer0_A3 Capture/Compare 1  */
__no_init volatile unsigned short TA0CCR1 @ 0x0174;



  /* Timer0_A3 Capture/Compare 2  */
__no_init volatile unsigned short TA0CCR2 @ 0x0176;



/* Alternate register names */
#line 1780 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\io430g2553.h"
/* Alternate register names 2 */
#line 1793 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\io430g2553.h"

#line 1806 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\io430g2553.h"

#line 1823 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\io430g2553.h"
/* T0_A3IV Definitions */
#line 1830 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\io430g2553.h"

/*-------------------------------------------------------------------------
 *   Timer1_A3
 *-------------------------------------------------------------------------*/



  /* Timer1_A3 Interrupt Vector Word  */
__no_init volatile unsigned const short TA1IV @ 0x011E;



__no_init volatile union
{
  unsigned short TA1CTL;   /* Timer1_A3 Control  */

  struct
  {
    unsigned short TAIFG           : 1; /* Timer A counter interrupt flag  */
    unsigned short TAIE            : 1; /* Timer A counter interrupt enable  */
    unsigned short TACLR           : 1; /* Timer A counter clear  */
    unsigned short                : 1;
    unsigned short MC0             : 1; /* Timer A mode control 0  */
    unsigned short MC1             : 1; /* Timer A mode control 1  */
    unsigned short ID0             : 1; /* Timer A clock input divider 0  */
    unsigned short ID1             : 1; /* Timer A clock input divider 1  */
    unsigned short TASSEL0         : 1; /* Timer A clock source select 0  */
    unsigned short TASSEL1         : 1; /* Timer A clock source select 1  */
  }TA1CTL_bit;
} @ 0x0180;


/*
enum {
  TAIFG           = 0x0001,
  TAIE            = 0x0002,
  TACLR           = 0x0004,
  MC0             = 0x0010,
  MC1             = 0x0020,
  ID0             = 0x0040,
  ID1             = 0x0080,
  TASSEL0         = 0x0100,
  TASSEL1         = 0x0200,
};

*/

__no_init volatile union
{
  unsigned short TA1CCTL0;   /* Timer1_A3 Capture/Compare Control 0  */

  struct
  {
    unsigned short CCIFG           : 1; /* Capture/compare interrupt flag  */
    unsigned short COV             : 1; /* Capture/compare overflow flag  */
    unsigned short OUT             : 1; /* PWM Output signal if output mode 0  */
    unsigned short CCI             : 1; /* Capture input signal (read)  */
    unsigned short CCIE            : 1; /* Capture/compare interrupt enable  */
    unsigned short OUTMOD0         : 1; /* Output mode 0  */
    unsigned short OUTMOD1         : 1; /* Output mode 1  */
    unsigned short OUTMOD2         : 1; /* Output mode 2  */
    unsigned short CAP             : 1; /* Capture mode: 1 /Compare mode : 0  */
    unsigned short                : 1;
    unsigned short SCCI            : 1; /* Latched capture signal (read)  */
    unsigned short SCS             : 1; /* Capture sychronize  */
    unsigned short CCIS0           : 1; /* Capture input select 0  */
    unsigned short CCIS1           : 1; /* Capture input select 1  */
    unsigned short CM0             : 1; /* Capture mode 0  */
    unsigned short CM1             : 1; /* Capture mode 1  */
  }TA1CCTL0_bit;
} @ 0x0182;


/*
enum {
  CCIFG           = 0x0001,
  COV             = 0x0002,
  OUT             = 0x0004,
  CCI             = 0x0008,
  CCIE            = 0x0010,
  OUTMOD0         = 0x0020,
  OUTMOD1         = 0x0040,
  OUTMOD2         = 0x0080,
  CAP             = 0x0100,
  SCCI            = 0x0400,
  SCS             = 0x0800,
  CCIS0           = 0x1000,
  CCIS1           = 0x2000,
  CM0             = 0x4000,
  CM1             = 0x8000,
};

*/

__no_init volatile union
{
  unsigned short TA1CCTL1;   /* Timer1_A3 Capture/Compare Control 1  */

  struct
  {
    unsigned short CCIFG           : 1; /* Capture/compare interrupt flag  */
    unsigned short COV             : 1; /* Capture/compare overflow flag  */
    unsigned short OUT             : 1; /* PWM Output signal if output mode 0  */
    unsigned short CCI             : 1; /* Capture input signal (read)  */
    unsigned short CCIE            : 1; /* Capture/compare interrupt enable  */
    unsigned short OUTMOD0         : 1; /* Output mode 0  */
    unsigned short OUTMOD1         : 1; /* Output mode 1  */
    unsigned short OUTMOD2         : 1; /* Output mode 2  */
    unsigned short CAP             : 1; /* Capture mode: 1 /Compare mode : 0  */
    unsigned short                : 1;
    unsigned short SCCI            : 1; /* Latched capture signal (read)  */
    unsigned short SCS             : 1; /* Capture sychronize  */
    unsigned short CCIS0           : 1; /* Capture input select 0  */
    unsigned short CCIS1           : 1; /* Capture input select 1  */
    unsigned short CM0             : 1; /* Capture mode 0  */
    unsigned short CM1             : 1; /* Capture mode 1  */
  }TA1CCTL1_bit;
} @ 0x0184;


/*
enum {
  CCIFG           = 0x0001,
  COV             = 0x0002,
  OUT             = 0x0004,
  CCI             = 0x0008,
  CCIE            = 0x0010,
  OUTMOD0         = 0x0020,
  OUTMOD1         = 0x0040,
  OUTMOD2         = 0x0080,
  CAP             = 0x0100,
  SCCI            = 0x0400,
  SCS             = 0x0800,
  CCIS0           = 0x1000,
  CCIS1           = 0x2000,
  CM0             = 0x4000,
  CM1             = 0x8000,
};

*/

__no_init volatile union
{
  unsigned short TA1CCTL2;   /* Timer1_A3 Capture/Compare Control 2  */

  struct
  {
    unsigned short CCIFG           : 1; /* Capture/compare interrupt flag  */
    unsigned short COV             : 1; /* Capture/compare overflow flag  */
    unsigned short OUT             : 1; /* PWM Output signal if output mode 0  */
    unsigned short CCI             : 1; /* Capture input signal (read)  */
    unsigned short CCIE            : 1; /* Capture/compare interrupt enable  */
    unsigned short OUTMOD0         : 1; /* Output mode 0  */
    unsigned short OUTMOD1         : 1; /* Output mode 1  */
    unsigned short OUTMOD2         : 1; /* Output mode 2  */
    unsigned short CAP             : 1; /* Capture mode: 1 /Compare mode : 0  */
    unsigned short                : 1;
    unsigned short SCCI            : 1; /* Latched capture signal (read)  */
    unsigned short SCS             : 1; /* Capture sychronize  */
    unsigned short CCIS0           : 1; /* Capture input select 0  */
    unsigned short CCIS1           : 1; /* Capture input select 1  */
    unsigned short CM0             : 1; /* Capture mode 0  */
    unsigned short CM1             : 1; /* Capture mode 1  */
  }TA1CCTL2_bit;
} @ 0x0186;


/*
enum {
  CCIFG           = 0x0001,
  COV             = 0x0002,
  OUT             = 0x0004,
  CCI             = 0x0008,
  CCIE            = 0x0010,
  OUTMOD0         = 0x0020,
  OUTMOD1         = 0x0040,
  OUTMOD2         = 0x0080,
  CAP             = 0x0100,
  SCCI            = 0x0400,
  SCS             = 0x0800,
  CCIS0           = 0x1000,
  CCIS1           = 0x2000,
  CM0             = 0x4000,
  CM1             = 0x8000,
};

*/

  /* Timer1_A3 Counter Register  */
__no_init volatile unsigned short TA1R @ 0x0190;



  /* Timer1_A3 Capture/Compare 0  */
__no_init volatile unsigned short TA1CCR0 @ 0x0192;



  /* Timer1_A3 Capture/Compare 1  */
__no_init volatile unsigned short TA1CCR1 @ 0x0194;



  /* Timer1_A3 Capture/Compare 2  */
__no_init volatile unsigned short TA1CCR2 @ 0x0196;



/* T1_A3IV Definitions */
#line 2045 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\io430g2553.h"

/*-------------------------------------------------------------------------
 *   USCI_A0  UART Mode
 *-------------------------------------------------------------------------*/


__no_init volatile union
{
  unsigned char UCA0CTL0;   /* USCI A0 Control Register 0  */

  struct
  {
    unsigned char UCSYNC          : 1; /* Sync-Mode  0:UART-Mode / 1:SPI-Mode  */
    unsigned char UCMODE0         : 1; /* Async. Mode: USCI Mode 0  */
    unsigned char UCMODE1         : 1; /* Async. Mode: USCI Mode 1  */
    unsigned char UCSPB           : 1; /* Async. Mode: Stop Bits  0:one / 1: two  */
    unsigned char UC7BIT          : 1; /* Async. Mode: Data Bits  0:8-bits / 1:7-bits  */
    unsigned char UCMSB           : 1; /* Async. Mode: MSB first  0:LSB / 1:MSB  */
    unsigned char UCPAR           : 1; /* Async. Mode: Parity     0:odd / 1:even  */
    unsigned char UCPEN           : 1; /* Async. Mode: Parity enable  */
  } UCA0CTL0_bit;

  unsigned char UCA0CTL0__SPI;   /*  */
  struct
  {
    unsigned char UCSYNC          : 1; /* Sync-Mode  0:UART-Mode / 1:SPI-Mode  */
    unsigned char UCMODE0         : 1; /* Async. Mode: USCI Mode 0  */
    unsigned char UCMODE1         : 1; /* Async. Mode: USCI Mode 1  */
    unsigned char UCMST           : 1; /* Sync. Mode: Master Select  */
    unsigned char UC7BIT          : 1; /* Async. Mode: Data Bits  0:8-bits / 1:7-bits  */
    unsigned char UCMSB           : 1; /* Async. Mode: MSB first  0:LSB / 1:MSB  */
    unsigned char UCCKPL          : 1; /* Sync. Mode: Clock Polarity  */
    unsigned char UCCKPH          : 1; /* Sync. Mode: Clock Phase  */
  } UCA0CTL0__SPI_bit;

} @ 0x0060;

enum {
  UCSYNC          = 0x0001,
  UCMODE0         = 0x0002,
  UCMODE1         = 0x0004,
  UCSPB           = 0x0008,
  UC7BIT          = 0x0010,
  UCMSB           = 0x0020,
  UCPAR           = 0x0040,
  UCPEN           = 0x0080
};

__no_init volatile union
{
  unsigned char UCA0CTL1;   /* USCI A0 Control Register 1  */

  struct
  {
    unsigned char UCSWRST         : 1; /* USCI Software Reset  */
    unsigned char UCTXBRK         : 1; /* Send next Data as Break  */
    unsigned char UCTXADDR        : 1; /* Send next Data as Address  */
    unsigned char UCDORM          : 1; /* Dormant (Sleep) Mode  */
    unsigned char UCBRKIE         : 1; /* Break interrupt enable  */
    unsigned char UCRXEIE         : 1; /* RX Error interrupt enable  */
    unsigned char UCSSEL0         : 1; /* USCI 0 Clock Source Select 0  */
    unsigned char UCSSEL1         : 1; /* USCI 0 Clock Source Select 1  */
  } UCA0CTL1_bit;

  unsigned char UCA0CTL1__SPI;   /*  */
  struct
  {
    unsigned char UCSWRST         : 1; /* USCI Software Reset  */
    unsigned char                : 5;
    unsigned char UCSSEL0         : 1; /* USCI 0 Clock Source Select 0  */
    unsigned char UCSSEL1         : 1; /* USCI 0 Clock Source Select 1  */
  } UCA0CTL1__SPI_bit;

} @ 0x0061;

enum {
  UCSWRST         = 0x0001,
  UCTXBRK         = 0x0002,
  UCTXADDR        = 0x0004,
  UCDORM          = 0x0008,
  UCBRKIE         = 0x0010,
  UCRXEIE         = 0x0020,
  UCSSEL0         = 0x0040,
  UCSSEL1         = 0x0080
};

__no_init volatile union
{
  unsigned char UCA0BR0;   /* USCI A0 Baud Rate 0  */
  unsigned char UCA0BR0__SPI;   /*  */
} @ 0x0062;

__no_init volatile union
{
  unsigned char UCA0BR1;   /* USCI A0 Baud Rate 1  */
  unsigned char UCA0BR1__SPI;   /*  */
} @ 0x0063;

__no_init volatile union
{
  unsigned char UCA0MCTL;   /* USCI A0 Modulation Control  */

  struct
  {
    unsigned char UCOS16          : 1; /* USCI 16-times Oversampling enable  */
    unsigned char UCBRS0          : 1; /* USCI Second Stage Modulation Select 0  */
    unsigned char UCBRS1          : 1; /* USCI Second Stage Modulation Select 1  */
    unsigned char UCBRS2          : 1; /* USCI Second Stage Modulation Select 2  */
    unsigned char UCBRF0          : 1; /* USCI First Stage Modulation Select 0  */
    unsigned char UCBRF1          : 1; /* USCI First Stage Modulation Select 1  */
    unsigned char UCBRF2          : 1; /* USCI First Stage Modulation Select 2  */
    unsigned char UCBRF3          : 1; /* USCI First Stage Modulation Select 3  */
  } UCA0MCTL_bit;

  unsigned char UCA0MCTL__SPI;   /*  */
} @ 0x0064;

enum {
  UCOS16          = 0x0001,
  UCBRS0          = 0x0002,
  UCBRS1          = 0x0004,
  UCBRS2          = 0x0008,
  UCBRF0          = 0x0010,
  UCBRF1          = 0x0020,
  UCBRF2          = 0x0040,
  UCBRF3          = 0x0080
};

__no_init volatile union
{
  unsigned char UCA0STAT;   /* USCI A0 Status Register  */

  struct
  {
    unsigned char UCBUSY          : 1; /* USCI Busy Flag  */
    unsigned char UCADDR          : 1; /* USCI Address received Flag  */
    unsigned char UCRXERR         : 1; /* USCI RX Error Flag  */
    unsigned char UCBRK           : 1; /* USCI Break received  */
    unsigned char UCPE            : 1; /* USCI Parity Error Flag  */
    unsigned char UCOE            : 1; /* USCI Overrun Error Flag  */
    unsigned char UCFE            : 1; /* USCI Frame Error Flag  */
    unsigned char UCLISTEN        : 1; /* USCI Listen mode  */
  } UCA0STAT_bit;

  unsigned char UCA0STAT__SPI;   /*  */
  struct
  {
    unsigned char UCBUSY          : 1; /* USCI Busy Flag  */
    unsigned char                : 4;
    unsigned char UCOE            : 1; /* USCI Overrun Error Flag  */
    unsigned char UCFE            : 1; /* USCI Frame Error Flag  */
    unsigned char UCLISTEN        : 1; /* USCI Listen mode  */
  } UCA0STAT__SPI_bit;

} @ 0x0065;

enum {
  UCBUSY          = 0x0001,
  UCADDR          = 0x0002,
  UCRXERR         = 0x0004,
  UCBRK           = 0x0008,
  UCPE            = 0x0010,
  UCOE            = 0x0020,
  UCFE            = 0x0040,
  UCLISTEN        = 0x0080
};

__no_init volatile union
{
  unsigned const char UCA0RXBUF;   /* USCI A0 Receive Buffer  */
  unsigned char UCA0RXBUF__SPI;   /*  */
} @ 0x0066;

__no_init volatile union
{
  unsigned char UCA0TXBUF;   /* USCI A0 Transmit Buffer  */
  unsigned char UCA0TXBUF__SPI;   /*  */
} @ 0x0067;


__no_init volatile union
{
  unsigned char UCA0ABCTL;   /* USCI A0 LIN Control  */

  struct
  {
    unsigned char UCABDEN         : 1; /* Auto Baud Rate detect enable  */
    unsigned char                : 1;
    unsigned char UCBTOE          : 1; /* Break Timeout error  */
    unsigned char UCSTOE          : 1; /* Sync-Field Timeout error  */
    unsigned char UCDELIM0        : 1; /* Break Sync Delimiter 0  */
    unsigned char UCDELIM1        : 1; /* Break Sync Delimiter 1  */
  }UCA0ABCTL_bit;
} @ 0x005D;


enum {
  UCABDEN         = 0x0001,
  UCBTOE          = 0x0004,
  UCSTOE          = 0x0008,
  UCDELIM0        = 0x0010,
  UCDELIM1        = 0x0020
};


__no_init volatile union
{
  unsigned char UCA0IRTCTL;   /* USCI A0 IrDA Transmit Control  */

  struct
  {
    unsigned char UCIREN          : 1; /* IRDA Encoder/Decoder enable  */
    unsigned char UCIRTXCLK       : 1; /* IRDA Transmit Pulse Clock Select  */
    unsigned char UCIRTXPL0       : 1; /* IRDA Transmit Pulse Length 0  */
    unsigned char UCIRTXPL1       : 1; /* IRDA Transmit Pulse Length 1  */
    unsigned char UCIRTXPL2       : 1; /* IRDA Transmit Pulse Length 2  */
    unsigned char UCIRTXPL3       : 1; /* IRDA Transmit Pulse Length 3  */
    unsigned char UCIRTXPL4       : 1; /* IRDA Transmit Pulse Length 4  */
    unsigned char UCIRTXPL5       : 1; /* IRDA Transmit Pulse Length 5  */
  }UCA0IRTCTL_bit;
} @ 0x005E;


enum {
  UCIREN          = 0x0001,
  UCIRTXCLK       = 0x0002,
  UCIRTXPL0       = 0x0004,
  UCIRTXPL1       = 0x0008,
  UCIRTXPL2       = 0x0010,
  UCIRTXPL3       = 0x0020,
  UCIRTXPL4       = 0x0040,
  UCIRTXPL5       = 0x0080
};


__no_init volatile union
{
  unsigned char UCA0IRRCTL;   /* USCI A0 IrDA Receive Control  */

  struct
  {
    unsigned char UCIRRXFE        : 1; /* IRDA Receive Filter enable  */
    unsigned char UCIRRXPL        : 1; /* IRDA Receive Input Polarity  */
    unsigned char UCIRRXFL0       : 1; /* IRDA Receive Filter Length 0  */
    unsigned char UCIRRXFL1       : 1; /* IRDA Receive Filter Length 1  */
    unsigned char UCIRRXFL2       : 1; /* IRDA Receive Filter Length 2  */
    unsigned char UCIRRXFL3       : 1; /* IRDA Receive Filter Length 3  */
    unsigned char UCIRRXFL4       : 1; /* IRDA Receive Filter Length 4  */
    unsigned char UCIRRXFL5       : 1; /* IRDA Receive Filter Length 5  */
  }UCA0IRRCTL_bit;
} @ 0x005F;


enum {
  UCIRRXFE        = 0x0001,
  UCIRRXPL        = 0x0002,
  UCIRRXFL0       = 0x0004,
  UCIRRXFL1       = 0x0008,
  UCIRRXFL2       = 0x0010,
  UCIRRXFL3       = 0x0020,
  UCIRRXFL4       = 0x0040,
  UCIRRXFL5       = 0x0080
};



#line 2320 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\io430g2553.h"

#line 2337 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\io430g2553.h"

#line 2347 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\io430g2553.h"

/*-------------------------------------------------------------------------
 *   USCI_A0  SPI Mode
 *-------------------------------------------------------------------------*/


enum {
/*  UCSYNC          = 0x0001, */
/*  UCMODE0         = 0x0002, */
/*  UCMODE1         = 0x0004, */
  UCMST           = 0x0008,
/*  UC7BIT          = 0x0010, */
/*  UCMSB           = 0x0020, */
  UCCKPL          = 0x0040,
  UCCKPH          = 0x0080
};

/*
enum {
  UCSWRST         = 0x0001,
  UCSSEL0         = 0x0040,
  UCSSEL1         = 0x0080,
};

*/
/*
enum {
  UCBUSY          = 0x0001,
  UCOE            = 0x0020,
  UCFE            = 0x0040,
  UCLISTEN        = 0x0080,
};

*/
/*-------------------------------------------------------------------------
 *   USCI_B0  SPI Mode
 *-------------------------------------------------------------------------*/


__no_init volatile union
{
  unsigned char UCB0CTL0__SPI;   /*  */

  struct
  {
    unsigned char UCSYNC          : 1; /* Sync-Mode  0:UART-Mode / 1:SPI-Mode  */
    unsigned char UCMODE0         : 1; /* Async. Mode: USCI Mode 0  */
    unsigned char UCMODE1         : 1; /* Async. Mode: USCI Mode 1  */
    unsigned char UCMST           : 1; /* Sync. Mode: Master Select  */
    unsigned char UC7BIT          : 1; /* Async. Mode: Data Bits  0:8-bits / 1:7-bits  */
    unsigned char UCMSB           : 1; /* Async. Mode: MSB first  0:LSB / 1:MSB  */
    unsigned char UCCKPL          : 1; /* Sync. Mode: Clock Polarity  */
    unsigned char UCCKPH          : 1; /* Sync. Mode: Clock Phase  */
  } UCB0CTL0__SPI_bit;

  unsigned char UCB0CTL0;   /* USCI B0 Control Register 0  */
  struct
  {
    unsigned char UCSYNC          : 1; /* Sync-Mode  0:UART-Mode / 1:SPI-Mode  */
    unsigned char UCMODE0         : 1; /* Async. Mode: USCI Mode 0  */
    unsigned char UCMODE1         : 1; /* Async. Mode: USCI Mode 1  */
    unsigned char UCMST           : 1; /* Sync. Mode: Master Select  */
    unsigned char                : 1;
    unsigned char UCMM            : 1; /* Multi-Master Environment  */
    unsigned char UCSLA10         : 1; /* 10-bit Slave Address Mode  */
    unsigned char UCA10           : 1; /* 10-bit Address Mode  */
  } UCB0CTL0_bit;

} @ 0x0068;

/*
enum {
  UCSYNC          = 0x0001,
  UCMODE0         = 0x0002,
  UCMODE1         = 0x0004,
  UCMST           = 0x0008,
  UC7BIT          = 0x0010,
  UCMSB           = 0x0020,
  UCCKPL          = 0x0040,
  UCCKPH          = 0x0080,
};

*/
__no_init volatile union
{
  unsigned char UCB0CTL1__SPI;   /*  */

  struct
  {
    unsigned char UCSWRST         : 1; /* USCI Software Reset  */
    unsigned char                : 5;
    unsigned char UCSSEL0         : 1; /* USCI 0 Clock Source Select 0  */
    unsigned char UCSSEL1         : 1; /* USCI 0 Clock Source Select 1  */
  } UCB0CTL1__SPI_bit;

  unsigned char UCB0CTL1;   /* USCI B0 Control Register 1  */
  struct
  {
    unsigned char UCSWRST         : 1; /* USCI Software Reset  */
    unsigned char UCTXSTT         : 1; /* Transmit START  */
    unsigned char UCTXSTP         : 1; /* Transmit STOP  */
    unsigned char UCTXNACK        : 1; /* Transmit NACK  */
    unsigned char UCTR            : 1; /* Transmit/Receive Select/Flag  */
    unsigned char                : 1;
    unsigned char UCSSEL0         : 1; /* USCI 0 Clock Source Select 0  */
    unsigned char UCSSEL1         : 1; /* USCI 0 Clock Source Select 1  */
  } UCB0CTL1_bit;

} @ 0x0069;

/*
enum {
  UCSWRST         = 0x0001,
  UCSSEL0         = 0x0040,
  UCSSEL1         = 0x0080,
};

*/
__no_init volatile union
{
  unsigned char UCB0BR0__SPI;   /*  */
  unsigned char UCB0BR0;   /* USCI B0 Baud Rate 0  */
} @ 0x006A;

__no_init volatile union
{
  unsigned char UCB0BR1__SPI;   /*  */
  unsigned char UCB0BR1;   /* USCI B0 Baud Rate 1  */
} @ 0x006B;

__no_init volatile union
{
  unsigned char UCB0STAT__SPI;   /*  */

  struct
  {
    unsigned char UCBUSY          : 1; /* USCI Busy Flag  */
    unsigned char                : 4;
    unsigned char UCOE            : 1; /* USCI Overrun Error Flag  */
    unsigned char UCFE            : 1; /* USCI Frame Error Flag  */
    unsigned char UCLISTEN        : 1; /* USCI Listen mode  */
  } UCB0STAT__SPI_bit;

  unsigned char UCB0STAT;   /* USCI B0 Status Register  */
  struct
  {
    unsigned char UCALIFG         : 1; /* Arbitration Lost interrupt Flag  */
    unsigned char UCSTTIFG        : 1; /* START Condition interrupt Flag  */
    unsigned char UCSTPIFG        : 1; /* STOP Condition interrupt Flag  */
    unsigned char UCNACKIFG       : 1; /* NAK Condition interrupt Flag  */
    unsigned char UCBBUSY         : 1; /* Bus Busy Flag  */
    unsigned char UCGC            : 1; /* General Call address received Flag  */
    unsigned char UCSCLLOW        : 1; /* SCL low  */
    unsigned char UCLISTEN        : 1; /* USCI Listen mode  */
  } UCB0STAT_bit;

} @ 0x006D;

/*
enum {
  UCBUSY          = 0x0001,
  UCOE            = 0x0020,
  UCFE            = 0x0040,
  UCLISTEN        = 0x0080,
};

*/
__no_init volatile union
{
  unsigned char UCB0RXBUF__SPI;   /*  */
  unsigned const char UCB0RXBUF;   /* USCI B0 Receive Buffer  */
} @ 0x006E;

__no_init volatile union
{
  unsigned char UCB0TXBUF__SPI;   /*  */
  unsigned char UCB0TXBUF;   /* USCI B0 Transmit Buffer  */
} @ 0x006F;

/*-------------------------------------------------------------------------
 *   USCI_B0  I2C Mode
 *-------------------------------------------------------------------------*/


enum {
/*  UCSYNC          = 0x0001, */
/*  UCMODE0         = 0x0002, */
/*  UCMODE1         = 0x0004, */
/*  UCMST           = 0x0008, */
  UCMM            = 0x0020,
  UCSLA10         = 0x0040,
  UCA10           = 0x0080
};

enum {
/*  UCSWRST         = 0x0001, */
  UCTXSTT         = 0x0002,
  UCTXSTP         = 0x0004,
  UCTXNACK        = 0x0008,
  UCTR            = 0x0010
/*  UCSSEL0         = 0x0040, */
/*  UCSSEL1         = 0x0080, */
};


__no_init volatile union
{
  unsigned char UCB0I2CIE;   /* USCI B0 I2C Interrupt Enable Register  */

  struct
  {
    unsigned char UCALIE          : 1; /* Arbitration Lost interrupt enable  */
    unsigned char UCSTTIE         : 1; /* START Condition interrupt enable  */
    unsigned char UCSTPIE         : 1; /* STOP Condition interrupt enable  */
    unsigned char UCNACKIE        : 1; /* NACK Condition interrupt enable  */
  }UCB0I2CIE_bit;
} @ 0x006C;


enum {
  UCALIE          = 0x0001,
  UCSTTIE         = 0x0002,
  UCSTPIE         = 0x0004,
  UCNACKIE        = 0x0008
};

enum {
  UCALIFG         = 0x0001,
  UCSTTIFG        = 0x0002,
  UCSTPIFG        = 0x0004,
  UCNACKIFG       = 0x0008,
  UCBBUSY         = 0x0010,
  UCGC            = 0x0020,
  UCSCLLOW        = 0x0040
/*  UCLISTEN        = 0x0080, */
};


__no_init volatile union
{
  unsigned short UCB0I2COA;   /* USCI B0 I2C Own Address  */

  struct
  {
    unsigned short UCOA0           : 1; /* I2C Own Address 0  */
    unsigned short UCOA1           : 1; /* I2C Own Address 1  */
    unsigned short UCOA2           : 1; /* I2C Own Address 2  */
    unsigned short UCOA3           : 1; /* I2C Own Address 3  */
    unsigned short UCOA4           : 1; /* I2C Own Address 4  */
    unsigned short UCOA5           : 1; /* I2C Own Address 5  */
    unsigned short UCOA6           : 1; /* I2C Own Address 6  */
    unsigned short UCOA7           : 1; /* I2C Own Address 7  */
    unsigned short UCOA8           : 1; /* I2C Own Address 8  */
    unsigned short UCOA9           : 1; /* I2C Own Address 9  */
    unsigned short                : 5;
    unsigned short UCGCEN          : 1; /* I2C General Call enable  */
  }UCB0I2COA_bit;
} @ 0x0118;


enum {
  UCOA0           = 0x0001,
  UCOA1           = 0x0002,
  UCOA2           = 0x0004,
  UCOA3           = 0x0008,
  UCOA4           = 0x0010,
  UCOA5           = 0x0020,
  UCOA6           = 0x0040,
  UCOA7           = 0x0080,
  UCOA8           = 0x0100,
  UCOA9           = 0x0200,
  UCGCEN          = 0x8000
};


__no_init volatile union
{
  unsigned short UCB0I2CSA;   /* USCI B0 I2C Slave Address  */

  struct
  {
    unsigned short UCSA0           : 1; /* I2C Slave Address 0  */
    unsigned short UCSA1           : 1; /* I2C Slave Address 1  */
    unsigned short UCSA2           : 1; /* I2C Slave Address 2  */
    unsigned short UCSA3           : 1; /* I2C Slave Address 3  */
    unsigned short UCSA4           : 1; /* I2C Slave Address 4  */
    unsigned short UCSA5           : 1; /* I2C Slave Address 5  */
    unsigned short UCSA6           : 1; /* I2C Slave Address 6  */
    unsigned short UCSA7           : 1; /* I2C Slave Address 7  */
    unsigned short UCSA8           : 1; /* I2C Slave Address 8  */
    unsigned short UCSA9           : 1; /* I2C Slave Address 9  */
  }UCB0I2CSA_bit;
} @ 0x011A;


enum {
  UCSA0           = 0x0001,
  UCSA1           = 0x0002,
  UCSA2           = 0x0004,
  UCSA3           = 0x0008,
  UCSA4           = 0x0010,
  UCSA5           = 0x0020,
  UCSA6           = 0x0040,
  UCSA7           = 0x0080,
  UCSA8           = 0x0100,
  UCSA9           = 0x0200
};





/*-------------------------------------------------------------------------
 *   Watchdog Timer
 *-------------------------------------------------------------------------*/



__no_init volatile union
{
  unsigned short WDTCTL;   /* Watchdog Timer Control  */

  struct
  {
    unsigned short WDTIS0          : 1; /*   */
    unsigned short WDTIS1          : 1; /*   */
    unsigned short WDTSSEL         : 1; /*   */
    unsigned short WDTCNTCL        : 1; /*   */
    unsigned short WDTTMSEL        : 1; /*   */
    unsigned short WDTNMI          : 1; /*   */
    unsigned short WDTNMIES        : 1; /*   */
    unsigned short WDTHOLD         : 1; /*   */
  }WDTCTL_bit;
} @ 0x0120;


enum {
  WDTIS0          = 0x0001,
  WDTIS1          = 0x0002,
  WDTSSEL         = 0x0004,
  WDTCNTCL        = 0x0008,
  WDTTMSEL        = 0x0010,
  WDTNMI          = 0x0020,
  WDTNMIES        = 0x0040,
  WDTHOLD         = 0x0080
};






/* WDT is clocked by fSMCLK (assumed 1MHz) */




/* WDT is clocked by fACLK (assumed 32KHz) */




/* WDT is clocked by fSMCLK (assumed 1MHz) */




/* WDT is clocked by fACLK (assumed 32KHz) */





/*-------------------------------------------------------------------------
 *   Calibration Data
 *-------------------------------------------------------------------------*/



  /* DCOCTL  Calibration Data for 16MHz  */
__no_init volatile unsigned const char CALDCO_16MHZ @ 0x10F8;



  /* BCSCTL1 Calibration Data for 16MHz  */
__no_init volatile unsigned const char CALBC1_16MHZ @ 0x10F9;



  /* DCOCTL  Calibration Data for 12MHz  */
__no_init volatile unsigned const char CALDCO_12MHZ @ 0x10FA;



  /* BCSCTL1 Calibration Data for 12MHz  */
__no_init volatile unsigned const char CALBC1_12MHZ @ 0x10FB;



  /* DCOCTL  Calibration Data for 8MHz  */
__no_init volatile unsigned const char CALDCO_8MHZ @ 0x10FC;



  /* BCSCTL1 Calibration Data for 8MHz  */
__no_init volatile unsigned const char CALBC1_8MHZ @ 0x10FD;



  /* DCOCTL  Calibration Data for 1MHz  */
__no_init volatile unsigned const char CALDCO_1MHZ @ 0x10FE;



  /* BCSCTL1 Calibration Data for 1MHz  */
__no_init volatile unsigned const char CALBC1_1MHZ @ 0x10FF;



/*-------------------------------------------------------------------------
 *   TLV Calibration Data
 *-------------------------------------------------------------------------*/



  /* TLV CHECK SUM  */
__no_init volatile unsigned const short TLV_CHECKSUM @ 0x10C0;



  /* TLV TAG_DCO30 TAG  */
__no_init volatile unsigned const char TLV_DCO_30_TAG @ 0x10F6;



  /* TLV TAG_DCO30 LEN  */
__no_init volatile unsigned const char TLV_DCO_30_LEN @ 0x10F7;



  /* TLV ADC10_1 TAG  */
__no_init volatile unsigned const char TLV_ADC10_1_TAG @ 0x10DA;



  /* TLV ADC10_1 LEN  */
__no_init volatile unsigned const char TLV_ADC10_1_LEN @ 0x10DB;


/* TLV Calibration Data Structure */




#line 2809 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\io430g2553.h"

#line 2818 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\io430g2553.h"




#pragma language=restore



/************************************************************
* Timer A interrupt vector value
************************************************************/

/* Compability definitions */






/************************************************************
* Interrupt Vectors (offset from 0xFFE0)
************************************************************/

#line 2855 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\io430g2553.h"


#line 768 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\io430.h"

/********************************************************************
 *  /MSP430Ixxxx Family/ 
 ********************************************************************/

#line 2135 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\io430.h"


#line 20 "C:\\Siddharth\\Personal\\Embedded C\\IAR_programs\\SpeedometerDesign\\drv.h"




void drv_ledInit();
void drv_hallInit();
void drv_blinkLED();
__interrupt void Port_1_ISR(void);

#line 18 "C:\\Siddharth\\Personal\\Embedded C\\IAR_programs\\SpeedometerDesign\\drv.c"
#line 1 "C:\\Siddharth\\Personal\\Embedded C\\IAR_programs\\SpeedometerDesign\\hal.h"
/*************************************************************************/
/*  File name: 	hal.h
*
*  Purpose:	Header file for the Hardware Abstraction Layer.
*               Constants used for application are set here.
*
*  Owner:  	Srividya Prasad, Siddharth D Srinivas, Gaurav Sai Palasari
*
*  Department:   ECE, ECE, EEE
*
*  Version History:
*  V4.0  30 July, 2023        Final version created
*/
/******************************************************************************/




#line 1 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\stdio.h"
/* stdio.h standard header */
/* Copyright 2003-2010 IAR Systems AB.  */




  #pragma system_include


#line 1 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\ycheck.h"
/* ycheck.h internal checking header file. */
/* Copyright 2005-2010 IAR Systems AB. */

/* Note that there is no include guard for this header. This is intentional. */


  #pragma system_include


/* __INTRINSIC
 *
 * Note: Redefined each time ycheck.h is included, i.e. for each
 * system header, to ensure that intrinsic support could be turned off
 * individually for each file.
 */










/* __AEABI_PORTABILITY_INTERNAL_LEVEL
 *
 * Note: Redefined each time ycheck.h is included, i.e. for each
 * system header, to ensure that ABI support could be turned off/on
 * individually for each file.
 *
 * Possible values for this preprocessor symbol:
 *
 * 0 - ABI portability mode is disabled.
 *
 * 1 - ABI portability mode (version 1) is enabled.
 *
 * All other values are reserved for future use.
 */






#line 67 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\ycheck.h"




/* A definiton for a function of what effects it has.
   NS  = no_state, errno, i.e. it uses no internal or external state. It may
         write to errno though
   NE  = no_state, i.e. it uses no internal or external state, not even
         writing to errno. 
   NRx = no_read(x), i.e. it doesn't read through pointer parameter x.
   NWx = no_write(x), i.e. it doesn't write through pointer parameter x.
   Rx  = returns x, i.e. the function will return parameter x.
   
   All the functions with effects also has "always_returns", 
   i.e. the function will always return.
*/

#line 103 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\ycheck.h"









#line 11 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\stdio.h"
#line 1 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\yvals.h"
/* yvals.h internal configuration header file. */
/* Copyright 2001-2010 IAR Systems AB. */





  #pragma system_include


#line 1 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\ycheck.h"
/* ycheck.h internal checking header file. */
/* Copyright 2005-2010 IAR Systems AB. */

/* Note that there is no include guard for this header. This is intentional. */


  #pragma system_include


/* __INTRINSIC
 *
 * Note: Redefined each time ycheck.h is included, i.e. for each
 * system header, to ensure that intrinsic support could be turned off
 * individually for each file.
 */










/* __AEABI_PORTABILITY_INTERNAL_LEVEL
 *
 * Note: Redefined each time ycheck.h is included, i.e. for each
 * system header, to ensure that ABI support could be turned off/on
 * individually for each file.
 *
 * Possible values for this preprocessor symbol:
 *
 * 0 - ABI portability mode is disabled.
 *
 * 1 - ABI portability mode (version 1) is enabled.
 *
 * All other values are reserved for future use.
 */






#line 67 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\ycheck.h"

#line 12 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\yvals.h"

                /* Convenience macros */









/* Used to refer to '__aeabi' symbols in the library. */ 


                /* Versions */










/*
 * Support for some C99 or other symbols
 *
 * This setting makes available some macros, functions, etc that are
 * beneficial.
 *
 * Default is to include them.
 *
 * Disabling this in C++ mode will not compile (some C++ functions uses C99
 * functionality).
 */


  /* Default turned on when compiling C++, EC++, or C99. */
#line 59 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\yvals.h"





#line 70 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\yvals.h"

                /* Configuration */
#line 1 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\DLib_Defaults.h"
/***************************************************
 *
 * DLib_Defaults.h is the library configuration manager.
 *
 * Copyright 2003-2010 IAR Systems AB.  
 *
 * This configuration header file performs the following tasks:
 *
 * 1. Includes the configuration header file, defined by _DLIB_CONFIG_FILE,
 *    that sets up a particular runtime environment.
 *
 * 2. Includes the product configuration header file, DLib_Product.h, that
 *    specifies default values for the product and makes sure that the
 *    configuration is valid.
 *
 * 3. Sets up default values for all remaining configuration symbols.
 *
 * This configuration header file, the one defined by _DLIB_CONFIG_FILE, and
 * DLib_Product.h configures how the runtime environment should behave. This
 * includes all system headers and the library itself, i.e. all system headers
 * includes this configuration header file, and the library has been built
 * using this configuration header file.
 *
 ***************************************************
 *
 * DO NOT MODIFY THIS FILE!
 *
 ***************************************************/





  #pragma system_include


/* Include the main configuration header file. */
#line 1 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\lib\\dlib\\dl430fn.h"
/* Customer-specific DLib configuration. */
/* Copyright (C) 2003 IAR Systems.  All rights reserved. */





  #pragma system_include


/* No changes to the defaults. */

#line 40 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\DLib_Defaults.h"
  /* _DLIB_CONFIG_FILE_STRING is the quoted variant of above */
#line 47 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\DLib_Defaults.h"

/* Include the product specific header file. */
#line 1 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\DLib_Product.h"
/* MSP430-specific DLib configuration. */
/* Copyright 2004-2008 IAR Systems AB. */





  #pragma system_include


/*
 * This is the MSP430-specific configuration file for DLib.
 *
 * This file is included right after the _DLIB_CONFIG_FILE (like
 * DLib_Config.h) file that the user can use to configure the library.
 * The file DLib_Defaults.h is then included to set up defaults for
 * all configuration variables that haven't got a value.
 */

/* Special placement for locale structures when building ropi libraries */





/* Adapt multithread support */






#line 51 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\DLib_Defaults.h"



/*
 * The remainder of the file sets up defaults for a number of
 * configuration symbols, each corresponds to a feature in the
 * libary.
 *
 * The value of the symbols should either be 1, if the feature should
 * be supported, or 0 if it shouldn't. (Except where otherwise
 * noted.)
 */


/*
 * Small or Large target
 *
 * This define determines whether the target is large or small. It must be 
 * setup in the DLib_Product header or in the compiler itself.
 *
 * For a small target some functionality in the library will not deliver 
 * the best available results. For instance the _accurate variants will not use
 * the extra precision packet for large arguments.
 * 
 */







/*
 * File handling
 *
 * Determines whether FILE descriptors and related functions exists or not.
 * When this feature is selected, i.e. set to 1, then FILE descriptors and
 * related functions (e.g. fprintf, fopen) exist. All files, even stdin,
 * stdout, and stderr will then be handled with a file system mechanism that
 * buffers files before accessing the lowlevel I/O interface (__open, __read,
 * __write, etc).
 *
 * If not selected, i.e. set to 0, then FILE descriptors and related functions
 * (e.g. fprintf, fopen) does not exist. All functions that normally uses
 * stderr will use stdout instead. Functions that uses stdout and stdin (like
 * printf and scanf) will access the lowlevel I/O interface directly (__open,
 * __read, __write, etc), i.e. there will not be any buffering.
 *
 * The default is not to have support for FILE descriptors.
 */





/*
 * Use static buffers for stdout
 *
 * This setting controls whether the stream stdout uses a static 80 bytes
 * buffer or uses a one byte buffer allocated in the file descriptor. This
 * setting is only applicable if the FILE descriptors are enabled above.
 *
 * Default is to use a static 80 byte buffer.
 */





/*
 * Support of locale interface
 *
 * "Locale" is the system in C that support language- and
 * contry-specific settings for a number of areas, including currency
 * symbols, date and time, and multibyte encodings.
 *
 * This setting determines whether the locale interface exist or not.
 * When this feature is selected, i.e. set to 1, the locale interface exist
 * (setlocale, etc). A number of preselected locales can be activated during
 * runtime. The preselected locales and encodings is choosen by defining any
 * number of _LOCALE_USE_xxx and _ENCODING_USE_xxx symbols. The application
 * will start with the "C" locale choosen. (Single byte encoding is always
 * supported in this mode.)
 *
 *
 * If not selected, i.e. set to 0, the locale interface (setlocale, etc) does
 * not exist. One preselected locale and one preselected encoding is then used
 * directly. That locale can not be changed during runtime. The preselected
 * locale and encoding is choosen by defining at most one of _LOCALE_USE_xxx
 * and at most one of _ENCODING_USE_xxx. The default is to use the "C" locale
 * and the single byte encoding, respectively.
 *
 * The default is not to have support for the locale interface with the "C"
 * locale and the single byte encoding.
 *
 * Supported locales
 * -----------------
 * _LOCALE_USE_C                  C standard locale (the default)
 * _LOCALE_USE_POSIX ISO-8859-1   Posix locale
 * _LOCALE_USE_CS_CZ ISO-8859-2   Czech language locale for Czech Republic
 * _LOCALE_USE_DA_DK ISO-8859-1   Danish language locale for Denmark
 * _LOCALE_USE_DA_EU ISO-8859-15  Danish language locale for Europe
 * _LOCALE_USE_DE_AT ISO-8859-1   German language locale for Austria
 * _LOCALE_USE_DE_BE ISO-8859-1   German language locale for Belgium
 * _LOCALE_USE_DE_CH ISO-8859-1   German language locale for Switzerland
 * _LOCALE_USE_DE_DE ISO-8859-1   German language locale for Germany
 * _LOCALE_USE_DE_EU ISO-8859-15  German language locale for Europe
 * _LOCALE_USE_DE_LU ISO-8859-1   German language locale for Luxemburg
 * _LOCALE_USE_EL_EU ISO-8859-7x  Greek language locale for Europe
 *                                (Euro symbol added)
 * _LOCALE_USE_EL_GR ISO-8859-7   Greek language locale for Greece
 * _LOCALE_USE_EN_AU ISO-8859-1   English language locale for Australia
 * _LOCALE_USE_EN_CA ISO-8859-1   English language locale for Canada
 * _LOCALE_USE_EN_DK ISO_8859-1   English language locale for Denmark
 * _LOCALE_USE_EN_EU ISO-8859-15  English language locale for Europe
 * _LOCALE_USE_EN_GB ISO-8859-1   English language locale for United Kingdom
 * _LOCALE_USE_EN_IE ISO-8859-1   English language locale for Ireland
 * _LOCALE_USE_EN_NZ ISO-8859-1   English language locale for New Zealand
 * _LOCALE_USE_EN_US ISO-8859-1   English language locale for USA
 * _LOCALE_USE_ES_AR ISO-8859-1   Spanish language locale for Argentina
 * _LOCALE_USE_ES_BO ISO-8859-1   Spanish language locale for Bolivia
 * _LOCALE_USE_ES_CL ISO-8859-1   Spanish language locale for Chile
 * _LOCALE_USE_ES_CO ISO-8859-1   Spanish language locale for Colombia
 * _LOCALE_USE_ES_DO ISO-8859-1   Spanish language locale for Dominican Republic
 * _LOCALE_USE_ES_EC ISO-8859-1   Spanish language locale for Equador
 * _LOCALE_USE_ES_ES ISO-8859-1   Spanish language locale for Spain
 * _LOCALE_USE_ES_EU ISO-8859-15  Spanish language locale for Europe
 * _LOCALE_USE_ES_GT ISO-8859-1   Spanish language locale for Guatemala
 * _LOCALE_USE_ES_HN ISO-8859-1   Spanish language locale for Honduras
 * _LOCALE_USE_ES_MX ISO-8859-1   Spanish language locale for Mexico
 * _LOCALE_USE_ES_PA ISO-8859-1   Spanish language locale for Panama
 * _LOCALE_USE_ES_PE ISO-8859-1   Spanish language locale for Peru
 * _LOCALE_USE_ES_PY ISO-8859-1   Spanish language locale for Paraguay
 * _LOCALE_USE_ES_SV ISO-8859-1   Spanish language locale for Salvador
 * _LOCALE_USE_ES_US ISO-8859-1   Spanish language locale for USA
 * _LOCALE_USE_ES_UY ISO-8859-1   Spanish language locale for Uruguay
 * _LOCALE_USE_ES_VE ISO-8859-1   Spanish language locale for Venezuela
 * _LOCALE_USE_ET_EE ISO-8859-1   Estonian language for Estonia
 * _LOCALE_USE_EU_ES ISO-8859-1   Basque language locale for Spain
 * _LOCALE_USE_FI_EU ISO-8859-15  Finnish language locale for Europe
 * _LOCALE_USE_FI_FI ISO-8859-1   Finnish language locale for Finland
 * _LOCALE_USE_FO_FO ISO-8859-1   Faroese language locale for Faroe Islands
 * _LOCALE_USE_FR_BE ISO-8859-1   French language locale for Belgium
 * _LOCALE_USE_FR_CA ISO-8859-1   French language locale for Canada
 * _LOCALE_USE_FR_CH ISO-8859-1   French language locale for Switzerland
 * _LOCALE_USE_FR_EU ISO-8859-15  French language locale for Europe
 * _LOCALE_USE_FR_FR ISO-8859-1   French language locale for France
 * _LOCALE_USE_FR_LU ISO-8859-1   French language locale for Luxemburg
 * _LOCALE_USE_GA_EU ISO-8859-15  Irish language locale for Europe
 * _LOCALE_USE_GA_IE ISO-8859-1   Irish language locale for Ireland
 * _LOCALE_USE_GL_ES ISO-8859-1   Galician language locale for Spain
 * _LOCALE_USE_HR_HR ISO-8859-2   Croatian language locale for Croatia
 * _LOCALE_USE_HU_HU ISO-8859-2   Hungarian language locale for Hungary
 * _LOCALE_USE_ID_ID ISO-8859-1   Indonesian language locale for Indonesia
 * _LOCALE_USE_IS_EU ISO-8859-15  Icelandic language locale for Europe
 * _LOCALE_USE_IS_IS ISO-8859-1   Icelandic language locale for Iceland
 * _LOCALE_USE_IT_EU ISO-8859-15  Italian language locale for Europe
 * _LOCALE_USE_IT_IT ISO-8859-1   Italian language locale for Italy
 * _LOCALE_USE_IW_IL ISO-8859-8   Hebrew language locale for Israel
 * _LOCALE_USE_KL_GL ISO-8859-1   Greenlandic language locale for Greenland
 * _LOCALE_USE_LT_LT   BALTIC     Lithuanian languagelocale for Lithuania
 * _LOCALE_USE_LV_LV   BALTIC     Latvian languagelocale for Latvia
 * _LOCALE_USE_NL_BE ISO-8859-1   Dutch language locale for Belgium
 * _LOCALE_USE_NL_EU ISO-8859-15  Dutch language locale for Europe
 * _LOCALE_USE_NL_NL ISO-8859-9   Dutch language locale for Netherlands
 * _LOCALE_USE_NO_EU ISO-8859-15  Norwegian language locale for Europe
 * _LOCALE_USE_NO_NO ISO-8859-1   Norwegian language locale for Norway
 * _LOCALE_USE_PL_PL ISO-8859-2   Polish language locale for Poland
 * _LOCALE_USE_PT_BR ISO-8859-1   Portugese language locale for Brazil
 * _LOCALE_USE_PT_EU ISO-8859-15  Portugese language locale for Europe
 * _LOCALE_USE_PT_PT ISO-8859-1   Portugese language locale for Portugal
 * _LOCALE_USE_RO_RO ISO-8859-2   Romanian language locale for Romania
 * _LOCALE_USE_RU_RU ISO-8859-5   Russian language locale for Russia
 * _LOCALE_USE_SL_SI ISO-8859-2   Slovenian language locale for Slovenia
 * _LOCALE_USE_SV_EU ISO-8859-15  Swedish language locale for Europe
 * _LOCALE_USE_SV_FI ISO-8859-1   Swedish language locale for Finland
 * _LOCALE_USE_SV_SE ISO-8859-1   Swedish language locale for Sweden
 * _LOCALE_USE_TR_TR ISO-8859-9   Turkish language locale for Turkey
 *
 *  Supported encodings
 *  -------------------
 * n/a                            Single byte (used if no other is defined).
 * _ENCODING_USE_UTF8             UTF8 encoding.
 */






/* We need to have the "C" locale if we have full locale support. */






/*
 * Support of multibytes in printf- and scanf-like functions
 *
 * This is the default value for _DLIB_PRINTF_MULTIBYTE and
 * _DLIB_SCANF_MULTIBYTE. See them for a description.
 *
 * Default is to not have support for multibytes in printf- and scanf-like
 * functions.
 */






/*
 * Throw handling in the EC++ library
 *
 * This setting determines what happens when the EC++ part of the library
 * fails (where a normal C++ library 'throws').
 *
 * The following alternatives exists (setting of the symbol):
 * 0                - The application does nothing, i.e. continues with the
 *                    next statement.
 * 1                - The application terminates by calling the 'abort'
 *                    function directly.
 * <anything else>  - An object of class "exception" is created.  This
 *                    object contains a string describing the problem.
 *                    This string is later emitted on "stderr" before
 *                    the application terminates by calling the 'abort'
 *                    function directly.
 *
 * Default is to do nothing.
 */






/*
 * Hexadecimal floating-point numbers in strtod
 *
 * If selected, i.e. set to 1, strtod supports C99 hexadecimal floating-point
 * numbers. This also enables hexadecimal floating-points in internal functions
 * used for converting strings and wide strings to float, double, and long
 * double.
 *
 * If not selected, i.e. set to 0, C99 hexadecimal floating-point numbers
 * aren't supported.
 *
 * Default is not to support hexadecimal floating-point numbers.
 */






/*
 * Printf configuration symbols.
 *
 * All the configuration symbols described further on controls the behaviour
 * of printf, sprintf, and the other printf variants.
 *
 * The library proves four formatters for printf: 'tiny', 'small',
 * 'large', and 'default'.  The setup in this file controls all except
 * 'tiny'.  Note that both small' and 'large' explicitly removes
 * some features.
 */

/*
 * Handle multibytes in printf
 *
 * This setting controls whether multibytes and wchar_ts are supported in
 * printf. Set to 1 to support them, otherwise set to 0.
 *
 * See _DLIB_FORMATTED_MULTIBYTE for the default setting.
 */





/*
 * Long long formatting in printf
 *
 * This setting controls long long support (%lld) in printf. Set to 1 to
 * support it, otherwise set to 0.

 * Note, if long long should not be supported and 'intmax_t' is larger than
 * an ordinary 'long', then %jd and %jn will not be supported.
 *
 * Default is to support long long formatting.
 */

#line 351 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\DLib_Defaults.h"






/*
 * Floating-point formatting in printf
 *
 * This setting controls whether printf supports floating-point formatting.
 * Set to 1 to support them, otherwise set to 0.
 *
 * Default is to support floating-point formatting.
 */





/*
 * Hexadecimal floating-point formatting in printf
 *
 * This setting controls whether the %a format, i.e. the output of
 * floating-point numbers in the C99 hexadecimal format. Set to 1 to support
 * it, otherwise set to 0.
 *
 * Default is to support %a in printf.
 */





/*
 * Output count formatting in printf
 *
 * This setting controls whether the output count specifier (%n) is supported
 * or not in printf. Set to 1 to support it, otherwise set to 0.
 *
 * Default is to support %n in printf.
 */





/*
 * Support of qualifiers in printf
 *
 * This setting controls whether qualifiers that enlarges the input value
 * [hlLjtz] is supported in printf or not. Set to 1 to support them, otherwise
 * set to 0. See also _DLIB_PRINTF_INT_TYPE_IS_INT and
 * _DLIB_PRINTF_INT_TYPE_IS_LONG.
 *
 * Default is to support [hlLjtz] qualifiers in printf.
 */





/*
 * Support of flags in printf
 *
 * This setting controls whether flags (-+ #0) is supported in printf or not.
 * Set to 1 to support them, otherwise set to 0.
 *
 * Default is to support flags in printf.
 */





/*
 * Support widths and precisions in printf
 *
 * This setting controls whether widths and precisions are supported in printf.
 * Set to 1 to support them, otherwise set to 0.
 *
 * Default is to support widths and precisions in printf.
 */





/*
 * Support of unsigned integer formatting in printf
 *
 * This setting controls whether unsigned integer formatting is supported in
 * printf. Set to 1 to support it, otherwise set to 0.
 *
 * Default is to support unsigned integer formatting in printf.
 */





/*
 * Support of signed integer formatting in printf
 *
 * This setting controls whether signed integer formatting is supported in
 * printf. Set to 1 to support it, otherwise set to 0.
 *
 * Default is to support signed integer formatting in printf.
 */





/*
 * Support of formatting anything larger than int in printf
 *
 * This setting controls if 'int' should be used internally in printf, rather
 * than the largest existing integer type. If 'int' is used, any integer or
 * pointer type formatting use 'int' as internal type even though the
 * formatted type is larger. Set to 1 to use 'int' as internal type, otherwise
 * set to 0.
 *
 * See also next configuration.
 *
 * Default is to internally use largest existing internally type.
 */





/*
 * Support of formatting anything larger than long in printf
 *
 * This setting controls if 'long' should be used internally in printf, rather
 * than the largest existing integer type. If 'long' is used, any integer or
 * pointer type formatting use 'long' as internal type even though the
 * formatted type is larger. Set to 1 to use 'long' as internal type,
 * otherwise set to 0.
 *
 * See also previous configuration.
 *
 * Default is to internally use largest existing internally type.
 */









/*
 * Emit a char a time in printf
 *
 * This setting controls internal output handling. If selected, i.e. set to 1,
 * then printf emits one character at a time, which requires less code but
 * can be slightly slower for some types of output.
 *
 * If not selected, i.e. set to 0, then printf buffers some outputs.
 *
 * Note that it is recommended to either use full file support (see
 * _DLIB_FILE_DESCRIPTOR) or -- for debug output -- use the linker
 * option "-e__write_buffered=__write" to enable buffered I/O rather
 * than deselecting this feature.
 */






/*
 * Scanf configuration symbols.
 *
 * All the configuration symbols described here controls the
 * behaviour of scanf, sscanf, and the other scanf variants.
 *
 * The library proves three formatters for scanf: 'small', 'large',
 * and 'default'.  The setup in this file controls all, however both
 * 'small' and 'large' explicitly removes some features.
 */

/*
 * Handle multibytes in scanf
 *
 * This setting controls whether multibytes and wchar_t:s are supported in
 * scanf. Set to 1 to support them, otherwise set to 0.
 *
 * See _DLIB_FORMATTED_MULTIBYTE for the default.
 */





/*
 * Long long formatting in scanf
 *
 * This setting controls whether scanf supports long long support (%lld). It
 * also controls, if 'intmax_t' is larger than an ordinary 'long', i.e. how
 * the %jd and %jn specifiers behaves. Set to 1 to support them, otherwise set
 * to 0.
 *
 * Default is to support long long formatting in scanf.
 */

#line 566 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\DLib_Defaults.h"





/*
 * Support widths in scanf
 *
 * This controls whether scanf supports widths. Set to 1 to support them,
 * otherwise set to 0.
 *
 * Default is to support widths in scanf.
 */





/*
 * Support qualifiers [hjltzL] in scanf
 *
 * This setting controls whether scanf supports qualifiers [hjltzL] or not. Set
 * to 1 to support them, otherwise set to 0.
 *
 * Default is to support qualifiers in scanf.
 */





/*
 * Support floating-point formatting in scanf
 *
 * This setting controls whether scanf supports floating-point formatting. Set
 * to 1 to support them, otherwise set to 0.
 *
 * Default is to support floating-point formatting in scanf.
 */





/*
 * Support output count formatting (%n)
 *
 * This setting controls whether scanf supports output count formatting (%n).
 * Set to 1 to support it, otherwise set to 0.
 *
 * Default is to support output count formatting in scanf.
 */





/*
 * Support scansets ([]) in scanf
 *
 * This setting controls whether scanf supports scansets ([]) or not. Set to 1
 * to support them, otherwise set to 0.
 *
 * Default is to support scansets in scanf.
 */





/*
 * Support signed integer formatting in scanf
 *
 * This setting controls whether scanf supports signed integer formatting or
 * not. Set to 1 to support them, otherwise set to 0.
 *
 * Default is to support signed integer formatting in scanf.
 */





/*
 * Support unsigned integer formatting in scanf
 *
 * This setting controls whether scanf supports unsigned integer formatting or
 * not. Set to 1 to support them, otherwise set to 0.
 *
 * Default is to support unsigned integer formatting in scanf.
 */





/*
 * Support assignment suppressing [*] in scanf
 *
 * This setting controls whether scanf supports assignment suppressing [*] or
 * not. Set to 1 to support them, otherwise set to 0.
 *
 * Default is to support assignment suppressing in scanf.
 */





/*
 * Handle multibytes in asctime and strftime.
 *
 * This setting controls whether multibytes and wchar_ts are
 * supported.Set to 1 to support them, otherwise set to 0.
 *
 * See _DLIB_FORMATTED_MULTIBYTE for the default setting.
 */





/*
 * True if "qsort" should be implemented using bubble sort.
 *
 * Bubble sort is less efficient than quick sort but requires less RAM
 * and ROM resources.
 */





/*
 * Set Buffert size used in qsort
 */





/*
 * The default "rand" function uses an array of 32 long:s of memory to
 * store the current state.
 *
 * The simple "rand" function uses only a single long. However, the
 * quality of the generated psuedo-random numbers are not as good as
 * the default implementation.
 */





/*
 * Wide character and multi byte character support in library.
 */





/*
 * Set attributes on the function used by the C-SPY debug interface to set a
 * breakpoint in.
 */





/*
 * Support threading in the library
 *
 * 0    No thread support
 * 1    Thread support with a, b, and d.
 * 2    Thread support with a, b, and e.
 * 3    Thread support with all thread-local storage in a dynamically allocated
 *        memory area and a, and b.
 *      a. Lock on heap accesses
 *      b. Optional lock on file accesses (see _DLIB_FILE_OP_LOCKS below)
 *      d. Use an external thread-local storage interface for all the
 *         libraries static and global variables.
 *      e. Static and global variables aren't safe for access from several
 *         threads.
 *
 * Note that if locks are used the following symbols must be defined:
 *
 *   _DLIB_THREAD_LOCK_ONCE_TYPE
 *   _DLIB_THREAD_LOCK_ONCE_MACRO(control_variable, init_function)
 *   _DLIB_THREAD_LOCK_ONCE_TYPE_INIT
 *
 * They will be used to initialize the needed locks only once. TYPE is the
 * type for the static control variable, MACRO is the expression that will be
 * evaluated at each usage of a lock, and INIT is the initializer for the
 * control variable.
 *
 * Note that if thread model 3 is used the symbol _DLIB_TLS_POINTER must be
 * defined. It is a thread local pointer to a dynamic memory area that
 * contains all used TLS variables for the library. Optionally the following
 * symbols can be defined as well (default is to use the default const data
 * and data memories):
 *
 *   _DLIB_TLS_INITIALIZER_MEMORY The memory to place the initializers for the
 *                                TLS memory area
 *   _DLIB_TLS_MEMORY             The memory to use for the TLS memory area. A
 *                                pointer to this memory must be castable to a
 *                                default pointer and back.
 *   _DLIB_TLS_REQUIRE_INIT       Set to 1 to require __cstart_init_tls
 *                                when needed to initialize the TLS data
 *                                segment for the main thread.
 *   _DLIB_TLS_SEGMENT_DATA       The name of the TLS RAM data segment
 *   _DLIB_TLS_SEGMENT_INIT       The name of the used to initialize the
 *                                TLS data segment.
 *
 * See DLib_Threads.h for a description of what interfaces needs to be
 * defined for thread support.
 */





/*
 * Used by products where one runtime library can be used by applications
 * with different data models, in order to reduce the total number of
 * libraries required. Typically, this is used when the pointer types does
 * not change over the data models used, but the placement of data variables
 * or/and constant variables do.
 *
 * If defined, this symbol is typically defined to the memory attribute that
 * is used by the runtime library. The actual define must use a
 * _Pragma("type_attribute = xxx") construct. In the header files, it is used
 * on all statically linked data objects seen by the application.
 */




#line 812 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\DLib_Defaults.h"


/*
 * Turn on support for the Target-specific ABI. The ABI is based on the
 * ARM AEABI. A target, except ARM, may deviate from it.
 */

#line 826 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\DLib_Defaults.h"

#line 857 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\DLib_Defaults.h"


/*
 * Turn on usage of a pragma to tell the linker the number of elements used
 * in a setjmp jmp_buf.
 */





/*
 * If true, the product supplies a "DLib_Product_string.h" file that
 * is included from "string.h".
 */





/*
 * Determine whether the math fma routines are fast or not.
 */




/*
 * Rtti support.
 */

#line 899 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\DLib_Defaults.h"

/*
 * Use the "pointers to short" or "pointers to long" implementation of 
 * the basic floating point routines (like Dnorm, Dtest, Dscale, and Dunscale).
 */




/*
 * Use 64-bit long long as intermediary type in Dtest, and fabs.
 * Default is to do this if long long is 64-bits.
 */




/*
 * Favor speed versus some size enlargements in floating point functions.
 */




/*
 * Include dlmalloc as an alternative heap manager in product.
 *
 * Typically, an application will use a "malloc" heap manager that is
 * relatively small but not that efficient. An application can
 * optionally use the "dlmalloc" package, which provides a more
 * effective "malloc" heap manager, if it is included in the product
 * and supported by the settings.
 *
 * See the product documentation on how to use it, and whether or not
 * it is included in the product.
 */

  /* size_t/ptrdiff_t must be a 4 bytes unsigned integer. */
#line 943 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\DLib_Defaults.h"





/*
 * Allow the 64-bit time_t interface?
 *
 * Default is yes if long long is 64 bits.
 */

  #pragma language = save 
  #pragma language = extended





  #pragma language = restore






/*
 * Is time_t 64 or 32 bits?
 *
 * Default is 32 bits.
 */




/*
 * Do we include math functions that demands lots of constant bytes?
 * (like erf, erfc, expm1, fma, lgamma, tgamma, and *_accurate)
 *
 */




/*
 * Set this to __weak, if supported.
 *
 */
#line 997 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\DLib_Defaults.h"


/*
 * Deleted options
 *
 */







#line 73 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\yvals.h"











                /* Floating-point */

/*
 * Whenever a floating-point type is equal to another, we try to fold those
 * two types into one. This means that if float == double then we fold float to
 * use double internally. Example sinf(float) will use _Sin(double, uint).
 *
 * _X_FNAME is a redirector for internal support routines. The X can be
 *          D (double), F (float), or L (long double). It redirects by using
 *          another prefix. Example calls to Dtest will be __iar_Dtest,
 *          __iar_FDtest, or __iarLDtest.
 * _X_FUN   is a redirector for functions visible to the customer. As above, the
 *          X can be D, F, or L. It redirects by using another suffix. Example
 *          calls to sin will be sin, sinf, or sinl.
 * _X_TYPE  The type that one type is folded to.
 * _X_PTRCAST is a redirector for a cast operation involving a pointer.
 * _X_CAST  is a redirector for a cast involving the float type.
 *
 * _FLOAT_IS_DOUBLE signals that all internal float routines aren't needed.
 * _LONG_DOUBLE_IS_DOUBLE signals that all internal long double routines
 *                        aren't needed.
 */
#line 147 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\yvals.h"





                /* NAMING PROPERTIES */


/* Has support for fixed point types */




/* Has support for secure functions (printf_s, scanf_s, etc) */
/* Will not compile if enabled */
#line 170 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\yvals.h"

/* Has support for complex C types */




/* If is Embedded C++ language */






/* If is true C++ language */






/* True C++ language setup */
#line 233 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\yvals.h"











                /* NAMESPACE CONTROL */
#line 292 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\yvals.h"









#line 308 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\yvals.h"








#line 1 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\xencoding_limits.h"
/* xencoding_limits.h internal header file */
/* Copyright 2003-2010 IAR Systems AB.  */





  #pragma system_include


#line 1 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\ycheck.h"
/* ycheck.h internal checking header file. */
/* Copyright 2005-2010 IAR Systems AB. */

/* Note that there is no include guard for this header. This is intentional. */


  #pragma system_include


/* __INTRINSIC
 *
 * Note: Redefined each time ycheck.h is included, i.e. for each
 * system header, to ensure that intrinsic support could be turned off
 * individually for each file.
 */










/* __AEABI_PORTABILITY_INTERNAL_LEVEL
 *
 * Note: Redefined each time ycheck.h is included, i.e. for each
 * system header, to ensure that ABI support could be turned off/on
 * individually for each file.
 *
 * Possible values for this preprocessor symbol:
 *
 * 0 - ABI portability mode is disabled.
 *
 * 1 - ABI portability mode (version 1) is enabled.
 *
 * All other values are reserved for future use.
 */






#line 67 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\ycheck.h"

#line 12 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\xencoding_limits.h"
#line 1 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\yvals.h"
/* yvals.h internal configuration header file. */
/* Copyright 2001-2010 IAR Systems AB. */

#line 711 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\yvals.h"

/*
 * Copyright (c) 1992-2009 by P.J. Plauger.  ALL RIGHTS RESERVED.
 * Consult your license regarding permissions and restrictions.
V5.04:0576 */
#line 13 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\xencoding_limits.h"

                                /* Multibyte encoding length. */


#line 24 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\xencoding_limits.h"




#line 42 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\xencoding_limits.h"

                                /* Utility macro */














#line 317 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\yvals.h"



                /* FLOATING-POINT PROPERTIES */

                /* float properties */
#line 335 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\yvals.h"

                /* double properties */
#line 360 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\yvals.h"

                /* long double properties */
                /* (must be same as double) */




#line 382 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\yvals.h"


                /* INTEGER PROPERTIES */

                                /* MB_LEN_MAX */







  #pragma language=save
  #pragma language=extended
  typedef long long _Longlong;
  typedef unsigned long long _ULonglong;
  #pragma language=restore
#line 405 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\yvals.h"






  typedef unsigned short int _Wchart;
  typedef unsigned short int _Wintt;


#line 424 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\yvals.h"

#line 432 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\yvals.h"

                /* POINTER PROPERTIES */


typedef signed int  _Ptrdifft;
typedef unsigned int     _Sizet;

/* IAR doesn't support restrict  */


                /* stdarg PROPERTIES */






/* This struct definition must not be inside namespace std, or
   overloading will be wrong in full C++ */
  typedef struct __va_list
  {
    char  *_Ap;
  } __va_list;

  typedef __va_list __Va_list;





__intrinsic __nounwind void __iar_Atexit(void (*)(void));


#line 475 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\yvals.h"
  typedef struct
  {       /* state of a multibyte translation */
    unsigned long _Wchar;      /* Used as an intermediary value (up to 32-bits) */
    unsigned long _State;      /* Used as a state value (only some bits used) */
  } _Mbstatet;











typedef struct
{       /* file position */



  long _Off;    /* can be system dependent */

  _Mbstatet _Wstate;
} _Fpost;







                /* THREAD AND LOCALE CONTROL */

#line 1 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\DLib_Threads.h"
/***************************************************
 *
 * DLib_Threads.h is the library threads manager.
 *
 * Copyright 2004-2010 IAR Systems AB.  
 *
 * This configuration header file sets up how the thread support in the library
 * should work.
 *
 ***************************************************
 *
 * DO NOT MODIFY THIS FILE!
 *
 ***************************************************/





  #pragma system_include


/*
 * DLib can support a multithreaded environment. The preprocessor symbol 
 * _DLIB_THREAD_SUPPORT governs the support. It can be 0 (no support), 
 * 1 (currently not supported), 2 (locks only), and 3 (simulated TLS and locks).
 */

/*
 * This header sets the following symbols that governs the rest of the
 * library:
 * ------------------------------------------
 * _DLIB_MULTI_THREAD     0 No thread support
 *                        1 Multithread support
 * _DLIB_GLOBAL_VARIABLES 0 Use external TLS interface for the libraries global
 *                          and static variables
 *                        1 Use a lock for accesses to the locale and no 
 *                          security for accesses to other global and static
 *                          variables in the library
 * _DLIB_FILE_OP_LOCKS    0 No file-atomic locks
 *                        1 File-atomic locks

 * _DLIB_COMPILER_TLS     0 No Thread-Local-Storage support in the compiler
 *                        1 Thread-Local-Storage support in the compiler
 * _DLIB_TLS_QUAL         The TLS qualifier, define only if _COMPILER_TLS == 1
 *
 * _DLIB_THREAD_MACRO_SETUP_DONE Whether to use the standard definitions of
 *                               TLS macros defined in xtls.h or the definitions
 *                               are provided here.
 *                        0 Use default macros
 *                        1 Macros defined for xtls.h
 *
 * _DLIB_THREAD_LOCK_ONCE_TYPE
 *                        type for control variable in once-initialization of 
 *                        locks
 * _DLIB_THREAD_LOCK_ONCE_MACRO(control_variable, init_function)
 *                        expression that will be evaluated at each lock access
 *                        to determine if an initialization must be done
 * _DLIB_THREAD_LOCK_ONCE_TYPE_INIT
 *                        initial value for the control variable
 *
 ****************************************************************************
 * Description
 * -----------
 *
 * If locks are to be used (_DLIB_MULTI_THREAD != 0), the following options
 * has to be used in ilink: 
 *   --redirect __iar_Locksyslock=__iar_Locksyslock_mtx
 *   --redirect __iar_Unlocksyslock=__iar_Unlocksyslock_mtx
 *   --redirect __iar_Lockfilelock=__iar_Lockfilelock_mtx
 *   --redirect __iar_Unlockfilelock=__iar_Unlockfilelock_mtx
 *   --keep     __iar_Locksyslock_mtx
 * and, if C++ is used, also:
 *   --redirect __iar_Initdynamicfilelock=__iar_Initdynamicfilelock_mtx
 *   --redirect __iar_Dstdynamicfilelock=__iar_Dstdynamicfilelock_mtx
 *   --redirect __iar_Lockdynamicfilelock=__iar_Lockdynamicfilelock_mtx
 *   --redirect __iar_Unlockdynamicfilelock=__iar_Unlockdynamicfilelock_mtx
 * Xlink uses similar options (-e and -g). The following lock interface must
 * also be implemented: 
 *   typedef void *__iar_Rmtx;                   // Lock info object
 *
 *   void __iar_system_Mtxinit(__iar_Rmtx *);    // Initialize a system lock
 *   void __iar_system_Mtxdst(__iar_Rmtx *);     // Destroy a system lock
 *   void __iar_system_Mtxlock(__iar_Rmtx *);    // Lock a system lock
 *   void __iar_system_Mtxunlock(__iar_Rmtx *);  // Unlock a system lock
 * The interface handles locks for the heap, the locale, the file system
 * structure, the initialization of statics in functions, etc. 
 *
 * The following lock interface is optional to be implemented:
 *   void __iar_file_Mtxinit(__iar_Rmtx *);    // Initialize a file lock
 *   void __iar_file_Mtxdst(__iar_Rmtx *);     // Destroy a file lock
 *   void __iar_file_Mtxlock(__iar_Rmtx *);    // Lock a file lock
 *   void __iar_file_Mtxunlock(__iar_Rmtx *);  // Unlock a file lock
 * The interface handles locks for each file stream.
 * 
 * These three once-initialization symbols must also be defined, if the 
 * default initialization later on in this file doesn't work (done in 
 * DLib_product.h):
 *
 *   _DLIB_THREAD_LOCK_ONCE_TYPE
 *   _DLIB_THREAD_LOCK_ONCE_MACRO(control_variable, init_function)
 *   _DLIB_THREAD_LOCK_ONCE_TYPE_INIT
 *
 * If an external TLS interface is used, the following must
 * be defined:
 *   typedef int __iar_Tlskey_t;
 *   typedef void (*__iar_Tlsdtor_t)(void *);
 *   int __iar_Tlsalloc(__iar_Tlskey_t *, __iar_Tlsdtor_t); 
 *                                                    // Allocate a TLS element
 *   int __iar_Tlsfree(__iar_Tlskey_t);               // Free a TLS element
 *   int __iar_Tlsset(__iar_Tlskey_t, void *);        // Set a TLS element
 *   void *__iar_Tlsget(__iar_Tlskey_t);              // Get a TLS element
 *
 */

/* We don't have a compiler that supports tls declarations */





  /* No support for threading. */





#line 296 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\DLib_Threads.h"

  
  typedef void *__iar_Rmtx;
  
#line 326 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\DLib_Threads.h"



#line 348 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\DLib_Threads.h"












#line 510 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\yvals.h"

#line 520 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\yvals.h"

                /* THREAD-LOCAL STORAGE */
#line 528 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\yvals.h"


                /* MULTITHREAD PROPERTIES */
#line 568 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\yvals.h"

                /* LOCK MACROS */
#line 576 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\yvals.h"

#line 694 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\yvals.h"

                /* MISCELLANEOUS MACROS AND FUNCTIONS*/





#line 709 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\yvals.h"



/*
 * Copyright (c) 1992-2009 by P.J. Plauger.  ALL RIGHTS RESERVED.
 * Consult your license regarding permissions and restrictions.
V5.04:0576 */
#line 12 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\stdio.h"
#line 1 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\ysizet.h"
/* ysizet.h internal header file. */
/* Copyright 2003-2010 IAR Systems AB.  */





  #pragma system_include


#line 1 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\ycheck.h"
/* ycheck.h internal checking header file. */
/* Copyright 2005-2010 IAR Systems AB. */

/* Note that there is no include guard for this header. This is intentional. */


  #pragma system_include


/* __INTRINSIC
 *
 * Note: Redefined each time ycheck.h is included, i.e. for each
 * system header, to ensure that intrinsic support could be turned off
 * individually for each file.
 */










/* __AEABI_PORTABILITY_INTERNAL_LEVEL
 *
 * Note: Redefined each time ycheck.h is included, i.e. for each
 * system header, to ensure that ABI support could be turned off/on
 * individually for each file.
 *
 * Possible values for this preprocessor symbol:
 *
 * 0 - ABI portability mode is disabled.
 *
 * 1 - ABI portability mode (version 1) is enabled.
 *
 * All other values are reserved for future use.
 */






#line 67 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\ycheck.h"

#line 12 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\ysizet.h"



                /* type definitions */




typedef _Sizet size_t;




typedef unsigned int __data16_size_t;











#line 13 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\stdio.h"
#line 1 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\ystdio.h"
/* ystdio.h internal header */
/* Copyright 2009-2010 IAR Systems AB. */




  #pragma system_include







#line 58 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\ystdio.h"
  
/* File system functions that have debug variants. They are agnostic on 
   whether the library is full or normal. */

__intrinsic __nounwind int remove(const char *);
__intrinsic __nounwind int rename(const char *, const char *);











/*
 * Copyright (c) 1992-2009 by P.J. Plauger.  ALL RIGHTS RESERVED.
 * Consult your license regarding permissions and restrictions.
V5.042:0576 */
#line 14 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\stdio.h"



/* Module consistency. */
#pragma rtmodel="__dlib_file_descriptor","0"

                /* macros */








#line 66 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\stdio.h"

#line 88 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\stdio.h"

#line 99 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\stdio.h"










                /* type definitions */
typedef _Fpost fpos_t;

                /* printf and scanf pragma support */
#pragma language=save
#pragma language=extended

#line 125 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\stdio.h"

#line 177 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\stdio.h"


             /* Corresponds to fgets(char *, int, stdin); */
_Pragma("function_effects = no_read(1), always_returns")    __intrinsic __nounwind char * __gets(char *, int);
_Pragma("function_effects = no_read(1), always_returns")    __intrinsic __nounwind char * gets(char *);
_Pragma("function_effects = no_write(1), always_returns")    __intrinsic __nounwind void perror(const char *);
_Pragma("function_effects = no_write(1), always_returns")    _Pragma("__printf_args") _Pragma("library_default_requirements _Printf = unknown") __intrinsic __nounwind int printf(const char *, ...);
_Pragma("function_effects = no_write(1), always_returns")    __intrinsic __nounwind int puts(const char *);
_Pragma("function_effects = no_write(1), always_returns")    _Pragma("__scanf_args") _Pragma("library_default_requirements _Scanf = unknown")  __intrinsic __nounwind int scanf(const char *, ...);
_Pragma("function_effects = no_read(1), no_write(2), always_returns") _Pragma("__printf_args") _Pragma("library_default_requirements _Printf = unknown") __intrinsic __nounwind int sprintf(char *, 
                                                 const char *, ...);
_Pragma("function_effects = no_write(1,2), always_returns") _Pragma("__scanf_args") _Pragma("library_default_requirements _Scanf = unknown")  __intrinsic __nounwind int sscanf(const char *, 
                                                const char *, ...);
             __intrinsic __nounwind char * tmpnam(char *);
             /* Corresponds to "ungetc(c, stdout)" */
             __intrinsic __nounwind int __ungetchar(int);
_Pragma("function_effects = no_write(1), always_returns")    _Pragma("__printf_args") _Pragma("library_default_requirements _Printf = unknown") __intrinsic __nounwind int vprintf(const char *,
                                                 __Va_list);

  _Pragma("function_effects = no_write(1), always_returns")    _Pragma("__scanf_args") _Pragma("library_default_requirements _Scanf = unknown")  __intrinsic __nounwind int vscanf(const char *, 
                                                  __Va_list);
  _Pragma("function_effects = no_write(1,2), always_returns") _Pragma("__scanf_args") _Pragma("library_default_requirements _Scanf = unknown")  __intrinsic __nounwind int vsscanf(const char *, 
                                                   const char *, 
                                                   __Va_list);

_Pragma("function_effects = no_read(1), no_write(2), always_returns")  _Pragma("__printf_args") _Pragma("library_default_requirements _Printf = unknown") __intrinsic __nounwind int vsprintf(char *, 
                                                   const char *,
                                                   __Va_list);
              /* Corresponds to fwrite(p, x, y, stdout); */
_Pragma("function_effects = no_write(1), always_returns")      __intrinsic __nounwind size_t __write_array(const void *, size_t, size_t);

  _Pragma("function_effects = no_read(1), no_write(3), always_returns") _Pragma("__printf_args") _Pragma("library_default_requirements _Printf = unknown") __intrinsic __nounwind int snprintf(char *, size_t, 
                                                    const char *, ...);
  _Pragma("function_effects = no_read(1), no_write(3), always_returns") _Pragma("__printf_args") _Pragma("library_default_requirements _Printf = unknown") __intrinsic __nounwind int vsnprintf(char *, size_t,
                                                     const char *, 
                                                     __Va_list);


              __intrinsic __nounwind int getchar(void);
              __intrinsic __nounwind int putchar(int);



#pragma language=restore

#line 238 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\stdio.h"




#line 292 "C:\\Program Files\\IAR Systems\\Embedded Workbench 9.1\\430\\inc\\dlib\\c\\stdio.h"

/*
 * Copyright (c) 1992-2002 by P.J. Plauger.  ALL RIGHTS RESERVED.
 * Consult your license regarding permissions and restrictions.
V5.04:0576 */
#line 21 "C:\\Siddharth\\Personal\\Embedded C\\IAR_programs\\SpeedometerDesign\\hal.h"





void hal_setWheelDia(float wd);
void hal_ledInit();
void hal_hallInit();
void resetPulseCount();

UINT16 getPPS();
UINT16 getRPM();
FLOAT32 getSpeed_mps();
void hal_printRPM();
void hal_printPPS();
void hal_printSpeed_mps();

void hal_blinkLED();
void incPulseCount();
void hal_process_HallSensor();
typedef void(* p_fHallSensorCallback_t)();
void hal_setCB_HallSensor(p_fHallSensorCallback_t p_fHallSensorCallback_t_in);
void hal_isr_hallsensorCB();



#line 19 "C:\\Siddharth\\Personal\\Embedded C\\IAR_programs\\SpeedometerDesign\\drv.c"

/*******************************************

  Name: drv_ledInit
  Parameters: None
  Return: None
  Description: Initializes the LED pin as an output and turns off the LED.*/

void drv_ledInit() 
{
  // Configure LED pin as output
  P1DIR |= (0x0001);
  P1OUT &= ~(0x0001);
}

/*******************************************

  Name: drv_hallInit
  Parameters: None
  Return: None
  Description: Initializes the Hall effect sensor pin as an input with 
  interrupt on rising edge.*/

void drv_hallInit()
{
  // Configure Hall effect sensor pin as input with interrupt
  P1DIR &= ~(0x0008);
  // P1IES |= HALL_SENSOR_PIN; // Interrupt on falling edge
  P1IES &= ~(0x0008); //Interrupt on rising edge 
  P1IE |= (0x0008);  // Enable interrupt for the pin
}

/*******************************************

  Name: drv_blinkLED
  Parameters: None
  Return: None
  Description: Toggles the state of the LED, creating a blinking effect.*/

void drv_blinkLED()
{
  P1OUT ^= (0x0001); // Toggle the LED
  for(int i=0;i<=200;i++);
  P1OUT ^= (0x0001); // Toggle the LED
}

/*******************************************

  Name: Port_1_ISR
  Parameters: None
  Return: None
  Description: Interrupt service routine for Port 1. Handles the interrupt
  from the Hall effect sensor pin, clears the interrupt flag, 
  and calls the hal_isr_hallsensorCB() function. */

// Port 1 interrupt service routine
#pragma vector = (2u * 2u)
__interrupt void Port_1_ISR(void)
{
  if (P1IFG & (0x0008)) // Check if the interrupt is from the Hall effect sensor pin
  {
    P1IFG &= ~(0x0008); // Clear the interrupt flag
    
    hal_isr_hallsensorCB();
  }
}

