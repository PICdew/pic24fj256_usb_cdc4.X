/**
  System Interrupts Generated Driver File 

  @Company:
    Microchip Technology Inc.

  @File Name:
    pin_manager.c

  @Summary:
    This is the generated manager file for the MPLAB(c) Code Configurator device.  This manager
    configures the pins direction, initial state, analog setting.
    The peripheral pin select, PPS, configuration is also handled by this manager.

  @Description:
    This source file provides implementations for MPLAB(c) Code Configurator interrupts.
    Generation Information : 
        Product Revision  :  MPLAB(c) Code Configurator - 4.15.1
        Device            :  PIC24FJ256DA206
    The generated drivers are tested against the following:
        Compiler          :  XC16 1.30
        MPLAB             :  MPLAB X 3.45

    Copyright (c) 2013 - 2015 released Microchip Technology Inc.  All rights reserved.

    Microchip licenses to you the right to use, modify, copy and distribute
    Software only when embedded on a Microchip microcontroller or digital signal
    controller that is integrated into your product or third party product
    (pursuant to the sublicense terms in the accompanying license agreement).

    You should refer to the license agreement accompanying this Software for
    additional information regarding your rights and obligations.

    SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
    EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
    MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
    IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
    CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
    OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
    INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
    CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
    SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
    (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

*/


/**
    Section: Includes
*/
#include <xc.h>
#include "pin_manager.h"

/**
    void PIN_MANAGER_Initialize(void)
*/
void PIN_MANAGER_Initialize(void)
{
    /****************************************************************************
     * Setting the Output Latch SFR(s)
     ***************************************************************************/
    LATB = 0x0000;
    LATC = 0x0000;
    LATD = 0x022A;
    LATE = 0x0000;
    LATF = 0x0000;
    LATG = 0x0000;

    /****************************************************************************
     * Setting the GPIO Direction SFR(s)
     ***************************************************************************/
    TRISB = 0xFD7F;
    TRISC = 0xF000;
    TRISD = 0x0DEB;
    TRISE = 0x00FF;
    TRISF = 0x009B;
    TRISG = 0x014C;

    /****************************************************************************
     * Setting the Weak Pull Up and Weak Pull Down SFR(s)
     ***************************************************************************/
    CNPD1 = 0x0000;
    CNPD2 = 0x0000;
    CNPD3 = 0x0000;
    CNPD4 = 0x0000;
    CNPD5 = 0x0000;
    CNPD6 = 0x0000;
    CNPU1 = 0x0000;
    CNPU2 = 0x0000;
    CNPU3 = 0x0000;
    CNPU4 = 0x0000;
    CNPU5 = 0x0000;
    CNPU6 = 0x0000;

    /****************************************************************************
     * Setting the Open Drain SFR(s)
     ***************************************************************************/
    ODCB = 0x0000;
    ODCC = 0x0000;
    ODCD = 0x0000;
    ODCE = 0x0000;
    ODCF = 0x0000;
    ODCG = 0x0000;

    /****************************************************************************
     * Setting the Analog/Digital Configuration SFR(s)
     ***************************************************************************/
    ANSB = 0xFEBC;
    ANSC = 0x6000;
    ANSD = 0x00C0;
    ANSF = 0x0001;
    ANSG = 0x0280;

    /****************************************************************************
     * Set the PPS
     ***************************************************************************/
    __builtin_write_OSCCONL(OSCCON & 0xbf); // unlock PPS

    RPINR27bits.U4RXR = 0x0008;   //RB8->UART4:U4RX;
    RPOR12bits.RP25R = 0x0005;   //RD4->UART2:U2TX;
    RPOR8bits.RP17R = 0x0004;   //RF5->UART1:U1RTS;
    RPOR13bits.RP26R = 0x001D;   //RG7->UART3:U3RTS;
    RPOR13bits.RP27R = 0x001C;   //RG9->UART3:U3TX;
    RPOR2bits.RP4R = 0x0003;   //RD9->UART1:U1TX;
    RPINR27bits.U4CTSR = 0x0006;   //RB6->UART4:U4CTS;
    RPOR11bits.RP23R = 0x0006;   //RD2->UART2:U2RTS;
    RPOR4bits.RP9R = 0x001E;   //RB9->UART4:U4TX;
    RPINR19bits.U2RXR = 0x0016;   //RD3->UART2:U2RX;
    RPINR19bits.U2CTSR = 0x0018;   //RD1->UART2:U2CTS;
    RPINR18bits.U1CTSR = 0x000A;   //RF4->UART1:U1CTS;
    RPOR3bits.RP7R = 0x001F;   //RB7->UART4:U4RTS;
    RPINR18bits.U1RXR = 0x0002;   //RD8->UART1:U1RX;
    RPINR21bits.U3CTSR = 0x0015;   //RG6->UART3:U3CTS;
    RPINR17bits.U3RXR = 0x0013;   //RG8->UART3:U3RX;

    __builtin_write_OSCCONL(OSCCON | 0x40); // lock   PPS

}

