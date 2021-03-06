/**
  System Interrupts Generated Driver File 

  @Company:
    Microchip Technology Inc.

  @File Name:
    interrupt_manager.h

  @Summary:
    This is the generated driver implementation file for setting up the
    interrupts using MPLAB(c) Code Configurator

  @Description:
    This source file provides implementations for MPLAB(c) Code Configurator interrupts.
    Generation Information : 
        Product Revision  :  MPLAB(c) Code Configurator - pic24-dspic-pic32mm : v1.26
        Device            :  PIC24FJ256DA206
    The generated drivers are tested against the following:
        Compiler          :  XC16 1.30
        MPLAB             :  MPLAB X 3.45
*/
/*
    (c) 2016 Microchip Technology Inc. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
*/

/**
    Section: Includes
*/
#include <xc.h>

/**
    void INTERRUPT_Initialize (void)
*/
void INTERRUPT_Initialize (void)
{
    //    USBI: USB1 - USB1 Interrupt
    //    Priority: 1
        IPC21bits.USB1IP = 1;
    //    UERI: U2E - UART2 Error
    //    Priority: 1
        IPC16bits.U2ERIP = 1;
    //    UTXI: U2TX - UART2 Transmitter
    //    Priority: 1
        IPC7bits.U2TXIP = 1;
    //    URXI: U2RX - UART2 Receiver
    //    Priority: 1
        IPC7bits.U2RXIP = 1;
    //    UERI: U3E - UART3 Error
    //    Priority: 1
        IPC20bits.U3ERIP = 1;
    //    UTXI: U3TX - UART3 Transmitter
    //    Priority: 1
        IPC20bits.U3TXIP = 1;
    //    URXI: U3RX - UART3 Receiver
    //    Priority: 1
        IPC20bits.U3RXIP = 1;
    //    UERI: U4E - UART4 Error
    //    Priority: 1
        IPC21bits.U4ERIP = 1;
    //    UTXI: U4TX - UART4 Transmitter
    //    Priority: 1
        IPC22bits.U4TXIP = 1;
    //    URXI: U4RX - UART4 Receiver
    //    Priority: 1
        IPC22bits.U4RXIP = 1;
    //    UERI: U1E - UART1 Error
    //    Priority: 1
        IPC16bits.U1ERIP = 1;
    //    UTXI: U1TX - UART1 Transmitter
    //    Priority: 1
        IPC3bits.U1TXIP = 1;
    //    URXI: U1RX - UART1 Receiver
    //    Priority: 1
        IPC2bits.U1RXIP = 1;
}
