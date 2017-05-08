/**
  UART Generated Driver File 

  @Company
    Microchip Technology Inc.

  @File Name
    uart1.c

  @Summary
    This is the generated source file for the UART driver using MPLAB(c) Code Configurator

  @Description
    This source file provides APIs for driver for UART. 
    Generation Information : 
        Product Revision  :  MPLAB(c) Code Configurator - pic24-dspic-pic32mm : v1.26
        Device            :  PIC24FJ256DA206
    The generated drivers are tested against the following:
        Compiler          :  XC16 1.30
        MPLAB 	          :  MPLAB X 3.45
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
  Section: Included Files
*/

#include "uart.h"
#include "../mcc_generated_files/usb/usb_device_config.h"
#include "../mcc_generated_files/usb/usb_device_cdc.h"

/**
  Section: Data Type Definitions
*/

/** UART Driver Queue Status

  @Summary
    Defines the object required for the status of the queue.
*/

typedef union {
    struct {
            uint8_t full:1;
            uint8_t empty:1;
            uint8_t reserved:6;
    }s;
    uint8_t status;
} uartByteQStatus_t;

/** UART Driver Hardware Instance Object

  @Summary
    Defines the object required for the maintenance of the hardware instance.

*/
typedef struct uartBuffer_s {
    /* RX Byte Q */
    uint8_t *rxTail ;
    uint8_t *rxHead ;

    /* TX Byte Q */
    uint8_t *txTail ;
    uint8_t *txHead ;

    uartByteQStatus_t rxStatus ;
    uartByteQStatus_t txStatus ;
} uartBuffer_t;

struct uart_s {
    /* ID/index associated to this UART object */
    uint8_t id;
    /* Tx and Rx buffer for UART transmission using interrupts */
    uartBuffer_t buffer;
    /* Associated USB CDC line coding parameters */
    LINE_CODING lineCoding;
};

static uart_t uart_obj[CDC_MAX_COUNT] ;

/** UART Driver Queue Length

  @Summary
    Defines the length of the Transmit and Receive Buffers

*/

/* OUT_EP is from USB host to USB Device */
#define UART_CONFIG_TX_BYTEQ_LENGTH     (128)
/* IN_EP is from USB device to USB host */
#define UART_CONFIG_RX_BYTEQ_LENGTH     (128)


/** UART Driver Queue

  @Summary
    Defines the Transmit and Receive Buffers

*/

static uint8_t uart_txByteQ[CDC_MAX_COUNT][UART_CONFIG_TX_BYTEQ_LENGTH] ;
static uint8_t uart_rxByteQ[CDC_MAX_COUNT][UART_CONFIG_RX_BYTEQ_LENGTH] ;


/**
  Section: Driver Interface
*/
uint8_t UART_GetId(uart_t *obj) {
    return obj->id;
}


void UART_Initialize(uart_t* obj) {
    obj->lineCoding.dwDTERate = UART_DEFAULT_BAUDRATE;
    obj->lineCoding.bCharFormat = UART_DEFAULT_CHAR_FORMAT;
    obj->lineCoding.bDataBits = UART_DEFAULT_DATA_BITS;
    obj->lineCoding.bParityType = UART_DEFAULT_PARITY;
        
    UART_SetLineCoding(obj);
    
    obj->buffer.txHead = uart_txByteQ[obj->id];
    obj->buffer.txTail = uart_txByteQ[obj->id];
    obj->buffer.rxHead = uart_rxByteQ[obj->id];
    obj->buffer.rxTail = uart_rxByteQ[obj->id];
    obj->buffer.rxStatus.s.empty = true;
    obj->buffer.txStatus.s.empty = true;
    obj->buffer.txStatus.s.full = false;
    obj->buffer.rxStatus.s.full = false;
}


/**
  Section: UART Driver Client Routines
*/

uint8_t UART_Read(uart_t *obj) {
    uint8_t data = 0;

    data = *obj->buffer.rxHead;
    obj->buffer.rxHead++;

    if (obj->buffer.rxHead == (uart_rxByteQ[obj->id] + UART_CONFIG_RX_BYTEQ_LENGTH)) {
        obj->buffer.rxHead = uart_rxByteQ[obj->id];
    }

    if (obj->buffer.rxHead == obj->buffer.rxTail) {
        obj->buffer.rxStatus.s.empty = true;
    }

    obj->buffer.rxStatus.s.full = false;

    return data;
}


unsigned int UART_ReadBuffer(uart_t *obj, uint8_t *buffer, const unsigned int bufLen) {
    unsigned int numBytesRead = 0 ;
    while ( numBytesRead < ( bufLen )) {
        if( obj->buffer.rxStatus.s.empty) {
            break;
        } else {
            buffer[numBytesRead++] = UART_Read(obj) ;
        }
    }
    return numBytesRead ;
}

void UART_Write(uart_t *obj, const uint8_t byte) {
    switch(obj->id) {
        case 1:
            IEC1bits.U2TXIE = false;
            break;
        case 2:
            IEC5bits.U3TXIE = false;
            break;
        case 3:
            IEC5bits.U4TXIE = false;
            break;
        default:
            IEC0bits.U1TXIE = false;
            break;
    }
    
    *obj->buffer.txTail = byte;
    obj->buffer.txTail++;
    
    if (obj->buffer.txTail == (uart_txByteQ[obj->id] + UART_CONFIG_TX_BYTEQ_LENGTH)) {
        obj->buffer.txTail = uart_txByteQ[obj->id];
    }

    obj->buffer.txStatus.s.empty = false;

    if (obj->buffer.txHead == obj->buffer.txTail) {
        obj->buffer.txStatus.s.full = true;
    }

    switch(obj->id) {
        case 1:
            IEC1bits.U2TXIE = true;
            break;
        case 2:
            IEC5bits.U3TXIE = true;
            break;
        case 3:
            IEC5bits.U4TXIE = true;
            break;
        default:
            IEC0bits.U1TXIE = true;
            break;
    }
}

unsigned int UART_WriteBuffer(uart_t *obj, const uint8_t *buffer , const unsigned int bufLen ) {
    unsigned int numBytesWritten = 0 ;

    while ( numBytesWritten < ( bufLen )) {
        if((obj->buffer.txStatus.s.full)) {
            break;
        } else {
            UART_Write(obj, buffer[numBytesWritten++] ) ;
        }
    }

    return numBytesWritten ;

}

uartTransferStatus_t UART_TransferStatusGet (uart_t *obj) {
    uartTransferStatus_t status = 0;

    if(obj->buffer.txStatus.s.full) {
        status |= UART_TRANSFER_STATUS_TX_FULL;
    }

    if(obj->buffer.txStatus.s.empty) {
        status |= UART_TRANSFER_STATUS_TX_EMPTY;
    }

    if(obj->buffer.rxStatus.s.full) {
        status |= UART_TRANSFER_STATUS_RX_FULL;
    }

    if(obj->buffer.rxStatus.s.empty) {
        status |= UART_TRANSFER_STATUS_RX_EMPTY;
    } else {
        status |= UART_TRANSFER_STATUS_RX_DATA_PRESENT;
    }
    return status;
}

uint8_t UART_Peek(uart_t *obj, uint16_t offset) {
    if( (obj->buffer.rxHead + offset) > (uart_rxByteQ[obj->id] + UART_CONFIG_RX_BYTEQ_LENGTH)) {
      return uart_rxByteQ[obj->id][offset - (uart_rxByteQ[obj->id] + UART_CONFIG_RX_BYTEQ_LENGTH - obj->buffer.rxHead)];
    } else {
        return *(obj->buffer.rxHead + offset);
    }
}

unsigned int UART_ReceiveBufferSizeGet(uart_t *obj) {
    if(!obj->buffer.rxStatus.s.full) {
        if(obj->buffer.rxHead > obj->buffer.rxTail) {
            return(obj->buffer.rxHead - obj->buffer.rxTail);
        } else {
            return(UART_CONFIG_RX_BYTEQ_LENGTH - (obj->buffer.rxTail - obj->buffer.rxHead));
        } 
    }
    return 0;
}

unsigned int UART_TransmitBufferSizeGet(uart_t *obj) {
    if(!obj->buffer.txStatus.s.full) { 
        if(obj->buffer.txHead > obj->buffer.txTail) {
            return(obj->buffer.txHead - obj->buffer.txTail);
        } else {
            return(UART_CONFIG_TX_BYTEQ_LENGTH - (obj->buffer.txTail - obj->buffer.txHead));
        }
    }
    return 0;
}

bool UART_ReceiveBufferIsEmpty (uart_t *obj) {
    return(obj->buffer.rxStatus.s.empty);
}

bool UART_TransmitBufferIsFull(uart_t *obj) {
    return(obj->buffer.txStatus.s.full);
}

uartStatus_t UART_StatusGet (uart_t *obj) {
    switch (obj->id) {
        case 0:
            return U1STA;
        case 1:
            return U2STA;
        case 2:
            return U3STA;
        default:
            return U4STA;
    }
}

void UART_SetLineCoding(uart_t *obj) {
    uint16_t clkDiv, stopBit, parity;
    uint16_t uxVal;
    
    clkDiv = PERIP_CLK_FREQ / (obj->lineCoding.dwDTERate << 2) - 1;
    stopBit = (obj->lineCoding.bCharFormat == NUM_STOP_BITS_2)? 1 : 0;
    switch(obj->lineCoding.bParityType) {
        case PARITY_ODD:
            parity = 0b100;
            break;
        case PARITY_EVEN:
            parity = 0b010;
            break;
        default:
            parity = 0b000;
            break;
    }
    
    uxVal = ((0x8008 | stopBit | parity) & ~(1<<15));  // disabling UARTEN bit
    switch(obj->id) {
        case 1:
            // STSEL 1; IREN disabled; PDSEL 8N; UARTEN enabled; RTSMD disabled; USIDL disabled; WAKE disabled; ABAUD disabled; LPBACK disabled; BRGH enabled; RXINV disabled; UEN TX_RX; 
            U2MODE = uxVal;
            // UTXISEL0 TX_ONE_CHAR; UTXINV disabled; OERR NO_ERROR_cleared; URXISEL RX_ONE_CHAR; UTXBRK COMPLETED; UTXEN disabled; ADDEN disabled; 
            U2STA = 0x0000;
            // BaudRate = 9600; Frequency = 16000000 Hz; BRG 416; 
            U2BRG = clkDiv;

            IEC1bits.U2RXIE = 1;

             //Make sure to set LAT bit corresponding to TxPin as high before UART initialization
            U2MODEbits.UARTEN = 1;  // enabling UART ON bit
            U2STAbits.UTXEN = 1;
            break;
            
        case 2:
            // STSEL 1; IREN disabled; PDSEL 8N; UARTEN enabled; RTSMD disabled; USIDL disabled; WAKE disabled; ABAUD disabled; LPBACK disabled; BRGH enabled; RXINV disabled; UEN TX_RX; 
            U3MODE = uxVal;
            // UTXISEL0 TX_ONE_CHAR; UTXINV disabled; OERR NO_ERROR_cleared; URXISEL RX_ONE_CHAR; UTXBRK COMPLETED; UTXEN disabled; ADDEN disabled; 
            U3STA = 0x0000;
            // BaudRate = 9600; Frequency = 16000000 Hz; BRG 416; 
            U3BRG = clkDiv;

            IEC5bits.U3RXIE = 1;

             //Make sure to set LAT bit corresponding to TxPin as high before UART initialization
            U3MODEbits.UARTEN = 1;  // enabling UART ON bit
            U3STAbits.UTXEN = 1;
            break;
            
        case 3:
            // STSEL 1; IREN disabled; PDSEL 8N; UARTEN enabled; RTSMD disabled; USIDL disabled; WAKE disabled; ABAUD disabled; LPBACK disabled; BRGH enabled; RXINV disabled; UEN TX_RX; 
            U4MODE = uxVal;
            // UTXISEL0 TX_ONE_CHAR; UTXINV disabled; OERR NO_ERROR_cleared; URXISEL RX_ONE_CHAR; UTXBRK COMPLETED; UTXEN disabled; ADDEN disabled; 
            U4STA = 0x0000;
            // BaudRate = 9600; Frequency = 16000000 Hz; BRG 416; 
            U4BRG = clkDiv;

            IEC5bits.U4RXIE = 1;

             //Make sure to set LAT bit corresponding to TxPin as high before UART initialization
            U4MODEbits.UARTEN = 1;  // enabling UART ON bit
            U4STAbits.UTXEN = 1;
            break;
            
        default:
            // STSEL 1; IREN disabled; PDSEL 8N; UARTEN enabled; RTSMD disabled; USIDL disabled; WAKE disabled; ABAUD disabled; LPBACK disabled; BRGH enabled; RXINV disabled; UEN TX_RX; 
            U1MODE = uxVal;
            // UTXISEL0 TX_ONE_CHAR; UTXINV disabled; OERR NO_ERROR_cleared; URXISEL RX_ONE_CHAR; UTXBRK COMPLETED; UTXEN disabled; ADDEN disabled; 
            U1STA = 0x0000;
            // BaudRate = 9600; Frequency = 16000000 Hz; BRG 416; 
            U1BRG = clkDiv;

            IEC0bits.U1RXIE = 1;

             //Make sure to set LAT bit corresponding to TxPin as high before UART initialization
            U1MODEbits.UARTEN = 1;  // enabling UART ON bit
            U1STAbits.UTXEN = 1;
            break;
    }
}


/* === Utility function === */
void* UART_GetLineCodingById(uint8_t id) {
    if (id < CDC_MAX_COUNT) {
        return &(uart_obj[id].lineCoding._byte[0]);
    } else {
        return NULL;
    }
}

uart_t* UART_GetObjById(uint8_t id) {
    if (id < CDC_MAX_COUNT) {
        uart_obj[id].id = id;
        return &(uart_obj[id]);
    } else {
        return NULL;
    }
}

/* === UART ISR === */

/**
    Maintains the driver's transmitter state machine and implements its ISR
*/
void __attribute__ ( ( interrupt, no_auto_psv ) ) _U1TXInterrupt ( void ) {
    if(uart_obj[0].buffer.txStatus.s.empty) {
        IEC0bits.U1TXIE = false;
        return;
    }
    IFS0bits.U1TXIF = false;

    while(!(U1STAbits.UTXBF == 1)) {
        U1TXREG = *uart_obj[0].buffer.txHead;
        uart_obj[0].buffer.txHead++;

        if(uart_obj[0].buffer.txHead == (uart_txByteQ[0] + UART_CONFIG_TX_BYTEQ_LENGTH)) {
            uart_obj[0].buffer.txHead = uart_txByteQ[0];
        }
        uart_obj[0].buffer.txStatus.s.full = false;

        if(uart_obj[0].buffer.txHead == uart_obj[0].buffer.txTail) {
            uart_obj[0].buffer.txStatus.s.empty = true;
            break;
        }
    }
}

void __attribute__ ( ( interrupt, no_auto_psv ) ) _U1RXInterrupt( void ) {
    while((U1STAbits.URXDA == 1)) {
        *uart_obj[0].buffer.rxTail = U1RXREG;
        uart_obj[0].buffer.rxTail++;

        if(uart_obj[0].buffer.rxTail == (uart_rxByteQ[0] + UART_CONFIG_RX_BYTEQ_LENGTH)) {
            uart_obj[0].buffer.rxTail = uart_rxByteQ[0];
        }
        uart_obj[0].buffer.rxStatus.s.empty = false;
        
        if(uart_obj[0].buffer.rxTail == uart_obj[0].buffer.rxHead) {
            //Sets the flag RX full
            uart_obj[0].buffer.rxStatus.s.full = true;
            break;
        }
    }
    IFS0bits.U1RXIF = false;
}

void __attribute__ ( ( interrupt, no_auto_psv ) ) _U1ErrInterrupt ( void ) {
    if ((U1STAbits.OERR == 1)) {
        U1STAbits.OERR = 0;
    }
    IFS4bits.U1ERIF = false;
}

/**
    Maintains the driver's transmitter state machine and implements its ISR
*/
void __attribute__ ( ( interrupt, no_auto_psv ) ) _U2TXInterrupt ( void ) {
    if(uart_obj[1].buffer.txStatus.s.empty) {
        IEC1bits.U2TXIE = false;
        return;
    }
    IFS1bits.U2TXIF = false;

    while(!(U2STAbits.UTXBF == 1)) {
        U2TXREG = *uart_obj[1].buffer.txHead;
        uart_obj[1].buffer.txHead++;

        if(uart_obj[1].buffer.txHead == (uart_txByteQ[1] + UART_CONFIG_TX_BYTEQ_LENGTH)) {
            uart_obj[1].buffer.txHead = uart_txByteQ[1];
        }
        uart_obj[1].buffer.txStatus.s.full = false;

        if(uart_obj[1].buffer.txHead == uart_obj[1].buffer.txTail) {
            uart_obj[1].buffer.txStatus.s.empty = true;
            break;
        }
    }
}

void __attribute__ ( ( interrupt, no_auto_psv ) ) _U2RXInterrupt( void ) {
    while((U2STAbits.URXDA == 1)) {
        *uart_obj[1].buffer.rxTail = U2RXREG;
        uart_obj[1].buffer.rxTail++;

        if(uart_obj[1].buffer.rxTail == (uart_rxByteQ[1] + UART_CONFIG_RX_BYTEQ_LENGTH)) {
            uart_obj[1].buffer.rxTail = uart_rxByteQ[1];
        }
        uart_obj[1].buffer.rxStatus.s.empty = false;
        
        if(uart_obj[1].buffer.rxTail == uart_obj[1].buffer.rxHead) {
            //Sets the flag RX full
            uart_obj[1].buffer.rxStatus.s.full = true;
            break;
        }
    }
    IFS1bits.U2RXIF = false;
}

void __attribute__ ( ( interrupt, no_auto_psv ) ) _U2ErrInterrupt ( void ) {
    if ((U2STAbits.OERR == 1)) {
        U2STAbits.OERR = 0;
    }
    IFS4bits.U2ERIF = false;
}

/**
    Maintains the driver's transmitter state machine and implements its ISR
*/
void __attribute__ ( ( interrupt, no_auto_psv ) ) _U3TXInterrupt ( void ) {
    if(uart_obj[2].buffer.txStatus.s.empty) {
        IEC5bits.U3TXIE = false;
        return;
    }
    IFS5bits.U3TXIF = false;

    while(!(U3STAbits.UTXBF == 1)) {
        U3TXREG = *uart_obj[2].buffer.txHead;
        uart_obj[2].buffer.txHead++;

        if(uart_obj[2].buffer.txHead == (uart_txByteQ[2] + UART_CONFIG_TX_BYTEQ_LENGTH)) {
            uart_obj[2].buffer.txHead = uart_txByteQ[2];
        }
        uart_obj[2].buffer.txStatus.s.full = false;

        if(uart_obj[2].buffer.txHead == uart_obj[2].buffer.txTail) {
            uart_obj[2].buffer.txStatus.s.empty = true;
            break;
        }
    }
}

void __attribute__ ( ( interrupt, no_auto_psv ) ) _U3RXInterrupt( void ) {
    while((U3STAbits.URXDA == 1)) {
        *uart_obj[2].buffer.rxTail = U3RXREG;
        uart_obj[2].buffer.rxTail++;

        if(uart_obj[2].buffer.rxTail == (uart_rxByteQ[2] + UART_CONFIG_RX_BYTEQ_LENGTH)) {
            uart_obj[2].buffer.rxTail = uart_rxByteQ[2];
        }
        uart_obj[2].buffer.rxStatus.s.empty = false;
        
        if(uart_obj[2].buffer.rxTail == uart_obj[2].buffer.rxHead) {
            //Sets the flag RX full
            uart_obj[2].buffer.rxStatus.s.full = true;
            break;
        }   
    }
    IFS5bits.U3RXIF = false;
}

void __attribute__ ( ( interrupt, no_auto_psv ) ) _U3ErrInterrupt ( void ) {
    if ((U3STAbits.OERR == 1)) {
        U3STAbits.OERR = 0;
    }
    IFS5bits.U3ERIF = false;
}

/**
    Maintains the driver's transmitter state machine and implements its ISR
*/
void __attribute__ ( ( interrupt, no_auto_psv ) ) _U4TXInterrupt ( void ) {
    if(uart_obj[3].buffer.txStatus.s.empty) {
        IEC5bits.U4TXIE = false;
        return;
    }
    IFS5bits.U4TXIF = false;

    while(!(U4STAbits.UTXBF == 1)) {
        U4TXREG = *uart_obj[3].buffer.txHead;
        uart_obj[3].buffer.txHead++;

        if(uart_obj[3].buffer.txHead == (uart_txByteQ[3] + UART_CONFIG_TX_BYTEQ_LENGTH)) {
            uart_obj[3].buffer.txHead = uart_txByteQ[3];
        }
        uart_obj[3].buffer.txStatus.s.full = false;
        
        if(uart_obj[3].buffer.txHead == uart_obj[3].buffer.txTail) {
            uart_obj[3].buffer.txStatus.s.empty = true;
            break;
        }
    }
}

void __attribute__ ( ( interrupt, no_auto_psv ) ) _U4RXInterrupt( void ) {
    while((U4STAbits.URXDA == 1)) {
        *uart_obj[3].buffer.rxTail = U4RXREG;
        uart_obj[3].buffer.rxTail++;

        if(uart_obj[3].buffer.rxTail == (uart_rxByteQ[3] + UART_CONFIG_RX_BYTEQ_LENGTH)) {
            uart_obj[3].buffer.rxTail = uart_rxByteQ[3];
        }
        uart_obj[3].buffer.rxStatus.s.empty = false;
        
        if(uart_obj[3].buffer.rxTail == uart_obj[3].buffer.rxHead) {
            //Sets the flag RX full
            uart_obj[3].buffer.rxStatus.s.full = true;
            break;
        }
    }

    IFS5bits.U4RXIF = false;
}

void __attribute__ ( ( interrupt, no_auto_psv ) ) _U4ErrInterrupt ( void ) {
    if ((U4STAbits.OERR == 1)) {
        U4STAbits.OERR = 0;
    }
    IFS5bits.U4ERIF = false;
}
/* === END: UART ISR === */

/**
  End of File
*/

