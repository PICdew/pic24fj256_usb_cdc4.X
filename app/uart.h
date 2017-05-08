/**
  UART Generated Driver API Header File 

  @Company
    Microchip Technology Inc.

  @File Name
    uart.h

  @Summary
    This is the generated header file for the UART driver using MPLAB(c) Code Configurator

  @Description
    This header file provides APIs for driver for UART. 
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

#ifndef _UART_H
#define _UART_H

/**
 Section: Included Files
*/

#include <xc.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
  
        
#define PERIP_CLK_FREQ              (16000000)
#define UART_DEFAULT_BAUDRATE       (115200)
#define UART_DEFAULT_PARITY         PARITY_NONE
#define UART_DEFAULT_CHAR_FORMAT   NUM_STOP_BITS_1
#define UART_DEFAULT_DATA_BITS      (8)


/**
  Section: Data Types
*/
        
/** UART Driver Hardware Flags

  @Summary
    Specifies the status of the hardware receive or transmit

  @Description
    This type specifies the status of the hardware receive or transmit.
    More than one of these values may be OR'd together to create a complete
    status value.  To test a value of this type, the bit of interest must be
    AND'ed with value and checked to see if the result is non-zero.
*/
typedef enum
{
    /* Indicates that Receive buffer has data, at least one more character can be read */
    UART_RX_DATA_AVAILABLE
        /*DOM-IGNORE-BEGIN*/  = (1 << 0) /*DOM-IGNORE-END*/,
    
    /* Indicates that Receive buffer has overflowed */
    UART_RX_OVERRUN_ERROR
        /*DOM-IGNORE-BEGIN*/  = (1 << 1) /*DOM-IGNORE-END*/,

    /* Indicates that Framing error has been detected for the current character */
    UART_FRAMING_ERROR
        /*DOM-IGNORE-BEGIN*/  = (1 << 2) /*DOM-IGNORE-END*/,

    /* Indicates that Parity error has been detected for the current character */
    UART_PARITY_ERROR
        /*DOM-IGNORE-BEGIN*/  = (1 << 3) /*DOM-IGNORE-END*/,

    /* Indicates that Receiver is Idle */
    UART_RECEIVER_IDLE
        /*DOM-IGNORE-BEGIN*/  = (1 << 4) /*DOM-IGNORE-END*/,

    /* Indicates that the last transmission has completed */
    UART_TX_COMPLETE
        /*DOM-IGNORE-BEGIN*/  = (1 << 8) /*DOM-IGNORE-END*/,

    /* Indicates that Transmit buffer is full */
    UART_TX_FULL
        /*DOM-IGNORE-BEGIN*/  = (1 << 9) /*DOM-IGNORE-END*/

} uartStatus_t;



/** UART Driver Transfer Flags

  @Summary
    Specifies the status of the receive or transmit

  @Description
    This type specifies the status of the receive or transmit operation.
    More than one of these values may be OR'd together to create a complete
    status value.  To test a value of this type, the bit of interest must be
    AND'ed with value and checked to see if the result is non-zero.
*/

typedef enum
{
    /* Indicates that the core driver buffer is full */
    UART_TRANSFER_STATUS_RX_FULL
        /*DOM-IGNORE-BEGIN*/  = (1 << 0) /*DOM-IGNORE-END*/,

    /* Indicates that at least one byte of Data has been received */
    UART_TRANSFER_STATUS_RX_DATA_PRESENT
        /*DOM-IGNORE-BEGIN*/  = (1 << 1) /*DOM-IGNORE-END*/,

    /* Indicates that the core driver receiver buffer is empty */
    UART_TRANSFER_STATUS_RX_EMPTY
        /*DOM-IGNORE-BEGIN*/  = (1 << 2) /*DOM-IGNORE-END*/,

    /* Indicates that the core driver transmitter buffer is full */
    UART_TRANSFER_STATUS_TX_FULL
        /*DOM-IGNORE-BEGIN*/  = (1 << 3) /*DOM-IGNORE-END*/,

    /* Indicates that the core driver transmitter buffer is empty */
    UART_TRANSFER_STATUS_TX_EMPTY
        /*DOM-IGNORE-BEGIN*/  = (1 << 4) /*DOM-IGNORE-END*/

} uartTransferStatus_t;


typedef struct uart_s uart_t;


/**
  Section: UART Driver Routines
*/


/**
  @Summary
    Initializes the UART instance : 1

  @Description
    This routine initializes the UART driver instance for : 1
    index.
    This routine must be called before any other UART routine is called.
    
  @Preconditions
    None.

  @Returns
    None.

  @Param
    id: UART number, 0 to 3, inclusive.

  @Comment
    
 
  @Example
    <code>
        const uint8_t writeBuffer[35] = "1234567890ABCDEFGHIJKLMNOP\n" ;
        unsigned int numBytes = 0;
        int  writebufferLen = strlen((char *)writeBuffer);
        UART_Initialize();
        while(numBytes < writebufferLen)
        {    
            int bytesToWrite = UART_TransmitBufferSizeGet();
            numBytes += UART_WriteBuffer ( writeBuffer+numBytes, bytesToWrite)  ;
           
        }
    </code>

*/
void UART_Initialize(uart_t *obj);


/**
  @Summary
    Read a byte of data from the UART

  @Description
    This routine reads a byte of data from the UART.

  @Preconditions
    UART_Initializer function should have been called 
    before calling this function. The transfer status should be checked to see 
    if the receiver is not empty before calling this function.

  @Param
    id: UART number, 0 to 3, inclusive.

  @Returns
    A data byte received by the driver.

  @Example
    <code>
    char            myBuffer[MY_BUFFER_SIZE];
    unsigned int    numBytes;

    numBytes = 0;
    do
    {
        if( UART_TRANSFER_STATUS_RX_DATA_PRESENT & UART_TransferStatusGet() )
        {
            myBuffer[numBytes++] = UART_Read();
        }

        // Do something else...

    } while( numBytes < MY_BUFFER_SIZE);
    </code>
*/
uint8_t UART_Read(uart_t *obj);


/**
  @Summary
    Returns the number of bytes read by the UART peripheral

  @Description
    This routine returns the number of bytes read by the Peripheral and fills the
    application read buffer with the read data.

  @Preconditions
    UART_Initializer function should have been called 
    before calling this function

  @Param
    id           - UART number, 0 to 3, inclusive.
    buffer       - Buffer into which the data read from the UART

  @Param
    numbytes     - Total number of bytes that need to be read from the UART
                   (must be equal to or less than the size of the buffer)

  @Returns
    Number of bytes actually copied into the caller's buffer or -1 if there
    is an error.

  @Example
    <code>
    char                     myBuffer[MY_BUFFER_SIZE];
    unsigned int             numBytes;
    UART_TRANSFER_STATUS status ;

    // Pre-initialize myBuffer with MY_BUFFER_SIZE bytes of valid data.

    numBytes = 0;
    while( numBytes < MY_BUFFER_SIZE);
    {
        status = UART_TransferStatusGet ( ) ;
        if (status & UART_TRANSFER_STATUS_RX_FULL)
        {
            numBytes += UART_ReadBuffer( myBuffer + numBytes, MY_BUFFER_SIZE - numBytes )  ;
            if(numBytes < readbufferLen)
            {
                continue;
            }
            else
            {
                break;
            }
        }
        else
        {
            continue;
        }

        // Do something else...
    }
    </code>
*/
unsigned int UART_ReadBuffer(uart_t *obj, uint8_t *buffer ,  const unsigned int numbytes);


/**
  @Summary
    Writes a byte of data to the UART

  @Description
    This routine writes a byte of data to the UART.

  @Preconditions
    UART_Initializer function should have been called 
    before calling this function. The transfer status should be checked to see if
    transmitter is not full before calling this function.

  @Param
    id           - UART number, 0 to 3, inclusive.
    byte         - Data byte to write to the UART

  @Returns
    None.

  @Example
    <code>
    char            myBuffer[MY_BUFFER_SIZE];
    unsigned int    numBytes;

    // Pre-initialize myBuffer with MY_BUFFER_SIZE bytes of valid data.

    numBytes = 0;
    while( numBytes < MY_BUFFER_SIZE);
    {
        if( !(UART_TRANSFER_STATUS_TX_FULL & UART_TransferStatusGet()) )
        {
            UART_Write(handle, myBuffer[numBytes++]);
        }

        // Do something else...
    }
    </code>
*/
void UART_Write(uart_t *obj, const uint8_t byte);


/**
  @Summary
    Returns the number of bytes written into the internal buffer

  @Description
    This API transfers the data from application buffer to internal buffer and 
    returns the number of bytes added in that queue

  @Preconditions
    UART_Initializer function should have been called 
    before calling this function

  @Example
    <code>
    char                     myBuffer[MY_BUFFER_SIZE];
    unsigned int             numBytes;
    UART_TRANSFER_STATUS status ;

    // Pre-initialize myBuffer with MY_BUFFER_SIZE bytes of valid data.

    numBytes = 0;
    while( numBytes < MY_BUFFER_SIZE);
    {
        status = UART_TransferStatusGet ( ) ;
        if (status & UART_TRANSFER_STATUS_TX_EMPTY)
        {
            numBytes += UART_WriteBuffer ( myBuffer + numBytes, MY_BUFFER_SIZE - numBytes )  ;
            if(numBytes < writebufferLen)
            {
                continue;
            }
            else
            {
                break;
            }
        }
        else
        {
            continue;
        }

        // Do something else...
    }
    </code>
*/
unsigned int UART_WriteBuffer(uart_t *obj, const uint8_t *buffer , const unsigned int numbytes );


/**
  @Summary
    Returns the transmitter and receiver transfer status

  @Description
    This returns the transmitter and receiver transfer status.The returned status 
    may contain a value with more than one of the bits
    specified in the UART_TRANSFER_STATUS enumeration set.  
    The caller should perform an "AND" with the bit of interest and verify if the
    result is non-zero (as shown in the example) to verify the desired status
    bit.

  @Preconditions
    UART_Initializer function should have been called 
    before calling this function

  @Param
    id           - UART number, 0 to 3, inclusive.

  @Returns
    A UART_TRANSFER_STATUS value describing the current status 
    of the transfer.

  @Example
    Refer to UART_ReadBuffer and UART_WriteBuffer for example

*/
uartTransferStatus_t UART_TransferStatusGet (uart_t *obj);


/**
  @Summary
    Returns the character in the read sequence at the offset provided, without
    extracting it

  @Description
    This routine returns the character in the read sequence at the offset provided,
    without extracting it
 
  @Param
    id           - UART number, 0 to 3, inclusive.
    
  @Example 
    <code>
    const uint8_t readBuffer[5];
    unsigned int data, numBytes = 0;
    unsigned int readbufferLen = sizeof(readBuffer);
    UART_Initializer();
    
    while(numBytes < readbufferLen)        
    {   
        UART_TasksReceive ( );
        //Check for data at a particular place in the buffer
        data = UART_Peek(3);
        if(data == 5)
        {
            //discard all other data if byte that is wanted is received.    
            //continue other operation
            numBytes += UART_ReadBuffer ( readBuffer + numBytes , readbufferLen ) ;
        }
        else
        {
            break;
        }
    }
    </code>
 
*/
uint8_t UART_Peek(uart_t *obj, uint16_t offset);


/**
  @Summary
    Returns the size of the receive buffer

  @Description
    This routine returns the size of the receive buffer.

  @Param
    id           - UART number, 0 to 3, inclusive.

  @Returns
    Size of receive buffer.
    
  @Example 
    <code>
    const uint8_t readBuffer[5];
    unsigned int size, numBytes = 0;
    unsigned int readbufferLen = sizeof(readBuffer);
    UART__Initializer();
    
    while(size < readbufferLen)
	{
	    UART_TasksReceive ( );
	    size = UART_ReceiveBufferSizeGet();
	}
    numBytes = UART_ReadBuffer ( readBuffer , readbufferLen ) ;
    </code>
 
*/
unsigned int UART_ReceiveBufferSizeGet(uart_t *obj);


/**
  @Summary
    Returns the size of the transmit buffer

  @Description
    This routine returns the size of the transmit buffer.

 @Param
    id           - UART number, 0 to 3, inclusive.
 
 @Returns
    Size of transmit buffer.

 @Example
    Refer to UART_Initializer(); for example.
*/
unsigned int UART_TransmitBufferSizeGet(uart_t *obj);


/**
  @Summary
    Returns the status of the receive buffer

  @Description
    This routine returns if the receive buffer is empty or not.

  @Param
    id           - UART number, 0 to 3, inclusive.
 
  @Returns
    True if the receive buffer is empty
    False if the receive buffer is not empty
    
  @Example
    <code>
    char                     myBuffer[MY_BUFFER_SIZE];
    unsigned int             numBytes;
    UART_TRANSFER_STATUS status ;

    // Pre-initialize myBuffer with MY_BUFFER_SIZE bytes of valid data.

    numBytes = 0;
    while( numBytes < MY_BUFFER_SIZE);
    {
        status = UART_TransferStatusGet ( ) ;
        if (!UART_ReceiveBufferIsEmpty())
        {
            numBytes += UART_ReadBuffer( myBuffer + numBytes, MY_BUFFER_SIZE - numBytes )  ;
            if(numBytes < readbufferLen)
            {
                continue;
            }
            else
            {
                break;
            }
        }
        else
        {
            continue;
        }

        // Do something else...
    }
    </code>
 
*/
bool UART_ReceiveBufferIsEmpty (uart_t *obj);


/**
  @Summary
    Returns the status of the transmit buffer

  @Description
    This routine returns if the transmit buffer is full or not.

 @Param
    id           - UART number, 0 to 3, inclusive.
 
 @Returns
    True if the transmit buffer is full
    False if the transmit buffer is not full

 @Example
    Refer to UART_Initializer() for example.
 
*/
bool UART_TransmitBufferIsFull (uart_t *obj);


/**
  @Summary
    Returns the transmitter and receiver status

  @Description
    This returns the transmitter and receiver status. The returned status may 
    contain a value with more than one of the bits
    specified in the UART_STATUS enumeration set.  
    The caller should perform an "AND" with the bit of interest and verify if the
    result is non-zero (as shown in the example) to verify the desired status
    bit.

  @Preconditions
    UART_Initializer function should have been called 
    before calling this function

  @Param
    None.

  @Returns
    A UART_STATUS value describing the current status 
    of the transfer.

  @Example
    <code>
        while(!(UART_StatusGet & UART_TX_COMPLETE ))
        {
           // Wait for the tranmission to complete
        }
    </code>
*/
uartStatus_t UART_StatusGet (uart_t *obj);

/**
 * @brief Set/Update UART baud rate, parity type and stop bits from its internal
 *      line coding information obtained from USB CDC SetLineCoding function.
 * @param obj UART object to update.
 */
void UART_SetLineCoding(uart_t *obj);

/**
 * @brief Get the pointer to line coding data associated to an UART object.
 * @param id ID/Index number of UART object whose line coding pointer is required.
 * @return Pointer to line coding associated to an UART object. NULL if no UART 
 *      object can be found that corresponds to id.
 */
void* UART_GetLineCodingById(uint8_t id);

/**
 * @brief Get the UART object associated with id.
 * @param id ID/Index number of an UART object.
 * @return Pointer to the UART object associated with id. NULL is no UART object 
 *      is associated with id.
 */
uart_t* UART_GetObjById(uint8_t id);

/**
 * @brief Get ID of an UART object.
 * @param obj UART object.
 * @return ID/Index number associated with the UART object.
 */
uint8_t UART_GetId(uart_t *obj);

#ifdef __cplusplus  // Provide C++ Compatibility

    }

#endif
    
#endif  // _UART_H

/*
  End of File
*/

