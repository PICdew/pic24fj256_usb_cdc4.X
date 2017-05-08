/*******************************************************************************
Copyright 2016 Microchip Technology Inc. (www.microchip.com)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

To request to license the code under the MLA license (www.microchip.com/mla_license), 
please contact mla_licensing@microchip.com
*******************************************************************************/

/** INCLUDES *******************************************************/
#include <stdint.h>
#include <string.h>
#include <stddef.h>

#include "app_device_cdc_to_uart.h"
#include "uart.h"

/** VARIABLES ******************************************************/

//static bool buttonPressed;
//static char buttonMessage[] = "Button pressed.\r\n";
static uint8_t USB_Out_Buffer[CDC_DATA_OUT_EP_SIZE];
static uint8_t RS232_Out_Data[CDC_DATA_IN_EP_SIZE];


/*********************************************************************
* Function: void APP_DeviceCDCEmulatorTasks(void);
*
* Overview: Keeps the demo running.
*
* PreCondition: The demo should have been initialized and started via
*   the APP_DeviceCDCEmulatorInitialize() and APP_DeviceCDCEmulatorStart() demos
*   respectively.
*
* Input: None
*
* Output: None
*
********************************************************************/
void APP_DeviceCDCEmulatorTasks() {
    int id;
    uint16_t nUsbByte;
    unsigned int nUartByte;
    uart_t *obj;
    
    /* If the USB device isn't configured yet, we can't really do anything
     * else since we don't have a host to talk to.  So jump back to the
     * top of the while loop. */
    if( USBGetDeviceState() < CONFIGURED_STATE ) {
        return;
    }

    /* If we are currently suspended, then we need to see if we need to
     * issue a remote wakeup.  In either case, we shouldn't process any
     * keyboard commands since we aren't currently communicating to the host
     * thus just continue back to the start of the while loop. */
    if( USBIsDeviceSuspended()== true ) {
        return;
    }

    for (id = 0; id < CDC_MAX_COUNT; id++) {
        obj = UART_GetObjById(id);
        /* TODO: Assert obj != NULL */
        
        nUsbByte = getsUSBUSART(id, RS232_Out_Data, CDC_DATA_IN_EP_SIZE);
        
        if (nUsbByte > 0) {
            nUartByte = UART_WriteBuffer(obj, RS232_Out_Data, nUsbByte);
        }
        
        if (cdc_trf_state[id] == CDC_TX_READY) {
            nUartByte = UART_ReadBuffer(obj, USB_Out_Buffer, CDC_DATA_OUT_EP_SIZE);
            #if defined(USB_CDC_SUPPORT_HARDWARE_FLOW_CONTROL)
                //Drive RTS pin, to let UART device attached know if it is allowed to
                //send more data or not.  If the receive buffer is almost full, we
                //deassert RTS.
                if(NextUSBOut <= (CDC_DATA_OUT_EP_SIZE - 5u))
                {
                    UART1_RTS = USB_CDC_RTS_ACTIVE_LEVEL;
                }
                else
                {
                    UART1_RTS = (USB_CDC_RTS_ACTIVE_LEVEL ^ 1);
                }
            #endif
            if (nUartByte > 0) {
                putUSBUSART(id, &USB_Out_Buffer[0], nUartByte);
            }
        }
        CDCTxService(id);
    }
}


void APP_DeviceCDCSetLineCoding(CTRL_TRF_PARAMS context) {
    uart_t *obj = (uart_t*) context;
    
    if (obj != NULL) {
        UART_SetLineCoding(obj);
    }
}
