/*******************************************************************************
  USART User Rover Driver Source File.

  File Name:
    app.c

  Summary:
    USART driver for the User Rover.

  Description:
    This file contains the USART driver needed to make the User Rover function.
    The main purpose of the User USART driver is to recieve single character 
    instructions from the Raspberry Pi.  This driver also contains a state to send
    data so that error messages may be sent to the Pi
 *
 ******************************************************************************/


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include "app.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Variable Definitions
// *****************************************************************************
// *****************************************************************************
/* USART Driver Demo Banner Message */

char preMsg[] = "\r\n User Rover Connected";
char temp[APP_BUFFER_SIZE];

/* User Application Data Structure */
APP_DATA appData;

extern SYSTEM_OBJECTS sysObj;

// *****************************************************************************
// *****************************************************************************
// Section: Application Data Initialization and State Machine
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize( void )

  Remarks:
    Initializes all appData values for use in the Driver
 */

void APP_Initialize( void )
{
    /* Set the App. previous state to its initial state. */
    appData.prevState         = APP_DRV_OPEN;
    /* Set the App. current state to its initial state. */
    appData.currentState      = APP_DRV_OPEN;
    /* Initialise buffer Size */
    appData.bufferSize        = 0;
    /* Demo App. message index */
    appData.usrMsgIndex       = 0;
    /* Set the USART Clent Statud to error */
    appData.usartStatus       = DRV_USART_CLIENT_STATUS_ERROR;
    /* Set the USART buffer handler to invalid */
    appData.usartBufferHandle = DRV_HANDLE_INVALID;
    /* Set the USART handler to invalid */
    appData.usartHandle       = DRV_HANDLE_INVALID;
    /* Set the USART buffer event to invalid */
    appData.usartBufferEvent  = DRV_USART_BUFFER_EVENT_ERROR;
    /* Set the initial state of event flags for driver messages */
    appData.drvBufferEventComplete = false;
    /* Set the initial state of event flags for user messages */
    appData.usrBufferEventComplete = false;
    /* Clear Application Buffer */
    strcpy(appData.buffer, "");
}


void APP_BufferEventHandler(DRV_USART_BUFFER_EVENT buffEvent,
                            DRV_USART_BUFFER_HANDLE hBufferEvent,
                            uintptr_t context)
{
    switch(buffEvent)
    {
        /* Buffer event is completed successfully */
        case DRV_USART_BUFFER_EVENT_COMPLETE:
        {
            if(context == APP_DRV_CONTEXT)
            {
                /* Update buffer event status */
                appData.drvBufferEventComplete = true;
            }
            else if (context == APP_USR_CONTEXT)
            { 
                /* Update buffer event status */
                appData.usrBufferEventComplete = true;   
            }
        }
            break;

        /* Buffer event has some error */
        case DRV_USART_BUFFER_EVENT_ERROR:
            break;

        /* Buffer event has aborted */
        case DRV_USART_BUFFER_EVENT_ABORT:
            break;
    }
}


/*******************************************************************************
  Function:
    void APP_Tasks( void )

  Remarks:
    What runs the driver.  Sends and receives to the UART buffer
 */
void APP_Tasks( void )
{
    /* Check the Application State*/
    switch ( appData.currentState )
    {
        /* Open USART Driver and set the Buffer Event Handling */
        case APP_DRV_OPEN:
        {
            appData.usartHandle = DRV_USART_Open(APP_USART_DRIVER_INDEX,
                         (DRV_IO_INTENT_READWRITE | DRV_IO_INTENT_NONBLOCKING));

            if (appData.usartHandle != DRV_HANDLE_INVALID )
            {
                DRV_USART_BufferEventHandlerSet(appData.usartHandle,
                                       APP_BufferEventHandler, APP_DRV_CONTEXT);
                appData.prevState    = APP_DRV_OPEN;
                appData.currentState = APP_DRV_READY;
            }
            else
            {
                appData.currentState = APP_ERROR;
            }
        }
        break;

        case APP_DRV_READY:
        {
            appData.usartStatus = DRV_USART_ClientStatus( appData.usartHandle );

            if ( appData.usartStatus == DRV_USART_CLIENT_STATUS_READY )
            {
                strcpy(appData.buffer, "\r\n");
                appData.bufferSize = min(APP_BUFFER_SIZE,
                                                strlen(appData.buffer));
                DRV_USART_BufferAddWrite( appData.usartHandle,
                                          &(appData.usartBufferHandle),
                                          appData.buffer, appData.bufferSize );

                if ( appData.usartBufferHandle == DRV_HANDLE_INVALID )
                {
                    appData.currentState = APP_ERROR;
                }
                else
                {
                    appData.prevState    = APP_DRV_READY;
                    appData.currentState = APP_WAIT_FOR_DONE;
                }
            }
            else
            {
                appData.currentState = APP_ERROR;
            }
        }
        break;

        case APP_WAIT_FOR_DONE:
        {
            if(appData.drvBufferEventComplete)
            {
                appData.drvBufferEventComplete = false;
                App_GetNextTaskState(appData.prevState);
            }
            else if(appData.usrBufferEventComplete)
            {
                appData.usrBufferEventComplete = false;
                App_GetNextTaskState(appData.prevState);
            }
        }
        break;

          //writes to terminal that the connection was a success
        case APP_DRV_MSG_WRITE:
        {
            appData.usartStatus = DRV_USART_ClientStatus( appData.usartHandle );

            if ( appData.usartStatus == DRV_USART_CLIENT_STATUS_READY )
            {
                strcpy( appData.buffer, preMsg );
                appData.bufferSize = min(APP_BUFFER_SIZE,
                                                       strlen(appData.buffer));
                DRV_USART_BufferAddWrite( appData.usartHandle,
                                          &(appData.usartBufferHandle),
                                          appData.buffer, appData.bufferSize);

                if ( appData.usartBufferHandle == DRV_HANDLE_INVALID )
                {
                    appData.currentState = APP_ERROR;
                }
                else
                {
                    appData.prevState    = APP_DRV_MSG_WRITE;
                    appData.currentState = APP_WAIT_FOR_DONE;
                }
            }
        }
        break;

            //reads from terminal
            //eventually will read from serial port on PI
        case APP_USR_MSG_READ:
        {
            appData.usartStatus = DRV_USART_ClientStatus( appData.usartHandle );
            if ( appData.usartStatus == DRV_USART_CLIENT_STATUS_READY )
            {
                strcpy( appData.buffer, " " );
                appData.bufferSize = min(APP_BUFFER_SIZE,
                                                       strlen(appData.buffer));
                DRV_USART_BufferAddRead( appData.usartHandle,
                                         &(appData.usartBufferHandle),
                                         appData.buffer, appData.bufferSize);

                if ( appData.usartBufferHandle == DRV_HANDLE_INVALID )
                {
                    appData.currentState = APP_ERROR;
                }
                else
                {
                    appData.prevState    = APP_USR_MSG_READ;
                    appData.currentState = APP_WAIT_FOR_DONE;
                }
            }
        }
        break;

        case APP_USR_MSG_WRITE:
        {
            appData.usartStatus = DRV_USART_ClientStatus( appData.usartHandle );

            if ( appData.usartStatus == DRV_USART_CLIENT_STATUS_READY )
            {
                strcpy(temp, "\r\nSuccessfully received: ");
                strcat( temp, appData.buffer );
                strcpy(appData.buffer, temp);
                
                appData.bufferSize = min(APP_BUFFER_SIZE,
                                                       strlen(appData.buffer));
                DRV_USART_BufferAddWrite( appData.usartHandle,
                                          &(appData.usartBufferHandle),
                                          appData.buffer, appData.bufferSize);

                if ( appData.usartBufferHandle == DRV_HANDLE_INVALID )
                {
                    appData.currentState = APP_ERROR;
                }
                else
                {
                    appData.prevState    = APP_USR_MSG_WRITE;
                    appData.currentState = APP_WAIT_FOR_DONE;
                }
            }
        }
        break;

        case APP_IDLE:
        {
            /* Close USART Driver */
            DRV_USART_Close( appData.usartHandle );
            /* Deinitialize the driver */
            DRV_USART_Deinitialize( sysObj.drvUsart0 );
            /* The appliction comes here when the demo has completed
             * successfully. Switch on LED D5. */
        }
        break;

        case APP_ERROR:
        {
            /* The appliction comes here when the demo
             * has failed. need to switch LED on to signal comm error.*/
            
        }
        break;

        default:
            break;
    }
}

/*******************************************************************************
  Function:
    void App_UpdateTaskState( void )

  Remarks:
    See prototype in app.h.
 */

void App_GetNextTaskState(uint32_t appState)
{
    switch ( appState )
    {
        case APP_DRV_READY:
            appData.currentState = APP_DRV_MSG_WRITE;
            break;

        case APP_DRV_MSG_WRITE:
            /* Insert newline character, before accepting data from user */
            strcpy(appData.buffer, "\n");
            appData.bufferSize = APP_NO_OF_BYTES_TO_READ;
            /* Set the buffer event for user data */
            DRV_USART_BufferEventHandlerSet(appData.usartHandle,
                              APP_BufferEventHandler, APP_USR_CONTEXT);
            /* Set the next Demo App. State */
            appData.currentState = APP_USR_MSG_READ;
            
        
        break;

        case APP_USR_MSG_READ:
            appData.currentState = APP_USR_MSG_WRITE;
            break;

        case APP_USR_MSG_WRITE:
            appData.currentState = APP_USR_MSG_READ;
            break;

    }
}

/*******************************************************************************
 End of File
*/

