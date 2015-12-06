/*******************************************************************************
  USART User Rover Driver Source File.

  File Name:
    app.c

  Summary:
    USART driver for the User Rover.

  Description:
    This file contains the USART driver needed to make the User Rover function.
    The main purpose of the User USART driver is to recEIve single character 
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

#ifdef APP_TEST
    char temp[INSTRUCTION_BUFFER_SIZE];
    char test[INSTRUCTION_BUFFER_SIZE];
#endif

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
    /* Set the flag for Queue fill*/
    appData.queued            = false;
    /* Set the initialized flag to false*/
    appData.initialized       = false;
    /* Set the error flag to false*/
    appData.error             = false;
    /* set the error_sent flag to true*/
    appData.error_sent        = true;
    
    MsgQueue_Error_Log = xQueueCreate(APP_MAX_ERROR_LOG, APP_ERROR_BUFFER_SIZE);
    if(MsgQueue_Error_Log == 0)
    {
        //Big failure
        //Need to set an LED on to alert us of this error as
        //we wont be able to send the error code through UART without this
        //Message Queue.
        
    }

    MsgQueue_User_Directions = xQueueCreate(APP_BUFFER_SIZE, sizeof( char ));
    if( MsgQueue_User_Directions == 0 )
    {
        //BAD
        //fatal error
    }
    
}

//if error occurs before UART is opened the error will be fatal
//otherwise the system will try to return to a functioning state
void setError(char* error)
{
    xQueueSendToBack(MsgQueue_Error_Log, error, APP_NUMBER_OF_TICKS);
    appData.error = true;
    
}

//will take the instruction set and put it into a message queue
//fills the user_instructions with the data from the Pi after the 
//Pi has finished sending the entire instruction set
void fillQueue()
{
    int count = 0;
    appData.queued = true;
    for(count; count <= strlen(appData.InstructionSet); count++)
    {
        xQueueSendToBack(MsgQueue_User_Directions, 
                          (void *) &appData.InstructionSet[count], 
                          APP_NUMBER_OF_TICKS);
    }
    
    appData.currentState = APP_USR_MSG_WRITE;
    
}

//copies the current buffer character to the instruction set array
void AddInstr()
{
    if(APP_TEST)
    {
        if(temp[0] == NULL)  
        strcpy(temp, appData.buffer);
        else
        {
            strcat(temp, ",");
            strcat(temp, appData.buffer); 
        }
    }
    if(appData.InstructionSet[0] == NULL)  
        strcpy(appData.InstructionSet, appData.buffer);
    else
    {
        strcat(appData.InstructionSet, appData.buffer); 
    }
    if(appData.buffer[0] == 'E' && appData.queued == false)
        fillQueue();
    if(APP_ERROR_TESTING)
    {
        if(appData.buffer[0] == '1')
            setError("Testing Error Message Queue");
    }
        
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
        {
            setError("Buffer Event Error");
        }
            break;

        /* Buffer event has aborted */
        case DRV_USART_BUFFER_EVENT_ABORT:
        {
            setError("Buffer Event has Aborted");
        }
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
    if(appData.error && appData.error_sent)
    {
        appData.currentState = APP_ERROR;
        appData.error_sent = false;
    }
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
                appData.currentState = APP_ERROR; //fatal error
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
                    setError("Invalid Driver Handle in DRV_READY");
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
                appData.error_sent = true;
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
                    setError("Invalid Driver Handle in DRV_MSG_WRITE");
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
                    setError("Invalid Driver Handle in USR_MSG_READ");
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
                if(APP_TEST) //Tests the rover on receiving the instruction set
                {
                strcpy(test, "\r\n");
                strcat(test, temp );
                
                appData.bufferSize = min(APP_BUFFER_SIZE,
                                                       strlen(test));
                DRV_USART_BufferAddWrite( appData.usartHandle,
                                          &(appData.usartBufferHandle),
                                          test, appData.bufferSize);
                }
                if ( appData.usartBufferHandle == DRV_HANDLE_INVALID )
                {
                    appData.prevState    = APP_USR_MSG_WRITE;
                    setError("Invalid Driver Handle");
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
            /* The appliction comes here when the rover has completed its path
             * successfully. Need to implement in code. */
        }
        break;

        case APP_ERROR:
        {
            char error_buffer[APP_BUFFER_SIZE];
            char pre_error[APP_ERROR_BUFFER_SIZE];
            if(uxQueueMessagesWaiting(MsgQueue_Error_Log) > 0 && appData.prevState != APP_DRV_OPEN)
            {
                //set the handler
                DRV_USART_BufferEventHandlerSet(appData.usartHandle,
                                           APP_BufferEventHandler, APP_DRV_CONTEXT);

                appData.usartStatus = DRV_USART_ClientStatus( appData.usartHandle );

                if ( appData.usartStatus == DRV_USART_CLIENT_STATUS_READY )
                {
                    if(xQueueReceive(MsgQueue_Error_Log, &pre_error, APP_NUMBER_OF_TICKS))
                    {
                        strcpy( error_buffer, "\r\n" );
                        strcat( error_buffer, pre_error);
                        appData.bufferSize = min(APP_BUFFER_SIZE,
                                                               strlen(error_buffer));
                        DRV_USART_BufferAddWrite( appData.usartHandle,
                                                  &(appData.usartBufferHandle),
                                                  error_buffer, appData.bufferSize);
                    }
                    if ( appData.usartBufferHandle == DRV_HANDLE_INVALID )
                    {
                        //set LED, this is fatal error
                    }
                    else
                    {
                        appData.prevState    = APP_ERROR;
                        appData.currentState = APP_WAIT_FOR_DONE;
                    }
                }
            }
            else
            {
                //comm error(fatal), set LED
                //state will stay in the error state
            }
            
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
            strcpy(appData.buffer, "\n");
            appData.bufferSize = APP_NO_OF_BYTES_TO_READ;
            /* Set the buffer event for user data */
            DRV_USART_BufferEventHandlerSet(appData.usartHandle,
                              APP_BufferEventHandler, APP_USR_CONTEXT);
            /* Set the next Demo App. State */
            appData.currentState = APP_USR_MSG_READ; 
            appData.initialized = true; //safeguard in error handling
        break;

        case APP_USR_MSG_READ:
            AddInstr();
            if(APP_TEST || appData.queued == true )
                appData.currentState = APP_USR_MSG_WRITE;
            break;

        case APP_USR_MSG_WRITE:
            if(APP_TEST)
                appData.currentState = APP_USR_MSG_READ;
            else
                appData.currentState = APP_USR_MSG_WRITE;
            break;
            
        case APP_ERROR:
            if(uxQueueMessagesWaiting(MsgQueue_Error_Log) > 0)
            {
                appData.currentState = APP_ERROR;
            }
            else //no errors left to be written
            {
                appData.error = false;
                if(appData.initialized)
                {
                    DRV_USART_BufferEventHandlerSet(appData.usartHandle,
                              APP_BufferEventHandler, APP_USR_CONTEXT);
                    
                    if(appData.queued)
                        appData.currentState = APP_USR_MSG_WRITE;
                    else
                        appData.currentState = APP_USR_MSG_READ;
                    
                }
                else
                    appData.currentState = APP_DRV_MSG_WRITE;       
            }
            
            break;

    }
}

/*******************************************************************************
 End of File
*/

