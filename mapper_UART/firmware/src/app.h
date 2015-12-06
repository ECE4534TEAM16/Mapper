/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    app.h

  Summary:
    This header file provides prototypes and definitions for the application.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APP_Initialize" and "APP_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APP_STATES" definition).  Both
    are defined here for convenience.
*******************************************************************************/

#ifndef _APP_H
#define _APP_H


// Section: Included Files

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "FreeRTOS.h"
#include "timers.h"
#include <time.h>
#include "messageQueue.h"
#include "system_definitions.h"
#include "system_config.h"
#include "system/system.h"
#include "system/clk/sys_clk.h"
#include "driver/usart/drv_usart.h"
#include "system/devcon/sys_devcon.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END 

// Section: Type Definitions
    extern SYS_MODULE_OBJ    usartModule;

// Section: Application Configuration
    #define APP_TEST                true 
    #define APP_ERROR_TESTING       false
    #define APP_QUEUE_SIZE          5*sizeof(char) 
    #define APP_MAX_ERROR_LOG       5
    #define APP_NUMBER_OF_TICKS     5
    #define APP_NO_OF_BYTES_TO_READ 5          
    #define APP_BUFFER_SIZE         80
    #define APP_ERROR_BUFFER_SIZE   75      //must be 5 less than APP_BUFFER_SIZE
    #define INSTRUCTION_BUFFER_SIZE 160
    #define APP_UART_BAUDRATE       57600
    #define APP_USR_ESC_KEY         0x1B
    #define APP_USR_RETURN_KEY      0x0D
    #define APP_USART_DRIVER_INDEX  DRV_USART_INDEX_0

    #define APP_DRV_CONTEXT         1
    #define APP_USR_CONTEXT         2

// *****************************************************************************
/* Application states

  Summary:
    Application states enumeration

  Description:
    This enumeration defines the valid application states.  These states
    determine the behavior of the application at various times.
*/

typedef enum
{
    /* In this state, the application opens the driver. */
    APP_DRV_OPEN,

    /* In this state driver will be ready to accept buffer */
    APP_DRV_READY,
            
    /* In this state driver prints demo banner message */
    APP_DRV_MSG_WRITE,

    /* In this state driver waits for the buffer event to get complete. */
    APP_WAIT_FOR_DONE,

    /* In this state, the App. accepts data from user*/
    APP_USR_MSG_READ,

    /* In this state, the App. writes user data to output terminal */
    APP_USR_MSG_WRITE,

    /* The application does nothing in the idle state. */
    APP_IDLE,

    /* This state indicates an error has occurred. */
    APP_ERROR,
            
} APP_STATES;


// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    Application strings and buffers are be defined outside this structure.
 */

typedef struct
{
    /* Application current state */
    APP_STATES currentState;

    /* Application Previous state */
    APP_STATES prevState;

    /* USART buffer for display */
    char buffer[APP_BUFFER_SIZE];

    /* Data Size in the USART Buffer */
    uint32_t bufferSize;

    /* Demo Application Message Indices */
    uint32_t usrMsgIndex;
    
    /* Cleint Status */
    DRV_USART_CLIENT_STATUS usartStatus;

    /* USART driver handle */
    DRV_HANDLE usartHandle;

    /* Handle returned by USART for buffer submitted */
    DRV_USART_BUFFER_HANDLE usartBufferHandle;

    /* Handle returned by USART for buffer event */
    DRV_USART_BUFFER_EVENT  usartBufferEvent;

    /* Flag to indicate the driver message is been processed */
    bool drvBufferEventComplete;

    /* Flag to indicate the user message is been processed */
    bool usrBufferEventComplete;
    
    char minBuffer[80];
    
    char InstructionSet[INSTRUCTION_BUFFER_SIZE];
    
    bool queued;
    
    bool initialized;
    
    bool error;
    
    bool error_sent;
    
    bool start;

} APP_DATA;


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************
/* These routines are called by drivers when certain events occur.
*/

	
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Summary:
     MPLAB Harmony application initialization routine.

  Description:
    This function initializes the Harmony application.  It places the 
    application in its initial state and prepares it to run so that its 
    APP_Tasks function can be called.

  Precondition:
    All other system initialization routines should be called before calling
    this routine (in "SYS_Initialize").

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void APP_Initialize ( void );


/*******************************************************************************
  Function:
    void APP_Tasks ( void )

  Summary:
    MPLAB Harmony Demo application tasks function

  Description:
    This routine is the Harmony Demo application's tasks function.  It
    defines the application's state machine and core logic.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void APP_Tasks( void );



void App_GetNextTaskState ( uint32_t appState );
void check_Start();
void setError(char* error);


#endif /* _APP_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/*******************************************************************************
 End of File
 */

