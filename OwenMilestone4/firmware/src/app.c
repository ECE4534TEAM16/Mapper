/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

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
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback funtions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary local functions.
*/

void GPIOOutputStringDebug(char* s, int length)
{
    if(!appData.debug)
        return;
    int i = 0;
    while( i < length )
    {
        vTaskDelay(25 / portTICK_PERIOD_MS);
        PLIB_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_E, s[i]);
    }
    vTaskDelay(25 / portTICK_PERIOD_MS);
    PLIB_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_E, (char)0);

    
}

void InterpretDataThread()
{
    uint8_t fakeIRData[] = {
        51, 51, 51, 39, 15, 39, 51, 51, 51, 51, 51,
        51, 51, 51, 57, 60, 57, 51, 51, 51, 51, 51,
        0

        //60, 00 111100 drifting far right
        //57, 00 111001 drifting right
        //51, 00 110011 on course
        //39, 00 100111 drifting left
        //15, 00 001111 drifting far left
        //3,  00 000011 left turn
        //48, 00 110000 right turn
        //0,  00 000000 intersection both sides
    };
    //write these values to a message cue
    int i = 0;
    
    char lastValue = 51;
    while(true)
    {  
        //read from IR message queue;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        char nextValue = fakeIRData[ i % (sizeof(fakeIRData)/sizeof(uint8_t))];
        i++;
        
        if(appData.debugInterpretData)
        {
            if(nextValue == lastValue)
                PLIB_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_E, 0);
            else
                PLIB_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_E, (uint8_t)nextValue);
        }
        
        if(nextValue == lastValue)
        {
            continue;
        }
        else
        {
            lastValue = nextValue;
            
            StandardMessage msg;
            msg.messageType = 'i';
            msg.ucData[0] = (uint8_t)nextValue;
            xQueueSend(MsgQueue_MapEncoder_Interrupt, &msg, 25);
        }
        
        //PLIB_PORTS_PinToggle (PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_3);
        //vTaskDelay(4000 / portTICK_PERIOD_MS);
    }
}

void MapperControlThread()
{ 
    short leftEncoderCount = 0;
    short rightEncoderCount = 0;
    
    while(true)
    {
        StandardMessage messageFromMapEncoderInterrupt;
        if( xQueueReceive( MsgQueue_MapEncoder_Interrupt, &messageFromMapEncoderInterrupt, 100) )
        {
            if(messageFromMapEncoderInterrupt.messageType == 'i')
            {
                /*
                switch ( messageFromMapEncoderInterrupt.ucData[0] )
                {
                    case 60: // 00 111100 drifting far right
                        GPIOOutputStringDebug("Correct left hard", 17); 
                        
                        //correct left hard
                        break;
                    case 57: // 00 111001 drifting right
                        GPIOOutputStringDebug("Correct left", 12); 
                        
                        // correct left
                        break;
                    case 51: // 00 110011 on course
                        GPIOOutputStringDebug("Go straight", 11); 
                        
                        // drive straight
                        break;
                    case 39: // 00 100111 drifting left
                        GPIOOutputStringDebug("Correct right", 13); 
                        
                        // correct right
                        break;
                    case 15: // 00 001111 drifting far left 
                        GPIOOutputStringDebug("Correct far right", 17); 
                        
                        // correct right hard
                        break;
                    // everything above this point should be sent directly to
                    //  motor control as a movement command
                        
                    // everything below this point will cause the robot to start
                    //  an intersection mapping subrouting
                    case 3:  // 00 000011 left turn
                        GPIOOutputStringDebug("Mapping an intersection", 23); 
                        
                        // map the intersection
                        break;
                    case 48: // 00 110000 right turn
                        GPIOOutputStringDebug("Mapping an intersection", 23);
                        
                        // map the intersection
                        break;
                    case 0:   // 00 000000 intersection both sides
                        GPIOOutputStringDebug("Mapping an intersection", 23);
                        
                        // map the intersection
                        break;
                }*/
            }
            
            if(messageFromMapEncoderInterrupt.messageType == 'e')
            {
                if(messageFromMapEncoderInterrupt.ucData[0] == 'l') // left encoder moves 1 cm
                {
                    leftEncoderCount += 1;
                }
                else if(messageFromMapEncoderInterrupt.ucData[0] == 'r') // right encoder moves 1 cm
                {
                    rightEncoderCount += 1;
                }
            }
        }
        
        if(appData.debugEncoders)
        {
            PLIB_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_E, 'l');
            vTaskDelay(25 / portTICK_PERIOD_MS);
            PLIB_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_E, (char)leftEncoderCount);
            vTaskDelay(25 / portTICK_PERIOD_MS);            
            PLIB_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_E, 'r');
            vTaskDelay(25 / portTICK_PERIOD_MS);            
            PLIB_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_E, (char)rightEncoderCount);
            vTaskDelay(25 / portTICK_PERIOD_MS);
            PLIB_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_E, (char)0);
            vTaskDelay(25 / portTICK_PERIOD_MS);
        }
    }
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
    appData.debugInterpretData = true;
    appData.debugEncoders = false;
    appData.debug = false;
    
    // these two timers run external interrupts
    DRV_TMR0_Start();
    DRV_TMR1_Start();
    
    // these lines configure LEDs for operation
    PLIB_PORTS_PinDirectionOutputSet (PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_3);
    PLIB_PORTS_PinSet (PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_3);
    
    // this code declares my message queues, declared in the header
    MsgQueue_MapEncoder_Interrupt = xQueueCreate( 50, sizeof( StandardMessage ) );
    MsgQueue_MapSensor_Interrupt = xQueueCreate( 50, sizeof( StandardMessage ) );
    //vTaskDelay(100 / portTICK_PERIOD_MS);
    //MsgQueue_MapSensor_Thread = xQueueCreate( 50, sizeof( StandardMessage ) );
    
    // this code declares some GPIO output pins as outputs for me
    PLIB_PORTS_DirectionOutputSet (PORTS_ID_0, PORT_CHANNEL_E, 0xFF);
    PLIB_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_E, 'a');
    
    // this code starts tasks for mapper control threads
    TaskHandle_t Handle_MapperControlThread;
    xTaskCreate((TaskFunction_t) MapperControlThread, 
                "MapperControlThread", 
                1024, NULL, 1, NULL);
    TaskHandle_t Handle_InterpretDataThread;
    xTaskCreate((TaskFunction_t) InterpretDataThread, 
                "InterpretDataThread", 
                1024, NULL, 1, NULL);
    vTaskStartScheduler();

}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{
    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            break;
        }

        /* TODO: implement your application state machine.*/

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}
 

/*******************************************************************************
 End of File
 */
