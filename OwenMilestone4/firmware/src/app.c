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

void ClearAllQueues()
{
    xQueueReset(MsgQueue_MapEncoder_Interrupt);
    xQueueReset(MsgQueue_MapSensor_Interrupt);
    xQueueReset(MsgQueue_MapSensor_Thread);
    xQueueReset(MsgQueue_MapAlgorithm_Instructions);
}

void GPIOOutputStringDebug(char* s, int length)
{
    int i = 0;
    while( i < length )
    {
        vTaskDelay(25 / portTICK_PERIOD_MS);
        PLIB_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_B, s[i]);
    }
    vTaskDelay(25 / portTICK_PERIOD_MS);
    PLIB_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_B, (char)0);
}

void moveRobot(int leftSpeed, int rightSpeed)
{
    if(leftSpeed > 0)
    {
        PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_1);
    }
    else
    {
        PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_1);
    }
    
    if(rightSpeed > 0)
    {
        PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_14);
    }
    else
    {
        PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_14);
    }
    
    PLIB_OC_PulseWidth16BitSet(1,leftSpeed);
    PLIB_OC_PulseWidth16BitSet(0,rightSpeed);
}

void readIR()
{
    PLIB_PORTS_DirectionInputSet(PORTS_ID_0, PORT_CHANNEL_E, 0xFF);

    const TickType_t waitDecay = 4 / portTICK_PERIOD_MS;
    vTaskDelay(waitDecay);
    
    char IRData = PLIB_PORTS_Read(PORTS_ID_0, PORT_CHANNEL_E);
    IRData = IRData & 0x3F;
    
    PLIB_PORTS_DirectionOutputSet(PORTS_ID_0, PORT_CHANNEL_E, 0xFF);
    PLIB_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_E, 0xFF);
    
    StandardMessage msg;
    msg.data = IRData;
    if( xQueueSendFromISR( MsgQueue_MapSensor_Interrupt, &msg, 100) );
    
    //    uint8_t fakeIRData[] = {60,57,51,39,15,3,48,0};
    //    appData.IRData = fakeIRData[rand() % 8];
}

void TurnRight()
{
    char rightEncoderCount = 0;
    char leftEncoderCount = 0;

    xQueueReset(MsgQueue_MapEncoder_Interrupt);
    moveRobot(800, 800);
    while(rightEncoderCount < 12 && leftEncoderCount < 12)
    {
        IDT_UpdateDistance(&rightEncoderCount, &leftEncoderCount);
        
    }
    rightEncoderCount = 0;
    xQueueReset(MsgQueue_MapEncoder_Interrupt);
    moveRobot(-800,800);
    while(rightEncoderCount < 8)
        IDT_UpdateDistance(&rightEncoderCount, &leftEncoderCount);
    moveRobot(0,0);
}

void TurnLeft()
{
    char rightEncoderCount = 0;
    char leftEncoderCount = 0;

    xQueueReset(MsgQueue_MapEncoder_Interrupt);
    moveRobot(800, 800);
    while(rightEncoderCount < 12 && leftEncoderCount < 12)
        IDT_UpdateDistance(&rightEncoderCount, &leftEncoderCount);

    rightEncoderCount = 0;
    xQueueReset(MsgQueue_MapEncoder_Interrupt);
    moveRobot(-800,800);
    while(leftEncoderCount < 8)
        IDT_UpdateDistance(&rightEncoderCount, &leftEncoderCount);
    moveRobot(0,0);
}

void MoveDistance(int d) // d is distance in cm
{
    xQueueReset(MsgQueue_MapEncoder_Interrupt);
    moveRobot(800, 800);
    if(d < 0)
    {
        moveRobot(-800, -800);
        d = -1*d;
    }
    char rightEncoderCount = 0;
    char leftEncoderCount = 0;
    while( rightEncoderCount < d && leftEncoderCount < d )
    {
        IDT_UpdateDistance(&rightEncoderCount, &leftEncoderCount);
    }
    moveRobot(0,0);
}

void IDT_UpdateDistance(char* rightEncoderCount, char* leftEncoderCount)
{
    StandardMessage messageFromMapEncoderInterrupt;
    if( xQueueReceive( MsgQueue_MapEncoder_Interrupt, &messageFromMapEncoderInterrupt, 0) )
    {
        if(messageFromMapEncoderInterrupt.data == 'L') // left encoder moves 1 cm
        {
            *leftEncoderCount += 1;
        }
        else if(messageFromMapEncoderInterrupt.data == 'R') // right encoder moves 1 cm
        {
            *rightEncoderCount += 1;
        }
    }
    /*
    if(appData.debugEncoders)
        {
            PLIB_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_B, 'l');
            vTaskDelay(25 / portTICK_PERIOD_MS);
            PLIB_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_B, (char)*leftEncoderCount);
            vTaskDelay(25 / portTICK_PERIOD_MS);            
            PLIB_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_B, 'r');
            vTaskDelay(25 / portTICK_PERIOD_MS);            
            PLIB_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_B, (char)*rightEncoderCount);
            vTaskDelay(25 / portTICK_PERIOD_MS);
            PLIB_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_B, (char)0);
            vTaskDelay(25 / portTICK_PERIOD_MS);
        }
   */
}

void IDT_CorrectDirection(char c)
{
    PLIB_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_B, c);
    switch(c)
    {
        //NORMAL
        case 12: //00110011
        {
            moveRobot(600,600);
            break;
        }
        
        //WOBBLES
        //Off left
        case 8: //00111001
        {
            moveRobot(600,0);
            break;
        }
        case 40: //00111100
        {
            moveRobot(600,0);
            break;
        }
        case 32: //00111001
        {
            moveRobot(600,0);
            break;
        }
        case 48: //00111100
        {
            moveRobot(600,0);
            break;
        }
        case 16: //00111001
        {
            moveRobot(600,0);
            break;
        }
        
        //Off right
        case 2: //00100111
        {
            moveRobot(0,600);
            break;
        }
        case 6: //00001111
        {
            moveRobot(0,600);
            break;
        }
        case 4: //00001111
        {
            moveRobot(0,600);
            break;
        }
        case 3: //00001111
        {
            moveRobot(0,600);
            break;
        }
        case 1: //00001111
        {
            moveRobot(0,600);
            break;
        }
        xQueueReset(MsgQueue_MapSensor_Interrupt);
    }
    
}

bool IDT_CheckForIntersection(char c)
{
    bool intersection = ( c == 47 || c == 15 || c == 62 || c == 60 || c == 63 ); //L, R, both
    if(intersection)
        moveRobot(0, 0);
    return intersection; 
}

void IDT_MapIntersection(char* leftPath, char* rightPath, char* forwardPath)
{
    *leftPath = 1;
    *rightPath = 1;
    *forwardPath = 1;
}

void IDT_SendToMapperThread(char rightEncoderCount, char leftEncoderCount, char leftPath, char rightPath, char forwardPath)
{
    EventData intersectionInformation;
    intersectionInformation.distFromLastIntersection = (rightEncoderCount < leftEncoderCount) ? rightEncoderCount : leftEncoderCount;
    intersectionInformation.forwardPathExists = forwardPath;
    intersectionInformation.rightPathExists = rightPath;
    intersectionInformation.leftPathExists = leftPath;
    intersectionInformation.absoluteDirection = '0';
    xQueueSend( MsgQueue_MapSensor_Thread, &intersectionInformation, 100);
}

void IDT_RecAndExInstruction()
{
    StandardMessage instruction;
    if( xQueueReceive( MsgQueue_MapAlgorithm_Instructions, &instruction, 0) )
    {
        PLIB_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_B, instruction.data);
        switch(instruction.data)
        {
            case 'r':
                TurnRight();
                break;
            case 'l':
                TurnLeft();
                break;
            case 'f':
                MoveDistance(4);
                break;
        }
    }
    ClearAllQueues();
}

void InterpretDataThread()
{
    //60, 00 111100 drifting far right
    //57, 00 111001 drifting right
    //51, 00 110011 on course
    //39, 00 100111 drifting left
    //15, 00 001111 drifting far left
    //3,  00 000011 left turn
    //48, 00 110000 right turn
    //0,  00 000000 intersection both sides
    
    char rightEncoderCount = 0; //SinceLastIntersection
    char leftEncoderCount = 0; //SinceLastIntersection
    char leftPath = 0, rightPath = 0, forwardPath = 0;
    
    //moveRobot(800, 800);
    
    while(true)
    {
        IDT_UpdateDistance(&rightEncoderCount, &leftEncoderCount);
        //read distance from the Encoders;
    
        StandardMessage messageFromMapSensorInterrupt;
        if( xQueueReceive( MsgQueue_MapSensor_Interrupt, &messageFromMapSensorInterrupt, 0) )
        {
            //PLIB_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_B, messageFromMapSensorInterrupt.data);
            IDT_CorrectDirection(messageFromMapSensorInterrupt.data);
            if( IDT_CheckForIntersection(messageFromMapSensorInterrupt.data) )
            {
                IDT_MapIntersection(&leftPath, &rightPath, &forwardPath);
                IDT_SendToMapperThread(rightEncoderCount, leftEncoderCount, leftPath, rightPath, forwardPath);
                IDT_RecAndExInstruction();
            }
        }
        
        //deals w/ finding intersections and getting notifications.
    }
    
}

void MapperControlThread()
{ 
    while(true)
    {
        //PLIB_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_B, 'c');
        EventData messageFromIDT;
        if( xQueueReceive( MsgQueue_MapSensor_Thread, &messageFromIDT, 0) )
        {
            PLIB_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_B, messageFromIDT.distFromLastIntersection);
        ///
        // Austin Mapper code should go here.
        ///

            StandardMessage instruction;
            instruction.data = 'R'; //Austin tells us which direction to go from here
            xQueueSend(MsgQueue_MapAlgorithm_Instructions, &instruction, 0);
            
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
    appData.debugInterpretData = false;
    appData.debugEncoders = true;
    appData.debug = false;
    
    irTimer = xTimerCreate("IR Timer", 100/portTICK_PERIOD_MS,pdTRUE, (void*) 1, readIR);
    xTimerStart(irTimer, 100);
    
    // these two timers run external interrupts
    DRV_TMR1_Start();
    DRV_TMR2_Start();

    // this timer is used by the external interrupt?
    DRV_TMR0_Start();
    
    // these commands set up the PWM for the motors
    DRV_OC0_Start();
    DRV_OC1_Start();
    PLIB_TMR_Period16BitSet(1,1000);
    PLIB_OC_PulseWidth16BitSet(0,0);
    PLIB_OC_PulseWidth16BitSet(1,0);
    PLIB_PORTS_PinDirectionOutputSet (PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_14);
    PLIB_PORTS_PinDirectionOutputSet (PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_1);
    
    // these lines configure LEDs for operation
    PLIB_PORTS_PinDirectionOutputSet (PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_3);
    PLIB_PORTS_PinSet (PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_3);
    
    // this code declares my message queues, declared in the header
    MsgQueue_MapEncoder_Interrupt = xQueueCreate( 50, sizeof( StandardMessage ) );
    MsgQueue_MapSensor_Interrupt = xQueueCreate( 50, sizeof( StandardMessage ) );
    MsgQueue_MapAlgorithm_Instructions = xQueueCreate( 50, sizeof( StandardMessage ) );
    MsgQueue_MapSensor_Thread = xQueueCreate( 50, sizeof( EventData ) );
    
    // this code declares some GPIO output pins as outputs for me
    PLIB_PORTS_DirectionOutputSet (PORTS_ID_0, PORT_CHANNEL_E, 0xFF);
    
    PLIB_PORTS_DirectionOutputSet (PORTS_ID_0, PORT_CHANNEL_B, 0xFF);
    PLIB_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_B, 'a');

    //moveRobot(800, 800);
    TurnRight();
    
    // this code starts tasks for mapper control threads
    TaskHandle_t Handle_MapperControlThread;
    xTaskCreate((TaskFunction_t) MapperControlThread, 
                "MapperControlThread", 
                1024, NULL, 1, NULL);
    TaskHandle_t Handle_InterpretDataThread;
    xTaskCreate((TaskFunction_t) InterpretDataThread, 
                "InterpretDataThread", 
                1024, NULL, 1, NULL);
//    vTaskStartScheduler();

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
            //moveRobot(500,500);
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
