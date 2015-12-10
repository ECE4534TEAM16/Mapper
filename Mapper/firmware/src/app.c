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

int ClosestValueFromTicks(int ticks)
{
    ticks = ticks + 68;
    return ticks / 136;
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
}

void TurnRight()
{
    char rightEncoderCount = 0;
    char leftEncoderCount = 0;
    xQueueReset(MsgQueue_MapEncoder_Interrupt);
    moveRobot(600,-600);
    while(rightEncoderCount < 56)
    {
        IDT_UpdateDistance(&rightEncoderCount, &leftEncoderCount);
    }
    moveRobot(0,0);
}

void TurnLeft()
{
    char rightEncoderCount = 0;
    char leftEncoderCount = 0;

    xQueueReset(MsgQueue_MapEncoder_Interrupt);
    moveRobot(-600,600);
    while(leftEncoderCount < 56)
        IDT_UpdateDistance(&rightEncoderCount, &leftEncoderCount);
    moveRobot(0,0);
}

void TurnAround()
{
    char rightEncoderCount = 0;
    char leftEncoderCount = 0;
    xQueueReset(MsgQueue_MapEncoder_Interrupt);
    moveRobot(600,-600);
    while(rightEncoderCount < 112)
    {
        IDT_UpdateDistance(&rightEncoderCount, &leftEncoderCount);
    }
    moveRobot(0,0);

}

void MoveDistance(int d, int speed) // 1/17th of an inch
{
    xQueueReset(MsgQueue_MapEncoder_Interrupt);
    int rightEncoderCount = 0;
    int leftEncoderCount = 0;
    if(d < 0)
    {
        moveRobot(-1*speed, -1*speed);
        d = -1*d;
    }
    else
    {
        moveRobot(speed, speed);
    }
    while( rightEncoderCount < d && leftEncoderCount < d )
    {
        IDT_UpdateDistance(&rightEncoderCount, &leftEncoderCount);
    }
    moveRobot(0,0);
}

void IDT_UpdateDistance(char* rightEncoderCount, char* leftEncoderCount)
{
    StandardMessage messageFromMapEncoderInterrupt;
    while( xQueueReceive( MsgQueue_MapEncoder_Interrupt, &messageFromMapEncoderInterrupt, 0) )
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
}

void IDT_CorrectDirection(char c)
{
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
            moveRobot(400,0);
            break;
        }
        case 40: //00111100
        {
            moveRobot(400,0);
            break;
        }
        case 32: //00111001
        {
            moveRobot(400,0);
            break;
        }
        case 48: //00111100
        {
            moveRobot(400,0);
            break;
        }
        case 16: //00111001
        {
            moveRobot(400,0);
            break;
        }
        
        //Off right
        case 2: //00100111
        {
            moveRobot(0,400);
            break;
        }
        case 6: //00001111
        {
            moveRobot(0,400);
            break;
        }
        case 4: //00001111
        {
            moveRobot(0,400);
            break;
        }
        case 3: //00001111
        {
            moveRobot(0,400);
            break;
        }
        case 1: //00001111
        {
            moveRobot(0,400);
            break;
        }
        xQueueReset(MsgQueue_MapSensor_Interrupt);
    }
    
}

bool IDT_CheckForEnd(char c)
{
    bool isEnd = c == 0;
    return isEnd;
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
    MoveDistance(1, 100);
    StandardMessage msg;
    xQueueReset(MsgQueue_MapSensor_Interrupt);
    xQueueReceive( MsgQueue_MapSensor_Interrupt, &msg, 500 );
    *leftPath = (char)(msg.data == 15 || msg.data == 47);
    *rightPath = (char)(msg.data == 60 || msg.data == 62);
    MoveDistance(39, 600);
    xQueueReset(MsgQueue_MapSensor_Interrupt);
    xQueueReceive( MsgQueue_MapSensor_Interrupt, &msg, 500 );
    *forwardPath = (msg.data != 0);
}

void ExecuteTurn(char data)
{
    PLIB_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_B, data);
    switch(data)
    {
        case 'r':
            TurnRight();
            break;
        case 'l':
            TurnLeft();
            break;
        case 'f':
            MoveDistance(16, 600);
            break;
        case 't':
            TurnAround();
            MoveDistance(16, 600);
            break;
        case 'S':
            moveRobot(0, 0);
            while(true);
    }
}

EventData MakeATurn(EventData in)
{ 
    PLIB_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_B, ']');
    PLIB_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_B, in.forwardPathExists);
    PLIB_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_B, in.leftPathExists);
    PLIB_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_B, in.rightPathExists);
    PLIB_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_B, in.distFromLastIntersection);
    PLIB_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_B, ']');
    
    char data[] = {'r', 'f', 'f', 'f', 'f', 't', 'r', 'r', 't', 'l', 'r', 'r', 't', 'f', 'l', 't', 'f', 'S'};

//    ExecuteTurn( approachIntersection(in.leftPathExists!=0, in.rightPathExists!=0, in.forwardPathExists!=0, in.distFromLastIntersection) );
    ExecuteTurn(data[appData.InstructionNumber]);
    appData.InstructionNumber ++;
    
    return in;

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
    appData.InstructionNumber = 0;
    
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
    
    // this code declares some GPIO output pins as outputs for me
    PLIB_PORTS_DirectionOutputSet (PORTS_ID_0, PORT_CHANNEL_E, 0xFF);
    
    PLIB_PORTS_DirectionOutputSet (PORTS_ID_0, PORT_CHANNEL_B, 0xFF);
    PLIB_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_B, 'a');

    
    appData.rightEncoderCount = 0; //SinceLastIntersection
    appData.leftEncoderCount = 0; //SinceLastIntersection
    appData.leftPath = 0;
    appData.rightPath = 0;
    appData.forwardPath = 0;
    
    // this code starts tasks for mapper control threads
    /*TaskHandle_t Handle_InterpretDataThread;
    xTaskCreate((TaskFunction_t) InterpretDataThread, 
                "InterpretDataThread", 
                1024, NULL, 1, NULL);
    vTaskStartScheduler();*/

}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{
    IDT_UpdateDistance(&appData.rightEncoderCount, &appData.leftEncoderCount);
    
    StandardMessage messageFromMapSensorInterrupt;
    if( xQueueReceive( MsgQueue_MapSensor_Interrupt, &messageFromMapSensorInterrupt, 0) )
    {
        IDT_CorrectDirection(messageFromMapSensorInterrupt.data);
        if( IDT_CheckForEnd(messageFromMapSensorInterrupt.data) )
        {
            EventData intersectionInformation;
            intersectionInformation.distFromLastIntersection = ClosestValueFromTicks( (appData.rightEncoderCount < appData.leftEncoderCount) ? appData.rightEncoderCount : appData.leftEncoderCount );
            intersectionInformation.forwardPathExists = 0;
            intersectionInformation.rightPathExists = 0;
            intersectionInformation.leftPathExists = 0;
            intersectionInformation.absoluteDirection = '0';
            intersectionInformation = MakeATurn(intersectionInformation);
            appData.rightEncoderCount = 80; //SinceLastIntersection
            appData.leftEncoderCount = 80;
        }
        else if( IDT_CheckForIntersection(messageFromMapSensorInterrupt.data) )
        {
            IDT_MapIntersection(&appData.leftPath, &appData.rightPath, &appData.forwardPath);
            EventData intersectionInformation;
            intersectionInformation.distFromLastIntersection = ClosestValueFromTicks( (appData.rightEncoderCount < appData.leftEncoderCount) ? appData.rightEncoderCount : appData.leftEncoderCount );
            intersectionInformation.forwardPathExists = appData.forwardPath;
            intersectionInformation.rightPathExists = appData.rightPath;
            intersectionInformation.leftPathExists = appData.leftPath;
            intersectionInformation.absoluteDirection = '0';
            intersectionInformation = MakeATurn(intersectionInformation);
            appData.rightEncoderCount = 20; //SinceLastIntersection
            appData.leftEncoderCount = 20;
        }
        xQueueReset(MsgQueue_MapSensor_Interrupt);
    }

    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            //moveRobot(500,500);
            break;
        }
        
        //deals w/ finding intersections and getting notifications.

        /* TODO: implement your application state machine.*/

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

////////////////////////////////////////////////////////////////////////
//
//  Maze solving algorithm
//

#include <stdio.h>
#include <iostream>

int Distance = 0;  // Acquired from Owen

char directionToRover(int Direction);
int getType(int left, int right, int forward);
char new_DetermineDirection(char left, char right, char forward, int xcoor, int ycoor, int Distance);
char traversedAdjacentPath(char left, char right, char forward, int xcoor, int ycoor, int intersection, int Distance, int exploredArray[], int typeArray[], int pointArray[], int point, int getXCoor[], int getYCoor[]);
char old_DetermineDirection(char left, char right, char forward, int xcoor, int ycoor, int intersection, int Distance, int exploredArray[], int typeArray[], int pointArray[], int point, int getXCoor[], int getYCoor[]);
char approachIntersection(char left, char right, char forward, int Distance);
int updateCoordinate(int xcoor, int ycoor, int point, bool update);
int updateDirection(int Direction, bool update);
int updateXCOOR(int Distance, bool update);
int updateYCOOR(int Distance, bool update);

// Function is called by Owen to return next rover direction
char approachIntersection(char left, char right, char forward, int Distance) {
	static int Total_Points = 1;
	static int intersection = 0;
	static int point = 1;
	static int xcoor = 10;
	xcoor = updateXCOOR(0, false);
	static int ycoor = 10;
	ycoor = updateYCOOR(0, false);
	int Coordinate = updateCoordinate(xcoor, ycoor, point, false);

	static int getXCoor[50];
	static int getYCoor[50];
	static int pointArray[50];
	static int typeArray[50];
	static int exploredArray[50];

	static bool initialize = true;
	while (initialize == true) {
		exploredArray[0] = 1;
		getXCoor[0] = xcoor;
		getYCoor[0] = ycoor;
		initialize = false;
	}

	intersection++;
	typeArray[intersection] = getType(left, right, forward);

	if (Coordinate > 0) {
		// Previous node identified
		pointArray[intersection] = Coordinate;
		exploredArray[Coordinate]++;
		exploredArray[intersection] = exploredArray[Coordinate] + 1;

		return old_DetermineDirection(left, right, forward, xcoor, ycoor, intersection, Distance, exploredArray, typeArray, pointArray, point, getXCoor, getYCoor);
	}
	else {
		// New node identified
		pointArray[intersection] = Total_Points;
		exploredArray[intersection]++;
		Total_Points++;

		// Save Coordinate
		updateCoordinate(xcoor, ycoor, point, true);
		//Coordinate[xcoor][ycoor] = point;
		getXCoor[point] = xcoor;
		getYCoor[point] = ycoor;
		point++;

		//Determine next direction
		return new_DetermineDirection(left, right, forward, xcoor, ycoor, Distance);
	}
}

// Analyzes previous direction and next direction in order
//	to send a turn signal that the rover can interperet.
//  Directions can be l, f , r, t
char directionToRover(int Direction) {
	static int prevDirection = 2;
	char newDirection = ' ';

	if (prevDirection == 1 && Direction == 1) {
		newDirection = 'f';
		prevDirection = 1;
	}
	else if (prevDirection == 1 && Direction == 2) {
		newDirection = 'l';
		prevDirection = 2;
	}
	else if (prevDirection == 1 && Direction == 3) {
		newDirection = 't';
		prevDirection = 3;
	}
	else if (prevDirection == 1 && Direction == 4) {
		newDirection = 'r';
		prevDirection = 4;
	}
	else if (prevDirection == 2 && Direction == 1) {
		newDirection = 'r';
		prevDirection = 1;
	}
	else if (prevDirection == 2 && Direction == 2) {
		newDirection = 'f';
		prevDirection = 2;
	}
	else if (prevDirection == 2 && Direction == 3) {
		newDirection = 'l';
		prevDirection = 3;
	}
	else if (prevDirection == 2 && Direction == 4) {
		newDirection = 't';
		prevDirection = 4;
	}
	else if (prevDirection == 3 && Direction == 1) {
		newDirection = 't';
		prevDirection = 1;
	}
	else if (prevDirection == 3 && Direction == 2) {
		newDirection = 'r';
		prevDirection = 2;
	}
	else if (prevDirection == 3 && Direction == 3) {
		newDirection = 'f';
		prevDirection = 3;
	}
	else if (prevDirection == 3 && Direction == 4) {
		newDirection = 'l';
		prevDirection = 4;
	}
	else if (prevDirection == 4 && Direction == 1) {
		newDirection = 'l';
		prevDirection = 1;
	}
	else if (prevDirection == 4 && Direction == 2) {
		newDirection = 't';
		prevDirection = 2;
	}
	else if (prevDirection == 4 && Direction == 3) {
		newDirection = 'r';
		prevDirection = 3;
	}
	else if (prevDirection == 4 && Direction == 4) {
		newDirection = 'f';
		prevDirection = 4;
	}
	return newDirection;
}

// Determines the type on intersection based on IR sensor data
int getType(int left, int right, int forward) {
	int type = 0;
	if (left) {
		type++;
	}
	if (right) {
		type++;
	}
	if (forward) {
		type++;
	}
	return type;
}

int updateDirection(int Direction, bool update) {
	static int currentDirection = 2;
	if (update == true) {
		currentDirection = Direction;
	}
	return currentDirection;
}

int updateCoordinate(int xcoor, int ycoor, int point, bool update) {
	static int Coordinate[50][50];
	if (update == true) {
		Coordinate[xcoor][ycoor] = point;
		return 0;
	}
	else if (update == false) {
		return Coordinate[xcoor][ycoor];
	}
}

int updateXCOOR(int Distance, bool update) {
	static int currentX = 10;
	if (update == true) {
		currentX += Distance;
	}
	return currentX;
}

int updateYCOOR(int Distance, bool update) {
	static int currentY = 10;
	if (update == true) {
		currentY += Distance;
	}
	return currentY;
}

char new_DetermineDirection(char left, char right, char forward, int xcoor, int ycoor, int Distance) {
	char nextDirection = ' ';
	int Direction = updateDirection(0, false);

	if (Direction == 1) {
		if (!left && !right && !forward) {
			Direction = updateDirection(3, true);
			updateXCOOR((-1 * Distance), true);
		}
		else if (forward) {
			Direction = updateDirection(1, true);
			updateXCOOR(Distance, true);
		}
		else if (left) {
			Direction = updateDirection(2, true);
			updateYCOOR((-1 * Distance), true);
		}
		else if (right) {
			Direction = updateDirection(4, true);
			updateYCOOR(Distance, true);
		}
		nextDirection = directionToRover(Direction);
	}
	else if (Direction == 2) {
		if (!left && !right && !forward) {
			Direction = updateDirection(4, true);
			updateYCOOR(Distance, true);
		}
		else if (right) {
			Direction = updateDirection(1, true);
			updateXCOOR(Distance, true);
		}
		else if (forward) {
			Direction = updateDirection(2, true);
			updateYCOOR((-1 * Distance), true);
		}
		else if (left) {
			Direction = updateDirection(3, true);
			updateXCOOR((-1 * Distance), true);
		}
		nextDirection = directionToRover(Direction);
	}
	else if (Direction == 3) {
		if (!left && !right && !forward) {
			Direction = updateDirection(1, true);
			updateXCOOR(Distance, true);
		}
		else if (right) {
			Direction = updateDirection(2, true);
			updateYCOOR((-1 * Distance), true);
		}
		else if (forward) {
			Direction = updateDirection(3, true);
			updateXCOOR((-1 * Distance), true);
		}
		else if (left) {
			Direction = updateDirection(4, true);
			updateYCOOR(Distance, true);
		}
		nextDirection = directionToRover(Direction);
	}
	else if (Direction == 4) {
		if (!left && !right && !forward) {
			Direction = updateDirection(2, true);
			updateYCOOR((-1 * Distance), true);
		}
		else if (left) {
			Direction = updateDirection(1, true);
			updateXCOOR(Distance, true);
		}
		else if (right) {
			Direction = updateDirection(3, true);
			updateXCOOR((-1 * Distance), true);
		}
		else if (forward) {
			Direction = updateDirection(4, true);
			updateYCOOR(Distance, true);
		}
		nextDirection = directionToRover(Direction);
	}
	return nextDirection;
}

// Determines what to do if all adjacent paths have already been traversed
char traversedAdjacentPath(char left, char right, char forward, int xcoor, int ycoor, int intersection, int Distance, int exploredArray[], int typeArray[], int pointArray[], int point, int getXCoor[], int getYCoor[]) {
	int stop = 1;
	int Direction = updateDirection(0, false);
	int Coordinate = updateCoordinate(xcoor, ycoor, point, false);
	static bool endMap = false;
	int isDeadEnd = getType(left, right, forward);
	if (isDeadEnd == 1) {
		return new_DetermineDirection(left, right, forward, xcoor, ycoor, Distance);
	}
	else {
		int i = 0;
		while (i < intersection) {
			stop = 1;
			if (pointArray[i] == Coordinate) {
				if (exploredArray[i - 1] < typeArray[i - 1]) {
					if (ycoor == getYCoor[pointArray[i - 1]]) {
						if (xcoor > getXCoor[pointArray[i - 1]]) {
							Direction = updateDirection(3, true);
							updateXCOOR((-1 * Distance), true);
						}
						else if (xcoor < getXCoor[pointArray[i - 1]]) {
							Direction = updateDirection(1, true);
							updateXCOOR(Distance, true);
						}
					}
					else if (xcoor == getXCoor[pointArray[i - 1]]) {
						if (ycoor > getYCoor[pointArray[i - 1]]) {
							Direction = updateDirection(2, true);
							updateYCOOR((-1 * Distance), true);
						}
						else if (ycoor < getYCoor[pointArray[i - 1]]) {
							Direction = updateDirection(4, true);
							updateYCOOR(Distance, true);
						}
					}
					stop--;
				}
				else if (exploredArray[i + 1] < typeArray[i + 1]) {
					if (ycoor == getYCoor[pointArray[i + 1]]) {
						if (xcoor > getXCoor[pointArray[i + 1]]) {
							Direction = updateDirection(3, true);
							updateXCOOR((-1 * Distance), true);
						}
						else if (xcoor < getXCoor[pointArray[i + 1]]) {
							Direction = updateDirection(1, true);
							updateXCOOR(Distance, true);
						}
					}
					else if (xcoor == getXCoor[pointArray[i + 1]]) {
						if (ycoor > getYCoor[pointArray[i + 1]]) {
							Direction = updateDirection(2, true);
							updateYCOOR((-1 * Distance), true);
						}
						else if (ycoor < getYCoor[pointArray[i + 1]]) {
							Direction = updateDirection(4, true);
							updateYCOOR(Distance, true);
						}
					}
					stop--;
				}
				else if ((exploredArray[i - 1] >= typeArray[i - 1]) && (exploredArray[i + 1] >= typeArray[i + 1])) {
					stop++;
				}
			}
			i++;
		}
	}
	if (stop == 2) {
		// DONE MAPPING!!!!
		return 'E';
	}
	else {
		return directionToRover(Direction);
	}
}

// This is the function that makes the algorith work!!!
//	Determines the direction to take after coming to an intersection
//  North=2, South=4, East=1, West=3
char old_DetermineDirection(char left, char right, char forward, int xcoor, int ycoor, int intersection, int Distance, int exploredArray[], int typeArray[], int pointArray[], int point, int getXCoor[], int getYCoor[]) {
	int Direction = updateDirection(0, false);
	bool directionNew = true;
	int compareType = 0;
	int Coordinate = updateCoordinate(xcoor, ycoor, point, false);

	if (Direction == 1) {
		if (forward) {
			directionNew = true;
			int i = 0;
			while (i < intersection) {
				if (xcoor < getXCoor[i] && ycoor == getYCoor[i]) {
					if (exploredArray[i] >= typeArray[i] && Coordinate != 0) {
						directionNew = false;
					}
				}
				i++;
			}
			if (directionNew == true) {
				Direction = updateDirection(1, true);
				updateXCOOR(Distance, true);
				return directionToRover(Direction);
			}
			else if (directionNew == false) {
				compareType++;
			}
		}
		if (left) {
			directionNew = true;
			int i = 0;
			while (i < intersection) {
				if (ycoor > getYCoor[i] && xcoor == getXCoor[i]) {
					if (exploredArray[i] >= typeArray[i] && Coordinate != 0) {
						directionNew = false;
					}
				}
				i++;
			}
			if (directionNew == true) {
				Direction = updateDirection(2, true);
				updateYCOOR((-1 * Distance), true);
				return directionToRover(Direction);
			}
			else if (directionNew == false) {
				compareType++;
			}
		}
		if (right) {
			directionNew = true;
			int i = 0;
			while (i < intersection) {
				if (ycoor < getYCoor[i] && xcoor == getXCoor[i]) {
					if (exploredArray[i] >= typeArray[i] && Coordinate != 0) {
						directionNew = false;
					}
				}
				i++;
			}
			if (directionNew == true) {
				Direction = updateDirection(4, true);
				updateYCOOR(Distance, true);
				return directionToRover(Direction);
			}
			else if (directionNew == false) {
				compareType++;
			}
		}
		if (compareType == getType(left, right, forward)) {
			return traversedAdjacentPath(left, right, forward, xcoor, ycoor, intersection, Distance, exploredArray, typeArray, pointArray, point, getXCoor, getYCoor);
		}
	}

	else if (Direction == 2) {
		if (right) {
			directionNew = true;
			int i = 0;
			while (i < intersection) {
				if (xcoor < getXCoor[i] && ycoor == getYCoor[i]) {
					if (exploredArray[i] >= typeArray[i] && Coordinate != 0) {
						directionNew = false;
					}
				}
				i++;
			}
			if (directionNew == true) {
				Direction = updateDirection(1, true);
				updateXCOOR(Distance, true);
				return directionToRover(Direction);
			}
			else if (directionNew == false) {
				compareType++;
			}
		}
		if (forward) {
			directionNew = true;
			int i = 0;
			while (i < intersection) {
				if (ycoor > getYCoor[i] && xcoor == getXCoor[i]) {
					if (exploredArray[i] >= typeArray[i] && Coordinate != 0) {
						directionNew = false;
					}
				}
				i++;
			}
			if (directionNew == true) {
				Direction = updateDirection(2, true);
				updateYCOOR((-1 * Distance), true);
				return directionToRover(Direction);
			}
			else if (directionNew == false) {
				compareType++;
			}
		}
		if (left) {
			directionNew = true;
			int i = 0;
			while (i < intersection) {
				if (xcoor > getXCoor[i] && ycoor == getYCoor[i]) {
					if (exploredArray[i] >= typeArray[i] && Coordinate != 0) {
						directionNew = false;
					}
				}
				i++;
			}
			if (directionNew == true) {
				Direction = updateDirection(3, true);
				updateXCOOR((-1 * Distance), true);
				return directionToRover(Direction);
			}
			else if (directionNew == false) {
				compareType++;
			}
		}
		if (compareType == getType(left, right, forward)) {
			return traversedAdjacentPath(left, right, forward, xcoor, ycoor, intersection, Distance, exploredArray, typeArray, pointArray, point, getXCoor, getYCoor);
		}
	}
	else if (Direction == 3) {
		if (right) {
			directionNew = true;
			int i = 0;
			while (i < intersection) {
				if (ycoor > getYCoor[i] && xcoor == getXCoor[i]) {
					if (exploredArray[i] > 0) {
						directionNew = false;
					}
				}
				i++;
			}
			if (directionNew == true) {
				Direction = updateDirection(2, true);
				updateYCOOR((-1 * Distance), true);
				return directionToRover(Direction);
			}
			else if (directionNew == false) {
				compareType++;
			}
		}
		if (forward) {
			directionNew = true;
			int i = 0;
			while (i < intersection) {
				if (xcoor > getXCoor[i] && ycoor == getYCoor[i]) {
					if (exploredArray[i] > 0 && typeArray[i] != 0) {
						directionNew = false;
					}
				}
				i++;
			}
			if (directionNew == true) {
				Direction = updateDirection(3, true);
				updateXCOOR((-1 * Distance), true);
				return directionToRover(Direction);
			}
			else if (directionNew == false) {
				compareType++;
			}
		}
		if (left) {
			directionNew = true;
			int i = 0;
			while (i < intersection) {
				if (ycoor < getYCoor[i] && xcoor == getXCoor[i]) {
					if (exploredArray[i] > 0) {
						directionNew = false;
					}
				}
				i++;
			}
			if (directionNew == true) {
				Direction = updateDirection(4, true);
				updateYCOOR(Distance, true);
				return directionToRover(Direction);
			}
			else if (directionNew == false) {
				compareType++;
			}
		}
		if (compareType == getType(left, right, forward)) {
			return traversedAdjacentPath(left, right, forward, xcoor, ycoor, intersection, Distance, exploredArray, typeArray, pointArray, point, getXCoor, getYCoor);
		}
	}
	else if (Direction == 4) {
		if (left) {
			directionNew = true;
			int i = 0;
			while (i < intersection) {
				if (xcoor < getXCoor[i] && ycoor == getYCoor[i]) {
					if (exploredArray[i] > 0) {
						directionNew = false;
					}
				}
				i++;
			}
			if (directionNew == true) {
				Direction = updateDirection(1, true);
				updateXCOOR(Distance, true);
				return directionToRover(Direction);
			}
			else if (directionNew == false) {
				compareType++;
			}
		}
		if (right) {
			directionNew = true;
			int i = 0;
			while (i < intersection) {
				if (xcoor > getXCoor[i] && ycoor == getYCoor[i]) {
					if (exploredArray[i] > 0) {
						directionNew = false;
					}
				}
				i++;
			}
			if (directionNew == true) {
				Direction = updateDirection(3, true);
				updateXCOOR((-1 * Distance), true);
				return directionToRover(Direction);
			}
			else if (directionNew == false) {
				compareType++;
			}
		}
		if (forward) {
			directionNew = true;
			int i = 0;
			while (i < intersection) {
				if (ycoor < getYCoor[i] && xcoor == getXCoor[i]) {
					if (exploredArray[i] > 0) {
						directionNew = false;
					}
				}
				i++;
			}
			if (directionNew == true) {
				Direction = updateDirection(4, true);
				updateYCOOR(Distance, true);
				return directionToRover(Direction);
			}
			else if (directionNew == false) {
				compareType++;
			}
		}
		if (compareType == getType(left, right, forward)) {
			return traversedAdjacentPath(left, right, forward, xcoor, ycoor, intersection, Distance, exploredArray, typeArray, pointArray, point, getXCoor, getYCoor);
		}
	}
}