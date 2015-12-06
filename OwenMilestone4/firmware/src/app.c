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
 

void getDataIR() {
	// North=2, South=4, East=1, West=3
	char dataIR = '0';
	// Read input from ir sensors here
	// Need right, left, and front data
}

// Analyzes previous direction and next direction in order
//	to send a turn signal that the rover can interperet
char directionToRover() {
	static int prevDirection = 2; // Directions can be l, f , r, t
	char newDirection;
    if (prevDirection == 1 && Direction == 1) {
		xcoor -= currentDistance;
        newDirection = 'f';
		prevDirection = 1;
	}
	else if (prevDirection == 1 && Direction == 2) {
		ycoor -= currentDistance;
		newDirection = 'l';
		prevDirection = 2;
	}
	else if (prevDirection == 1 && Direction == 3) {
		xcoor -= currentDistance;
		newDirection = 't';
		prevDirection = 3;
	}
	else if (prevDirection == 1 && Direction == 4) {
		ycoor += currentDistance;
		newDirection = 'r';
		prevDirection = 4;
	}
	else if (prevDirection == 2 && Direction == 1) {
		xcoor += currentDistance;
		newDirection = 'r';
		prevDirection = 1;
	}
	else if (prevDirection == 2 && Direction == 2) {
		ycoor -= currentDistance;
        newDirection = 'f';
		prevDirection = 2;
	}
	else if (prevDirection == 2 && Direction == 3) {
		xcoor -= currentDistance;
		newDirection = 'l';
		prevDirection = 3;
	}
	else if (prevDirection == 2 && Direction == 4) {
		ycoor += currentDistance;
		newDirection = 't';
		prevDirection = 4;
	}
	else if (prevDirection == 3 && Direction == 1) {
		xcoor += currentDistance;
		newDirection = 't';
		prevDirection = 1;
	}
	else if (prevDirection == 3 && Direction == 2) {
		ycoor -= currentDistance;
        newDirection = 'r';
		prevDirection = 2;
	}
	else if (prevDirection == 3 && Direction == 3) {
		xcoor -= currentDistance;
		newDirection = 'f';
		prevDirection = 3;
	}
	else if (prevDirection == 3 && Direction == 4) {
		ycoor += currentDistance;
		newDirection = 'l';
		prevDirection = 4;
	}
	else if (prevDirection == 4 && Direction == 1) {
		xcoor += currentDistance;
		newDirection = 'l';
		prevDirection = 1;
	}
	else if (prevDirection == 4 && Direction == 2) {
		ycoor -= currentDistance;
		newDirection = 't';
		prevDirection = 2;
	}
	else if (prevDirection == 4 && Direction == 3) {
		xcoor -= currentDistance;
		newDirection = 'r';
		prevDirection = 3;
	}
	else if (prevDirection == 4 && Direction == 4) {
		ycoor += currentDistance;
		newDirection = 'f';
		prevDirection = 4;
	}
}

// Updated multidimensional arrays that store coordinate data
//	- Assigns coordinates to a node
//	- Assigns node to x and y coordinates
void saveCoordinate() {
	// X coordinate of point is found in 0
	// Y coordinate of point is found in 1
	static int point = 0;
	Coordinate[xcoor][ycoor] = point;
	getCoor[point][0] = xcoor;
	getCoor[point][1] = ycoor;
	point++;
}

// Determines the type on intersection based on IR sensor data
int getType(char left, char right, char forward) {
	int type = 0;
	if (left) {
		type++;
	}
	if (forward) {
		type++;
	}
	if (right) {
		type++;
	}
	return type;
}

char new_DetermineDirection(char left, char right, char forward) {
	char toRet;
    if (Direction == 1) {
		if (!left && !right && !forward) {
			Direction = 3;
		}
		else if (forward) {
			Direction = 1;
		}
		else if (left) {
			Direction = 2;
		}
		else if (right) {
			Direction = 4;
		}
		toRet = directionToRover();
	}
	else if (Direction == 2) {
		if (!left && !right && !forward) {
			Direction = 4;
		}
		else if (right) {
			Direction = 1;
		}
		else if (forward) {
			Direction = 2;
		}
		else if (left) {
			Direction = 3;
		}
		toRet = directionToRover();
	}
	else if (Direction == 3) {
		if (!left && !right && !forward) {
			Direction = 1;
		}
		else if (right) {
			Direction = 2;
		}
		else if (forward) {
			Direction = 3;
		}
		else if (left) {
			Direction = 4;
		}
		toRet = directionToRover();
	}
	else if (Direction == 4) {
		if (!left && !right && !forward) {
			Direction = 2;
		}
		else if (left) {
			Direction = 1;
		}
		else if (right) {
			Direction = 3;
		}
		else if (forward) {
			Direction = 4;
		}
		toRet = directionToRover();
	}
    return;
}

// Determines what to do if all adjacent paths have already been traversed
void traversedAdjacentPath(char left, char right, char forward) {
	int stop = 1;
	static bool endMap = false;
	int isDeadEnd = getType(left, right, forward);
	if (isDeadEnd == 1) {
		new_DetermineDirection(left, right, forward);
	}
	else {
        int i = 0;
		while( i <= intersection ) {
			stop = 1;
			if (pointArray[i] == Coordinate[xcoor][ycoor]) {
				if (exploredArray[i - 1] < typeArray[i - 1]) {
					// Determine Direction
					if (ycoor == getCoor[pointArray[i - 1]][1]) {
						if (xcoor > getCoor[pointArray[i - 1]][0]) {
							Direction = 3;
						}
						else if (xcoor < getCoor[pointArray[i - 1]][0]) {
							Direction = 1;
						}
					}
					else if (xcoor == getCoor[pointArray[i - 1]][0]) {
						if (ycoor > getCoor[pointArray[i - 1]][1]) {
							Direction = 2;
						}
						else if (ycoor < getCoor[pointArray[i - 1]][1]) {
							Direction = 4;
						}
					}
					stop--;
				}
				else if (exploredArray[i + 1] < typeArray[i + 1]) {
					// Determine Direction
					if (ycoor == getCoor[pointArray[i + 1]][1]) {
						if (xcoor > getCoor[pointArray[i + 1]][0]) {
							Direction = 3;
						}
						else if (xcoor < getCoor[pointArray[i + 1]][0]) {
							Direction = 1;
						}
					}
					else if (xcoor == getCoor[pointArray[i + 1]][0]) {
						if (ycoor > getCoor[pointArray[i + 1]][1]) {
							Direction = 2;
						}
						else if (ycoor < getCoor[pointArray[i + 1]][1]) {
							Direction = 4;
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
	/*if (stop == 2) {
        return 'E';
	}
	else {
		printf("NEW DIRECTION: %d\n", Direction);
		directionToRover();
	}*/
}

// This is the function that makes the algorith work!!!
//	Determines the direction to take after coming to an intersection
char old_DetermineDirection(char left, char right, char forward) {
	// North=2, South=4, East=1, West=3
	bool directionNew = true; // place at top
	int compareType = 0; // place at top

    char toRet;
	if (Direction == 1) {
		if (forward) {
			directionNew = true;
			int i = 0;
            while( i <= intersection ) {
				if (xcoor < getCoor[i][0] && ycoor == getCoor[i][1]) {
					if (exploredArray[i] >= typeArray[i] && Coordinate[xcoor][ycoor] != 0) {
						directionNew = false;
					}
				}
                i++;
			}
			if (directionNew == true) {
				Direction = 1;
				toRet = directionToRover();
				return;
			}
			else if (directionNew == false) {
				compareType++;
			}
		}
		if (left) {
			directionNew = true;
			int i = 0;
            while( i <= intersection ) {
				if (ycoor > getCoor[i][1] && xcoor == getCoor[i][0]) {
					if (exploredArray[i] >= typeArray[i] && Coordinate[xcoor][ycoor] != 0) {
						directionNew = false;
					}
				}
                i++;
			}
			if (directionNew == true) {
				Direction = 2;
				toRet = directionToRover();
				return;
			}
			else if (directionNew == false) {
				compareType++;
			}
		}
		if (right) {
			directionNew = true;
			int i = 0;
            while( i <= intersection ) {
				if (ycoor < getCoor[i][1] && xcoor == getCoor[i][0]) {
					if (exploredArray[i] >= typeArray[i] && Coordinate[xcoor][ycoor] != 0) {
						directionNew = false;
					}
				}
                i++;
			}
			if (directionNew == true) {
				Direction = 4;
				toRet = directionToRover();
				return;
			}
			else if (directionNew == false) {
				compareType++;
			}
		}
		if (compareType == getType(left, right, forward)) {
			traversedAdjacentPath(left, right, forward);
		}
	}
	else if (Direction == 2) {
		if (right) {
			directionNew = true;
			int i = 0;
            while( i <= intersection ) {
				if (xcoor < getCoor[i][0] && ycoor == getCoor[i][1]) {
					if (exploredArray[i] >= typeArray[i] && Coordinate[xcoor][ycoor] != 0) {
						directionNew = false;
					}
				}
                i++;
            }
			if (directionNew == true) {
				Direction = 1;
				toRet = directionToRover();
				return;
			}
			else if (directionNew == false) {
				compareType++;
			}
		}
		if (forward) {
			directionNew = true;
			int i = 0;
            while( i <= intersection ) {
				if (ycoor > getCoor[i][1] && xcoor == getCoor[i][0]) {
					if (exploredArray[i] >= typeArray[i] && Coordinate[xcoor][ycoor] != 0) {
						directionNew = false;
					}
				}
                i++;
            }
			if (directionNew == true) {
				Direction = 2;
				toRet = directionToRover();
				return;
			}
			else if (directionNew == false) {
				compareType++;
			}
		}
		if (left) {
			directionNew = true;
			int i = 0;
            while( i <= intersection ) {
				if (xcoor > getCoor[i][0] && ycoor == getCoor[i][1]) {
					if (exploredArray[i] >= typeArray[i] && Coordinate[xcoor][ycoor] != 0) {
						directionNew = false;
					}
				}
                i++;
            }
			if (directionNew == true) {
				Direction = 3;
				toRet = directionToRover();
				return;
			}
			else if (directionNew == false) {
				compareType++;
			}
		}
		if (compareType == getType(left, right, forward)) {
			traversedAdjacentPath(left, right, forward);
		}
	}
	else if (Direction == 3) {
		if (right) {
			directionNew = true;
			int i = 0;
            while( i <= intersection ) {
				if (ycoor > getCoor[i][1] && xcoor == getCoor[i][0]) {
					if (exploredArray[i] > 0) {
						directionNew = false;
					}
				}
                i++;
            }
			if (directionNew == true) {
				Direction = 2;
				toRet = directionToRover();
				return;
			}
			else if (directionNew == false) {
				compareType++;
			}
		}
		if (forward) {
			directionNew = true;
			int i = 0;
            while( i <= intersection ) {
				if (xcoor > getCoor[i][0] && ycoor == getCoor[i][1]) {
					if (exploredArray[i] > 0 && typeArray[i] != 0) {
						directionNew = false;
					}
				}
                i++;
            }
			if (directionNew == true) {
				Direction = 3;
				toRet = directionToRover();
				return;
			}
			else if (directionNew == false) {
				compareType++;
			}
		}
		if (left) {
			directionNew = true;
			int i = 0;
            while( i <= intersection ) {
				if (ycoor < getCoor[i][1] && xcoor == getCoor[i][0]) {
					if (exploredArray[i] > 0) {
						directionNew = false;
					}
				}
                i++;
            }
			if (directionNew == true) {
				Direction = 4;
				toRet = directionToRover();
				return;
			}
			else if (directionNew == false) {
				compareType++;
			}
		}
		if (compareType == getType(left, right, forward)) {
			traversedAdjacentPath(left, right, forward);
		}
	}
	else if (Direction == 4) {
		if (left) {
			directionNew = true;
			int i = 0;
            while( i <= intersection ) {
				if (xcoor < getCoor[i][0] && ycoor == getCoor[i][1]) {
					if (exploredArray[i] > 0) {
						directionNew = false;
					}
				}
                i++;
            }
			if (directionNew == true) {
				Direction = 1;
				toRet = directionToRover();
				return;
			}
			else if (directionNew == false) {
				compareType++;
			}
		}
		if (right) {
			directionNew = true;
			int i = 0;
            while( i <= intersection ) {
				if (xcoor > getCoor[i][0] && ycoor == getCoor[i][1]) {
					if (exploredArray[i] > 0) {
						directionNew = false;
					}
				}
                i++;
            }
			if (directionNew == true) {
				Direction = 3;
				toRet = directionToRover();
				return;
			}
			else if (directionNew == false) {
				compareType++;
			}
		}
		if (forward) {
			directionNew = true;
			int i = 0;
            while( i <= intersection ) {
				if (ycoor < getCoor[i][1] && xcoor == getCoor[i][0]) {
					if (exploredArray[i] > 0) {
						directionNew = false;
					}
				}
                i++;
            }
			if (directionNew == true) {
				Direction = 4;
				toRet = directionToRover();
				return;
			}
			else if (directionNew == false) {
				compareType++;
			}
		}
		if (compareType == getType(left, right, forward)) {
			traversedAdjacentPath(left, right, forward);
		}
	}
    return toRet;
}

////////////////////////////////////////////////////////////////////
// THE WORK HORSES
//
void getBearing() {
	// Rover begins at a starting point '0' and sets initial values
	pointArray[0] = 0;
	exploredArray[0]++;
	saveCoordinate();
    xcoor = 10;
    ycoor = 10;
    Direction = 2;  // Rover always starts going North  
    intersection = 0;   
    nodeDistance = 0;
}

// Call this function when an intersection is seen!!!
char approachIntersection(char left, char right, char forward, char distance) {
	// ^^ Need to read from front pixel to notify rover of dead ends in original algorithm
	// This could be where the rover sees intersection and then moves forward a set number of places
	// If all IR sensors equal 0 (see white) then the rover knows it is at a dead end.

    getBearing();
    
	static int Total_Points = 1;

    currentDistance = distance;

    
	intersection++;
	typeArray[intersection] = getType(left, right, forward);

    char toRet;
    
	// Assuming that rover will never go back to the starting point
	if (Coordinate[xcoor][ycoor] > 0) {
		// Previous node identified (OLD POINT)
		pointArray[intersection] = Coordinate[xcoor][ycoor];
		exploredArray[Coordinate[xcoor][ycoor]]++;
		exploredArray[intersection] = exploredArray[Coordinate[xcoor][ycoor]] + 1;
		toRet = old_DetermineDirection(left, right, forward);
	}
	else {
		// New node identified
		pointArray[intersection] = Total_Points;
		exploredArray[intersection]++;
		Total_Points++;

		saveCoordinate();
		toRet = new_DetermineDirection(left, right, forward);
	}
    return toRet;
}


/*******************************************************************************
 End of File
 */
