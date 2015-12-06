#include <stdio.h>
#include <iostream>

// Important variables include:
//		- Right, Front, Left sensor data
//		- Distance from one node to the next

// Global Variables //
#define MAX_NODES	50

int xcoor = 6;
int ycoor = 30;
int Direction = 2;  // Rover always starts going North
int intersection = 0;
int nodeDistance = 0;

int Coordinate[MAX_NODES][MAX_NODES];
int getCoor[MAX_NODES][MAX_NODES];
int pointArray[MAX_NODES];
int typeArray[MAX_NODES];
int exploredArray[MAX_NODES];

// End Global Variables //

// Reads data from IR sensors on rover
void getDataIR() {
	// North=2, South=4, East=1, West=3
	char dataIR = '0';
	// Read input from ir sensors here
	// Need right, left, and front data
}

// Analyzes previous direction and next direction in order
//	to send a turn signal that the rover can interperet
void directionToRover() {
	static int prevDirection = 2; // Directions can be l, f , r, t
	if (prevDirection == 1 && Direction == 1) {
		printf("ROVER DIRECTION IS: f\n");
		printf("SENDING PACKET TO TYLER: %d %d %d %d %c\n", nodeDistance, dataIR.left.Red, dataIR.front.Red, dataIR.right.Red, 'f');
		nodeDistance = 0;
		prevDirection = 1;
	}
	else if (prevDirection == 1 && Direction == 2) {
		printf("ROVER DIRECTION IS: l\n");
		printf("SENDING PACKET TO TYLER: %d %d %d %d %c\n", nodeDistance, dataIR.left.Red, dataIR.front.Red, dataIR.right.Red, 'l');
		nodeDistance = 0;
		prevDirection = 2;
	}
	else if (prevDirection == 1 && Direction == 3) {
		printf("ROVER DIRECTION IS: t\n");
		printf("SENDING PACKET TO TYLER: %d %d %d %d %c\n", nodeDistance, dataIR.left.Red, dataIR.front.Red, dataIR.right.Red, 'r');
		nodeDistance = 0;
		prevDirection = 3;
	}
	else if (prevDirection == 1 && Direction == 4) {
		printf("ROVER DIRECTION IS: r\n");
		printf("SENDING PACKET TO TYLER: %d %d %d %d %c\n", nodeDistance, dataIR.left.Red, dataIR.front.Red, dataIR.right.Red, 'r');
		nodeDistance = 0;
		prevDirection = 4;
	}
	else if (prevDirection == 2 && Direction == 1) {
		printf("ROVER DIRECTION IS: r\n");
		printf("SENDING PACKET TO TYLER: %d %d %d %d %c\n", nodeDistance, dataIR.left.Red, dataIR.front.Red, dataIR.right.Red, 'r');
		nodeDistance = 0;
		prevDirection = 1;
	}
	else if (prevDirection == 2 && Direction == 2) {
		printf("ROVER DIRECTION IS: f\n");
		printf("SENDING PACKET TO TYLER: %d %d %d %d %c\n", nodeDistance, dataIR.left.Red, dataIR.front.Red, dataIR.right.Red, 'f');
		nodeDistance = 0;
		prevDirection = 2;
	}
	else if (prevDirection == 2 && Direction == 3) {
		printf("ROVER DIRECTION IS: l\n");
		printf("SENDING PACKET TO TYLER: %d %d %d %d %c\n", nodeDistance, dataIR.left.Red, dataIR.front.Red, dataIR.right.Red, 'l');
		nodeDistance = 0;
		prevDirection = 3;
	}
	else if (prevDirection == 2 && Direction == 4) {
		printf("ROVER DIRECTION IS: t\n");
		printf("SENDING PACKET TO TYLER: %d %d %d %d %c\n", nodeDistance, dataIR.left.Red, dataIR.front.Red, dataIR.right.Red, 'r');
		nodeDistance = 0;
		prevDirection = 4;
	}
	else if (prevDirection == 3 && Direction == 1) {
		printf("ROVER DIRECTION IS: t\n");
		printf("SENDING PACKET TO TYLER: %d %d %d %d %c\n", nodeDistance, dataIR.left.Red, dataIR.front.Red, dataIR.right.Red, 'r');
		nodeDistance = 0;
		prevDirection = 1;
	}
	else if (prevDirection == 3 && Direction == 2) {
		printf("ROVER DIRECTION IS: r\n");
		printf("SENDING PACKET TO TYLER: %d %d %d %d %c\n", nodeDistance, dataIR.left.Red, dataIR.front.Red, dataIR.right.Red, 'r');
		nodeDistance = 0;
		prevDirection = 2;
	}
	else if (prevDirection == 3 && Direction == 3) {
		printf("ROVER DIRECTION IS: f\n");
		printf("SENDING PACKET TO TYLER: %d %d %d %d %c\n", nodeDistance, dataIR.left.Red, dataIR.front.Red, dataIR.right.Red, 'f');
		nodeDistance = 0;
		prevDirection = 3;
	}
	else if (prevDirection == 3 && Direction == 4) {
		printf("ROVER DIRECTION IS: l\n");
		printf("SENDING PACKET TO TYLER: %d %d %d %d %c\n", nodeDistance, dataIR.left.Red, dataIR.front.Red, dataIR.right.Red, 'l');
		nodeDistance = 0;
		prevDirection = 4;
	}
	else if (prevDirection == 4 && Direction == 1) {
		printf("ROVER DIRECTION IS: l\n");
		printf("SENDING PACKET TO TYLER: %d %d %d %d %c\n", nodeDistance, dataIR.left.Red, dataIR.front.Red, dataIR.right.Red, 'l');
		nodeDistance = 0;
		prevDirection = 1;
	}
	else if (prevDirection == 4 && Direction == 2) {
		printf("ROVER DIRECTION IS: t\n");
		printf("SENDING PACKET TO TYLER: %d %d %d %d %c\n", nodeDistance, dataIR.left.Red, dataIR.front.Red, dataIR.right.Red, 't');
		nodeDistance = 0;
		prevDirection = 2;
	}
	else if (prevDirection == 4 && Direction == 3) {
		printf("ROVER DIRECTION IS: r\n");
		printf("SENDING PACKET TO TYLER: %d %d %d %d %c\n", nodeDistance, dataIR.left.Red, dataIR.front.Red, dataIR.right.Red, 'r');
		nodeDistance = 0;
		prevDirection = 3;
	}
	else if (prevDirection == 4 && Direction == 4) {
		printf("ROVER DIRECTION IS: f\n");
		printf("SENDING PACKET TO TYLER: %d %d %d %d %c\n", nodeDistance, dataIR.left.Red, dataIR.front.Red, dataIR.right.Red, 'f');
		nodeDistance = 0;
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
int getType() {
	int type = 0;
	if ((int)dataIR.left.Red != 255) {
		type++;
	}
	if ((int)dataIR.right.Red != 255) {
		type++;
	}
	if ((int)dataIR.front.Red != 255) {
		type++;
	}
	return type;
}

void new_DetermineDirection() {
	printf("PATH THAT IS NOT TRAVERSED\n");
	if (Direction == 1) {
		if ((int)dataIR.left.Red == 255 && (int)dataIR.right.Red == 255 && (int)dataIR.front.Red == 255) {
			// Dead end, turn around!
			printf("DEAD END\n");
			Direction = 3;
			xcoor--;
		}
		else if (dataIR.front.Red != 255) {
			Direction = 1;
			xcoor++;
		}
		else if (dataIR.left.Red != 255) {
			Direction = 2;
			ycoor--;
		}
		else if (dataIR.right.Red != 255) {
			Direction = 4;
			ycoor++;
		}
		printf("NEW DIRECTION: %d\n", Direction);
		directionToRover();
	}
	else if (Direction == 2) {
		if ((int)dataIR.left.Red == 255 && (int)dataIR.right.Red == 255 && (int)dataIR.front.Red == 255) {
			// Dead end, turn around!
			printf("DEAD END\n");
			Direction = 4;
			ycoor++;
		}
		else if ((int)dataIR.right.Red != 255) {
			Direction = 1;
			xcoor++;
		}
		else if ((int)dataIR.front.Red != 255) {
			Direction = 2;
			ycoor--;
		}
		else if ((int)dataIR.left.Red != 255) {
			Direction = 3;
			xcoor--;
		}
		printf("NEW DIRECTION: %d\n", Direction);
		directionToRover();
	}
	else if (Direction == 3) {
		if ((int)dataIR.left.Red == 255 && (int)dataIR.right.Red == 255 && (int)dataIR.front.Red == 255) {
			// Dead end, turn around!
			printf("DEAD END\n");
			Direction = 1;
			xcoor++;
		}
		else if (dataIR.right.Red != 255) {
			Direction = 2;
			ycoor--;
		}
		else if (dataIR.front.Red != 255) {
			Direction = 3;
			xcoor--;
		}
		else if (dataIR.left.Red != 255) {
			Direction = 4;
			ycoor++;
		}
		printf("NEW DIRECTION: %d\n", Direction);
		directionToRover();
	}
	else if (Direction == 4) {
		if ((int)dataIR.left.Red == 255 && (int)dataIR.right.Red == 255 && (int)dataIR.front.Red == 255) {
			// Dead end, turn around!
			printf("DEAD END\n");
			Direction = 2;
			ycoor--;
		}
		else if (dataIR.left.Red != 255) {
			Direction = 1;
			xcoor++;
		}
		else if (dataIR.right.Red != 255) {
			Direction = 3;
			xcoor--;
		}
		else if (dataIR.front.Red != 255) {
			Direction = 4;
			ycoor++;
		}
		printf("NEW DIRECTION: %d\n", Direction);
		directionToRover();
	}
}

// Determines what to do if all adjacent paths have already been traversed
void traversedAdjacentPath() {
	// All adjacent paths have already been traversed, look for nodes that still need to be visited!
	printf("ALL PATHS TRAVERSED\n");
	int stop = 1;
	static bool endMap = false;
	int isDeadEnd = getType();
	if (isDeadEnd == 1) {
		new_DetermineDirection();
	}
	else {
		for (int i = 0; i <= intersection; i++) {
			stop = 1;
			if (pointArray[i] == Coordinate[xcoor][ycoor]) {
				printf("COMPARING: %d TO %d\n", Coordinate[xcoor][ycoor], pointArray[i]);
				if (exploredArray[i - 1] < typeArray[i - 1]) {
					printf("FOUND NODE THAT STILL NEEDS TO BE EXPLORED. NODE IS: %d\n", pointArray[i - 1]);
					printf("XCOOR: %d\n", xcoor);
					printf("YCOOR: %d\n", ycoor);
					printf("GETXCOOR: %d\n", getCoor[pointArray[i - 1]][0]);
					printf("GETYCOOR: %d\n", getCoor[pointArray[i - 1]][1]);
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
					travel();
					stop--;
				}
				else if (exploredArray[i + 1] < typeArray[i + 1]) {
					printf("FOUND NODE THAT STILL NEEDS TO BE EXPLORED. NODE IS: %d\n", pointArray[i + 1]);
					printf("XCOOR: %d\n", xcoor);
					printf("YCOOR: %d\n", ycoor);
					printf("GETXCOOR: %d\n", getCoor[pointArray[i + 1]][0]);
					printf("GETYCOOR: %d\n", getCoor[pointArray[i + 1]][1]);
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
					travel();
					stop--;
				}
				else if ((exploredArray[i - 1] >= typeArray[i - 1]) && (exploredArray[i + 1] >= typeArray[i + 1])) {
					stop++;
				}
			}
		}
	}
	// Sketchy code for sketchy mazes
	/*if (isDeadEnd == 3) {
	// Backtrack just in case
	// uhh
	if (Direction = 1) {
	Direction = 3;
	}
	else if (Direction = 2) {
	Direction = 4;
	}
	else {
	Direction = Direction - 2;
	}
	travel();
	}
	else */
	if (stop == 2) {
		// DONE MAPPING!!!!
		printf("ROVER DIRECTION IS: e\n");
	}
	else {
		printf("NEW DIRECTION: %d\n", Direction);
		directionToRover();
	}
}

// This is the function that makes the algorith work!!!
//	Determines the direction to take after coming to an intersection
void old_DetermineDirection() {
	// North=2, South=4, East=1, West=3
	bool directionNew = true; // place at top
	int compareType = 0; // place at top

	if (Direction == 1) {
		if ((int)dataIR.front.Red != 255) {
			directionNew = true;
			for (int i = 0; i <= intersection; i++) {
				if (xcoor < getCoor[i][0] && ycoor == getCoor[i][1]) {
					if (exploredArray[i] >= typeArray[i] && Coordinate[xcoor][ycoor] != 0) {
						directionNew = false;
					}
				}
			}
			if (directionNew == true) {
				Direction = 1;
				xcoor++;
				printf("NEW DIRECTION: %d\n", Direction);
				directionToRover();
				return;
			}
			else if (directionNew == false) {
				compareType++;
			}
		}
		if ((int)dataIR.left.Red != 255) {
			directionNew = true;
			for (int i = 0; i <= intersection; i++) {
				if (ycoor > getCoor[i][1] && xcoor == getCoor[i][0]) {
					if (exploredArray[i] >= typeArray[i] && Coordinate[xcoor][ycoor] != 0) {
						directionNew = false;
					}
				}
			}
			if (directionNew == true) {
				Direction = 2;
				ycoor--;
				printf("NEW DIRECTION: %d\n", Direction);
				directionToRover();
				return;
			}
			else if (directionNew == false) {
				compareType++;
			}
		}
		if ((int)dataIR.right.Red != 255) {
			directionNew = true;
			for (int i = 0; i <= intersection; i++) {
				if (ycoor < getCoor[i][1] && xcoor == getCoor[i][0]) {
					if (exploredArray[i] >= typeArray[i] && Coordinate[xcoor][ycoor] != 0) {
						directionNew = false;
					}
				}
			}
			if (directionNew == true) {
				Direction = 4;
				ycoor++;
				printf("NEW DIRECTION: %d\n", Direction);
				directionToRover();
				return;
			}
			else if (directionNew == false) {
				compareType++;
			}
		}
		if (compareType == getType()) {
			traversedAdjacentPath();
		}
	}
	else if (Direction == 2) {
		if ((int)dataIR.right.Red != 255) {
			directionNew = true;
			for (int i = 0; i <= intersection; i++) {
				if (xcoor < getCoor[i][0] && ycoor == getCoor[i][1]) {
					if (exploredArray[i] >= typeArray[i] && Coordinate[xcoor][ycoor] != 0) {
						directionNew = false;
					}
				}
			}
			if (directionNew == true) {
				Direction = 1;
				xcoor++;
				printf("NEW DIRECTION: %d\n", Direction);
				directionToRover();
				return;
			}
			else if (directionNew == false) {
				compareType++;
			}
		}
		if ((int)dataIR.front.Red != 255) {
			directionNew = true;
			for (int i = 0; i <= intersection; i++) {
				if (ycoor > getCoor[i][1] && xcoor == getCoor[i][0]) {
					if (exploredArray[i] >= typeArray[i] && Coordinate[xcoor][ycoor] != 0) {
						directionNew = false;
					}
				}
			}
			if (directionNew == true) {
				Direction = 2;
				ycoor--;
				printf("NEW DIRECTION: %d\n", Direction);
				directionToRover();
				return;
			}
			else if (directionNew == false) {
				compareType++;
			}
		}
		if ((int)dataIR.left.Red != 255) {
			directionNew = true;
			for (int i = 0; i <= intersection; i++) {
				if (xcoor > getCoor[i][0] && ycoor == getCoor[i][1]) {
					if (exploredArray[i] >= typeArray[i] && Coordinate[xcoor][ycoor] != 0) {
						directionNew = false;
					}
				}
			}
			if (directionNew == true) {
				Direction = 3;
				xcoor--;
				printf("NEW DIRECTION: %d\n", Direction);
				directionToRover();
				return;
			}
			else if (directionNew == false) {
				compareType++;
			}
		}
		if (compareType == getType()) {
			traversedAdjacentPath();
		}
	}
	else if (Direction == 3) {
		if ((int)dataIR.right.Red != 255) {
			directionNew = true;
			for (int i = 0; i <= intersection; i++) {
				if (ycoor > getCoor[i][1] && xcoor == getCoor[i][0]) {
					if (exploredArray[i] > 0) {
						directionNew = false;
					}
				}
			}
			if (directionNew == true) {
				Direction = 2;
				ycoor--;
				printf("NEW DIRECTION: %d\n", Direction);
				directionToRover();
				return;
			}
			else if (directionNew == false) {
				compareType++;
			}
		}
		if ((int)dataIR.front.Red != 255) {
			directionNew = true;
			for (int i = 0; i <= intersection; i++) {
				if (xcoor > getCoor[i][0] && ycoor == getCoor[i][1]) {
					if (exploredArray[i] > 0 && typeArray[i] != 0) {
						directionNew = false;
					}
				}
			}
			if (directionNew == true) {
				Direction = 3;
				xcoor--;
				printf("NEW DIRECTION: %d\n", Direction);
				directionToRover();
				return;
			}
			else if (directionNew == false) {
				compareType++;
			}
		}
		if ((int)dataIR.left.Red != 255) {
			directionNew = true;
			for (int i = 0; i <= intersection; i++) {
				if (ycoor < getCoor[i][1] && xcoor == getCoor[i][0]) {
					if (exploredArray[i] > 0) {
						directionNew = false;
					}
				}
			}
			if (directionNew == true) {
				Direction = 4;
				ycoor++;
				printf("NEW DIRECTION: %d\n", Direction);
				directionToRover();
				return;
			}
			else if (directionNew == false) {
				compareType++;
			}
		}
		if (compareType == getType()) {
			traversedAdjacentPath();
		}
	}
	else if (Direction == 4) {
		if ((int)dataIR.left.Red != 255) {
			directionNew = true;
			for (int i = 0; i <= intersection; i++) {
				if (xcoor < getCoor[i][0] && ycoor == getCoor[i][1]) {
					if (exploredArray[i] > 0) {
						directionNew = false;
					}
				}
			}
			if (directionNew == true) {
				Direction = 1;
				xcoor++;
				printf("NEW DIRECTION: %d\n", Direction);
				directionToRover();
				return;
			}
			else if (directionNew == false) {
				compareType++;
			}
		}
		if ((int)dataIR.right.Red != 255) {
			directionNew = true;
			for (int i = 0; i <= intersection; i++) {
				if (xcoor > getCoor[i][0] && ycoor == getCoor[i][1]) {
					if (exploredArray[i] > 0) {
						directionNew = false;
					}
				}
			}
			if (directionNew == true) {
				Direction = 3;
				xcoor--;
				printf("NEW DIRECTION: %d\n", Direction);
				directionToRover();
				return;
			}
			else if (directionNew == false) {
				compareType++;
			}
		}
		if ((int)dataIR.front.Red != 255) {
			directionNew = true;
			for (int i = 0; i <= intersection; i++) {
				if (ycoor < getCoor[i][1] && xcoor == getCoor[i][0]) {
					if (exploredArray[i] > 0) {
						directionNew = false;
					}
				}
			}
			if (directionNew == true) {
				Direction = 4;
				ycoor++;
				printf("NEW DIRECTION: %d\n", Direction);
				directionToRover();
				return;
			}
			else if (directionNew == false) {
				compareType++;
			}
		}
		if (compareType == getType()) {
			traversedAdjacentPath();
		}
	}
}


////////////////////////////////////////////////////////////////////
// THE WORK HORSES
//
void getBearing() {
	// Rover begins at a starting point '0' and sets initial values
	pointArray[0] = 0;
	exploredArray[0]++;
	saveCoordinate();

	// Rover begins traveling
	travel();
}

// Call this function when an intersection is seen!!!
void approachIntersection() {
	// ^^ Need to read from front pixel to notify rover of dead ends in original algorithm
	// This could be where the rover sees intersection and then moves forward a set number of places
	// If all IR sensors equal 0 (see white) then the rover knows it is at a dead end.

	static int Total_Points = 1;

	getDataIR(imgTemp);
	intersection++;
	typeArray[intersection] = getType();

	// Assuming that rover will never go back to the starting point
	if (Coordinate[xcoor][ycoor] > 0) {
		// Previous node identified (OLD POINT)
		pointArray[intersection] = Coordinate[xcoor][ycoor];
		exploredArray[Coordinate[xcoor][ycoor]]++;
		exploredArray[intersection] = exploredArray[Coordinate[xcoor][ycoor]] + 1;
		old_DetermineDirection();
	}
	else {
		// New node identified
		pointArray[intersection] = Total_Points;
		exploredArray[intersection]++;
		Total_Points++;

		saveCoordinate();
		new_DetermineDirection();
	}
}