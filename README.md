# Mapper Rover

The mapper rover traverses a map and sends map information to a QT application

### Code description
	The mapper rover control code operates by checking encoder and IR sensor data readings as part of the application loop
	The mapper rover reads IR sensor data from the callback funtion of a FreeRTOS timer
	The mapper rover reads encoder sensor data from the ISR of a PIC32 counter

### Notes
	Because of incomplete sections of the rover, directions are hardcoded in to the rover code in the MakeATurn function in app.c
	
	The Direction Control repository had code used to generate the rover turn decisions before they were hardcoded into the mapper rover.

### Prerequisities

	MPLAB
	MPLAB harmony 
	FreeRTOS

### Built With

* MPLAB

## Authors

* **Owen Nugent**
