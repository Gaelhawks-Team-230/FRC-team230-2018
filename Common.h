/*
 * Common.h
 *
 *  Created on: Jan 13, 2018
 *      Author: Gaelhawks
 */

#ifndef SRC_COMMON_H_
#define SRC_COMMON_H_


#include "WPILib.h"

/**********************\
 *  Feature Enablers  *
\**********************/
#define USE_GYRO
//#define TEST_MODE
#define USE_WEBCAM
//#define USE_PIXY
//#define PRACTICE_BOT
//#define CHECK_LOOPTIMING

#include "TalonXIX_main.h"


/*******************\
 *  Global Values  *
\*******************/
#define PI	 						(3.14159265)
#define RAD_TO_DEG 					(180.0/PI)
#define DEG_TO_RAD					(PI/180.0)
#define X_SCALING_FACTOR 			(1)
#define Y_SCALING_FACTOR			(0.65)

#define JOYSTICK_DEADBAND			(0.3)
#define DRIVE_DEADBAND				(0.1)
#define GAMEPAD_DEADBAND			(0.5)

#define LOOPTIME 					(0.02)
#define PIXYTIME					(0.008)
#define SAMPLES_PER_SECOND			(1.0/LOOPTIME)
#define N1SEC  						((int) SAMPLES_PER_SECOND)

#ifdef USE_PIXY
#define PIXY_CUBE_I2C_ADDRESS       (0x5A)
#define MAX_PIXY_BLOCKS				2
#endif

typedef enum
{
	// example: USB_JOY_DRIVE	= 1,
	// example: USB_GAMEPAD		= 2,

} usbs;

typedef enum
{
	// example: CLIMBER_CURRENT_IN		= 2,
	CLAW_PDP 		= 2,

} pdp;

typedef enum
{
	PWM_BACK_LEFT			= 0,
	PWM_BACK_RIGHT			= 1,
	PWM_FRONT_LEFT			= 2,
	PWM_FRONT_RIGHT			= 3,
	PWM_CUBELIFT			= 4, //Random Numbers I added to build, most likely incorrect as of 2/4
	PWM_CLAW_SQUEEZE		= 5,
	PWM_CLAW_WHEELS     	= 6,
	PWM_CLIMBER				= 7,

} pwms;

typedef enum
{
	//example GYRO_ANALOG_INPUT   = 1,
	//example GRAB_ANALOG_INPUT   = 0,

} analogs;

typedef enum
{
	RELAY_CLAW_DROP			                = 0,

}relays;

typedef enum
{
	FRONT_LEFT_ENCODER_ONE                      = 0,
	FRONT_LEFT_ENCODER_TWO                      = 1,
	FRONT_RIGHT_ENCODER_ONE                     = 2,
	FRONT_RIGHT_ENCODER_TWO                     = 3,
	BACK_LEFT_ENCODER_ONE                       = 4,
	BACK_LEFT_ENCODER_TWO                       = 5,
	BACK_RIGHT_ENCODER_ONE                      = 6,
	BACK_RIGHT_ENCODER_TWO                      = 7,
	LIFT_ENCODER_ONE							= 8,
	LIFT_ENCODER_TWO							= 9,
	DIGIN_CUBE_SENSOR 			                = 17, //MXP Digital 7
	CLIMBER_ENCODER_CHANNEL_A					= 14, //MXP Digital 4
	CLIMBER_ENCODER_CHANNEL_B					= 15, //MXP Digital 5
	LED_LIGHT_OUTPUT_1                          = 22, //MXP 2 Arduino
	LED_LIGHT_OUTPUT_2                          = 23, //MXP 2 Arduino

} digitals;

/*
** List of gamepad (USB_GAMEPAD below) button and axis assignments
*/

typedef enum
{
	ENABLE_GYRO_BUTTON 			= 1,
	BUTTON_CLAW_RELEASE_TEST	= 15, //X button
	CLIMBER_ENCODER_RESET		= 3,  // red button
} joystick_buttons;

typedef enum
{
	ROTATE_AXIS                 = 5,
	SPEED_AXIS                  = 1,
	STRAFE_AXIS                 = 0,

} joystick_axes;

typedef enum
{
	GAMEPAD_SPEED_AXIS          = 3, //1,
	GAMEPAD_STRAFE_AXIS         = 2, //0,
	MANUAL_ELEVATOR_AXIS		= 1, //3,
	GAMEPAD_ROTATE_AXIS         = 0, //2,

} gamepad_axes;

typedef enum
{
	//example: DPAD_OPEN_GEARDOOR  				= 0,
	DPAD_CLIMBER_MANUAL_UP			= 0,
	DPAD_CLIMBER_STOP_WHERE_I_AM	= 90,
	DPAD_CLIMBER_MANUAL_DOWN		= 180,
	DPAD_CLIMBER_TO_TOP				= 270,
} gamepad_pov;

typedef enum
{
	BUTTON_LIFT_PICKUP_POSITION				= 1,
	BUTTON_LIFT_SWITCH_POSITION             = 2,
	BUTTON_LIFT_NEUTRAL_POSITION			= 3,
	BUTTON_LIFT_HIGHEST_POSITION            = 4,
	BUTTON_EJECT_CUBE						= 5,
	BUTTON_GATHER_CUBE						= 6,
	BUTTON_OPEN_CLAW						= 7,
	BUTTON_ADJUST_CUBE						= 8,
	//BUTTON_WHEELS_OUT						= 5, //Left trigger
	//BUTTON_WHEELS_IN						= 6, //Right trigger
	//BUTTON_OPEN_CLAW						= 7, //Left bumper
	//BUTTON_CLOSE_CLAW						= 8, //Right bumper
	BUTTON_STOP_WHEELS						= 9, //3 lines button
	BUTTON_LIFT_STOP						= 10,

} gamepad_buttons;




#endif /* SRC_COMMON_H_ */
