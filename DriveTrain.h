/*
 * DriveTrain.h
 *
 *  Created on: Jan 14, 2018
 *      Author: Gaelhawks
 */
#include "WPILib.h"
#include "Common.h"

#ifndef SRC_DRIVETRAIN_H_
#define SRC_DRIVETRAIN_H_

class TalonXIX;

#define K_GYRO		 (20.0) //Edit this value once calculated
#define K_ROBOT		 (610) //Edit this value once calculated
#define TAU_ROBOT	 (0.15) //Edit this value once calculated
#define GAMEPAD_GAIN        (0.2)
#define GAMEPAD_ROTATE_GAIN (0.25)
#define GAMEPAD_STRAFE_GAIN (0.5)

#define M_CMD			(0.7)

#define MAX_ROTATE  	(3.0) //Value in radians
#define GAMEPAD_M_CMD	(0.7)

#define MAX_COMMAND		(1.0)
#define MIN_COMMAND	    (-1.0)

#define MIN_MOVING_CMD  (0.1) //Absolute val of motor cmd needed for moving function to return true

class DriveTrain
{
private:

	VictorSP *frontLeftMotor;
	VictorSP *frontRightMotor;
	VictorSP *backLeftMotor;
	VictorSP *backRightMotor;

	TalonXIX *mainRobot;

#ifdef USE_GYRO
	ADXRS450_Gyro *gyro;
	double gyroVel;
	double gyroAngle;
	double gyroErr;
	double gyroErrInt;
#endif
	bool gyroOn;

	double frontLeftMotorCmd;
	double frontRightMotorCmd;
	double backLeftMotorCmd;
	double backRightMotorCmd;

	double Limit(double num);

public:
	DriveTrain(TalonXIX *robot);
	void DriveControl(float speedCmd, float rotateCmd, float sideCmd, float gamepadSpeedCmd, float gamepadRotateCmd, float gamepadSideCmd, bool isJoystickCmd, bool isAuto);
	void ResetGyro(void);
	bool IsRobotMoving(void);
	void GyroOn(void);
	void GyroOff(void);
	double GyroControl(double motorCmd);
	void GetGyroVelocity(void);
	double GetGyroAngle(void);
	void LocalReset(void);
	void UpdateDash(void);
	void StopAll(void);
};



#endif /* SRC_DRIVETRAIN_H_ */
