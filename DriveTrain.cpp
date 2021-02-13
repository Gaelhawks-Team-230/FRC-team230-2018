/*
 * DriveTrain.cpp
 *
 *  Created on: Jan 14, 2018
 *      Author: Gaelhawks
 */
#include "WPILib.h"
#include "TalonXIX_main.h"
#include "DriveTrain.h"

DriveTrain::DriveTrain(TalonXIX *robot)
{
	frontLeftMotor = new VictorSP(PWM_FRONT_LEFT);
	frontRightMotor = new VictorSP(PWM_FRONT_RIGHT);
	backLeftMotor = new VictorSP(PWM_BACK_LEFT);
	backRightMotor = new VictorSP(PWM_BACK_RIGHT);

	frontLeftMotorCmd = 0.0;
	frontRightMotorCmd = 0.0;
	backLeftMotorCmd = 0.0;
	backRightMotorCmd = 0.0;

	mainRobot = robot;

#ifdef USE_GYRO
	gyro = new ADXRS450_Gyro(SPI::kOnboardCS0);
	gyro->Calibrate();
	gyroVel = 0;
	gyroAngle = 0;
	gyroErr = 0;
	gyroErrInt = 0;
	gyroOn = true;
#else
	gyroOn = false;
#endif
	LocalReset();
}

void DriveTrain::DriveControl(float speedCmd, float rotateCmd, float sideCmd, float gamepadSpeedCmd, float gamepadRotateCmd, float gamepadSideCmd, bool isJoystickCmd, bool isAuto)
{
	double modifiedRotate;

	//printf("INPUT SpeedCmd: %f, RotateCmd: %f, StrafeCmd: %f \n", speedCmd, rotateCmd, sideCmd);

	if (isJoystickCmd)
	{
		if(!isAuto)
		{
			if(fabs(speedCmd) <= DRIVE_DEADBAND)
				speedCmd = 0.0;
			else if(speedCmd > 0.0)
				speedCmd = (speedCmd - DRIVE_DEADBAND)/(1.0 - DRIVE_DEADBAND);
			else
				speedCmd = (speedCmd + DRIVE_DEADBAND)/(1.0 - DRIVE_DEADBAND);

			if(fabs(rotateCmd) <= DRIVE_DEADBAND)
				rotateCmd = 0.0;
			else if(rotateCmd > 0.0)
				rotateCmd = (rotateCmd - DRIVE_DEADBAND)/(1.0 - DRIVE_DEADBAND);
			else
				rotateCmd = (rotateCmd + DRIVE_DEADBAND)/(1.0 - DRIVE_DEADBAND);

			if(fabs(sideCmd) <= DRIVE_DEADBAND)
				sideCmd = 0.0;
			else if(sideCmd > 0.0)
				sideCmd = (sideCmd - DRIVE_DEADBAND)/(1.0 - DRIVE_DEADBAND);
			else
				sideCmd = (sideCmd + DRIVE_DEADBAND)/(1.0 - DRIVE_DEADBAND);
		}


		if(!isAuto)
		{
			if(fabs(gamepadSpeedCmd) <= GAMEPAD_DEADBAND)
				gamepadSpeedCmd = 0.0;
			else if(gamepadSpeedCmd > 0.0)
				gamepadSpeedCmd = (gamepadSpeedCmd - GAMEPAD_DEADBAND)/(1.0 - GAMEPAD_DEADBAND);
			else
				gamepadSpeedCmd = (gamepadSpeedCmd + GAMEPAD_DEADBAND)/(1.0 - GAMEPAD_DEADBAND);

			if(fabs(gamepadRotateCmd) <= GAMEPAD_DEADBAND)
				gamepadRotateCmd = 0.0;
			else if(gamepadRotateCmd > 0.0)
				gamepadRotateCmd = (gamepadRotateCmd - GAMEPAD_DEADBAND)/(1.0 - GAMEPAD_DEADBAND);
			else
				gamepadRotateCmd = (gamepadRotateCmd + GAMEPAD_DEADBAND)/(1.0 - GAMEPAD_DEADBAND);

			if(fabs(gamepadSideCmd) <= GAMEPAD_DEADBAND)
				gamepadSideCmd = 0.0;
			else if(gamepadSideCmd > 0.0)
				gamepadSideCmd = (gamepadSideCmd - GAMEPAD_DEADBAND)/(1.0 - GAMEPAD_DEADBAND);
			else
				gamepadSideCmd = (gamepadSideCmd + GAMEPAD_DEADBAND)/(1.0 - GAMEPAD_DEADBAND);
		}

		// joystick shaping
		sideCmd = sideCmd * (1.0 + 0.5*M_CMD * (fabs(sideCmd) - 1.0));
		rotateCmd = rotateCmd * (1.0 + 0.5*M_CMD * (fabs(rotateCmd) - 1.0));
		speedCmd = speedCmd * (1.0 + 0.5*M_CMD * (fabs(speedCmd) - 1.0));

		gamepadSideCmd = gamepadSideCmd * (1.0 + 0.5*GAMEPAD_M_CMD * (fabs(gamepadSideCmd) - 1.0));
		gamepadRotateCmd = gamepadRotateCmd * (1.0 + 0.5*GAMEPAD_M_CMD * (fabs(gamepadRotateCmd) - 1.0));
		gamepadSpeedCmd = gamepadSpeedCmd * (1.0 + 0.5*GAMEPAD_M_CMD * (fabs(gamepadSpeedCmd) - 1.0));
	}

	speedCmd = speedCmd + (gamepadSpeedCmd * GAMEPAD_GAIN);
	rotateCmd = rotateCmd + (gamepadRotateCmd * GAMEPAD_ROTATE_GAIN);
	sideCmd = sideCmd + (-gamepadSideCmd * GAMEPAD_STRAFE_GAIN);

#ifdef USE_GYRO
	double teleopMaxRotate;
	GetGyroVelocity();

	if(gyroOn)
	{
		teleopMaxRotate =  MAX_ROTATE * RAD_TO_DEG; //Max speed robot can rotate at converted to degrees
		if(isAuto)
		{
			modifiedRotate = GyroControl(rotateCmd);
		}
		else
		{
			modifiedRotate = GyroControl(rotateCmd * teleopMaxRotate);
		}
	}
	else
	{
		teleopMaxRotate = 1.0; //max speed robot can rotate at
		modifiedRotate = rotateCmd * teleopMaxRotate;
	}
#else
	modifiedRotate = rotateCmd;
#endif

	//printf("speed: %f    rotate: %f		side: %f   %s\n", speedCmd, modifiedRotate, sideCmd, gyroOn?"GYRO":"NO gyro");
	//printf("%lf	%lf %lf %lf\n", modifiedRotate, rotateCmd, GetGyroAngle(), gyroVel);

	frontLeftMotorCmd = Limit(speedCmd + sideCmd + modifiedRotate);
	frontLeftMotor->Set(frontLeftMotorCmd);

	frontRightMotorCmd = Limit(speedCmd - sideCmd - modifiedRotate);
	frontRightMotor->Set(-1.0*frontRightMotorCmd);  // on opposite side of the robot, needs inverted command

	backLeftMotorCmd = Limit(speedCmd - sideCmd + modifiedRotate);
	backLeftMotor->Set(backLeftMotorCmd);

	backRightMotorCmd = Limit(speedCmd + sideCmd - modifiedRotate);
	backRightMotor->Set(-1.0*backRightMotorCmd); // on opposite side of the robot, needs inverted command
}

bool DriveTrain::IsRobotMoving() // Checks if robot is moving based on a minimum motor cmd
{
	if(fabs(frontLeftMotorCmd) < MIN_MOVING_CMD &&
	   fabs(frontRightMotorCmd) < MIN_MOVING_CMD &&
	   fabs(backLeftMotorCmd) < MIN_MOVING_CMD &&
	   fabs(backRightMotorCmd) < MIN_MOVING_CMD)
		return false;
	else
		return true;
}

void DriveTrain::ResetGyro()
{
#ifdef USE_GYRO
	gyro->Reset();
#endif
}

void DriveTrain::GyroOn()
{
#ifdef USE_GYRO
	gyroOn = true;
#else
	gyroOn = false;
#endif
}

void DriveTrain::GyroOff()
{
	gyroOn = false;
#ifdef USE_GYRO
	gyroErrInt = 0.0;
#endif
}

void DriveTrain::GetGyroVelocity()
{
#ifdef USE_GYRO
	gyroVel = gyro->GetRate(); //Change to negative value based on direction of drive train motors.
#endif
}

double DriveTrain::GetGyroAngle()
{
#ifdef USE_GYRO
	double gyroAng;
	gyroAng = gyro->GetAngle(); //Change to negative value based on direction of drive train motors.

	return gyroAng;
#else
    return 0.0;
#endif
}

double DriveTrain::GyroControl(double motorCmd)
{
#ifdef USE_GYRO
	GetGyroVelocity();

	gyroErr = motorCmd - gyroVel;
	gyroErrInt = gyroErrInt + (gyroErr * LOOPTIME);
	float newCmd = (K_GYRO / K_ROBOT) * (TAU_ROBOT * gyroErr + gyroErrInt);

	return newCmd;
#else
	return motorCmd;
#endif
}

double DriveTrain::Limit(double num)
{
	if(num > MAX_COMMAND)
		return MAX_COMMAND;
	else if(num < MIN_COMMAND)
		return MIN_COMMAND;
	else
		return num;
}

void DriveTrain::LocalReset()
{
	frontLeftMotorCmd = 0.0;
	frontRightMotorCmd = 0.0;
	backLeftMotorCmd = 0.0;
	backRightMotorCmd = 0.0;
	gyroOn = false;
#ifdef USE_GYRO
	gyro->Reset();
	gyroErrInt = 0.0;
#endif
	DriveControl(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, true, false);
}

void DriveTrain::UpdateDash()
{
#ifdef TEST_MODE
	SmartDashboard::PutNumber("Front Left", frontLeftMotorCmd);
	SmartDashboard::PutNumber("Front Right", frontRightMotorCmd);
	SmartDashboard::PutNumber("Back Left", backLeftMotorCmd);
	SmartDashboard::PutNumber("Back Right", backRightMotorCmd);
	SmartDashboard::PutNumber("Gyro Rate", gyroVel);
#endif

#ifdef USE_GYRO
	SmartDashboard::PutBoolean("Gyro", gyroOn);
	SmartDashboard::PutNumber("Gyro Reading", GetGyroAngle());
	//printf("Gyro Reading %lf	Gyro Rate %lf\n", GetGyroAngle(), gyroVel);
#endif
}

void DriveTrain::StopAll()
{
	LocalReset();
}
