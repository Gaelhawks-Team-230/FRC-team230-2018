#include "WPILib.h"
#include "Common.h"
#include "Climber.h"

//Climber mechanism
Climber::Climber(TalonXIX* pRobot)
{
	climberMotor = new VictorSP(PWM_CLIMBER);
//#ifdef PRACTICE_BOT
	climbEncoder = new Encoder(CLIMBER_ENCODER_CHANNEL_B, CLIMBER_ENCODER_CHANNEL_A, false);  // green arm
//#else
//	climbEncoder = new Encoder(CLIMBER_ENCODER_CHANNEL_A, CLIMBER_ENCODER_CHANNEL_B, false);  // orange arm
//#endif
	climbEncoder->SetDistancePerPulse(CLIMBER_DISTANCE_PER_PULSE);

	mainRobot = pRobot;
	isInitialized = false;
	initStage = 0;
	curCount = 10;
	lastCount = 1000;
	loopCount = 0;
	isReadyToClimb = false;

	LocalReset();
}

void Climber::LocalReset()
{
	currentPosition = ReadEncoder();
	goalPosition = currentPosition;
	oldPosition = currentPosition;
	positionCmd = currentPosition;

	posError = 0.0;
	curVelocity = 0.0;
	velocityError = 0.0;
	velocityCmd = 0.0;
	velocityErrorInt = 0.0;
	atClGoal = true;
	isManualMove = false;
	oldPCmd = 0.0;

	motorSpeed = 0.0;
	climberMotor->Set(motorSpeed);
}

void Climber::StopAll()
{
	LocalReset();
}

void Climber::ControlSystemInit()
{
	LocalReset();
}

/*void Climber::ResetEncoder()
{
	climbEncoder->Reset();
	currentPosition = ReadEncoder();
	goalPosition = currentPosition;
	oldPosition = currentPosition;
	positionCmd = currentPosition;
}*/

void Climber::StopWhereIAm()
{
	LocalReset();
}

bool Climber::IsAtGoal()
{
	if(fabs(goalPosition - currentPosition) <= CLIMBER_ALLOWANCE)
	{
		atClGoal = true;
	}
	else
	{
		atClGoal = false;
	}
	return atClGoal;
}

bool Climber::IsClimbing()
{
	return(isReadyToClimb);

	if(fabs(motorSpeed) >= 0.2 && isReadyToClimb)
	{
		return true;
	}
	else
	{
		return false;
	}
}

//#ifdef TEST_MODE	//SHOULD BE ifdef
void Climber::PracManualMove(double pracRaiseCmd)
{
	if(fabs(pracRaiseCmd) >= GAMEPAD_DEADBAND)
	{
		motorSpeed = pracRaiseCmd * CLIMBER_RELEASE_CONSTANT;
	}
	else
	{
		motorSpeed = 0.0;
	}
}
//#else
void Climber::ManualMove(double raiseCmd)
{
	currentPosition = ReadEncoder();

	if (fabs(raiseCmd) >= GAMEPAD_DEADBAND)
	{
		if (raiseCmd > 0.0)
		{
			if (currentPosition >= CLIMBER_TOP_STOP_HEIGHT)
			{
				return;
			}
			else
			{
				if(!isManualMove)
				{
					goalPosition = currentPosition;
					isManualMove = true;
				}
				else
				{
					goalPosition = goalPosition + MANUAL_DELTA;
				}
			}
		}
		else
		{
			if (currentPosition <= CLIMBER_BOTTOM_STOP_HEIGHT)
			{
				return;
			}
			else
			{
				if(!isManualMove)
				{
					goalPosition = currentPosition;
					isManualMove = true;
				}
				else
				{
					goalPosition = goalPosition - MANUAL_DELTA;
				}
			}
		}
	}
	else
	{
		return;
	}
}

void Climber::GiveGoalPosition(double releaseInches)
{
	isManualMove = false;
	goalPosition = releaseInches;
	goalPosition = TalonXIX::Limit(CLIMBER_BOTTOM_STOP_HEIGHT, CLIMBER_TOP_STOP_HEIGHT, goalPosition);
	if(goalPosition >= (CLIMBER_RELEASE_HEIGHT - 5.0))
	{
		isReadyToClimb = true;
	}
}

void Climber::ControlRelease()
{
	currentPosition = ReadEncoder();

	goalPosition = TalonXIX::Limit(CLIMBER_BOTTOM_STOP_HEIGHT, CLIMBER_TOP_STOP_HEIGHT, goalPosition);
	positionCmd = positionCmd + TalonXIX::Limit(-1.0 * CLIMBER_MAX_DELTA_POS, CLIMBER_MAX_DELTA_POS, goalPosition - positionCmd);

	posError = positionCmd - currentPosition;
	curVelocity = (currentPosition - oldPosition)/LOOPTIME;

	if (fabs(positionCmd - currentPosition) <= CLIMBER_ALLOWANCE)
	{
		velocityCmd = 0.0;
		atClGoal = true;
	}
	else
	{
		velocityCmd = CLIMBER_KP * posError + CLIMBER_KFF * ((positionCmd - oldPCmd)/LOOPTIME);
		atClGoal = false;
	}

	velocityError = velocityCmd - curVelocity;
	velocityErrorInt = velocityErrorInt + (velocityError * LOOPTIME);
	velocityErrorInt = TalonXIX::Limit(CLIMBER_MIN_VEL_ERROR_INT, CLIMBER_MAX_VEL_ERROR_INT, velocityErrorInt);

	motorSpeed = CLIMBER_K_DEADBAND/CLIMBER_K * (CLIMBER_TAU * velocityError + velocityErrorInt);
	motorSpeed = TalonXIX::Limit(CLIMBER_MIN_MOTOR_CMD, CLIMBER_MAX_MOTOR_CMD, motorSpeed);

	oldPosition = currentPosition;
	oldPCmd = positionCmd;
}
//#endif

void Climber::ReCalibrate()
{
	if (isInitialized)
	{
		isInitialized = false;
		initStage = 0;
	}
}

void Climber::StartAtBottom()
{
	switch(initStage)
	{
		case 0: //Set moving down Cmd
			motorSpeed = CLIMBER_CALIBRATION_CMD;
			printf("calibrating at %f\n", motorSpeed);
			loopCount = 0;
			curCount = 10;
			lastCount = 1000;
			initStage++;
			break;

		case 1:  // Wait(0.1);
			if(loopCount >= 5)
			{
				loopCount = 0;
				initStage++;
			}
			else
			{
				loopCount++;
			}
			break;

		case 2: //Check if delta is greater than 2 every 3 looptimes
			if(abs(lastCount - curCount) >= 2)
			{
				if(loopCount > 5)
				{
					lastCount = curCount;
					curCount = climbEncoder->Get();
					loopCount = 0;
					//printf(" raw %d get %d distance %lf\n", climbEncoder->GetRaw(), curCount, climbEncoder->GetDistance());
					printf(" %d != %d\n", lastCount, curCount);
				}
				else
				{
					loopCount++;
				}
			}
			else
			{
				printf("at bottom... ");
				motorSpeed = 0.0;
				loopCount = 0;
				initStage++;
			}
			break;

		case 3: //Wait(0.5)
			if(loopCount >= 25)
			{
				loopCount = 0;
				printf("RESETTING encoder\n");
				climbEncoder->Reset();
				LocalReset();
				isInitialized = true;
				GiveGoalPosition(CLIMBER_BOTTOM_STOP_HEIGHT);
			}
			else
			{
				loopCount++;
			}
			break;
	}
}

double Climber::ReadEncoder()
{
	return climbEncoder->GetDistance();
}

bool Climber::IsAtTop()
{
	if (ReadEncoder() < CLIMBER_TOP_STOP_HEIGHT)
	{
		return false;
	}
	return true;
}

bool Climber::IsAtBottom()
{
	if (ReadEncoder() > CLIMBER_BOTTOM_STOP_HEIGHT)
	{
		return false;
	}
	return true;
}

void Climber::UpdateDash()
{
	SmartDashboard::PutNumber("Climber MotorCmd: ", motorSpeed);
	SmartDashboard::PutNumber("Climber height: ", ReadEncoder());
	SmartDashboard::PutNumber("ClimbCurrent: ", mainRobot->pdp->GetCurrent(3));
	SmartDashboard::PutBoolean("ClimberatGoal: ", atClGoal);
	SmartDashboard::PutBoolean("isReadyToClimb: ", isReadyToClimb);
}

// All classes need a Service function, even if
void Climber::Service()
{
#ifndef TEST_MODE
	if(!isInitialized)
	{
		StartAtBottom();
	}
	else
	{
		ControlRelease();
	}

#endif
	climberMotor->Set(motorSpeed);
}
