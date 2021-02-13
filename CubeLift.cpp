#include "WPILib.h"
#include "Common.h"
#include "CubeLift.h"

// Controls the movement of the lift
CubeLift::CubeLift(TalonXIX* pRobot)
{
	encoder = new Encoder(LIFT_ENCODER_ONE, LIFT_ENCODER_TWO, false);
	encoder->SetDistancePerPulse(LIFT_DISTANCE_PER_PULSE);
	cubelift = new VictorSP(PWM_CUBELIFT);

	mainRobot = pRobot;

	LocalReset();
}

void CubeLift::LocalReset()
{
	currentPosition = GetHeight();
	goalPosition = currentPosition;
	oldPosition = currentPosition;
	positionCmd = currentPosition;

	positionError = 0.0;
	curVelocity = 0.0;
	atGoal = false;
	velocityCmd = 0.0;
	velocityError = 0.0;
	velocityErrorInt = 0.0;
	isManualMove = false;
	oldPCmd = 0.0;

	motorCmd = 0.0;
	cubelift->Set(motorCmd);
}

void CubeLift::StopAll()
{
	LocalReset();
}

double CubeLift::GetHeight()
{
	return encoder->GetDistance();
}

void CubeLift::ControlSystemInit()
{
	LocalReset();
}


#ifdef TEST_MODE
void CubeLift::PracManualMove(double pracLiftCmd)
{
	if(fabs(pracLiftCmd) >= LIFT_MANUAL_DEADBAND)
	{
		motorCmd = LIFT_CONSTANT * pracLiftCmd;
	}
	else
	{
		motorCmd = 0.0;
		return;
	}
}
#else

void CubeLift::ManualMove(double liftCmd)
{
	currentPosition = GetHeight();
	if (fabs(liftCmd) >= LIFT_MANUAL_DEADBAND)
	{
		if (liftCmd > 0.0)
		{
			if (currentPosition >= LIFT_TOP_STOP_HEIGHT)
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
			if (currentPosition <= LIFT_BOTTOM_STOP_HEIGHT)
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

void CubeLift::GiveGoalPosition(double heightInches)
{
	isManualMove = false;
	goalPosition = heightInches;
	goalPosition = TalonXIX::Limit(LIFT_BOTTOM_STOP_HEIGHT, LIFT_TOP_STOP_HEIGHT, goalPosition);
}

void CubeLift::ControlMove()
{
	currentPosition = GetHeight();

	goalPosition = TalonXIX::Limit(LIFT_BOTTOM_STOP_HEIGHT, LIFT_TOP_STOP_HEIGHT, goalPosition);
	positionCmd = positionCmd + TalonXIX::Limit(-1.0 * LIFT_MAX_DELTA_POS, LIFT_MAX_DELTA_POS, goalPosition - positionCmd);

	positionError = positionCmd - currentPosition;
	curVelocity = (currentPosition - oldPosition)/LOOPTIME;

	if (fabs(goalPosition - currentPosition) <= LIFT_ALLOWANCE)
	{
		velocityCmd = 0.0;
		atGoal = true;
	}

	else
	{
		velocityCmd = LIFT_KP * positionError + LIFT_KFF * ((positionCmd - oldPCmd)/LOOPTIME);
		atGoal = false;
	}

	velocityError = velocityCmd - curVelocity;
	velocityErrorInt = velocityErrorInt + (velocityError * LOOPTIME);
	velocityErrorInt = TalonXIX::Limit(LIFT_MIN_VELOCITY_ERROR_INT, LIFT_MAX_VELOCITY_ERROR_INT, velocityErrorInt);

	motorCmd = LIFT_K_BANDWITH/LIFT_K * (LIFT_TAU * velocityError + velocityErrorInt);
	motorCmd = TalonXIX::Limit(LIFT_MIN_MOTOR_CMD, LIFT_MAX_MOTOR_CMD, motorCmd);

	oldPosition = currentPosition;
	oldPCmd = positionCmd;
}
#endif

void CubeLift::StartAtBottom()
{
	int curCount = 10;
	int lastCount = 1000;
	int loopCount = 0;

	printf("Calibrating Lift... ");
	cubelift->Set(LIFT_CALIBRATION_CMD);

	while(curCount != lastCount)
	{
		lastCount = curCount;
		curCount = encoder->Get();
		loopCount++;
		if (loopCount >= 1000)
		{
			break;
		}
		Wait(2*LOOPTIME); //If there is no wait, then it will go through the while loop so fast that two consecutive readings may be the same even if the mechanism is moving.
	}

	printf("Lift at bottom!\n");
	cubelift->Set(0.0);
	encoder->Reset();
}

bool CubeLift::AtFloor()
{
	if (GetHeight() <= LIFT_BOTTOM_STOP_HEIGHT)
	{
		return(true);
	}
	else
	{
		return(false);
	}
}

bool CubeLift::AtMax()
{
	if (GetHeight() >= LIFT_TOP_STOP_HEIGHT)
	{
		return(true);
	}
	else
	{
		return(false);
	}
}

bool CubeLift::AtGoalPosition()
{
	//if(fabs(goalPosition - currentPosition) <= LIFT_ALLOWANCE)
	if(fabs(goalPosition - positionCmd) <= LIFT_AT_GOAL_ALLOWANCE)
	{
		atGoal = true;
	}
	else
	{
		atGoal = false;
	}
	return atGoal;
}

void CubeLift::LiftStop()
{
	currentPosition = GetHeight();
	goalPosition = currentPosition;
	oldPosition = currentPosition;
	positionCmd = currentPosition;
}

void CubeLift::UpdateDash()
{
#ifdef TEST_MODE
	SmartDashboard::PutNumber("Encoder Value: ", encoder->Get());
//	SmartDashboard::PutNumber("CubeLift Integral: ", velocityErrorInt);
	currentPosition = GetHeight();
#endif
	SmartDashboard::PutNumber("MotorCurrent: ", mainRobot->pdp->GetCurrent(13)); //make into #define
	SmartDashboard::PutNumber("Lift MotorCmd: ", motorCmd);
	SmartDashboard::PutNumber("CurHeight: ", currentPosition);
	SmartDashboard::PutNumber("GoalPosition: ", goalPosition);
	SmartDashboard::PutBoolean("At goal: ", atGoal);

}

void CubeLift::Service()
{
#ifndef TEST_MODE
	ControlMove();
#endif
	cubelift->Set(motorCmd);
	//printf(" %lf  %d  \n", motorCmd, encoder->GetRaw());
}
