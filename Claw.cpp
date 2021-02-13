#include "WPILib.h"
#include "Common.h"
#include "Claw.h"

// Mechanism to pick up power cubes, sense when a cube is being held, and release the claw
Claw::Claw(TalonXIX *pRobot)
{
	// Create objects needed by this class
	clawSqueeze = new VictorSP(PWM_CLAW_SQUEEZE);
	wheels = new VictorSP(PWM_CLAW_WHEELS); //from robot's pov
	clawDrop = new Relay(RELAY_CLAW_DROP);
	cubeSensor = new DigitalInput(DIGIN_CUBE_SENSOR);

	mainRobot = pRobot;

	// call the class initialization function
	LocalReset();

	if (hasCube)
		holdingCube = true;
}

// Set initial values for ALL objects and variables

void Claw::LocalReset()
{
	// Initialize ALL new objects
	StopWheels();
	clawDrop->Set(Relay::kOff);
	// Initialize ALL local variables
	hasCube = HaveCube();
	holdingCube = false;
	dropMode = false;
	dropCount = 0;
	clawMotorCmd = 0.0;
	movingClaw = false;
	squeezeMotorCmd = 0.0;
	waitToPauseCount = 0;
	ejectCount = 0;
	clawIsOpen = false;

	isGatheringCube = false;
	isEjectingCube = false;
	isAdjustingCube = false;
	gatherState = 0;
	ejectState = 0;
	adjustState = 0;
	gatherCubeCount = 0;
	ejectCubeCount = 0;
	adjustCubeCount = 0;
	moveClawTime = 0;
}

void Claw::StartingConfig()
{
	StopWheels();
	SqueezeCube();

}

void Claw::StopAll()
{
	LocalReset();
}


void Claw::PauseWheels()
{
	wheels->Set(0.0);
}

void Claw::StopWheels(void)
{
	clawMotorCmd = 0.0;
	holdingCube = false;
}


void Claw::StartWheels(bool wheelsIn)
{
	if (wheelsIn)
	{   // GRAB
		clawMotorCmd = GATHER_COMMAND;
		if (!holdingCube)
		{
			holdingCube = true;
			waitToPauseCount = 0;
		}
	}
	else
	{   // EJECT
		if (isAdjustingCube)
		{
			clawMotorCmd = ADJUST_COMMAND;
		}
		else
		{
			clawMotorCmd = EJECT_COMMAND;
		}
		if (holdingCube)
		{
			holdingCube = false; //cube pushed out
			ejectCount = 0;
		}
	}
}

bool Claw::HaveCube()
{
	if(clawIsOpen)
	{
		hasCube = false;
	}
	else
	{
		if (cubeSensor->Get())
		{
			hasCube = false;
		}
		else
		{
			hasCube = true;
		}
	}
	return hasCube;
}

// Release the claw from its storage location at the beginning of the match
void Claw::DropClaw()
{
	if (!dropMode)
	{
		dropCount = 0;
	}
	dropMode = true;
}


void Claw::SqueezeCube()
{
//#ifdef TEST_MODE
//	squeezeMotorCmd = 1.0;
//	clawIsOpen = false;
//#else
	//THIS WORKS
	if (squeezeMotorCmd != SQUEEZE_COMMAND)
	{
		squeezeTimeCount = 0;
	}
	squeezeMotorCmd = SQUEEZE_COMMAND;
	movingClaw = true;
	clawIsOpen = false;
	moveClawTime = SQUEEZE_TIME;
//#endif
}

void Claw::ReleaseCube()
{
//#ifdef TEST_MODE
//	squeezeMotorCmd = -1.0;
//	clawIsOpen = true;
//#else
	if (squeezeMotorCmd != RELEASE_COMMAND)
	{
		squeezeTimeCount = 0;
	}
	movingClaw = true;
	clawIsOpen = true;
	squeezeMotorCmd = RELEASE_COMMAND;
	moveClawTime = RELEASE_TIME;
//#endif
}

void Claw::StopSqueeze()
{
	squeezeMotorCmd = 0.0;
}

void Claw::GatherCube()
{
	if(!isGatheringCube)
	{
		isGatheringCube = true;
		isEjectingCube = false;
		isAdjustingCube = false;
		gatherState = IDLE_MODE;
	}
}
void Claw::EjectCube()
{
	if(!isEjectingCube)
	{
		//printf("TURNING ON EJECT CUBE\n");
		isGatheringCube = false;
		isEjectingCube = true;
		isAdjustingCube = false;
		ejectState = IDLE_MODE;
	}
}

void Claw::AdjustCube()
{
	if(!isAdjustingCube)
	{
		isGatheringCube = false;
		isEjectingCube = false;
		isAdjustingCube = true;
		adjustState = IDLE_MODE;
	}
}


void Claw::AdvancedCubeOperations()
{
	if (isGatheringCube)
	{
		switch(gatherState)
		{
			case IDLE_MODE:
				StopWheels();
				gatherCubeCount = 0;
				gatherState++;
				break;

			case SPIN_WHEELS_MODE:
				StartWheels(true);
				if (gatherCubeCount < GATHER_SPIN_TIMER)
				{
					gatherCubeCount++;
				}
				else
				{
					gatherCubeCount = 0;
					gatherState++;
				}
				break;

			case SQUEEZE_CUBE_MODE:
				SqueezeCube();
				isGatheringCube = false;
				gatherState = IDLE_MODE;
				break;
		}
	}

	else if (isEjectingCube)
	{
		//printf("Ejecting state %d ejectCubeCount %d (also ejectCount %d)\n", ejectState, ejectCubeCount, ejectCount);
		switch(ejectState)
		{
			case IDLE_MODE:
				StopWheels();
				ejectCubeCount = 0;
				ejectCount = 0;
				ejectState++;
				break;

			case SPIN_WHEELS_MODE:
				StartWheels(false);
				if (ejectCubeCount < EJECT_SPIN_TIMER)
				{
					ejectCubeCount++;
				}
				else
				{
					ejectCubeCount = 0;
					ejectState++;
				}
				break;

			case SQUEEZE_CUBE_MODE:
				ReleaseCube();
				isEjectingCube = false;
				ejectState = IDLE_MODE;
				break;
		}
	}

	else if (isAdjustingCube)
	{
		switch(adjustState)
		{
			case IDLE_MODE:
				StopWheels();
				adjustCubeCount = 0;
				holdingCube = false;
				ejectCount = 0;
				adjustState++;
				break;

			case SPIN_OUT_MODE:
				StartWheels(false);
				if (adjustCubeCount < ADJUST_SPIN_OUT_TIME)
				{
					adjustCubeCount++;
				}
				else
				{
					adjustCubeCount = 0;
					adjustState++;
				}
				break;

			case SPIN_IN_MODE:
				isAdjustingCube = false;
				adjustState = IDLE_MODE;
				GatherCube();
				break;
		}
	}
}


void Claw::Service()
{
	HaveCube();

	// times release of the claw
	if (dropMode)
	{
		//printf("drop kForward\n");
		clawDrop->Set(Relay::kForward);
		dropCount++;
		if(dropCount >= CLAW_RELEASE_TIME)
		{
			//printf("drop kOff\n");
			clawDrop->Set(Relay::kOff);
			dropMode = false;
		}
	}

	AdvancedCubeOperations();

	if (holdingCube)
	{
		// GRABBING
		if(!hasCube) //cube has not been intentionally released but is gone
		{
			wheels->Set(clawMotorCmd);
			waitToPauseCount = 0;
		}
		else
		{
			if (waitToPauseCount >= GRAB_PAUSE_WAIT)
			{
				PauseWheels();
			}
			waitToPauseCount++;
		}
	}
	else
	{   //EJECTING
		wheels->Set(clawMotorCmd);
		if (ejectCount >= EJECT_UNTIL_STOP_WAIT)
		{
			StopWheels();
		}
		ejectCount++;
	}

//#ifndef TEST_MODE
	if (movingClaw)
	{
		squeezeTimeCount++;

		if (squeezeTimeCount >= moveClawTime)
		{
			moveClawTime = 0;
			squeezeMotorCmd = 0.0;
			movingClaw = false;
		}
	}
//#endif
	clawSqueeze->Set(squeezeMotorCmd);
}

// Update the display of all relevant information in your class
//  - this can include motor/pwm command values, flags, state, etc.
void Claw::UpdateDash()
{
	SmartDashboard::PutBoolean("dropMode: ", dropMode);
	SmartDashboard::PutBoolean("hasCube: ", HaveCube());
#ifdef TEST_MODE
	SmartDashboard::PutBoolean("movingClaw: ", movingClaw);
#endif
	SmartDashboard::PutBoolean("clawIsOpen: ", clawIsOpen);
	SmartDashboard::PutNumber("wheels PWM: ", wheels->Get());
	SmartDashboard::PutNumber("squeezeMotorCmd: ", squeezeMotorCmd);
}

