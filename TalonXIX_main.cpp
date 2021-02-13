/*
 * TalonXIX_main.cpp
 *
 *  Created on: Jan 13, 2018
 *      Author: Gaelhawks
 */
#include "WPILib.h"
#include "TalonXIX_main.h"


TalonXIX::TalonXIX()
{
	joystick = new Joystick(0);
	gamepad = new Joystick(1);
	pdp = new PowerDistributionPanel();


	claw = new Claw(this);
	drive = new DriveTrain(this);
	distance = new DriveDistance();
	lift = new CubeLift(this);
	lights = new LightController(this);
	climber = new Climber(this);

#ifdef USE_PIXY
	cubeCamera = new PixyJunk(PIXY_CUBE_I2C_ADDRESS);
	testBlock = 0;
#endif

	AutoPositionChooser = new SendableChooser<char*>;
	AutoSetupChooser = new SendableChooser<int*>;

	targetPositionData = "";
	autoPosition = 'C';
	autoPositionString[0] = '\0';
	autoPositionString[1] = '\0';	// null-terminator
	autoSetupChoice = 0;
	autoTurnMultiplier = 1.0;
	autoMode = 0;
	autoLevel2 = 0;
	autoDelay = 0.0;
	autoStage = 0;
	driveCmd = 0.0;
	rotateCmd = 0.0;
	sideCmd = 0.0;
	validRead = false;

	centerRotateCmd = 0.0;
	centerRotateCount = 0.0;
	centerDistance = 0.0;

	LL_Level1 = 0;
	LL_Level2 = 0;
	LR_Level1 = 0;
	LR_Level2 = 0;
	RL_Level1 = 0;
	RL_Level2 = 0;
	RR_Level1 = 0;
	RR_Level2 = 0;

	dashCounter = 0;
	loopTime = LOOPTIME;
	startTime = 0.0;
	curTime = 0.0;
	waitTime = 0.0;
	loopOverTime = false;
	NumLoopsOverTime = 0;
	loopCount = 0;
	isBlueAlliance = false;
	isJoystickCmd = false;
	firstTime = true;
	isAuto = false;
}

void TalonXIX::RobotInit()
{
#ifdef USE_WEBCAM
	CameraServer::GetInstance()->StartAutomaticCapture();
#endif

	SmartDashboard::PutNumber("LL Level 1", 0);
	SmartDashboard::PutNumber("LL Level 2", 0);

	SmartDashboard::PutNumber("LR Level 1", 0);
	SmartDashboard::PutNumber("LR Level 2", 0);

	SmartDashboard::PutNumber("RL Level 1", 0);
	SmartDashboard::PutNumber("RL Level 2", 0);

	SmartDashboard::PutNumber("RR Level 1", 0);
	SmartDashboard::PutNumber("RR Level 2", 0);

	AutoPositionChooser->AddObject("Left", &LEFT_POS);
	AutoPositionChooser->AddDefault("Center", &CENTER_POS);
	AutoPositionChooser->AddObject("Right", &RIGHT_POS);
	SmartDashboard::PutData("Positions: ", AutoPositionChooser);

	AutoSetupChooser->AddDefault("Default", &defaultSetup);
	AutoSetupChooser->AddObject("Custom", &customSetup);
	SmartDashboard::PutData("Setup Options: ", AutoSetupChooser);

	ServiceDash();
}

// Runs when disabled
void TalonXIX::Disabled()
{
	while(IsDisabled())
	{
		UpdateModeSelections();
		Wait(loopTime);
	}
}

/**
 * This autonomous (along with the chooser code above) shows how to select between different autonomous modes
 * using the dashboard. The sendable chooser code works with the Java SmartDashboard. If you prefer the LabVIEW
 * Dashboard, remove all of the chooser code and uncomment the GetString line to get the auto name from the text box
 * below the Gyro
 *
 * You can add additional auto modes by adding additional comparisons to the if-else structure below with additional strings.
 * If using the SendableChooser make sure to add them to the chooser code above as well.
 */

void TalonXIX::InitializeAlliance()
{
	if(frc::DriverStation::GetInstance().GetAlliance() == DriverStation::kBlue)
		isBlueAlliance = true;
	else
		isBlueAlliance = false;
}


void TalonXIX::Autonomous()
{
	UpdateModeSelections();

	while(!validRead)
	{
		validRead = MatchModeAssignments();
	}

	drive->GyroOn();
	isAuto = true;

	if (firstTime)
	{
	//	lift->StartAtBottom();
		claw->DropClaw(); //must be last thing
		firstTime = false;
	}

	Wait(0.1);

	InitializeAlliance();

#ifdef CHECK_LOOPTIMING
	double endTime;
	double totalLoopTime;
	int loopCounter = 0;
#endif


	double initialStartTime;
	initialStartTime = GetTime();
	startTime = initialStartTime - loopTime;

	while (IsAutonomous() && IsEnabled())
	{
		startTime += loopTime;
#ifdef CHECK_LOOPTIMING
		loopCounter++;
#endif

#ifndef TEST_MODE
		switch(autoMode)
		{
			/*
			 * LEVEL 1 MODES
			 */
			case 1://Go Forward to auto line (REACH_BASELINE_AUTO)
				ReachBaselineMode();
				break;

			case 2://Deliver Cube to Switch starting in Center (CENTER_SWITCH_AUTO)
				CenterSwitchBaseMode();
				break;

			case 3://Deliver Cube to Switch starting on Side (SIDE_SWITCH_AUTO)
				SameSideSwitchBaseMode();
				break;

			case 4://Deliver Cube to Scale(If on the same side) Starting on Side (SIDE_SCALE_AUTO)
				SameSideScaleBaseMode();
				break;

			case 5://Deliver Cube to Scale(If on opposite side) Starting on Side (FAR_SIDE_SCALE_AUTO)
				FarSideScaleBaseMode();
				break;

			/*
			 * LEVEL 2 MODES
			 */
			case 10://After delivering to the Switch in Center, pickup cube from pyramid and drop in Switch
				CenterMultiCubeSwitchMode();
				break;

			case 11://After delivering to Switch on Side, pickup cube behind Switch and bring it to Switch
				SameSideScaleThenSwitchMode();
				break;

			case 12://After delivering to Switch on Side, pickup cube behind Switch and bring it to Scale
				SideMultiCubeScaleMode();
				break;

			case 13:
				FarSideScaleThenSwitch();
				break;

			case 14:
				if (targetPositionData[0] == LEFT_POS)
				{
					LeftCenterSwitchThenExchangeMode();
				}
				else
				{
					RightCenterSwitchThenExchangeMode();
				}
				break;

			case 15:
				SameSideSwitchThenFarScale();
				break;

			default:
			case 0://Do nothing
				WaitDelayOrDoNothingMode();
				break;
		}
#endif

		//printf("Auto Stage: %d\n", autoStage);

		/*
		** COMMUNITY SERVICE
		*/
		drive->DriveControl(driveCmd, rotateCmd, sideCmd, 0.0, 0.0, 0.0, false, isAuto);
		CommunityService();
		ServiceDash();

		// ENFORCE LOOP TIMING
		curTime = GetTime();
		waitTime = loopTime - (curTime - startTime);
		if (waitTime < 0.0)
		{
			printf("WARNING! LOOP IS OVERTIME by %f seconds\n", waitTime*-1);
			loopOverTime = true;
			NumLoopsOverTime++;
		}
		else
		{
			Wait(waitTime);
			loopOverTime = false;

#ifdef CHECK_LOOPTIMING
			endTime = GetTime();
			totalLoopTime = endTime - startTime;
			printf("startTime: %f  curTime : %f  endTime: %f	Wait Time: %f  This Loop Time: %f  Total Delta Time: %f [avg: %f] \n",
					startTime, curTime, endTime, waitTime, totalLoopTime, endTime-initialStartTime, (endTime-initialStartTime)/loopCounter);
#endif

		}
	}
	Omnicide();
};


/**
 * Runs the motors with arcade steering.
 */
void TalonXIX::OperatorControl()
{
	int loopCounter=0;
	double initialStartTime;
	isJoystickCmd = true;
	isAuto = false;

	InitializeAlliance();

	if (firstTime)
	{
	//	lift->StartAtBottom();
		claw->DropClaw(); //must be last thing
		firstTime = false;
	}

	lift->ControlSystemInit(); //When enabled keeps it at current height
	climber->ControlSystemInit();

#ifdef USE_PIXY
	int lastBlockCount = 0;
	int repeatCount = 0;
#endif
#ifdef CHECK_LOOPTIMING
	double endTime;
	double totalLoopTime;
#endif

	initialStartTime = GetTime();
	startTime = initialStartTime - loopTime;

	while (IsOperatorControl() && IsEnabled())
	{
		startTime += loopTime;
		loopCounter++;

		// GYRO stuff
		if(joystick->GetRawButton(ENABLE_GYRO_BUTTON))
		{
			drive->GyroOn();
		}
		else
		{
			drive->GyroOff();
        }

		// CLAW stuff
        if (joystick->GetRawButton(BUTTON_CLAW_RELEASE_TEST))
        {
        	claw->DropClaw();
        }

		if(gamepad->GetRawButton(BUTTON_GATHER_CUBE))
		{
			claw->GatherCube();
		}
		if(gamepad->GetRawButton(BUTTON_EJECT_CUBE))
		{
			claw->EjectCube();
		}
		if(gamepad->GetRawButton(BUTTON_ADJUST_CUBE))
		{
			claw->AdjustCube();
		}
		if(gamepad->GetRawButton(BUTTON_OPEN_CLAW))
		{
			claw->StopWheels();
			claw->ReleaseCube();
		}
		if(gamepad->GetRawButton(BUTTON_STOP_WHEELS))
		{
			claw->StopWheels();
		}


#ifdef TEST_MODE
	/*	if(gamepad->GetRawAxis(MANUAL_ELEVATOR_AXIS) > 0.1 || gamepad->GetRawAxis(MANUAL_ELEVATOR_AXIS) < -0.1 )
		{
			lift->PracManualMove(gamepad->GetRawAxis(MANUAL_ELEVATOR_AXIS));
		}
		else
		{
			lift->PracManualMove(0.0);
		}*/

		if(gamepad->GetRawAxis(MANUAL_CLIMBER_AXIS) > 0.5 || gamepad->GetRawAxis(MANUAL_CLIMBER_AXIS) < -0.5 )
		{
			climber->PracManualMove(gamepad->GetRawAxis(MANUAL_CLIMBER_AXIS));
		}
		else
		{
			climber->PracManualMove(0.0);
		}
#else
		// CUBE_LIFT stuff
		if(gamepad->GetRawButton(BUTTON_LIFT_PICKUP_POSITION))
		{
			lift->GiveGoalPosition(LIFT_PICKUP_POSITION);
		}
		if(gamepad->GetRawButton(BUTTON_LIFT_SWITCH_POSITION))
		{
			lift->GiveGoalPosition(LIFT_SWITCH_POSITION);
		}
		if(gamepad->GetRawButton(BUTTON_LIFT_NEUTRAL_POSITION))
		{
			lift->GiveGoalPosition(LIFT_NEUTRAL_POSITION);
		}
		if(gamepad->GetRawButton(BUTTON_LIFT_HIGHEST_POSITION))
		{
			lift->GiveGoalPosition(LIFT_HIGHEST_POSITION);
		}
		if(joystick->GetRawButton(BUTTON_LIFT_STOP))
		{
			lift->LiftStop();
		}
		lift->ManualMove(gamepad->GetRawAxis(MANUAL_ELEVATOR_AXIS) * -1.0);




		// CLIMBER stuff
		if(gamepad->GetPOV(0) == DPAD_CLIMBER_STOP_WHERE_I_AM)
		{
			climber->StopWhereIAm();
		}
		if(gamepad->GetPOV(0) == DPAD_CLIMBER_MANUAL_UP)
		{
			climber->ManualMove(1.0);
		}
		if(gamepad->GetPOV(0) == DPAD_CLIMBER_MANUAL_DOWN)
		{
			climber->ManualMove(-1.0);
		}
		if(gamepad->GetPOV(0) == DPAD_CLIMBER_TO_TOP)
		{
			climber->GiveGoalPosition(CLIMBER_RELEASE_HEIGHT);
		}
		if(joystick->GetRawButton(CLIMBER_ENCODER_RESET))
		{
			climber->ReCalibrate();
		}


#endif


#ifdef USE_PIXY
		testBlock = cubeCamera->GetBlocks(MAX_PIXY_BLOCKS);
		if (lastBlockCount != testBlock)
		//if (testBlock >= 0)
		{
			printf("Repeated for %d frames\n", repeatCount);
			repeatCount = 0;
			printf("Cube camera returned: Testblock = %d\n", testBlock);
			for(int i = 0; i < testBlock; i++)
			{
				printf("cube pixy: %d  ", i); //prints number of block to the console
				cubeCamera->PrintBlock(i); // prints x, y, width, and etc. to the console (the variables in the block object)
			}
			printf("\n\n");
		}
		else
		{
			repeatCount++;
		}
		lastBlockCount = testBlock;
#endif


		/*
		** COMMUNITY SERVICE
		*/
		driveCmd = (joystick->GetRawAxis(SPEED_AXIS) * -1.0);
	    sideCmd = joystick->GetRawAxis(STRAFE_AXIS);
		rotateCmd = joystick->GetRawAxis(ROTATE_AXIS);
		isJoystickCmd = true;

		//drive->DriveControl(driveCmd, rotateCmd, sideCmd, 0.0, 0.0, 0.0, isJoystickCmd, false);
		drive->DriveControl(driveCmd, rotateCmd, sideCmd, (gamepad->GetRawAxis(GAMEPAD_SPEED_AXIS) * -1), gamepad->GetRawAxis(GAMEPAD_ROTATE_AXIS), (gamepad->GetRawAxis(GAMEPAD_STRAFE_AXIS) * -1), isJoystickCmd, false);
		CommunityService();
		ServiceDash();

		// ENFORCE LOOP TIMING
		curTime = GetTime();
		waitTime = loopTime - (curTime - startTime);
		if (waitTime < 0.0)
		{
			printf("WARNING! LOOP IS OVERTIME by %f seconds\n", waitTime*-1);
			loopOverTime = true;
			NumLoopsOverTime++;
			//SmartDashboard::PutBoolean("loopOverTime: ", loopOverTime);
			//SmartDashboard::PutNumber("waitTime: ", waitTime);
		}
		else
		{
			Wait(waitTime);
			loopOverTime = false;

#ifdef CHECK_LOOPTIMING
			endTime = GetTime();
			totalLoopTime = endTime - startTime;
			//printf("startTime: %f  curTime : %f  endTime: %f	Wait Time: %f  This Loop Time: %f  Total Delta Time: %f [avg: %f] \n",
			//		startTime, curTime, endTime, waitTime, totalLoopTime, endTime-initialStartTime, (endTime-initialStartTime)/loopCounter);
			printf("Wait Time: %f  \n",	   waitTime);
#endif
		}
	}
	Omnicide();
}

void TalonXIX::StopAll()
{
	loopCount = 0;
	autoStage = 0;
	validRead = false;
	driveCmd = 0.0;
	rotateCmd = 0.0;
	sideCmd = 0.0;
}

/**
 * Runs during test mode
 */
void TalonXIX::Test()
{

}


double TalonXIX::Limit(double min, double max, double curValue)
{
	if (curValue > max)
		return max;
	if (curValue < min)
		return min;
	return curValue;
}

void TalonXIX::RobotStartConfig()
{
	lights->StartingConfig();
	claw->StartingConfig();
}

void TalonXIX::Omnicide()
{
	distance->StopAll();
	drive->StopAll();
	lights->StopAll();
	climber->StopAll();
	lift->StopAll();
	claw->StopAll();
	StopAll();
}

void TalonXIX::CommunityService()
{
	lights->Service();
	climber->Service();
	lift->Service();
	claw->Service();
}

void TalonXIX::ServiceDash()
{
	if (dashCounter == 20)
	{
		distance->UpdateDash();
		drive->UpdateDash();
		lights->UpdateDash();
	    climber->UpdateDash();
	    lift->UpdateDash();
	    claw->UpdateDash();
		dashCounter = 0;
	}
	else
	{
		dashCounter++;
	}
}


START_ROBOT_CLASS(TalonXIX)




