/*
 * AutonomousMode.cpp
 *
 *  Created on: Jan 14, 2018
 *      Author: jacob
 */


#include "Common.h"
#include "AutonomousModes.h"
#include "WPILib.h"
#include "TalonXIX_main.h"


/*Auto modes
 * Assumptions:
 * 		1. Priority means priority, perform mode regardless of orientation
 * 		(only exception may possibly be if switch ends up on opposite side and scale is on same side and we're not in center)
 * 		2. No priority means make best choice based on scenario (depending on color layout and what friends are doing and our position)
 *
 * Default: Center Position Turn to switch (left or right depending on color), drop cube
 * if(Switch Side == Scale slide && no priority) -> After dropping in switch, pick up cube and drop it in scale
 * else -> continue picking up cubes and dropping into corresponding switch side
 *
 * Side Positions:
 *	if(switch priority) -> switch is priority, check the best way to cross (either across close or across far) if necessary
 *	if(scale priority) -> scale is priority regardless
 *	if(no priority) -> Do which ever target is on your side, and if both are on opposite side, do scale
 *							if both on same side, and multi-cube option on, do both
 */

#ifndef TEST_MODE

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////  AUTO LEVEL 1  ///////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// MODE 0/Negative Number
void TalonXIX::WaitDelayOrDoNothingMode()
{
	if (loopCount < (autoDelay * (N1SEC)))
	{
		loopCount++;
	}
	else
	{
		autoMode = autoLevel2;
		driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
		loopCount = 0;
		autoStage = 0;
	}
}

// MODE 1
void TalonXIX::ReachBaselineMode()
{
	switch(autoStage)
	{
		case 0: //Reset
			driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
			distance->LocalReset();
			loopCount = 0;
			autoStage++;
			break;

		case 1: //Turn to line up with switch depending on where the switch target is positioned
			if(distance->GetForwardsDistance() < DISTANCE_TO_SWITCH)
			{
				driveCmd = 0.4; rotateCmd = 0.0; sideCmd = 0.0;
				//Lift set to switch position
				loopCount++;
			}
			else
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;
				autoStage++;
			}
			break;

		case 2: //Turn to line up with switch depending on where the switch target is positioned
			if((distance->GetForwardsDistance() * -1) < DISTANCE_TO_SWITCH)
			{
				driveCmd = -0.4; rotateCmd = 0.0; sideCmd = 0.0;
				//Lift set to switch position
				loopCount++;
			}
			else
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;
				autoStage++;
			}
			autoStage++;
			break;

		case 3: //Move on to Level 2
			autoMode = autoLevel2;
			driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
			loopCount = 0;
			autoStage = 0;
			break;
	}
}

// MODE 2
void TalonXIX::CenterSwitchBaseMode()
{

	switch(autoStage)
	{
		case 0: //Reset
			if(targetPositionData[0] == LEFT_POS)
			{
				centerRotateCmd = -50.0;
				centerRotateCount = AUTO_TURN_HALF_SEC;
			}
			else
			{
				centerRotateCmd = 40.0;
				centerRotateCount = AUTO_TURN_HALF_SEC;
			}
			driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
			distance->LocalReset();
			loopCount = 0;
			autoStage++;
			break;

		case 1: //Turn to line up with switch depending on where the switch target is positioned
			if(loopCount < centerRotateCount)
			{
				driveCmd = 0.0; rotateCmd = centerRotateCmd; sideCmd = 0.0;
				loopCount++;
			}
			else
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				lift->GiveGoalPosition(LIFT_SWITCH_POSITION);
				if(targetPositionData[0] == LEFT_POS)
				{
					centerDistance = LEFT_DISTANCE_TO_SWITCH_AT_ANGLE;
				}
				else
				{
					centerDistance = RIGHT_DISTANCE_TO_SWITCH_AT_ANGLE;
				}
				loopCount = 0;
				autoStage++;
			}
			break;

		case 2://Drive from alliance wall to switch at either 20 or 25 degrees depending on the side of the switch
			if(distance->GetForwardsDistance() < centerDistance)
			{
				driveCmd = 0.5; rotateCmd = 0.0; sideCmd = 0.0;
			}
			else
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;
				autoStage++;
			}
			//autoStage++;	//FOR TESTING PURPOSES
			break;

		case 3: //Wait for lift to be at goal position
			if (lift->AtGoalPosition())
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				claw->EjectCube();
				distance->ResetEncoders();
				loopCount = 0;
				autoStage++;
			}
			break;

		case 4: //Release Cube into switch
			if(loopCount < CUBE_LOAD_WAIT)
			{
				loopCount++;
			}
			else
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;
				autoStage++;
			}
			break;

		case 5: //Move on to Level 2
			autoMode = autoLevel2;
			driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
			loopCount = 0;
			autoStage = 0;
			break;
	}
}

// MODE 3
void TalonXIX::SameSideSwitchBaseMode()
{
	switch(autoStage)
	{
		case 0: //Reset
			driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
			distance->ResetEncoders();
			loopCount = 0;
			autoStage++;
			break;

		case 1: //Drives to side of switch, centered
			if(distance->GetForwardsDistance() < DISTANCE_WALL_TO_SWITCH_SIDE)
			{
				driveCmd = 0.6; rotateCmd = 0.0; sideCmd = 0.0;
			}
			else
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				lift->GiveGoalPosition(LIFT_SWITCH_POSITION);
				loopCount = 0;
				autoStage++;
			}
			break;

		case 2: //Turn 90 degrees toward Switch
			if(loopCount < AUTO_TURN_ONE_SEC)
			{
				driveCmd = 0.0; rotateCmd = 90.0; sideCmd = 0.0;
				loopCount++;
			}
			else
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;
				autoStage++;
			}
			break;

		case 3: //Once lined up, drive up against switch
			if(distance->GetForwardsDistance() < DISTANCE_SIDE_LINEDUP_TO_SWITCH)
			{
				driveCmd = 0.5; rotateCmd = 0.0; sideCmd = 0.0;
				//Lift set to switch position
			}
			else
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				claw->EjectCube();
				loopCount = 0;
				autoStage++;
			}
			break;

		case 4: //Release Cube into switch
			if(loopCount < CUBE_LOAD_WAIT)
			{
				loopCount++;
			}
			else
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;
				autoStage++;
			}
			break;

		case 5: //Back up a bit
			if((distance->GetForwardsDistance() * -1) < SMALL_BACK_UP)
			{
				driveCmd = -0.2; rotateCmd = 0.0; sideCmd = 0.0;
			}
			else
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;
				autoStage++;
			}
			break;

		case 6: //Move on to Level 2
			autoMode = autoLevel2;
			driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
			loopCount = 0;
			autoStage = 0;
			break;
	}

	rotateCmd *= autoTurnMultiplier;

}

// MODE 4
void TalonXIX::SameSideScaleBaseMode()
{

	switch(autoStage)
	{
		case 0: //Reset
			driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
			distance->ResetEncoders();
			loopCount = 0;
			autoStage++;
			break;

		case 1: //Drive to corner of scale
			if (distance->GetForwardsDistance() < DISTANCE_WALL_TO_SCALE)
			{
				driveCmd = 0.7; rotateCmd = 0.0; sideCmd = 0.0;
				lift->GiveGoalPosition(LIFT_HIGHEST_POSITION);
			}
			else
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;
				autoStage++;
			}
			break;

		case 2://Rotate to line up at scale
			if(loopCount < AUTO_TURN_ONE_SEC)
			{
				driveCmd = 0.0; rotateCmd = 80.0; sideCmd = 0.0;
				loopCount++;
			}
			else
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;
				autoStage++;
			}
			break;

		case 3: //Forward a bit
			if (distance->GetForwardsDistance() < SMALL_BACK_UP)
			{
				driveCmd = 0.3; rotateCmd = 0.0; sideCmd = 0.0;
			}
			else
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;
				claw->EjectCube();
				autoStage++;
			}

		case 4: //Wait 0.5s
			if(loopCount < CUBE_LOAD_WAIT)
			{
				loopCount++;
			}
			else
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;
				autoStage++;
			}
			break;

		case 5: //Back up a bit
			if((distance->GetForwardsDistance() * -1) < SCALE_BACKUP_SAME_SIDE)
			{
				driveCmd = -0.35; rotateCmd = 0.0; sideCmd = 0.0;
			}
			else
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;
				autoStage++;
			}
			break;

		case 6: //Move on to Level 2
			autoMode = autoLevel2;
			driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
			loopCount = 0;
			autoStage = 0;
			break;
	}

	rotateCmd *= autoTurnMultiplier;

}

// MODE 5
void TalonXIX::FarSideScaleBaseMode()
{

	switch(autoStage)
		{
			case 0: //Reset
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;
				autoStage++;
				break;

			case 1: //Drive to corner of scale
				if (distance->GetForwardsDistance() < DISTANCE_WALL_TO_MID_PLATFORM_ZONE)
				{
					driveCmd = 0.5; rotateCmd = 0.0; sideCmd = 0.0;
				}
				else
				{
					driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
					distance->ResetEncoders();
					loopCount = 0;
					autoStage++;
				}
				break;

			case 2: //Turn 90 degrees
				if(loopCount < AUTO_TURN_ONE_SEC)
				{
					driveCmd = 0.0; rotateCmd = 90.0; sideCmd = 0.0;
					loopCount++;
				}
				else
				{
					driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
					distance->ResetEncoders();
					loopCount = 0;
					autoStage++;
				}
				break;

			case 3: //Cross platform zone to scale
				if(distance->GetForwardsDistance() < DISTANCE_CROSS_PLATFORM_ZONE)
				{
					driveCmd = 0.5; rotateCmd = 0.0; sideCmd = 0.0;
					lift->GiveGoalPosition(LIFT_HIGHEST_POSITION);
				}
				else
				{
					driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
					distance->ResetEncoders();
					loopCount = 0;
					autoStage++;
				}
				break;

			case 4: //Rotate 110 degrees towards scale
				if(loopCount < AUTO_TURN_ONE_SEC)
				{
					driveCmd = 0.0; rotateCmd = -110.0; sideCmd = 0.0;
					loopCount++;
				}
				else
				{
					driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
					distance->ResetEncoders();
					loopCount = 0;
					autoStage++;
				}
				break;

			case 5: //Forward to Scale
				if(distance->GetForwardsDistance() < DISTANCE_MIDPLATFORM_ZONE_TO_SCALE)
				{
					driveCmd = 0.3; rotateCmd = 0.0; sideCmd = 0.0;
				}
				else
				{
					driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
					distance->ResetEncoders();
					loopCount = 0;
					claw->EjectCube();
					autoStage++;
				}
				break;

			case 6: //Wait(0.5)
				if(loopCount < CUBE_LOAD_WAIT)
				{
					loopCount++;
				}
				else
				{
					driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
					distance->ResetEncoders();
					loopCount = 0;
					autoStage++;
				}
				break;

			case 7: //Back up a little
				if((distance->GetForwardsDistance() * -1.0) < SCALE_BACKUP)
				{
					driveCmd = -0.3; rotateCmd = 0.0; sideCmd = 0.0;
					lift->GiveGoalPosition(LIFT_PICKUP_POSITION);
				}
				else
				{
					driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
					distance->ResetEncoders();
					loopCount = 0;
					autoStage++;
				}
				break;

			case 8: //Turn 135 degrees to cube
				if (loopCount < AUTO_TURN_1p5_SEC)
				{
					driveCmd = 0.0; rotateCmd = -90.0; sideCmd = 0.0;
					loopCount++;
				}
				else
				{
					driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
					distance->ResetEncoders();
					loopCount = 0;
					autoStage++;
				}
				break;

			case 9: //Drive to side cube while grabbing
				if (distance->GetForwardsDistance() < DISTANCE_FAR_SIDE_SCALE_TO_SIDECUBE)
				{
					driveCmd = 0.4; rotateCmd = 0.0; sideCmd = 0.0;
					//Bring lift to ground position
					if(loopCount > AUTO_CLAW_OPEN_DELAY)
					{
						claw->EjectCube();
					}
					else
					{
						loopCount++;
					}
				}
				else
				{
					driveCmd = 0.1; rotateCmd = 0.0; sideCmd = 0.0;
					distance->ResetEncoders();
					claw->GatherCube();
					loopCount = 0;
					autoStage++;
				}
				break;

			case 10: //Keep grabbing until we have cube
				if (claw->HaveCube()) //Once we have the cube, move on
				{
					driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
					distance->ResetEncoders();
					lift->GiveGoalPosition(LIFT_SWITCH_POSITION);
					loopCount = 0;
					autoStage++;
				}
				break;

			case 11: //Move on to Level 2
				autoMode = autoLevel2;
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				loopCount = 0;
				autoStage = 0;
				break;
		}

		rotateCmd *= autoTurnMultiplier;
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////  AUTO LEVEL 2  //////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// MODE 10
void TalonXIX::CenterMultiCubeSwitchMode()
{

	switch(autoStage)
	{
		case 0: //Reset
			if(targetPositionData[0] == LEFT_POS)
			{
				centerRotateCmd = 55.0;
			}
			else
			{
				centerRotateCmd = 50.0;
			}
			driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
			distance->ResetEncoders();
			loopCount = 0;
			autoStage++;
			break;

		case 1: //Back up from switch
			if ((distance->GetForwardsDistance() * -1) < DISTANCE_SWITCH_TO_PYRAMID)
			{
				driveCmd = -0.4; rotateCmd = 0.0; sideCmd = 0.0;
				lift->GiveGoalPosition(LIFT_PICKUP_POSITION);
			}
			else
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;
				autoStage++;
			}
			break;

		case 2: //Rotate 90 degrees towards cube pyramid
			if (loopCount < AUTO_TURN_ONE_SEC)
			{
				driveCmd = 0.0; rotateCmd = centerRotateCmd; sideCmd = 0.0;
				loopCount++;
			}
			else
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				claw->EjectCube();
				loopCount = 0;
				autoStage++;
			}
			break;

		case 3: //Drive to cube pyramid
			if (distance->GetForwardsDistance() < DISTANCE_TO_PYRAMID_CENTER_CUBE)
			{
				driveCmd = 0.4; rotateCmd = 0.0; sideCmd = 0.0;
			}
			else
			{
				driveCmd = 0.1; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;
				claw->GatherCube();
				autoStage++;
			}
			break;

		case 4: //Keep grabbing cube until claw has cube
			if (claw->HaveCube()) //Once we have cube we move on
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;
				autoStage++;
			}
			break;

		case 5: //Drive back to switch
			if ((distance->GetForwardsDistance() * -1) < DISTANCE_TO_PYRAMID_CENTER_CUBE)
			{
				driveCmd = -0.4; rotateCmd = 0.0; sideCmd = 0.0;
			}
			else
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;
				autoStage++;
			}
			break;

		case 6: //Turn to switch
			if (loopCount < AUTO_TURN_ONE_SEC)
			{
				driveCmd = 0.0; rotateCmd = (centerRotateCmd - 5) * -1; sideCmd = 0.0;
				loopCount++;
			}
			else
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;
				autoStage++;
			}
			break;

		case 7: //Drive to switch
			if (distance->GetForwardsDistance() < DISTANCE_PYRAMID_TO_SWITCH)
			{
				driveCmd = 0.4; rotateCmd = 0.0; sideCmd = 0.0;
				lift->GiveGoalPosition(LIFT_SWITCH_POSITION);
			}
			else
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;
				claw->EjectCube();
				autoStage++;
			}
			break;

		case 8: //Wait(0.5)
			if (loopCount < CUBE_LOAD_WAIT)
			{
				loopCount++;
			}
			else
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;
				autoStage++;
			}
			break;
	}

	rotateCmd *= autoTurnMultiplier;
}

// MODE 11
void TalonXIX::SameSideScaleThenSwitchMode()
{

	switch(autoStage)
	{
		case 0: //Reset
			driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
			distance->ResetEncoders();
			loopCount = 0;
			autoStage++;
			break;

		case 1: //Turn 70 degrees to cube
			if (loopCount < AUTO_TURN_ONE_SEC)
			{
				driveCmd = 0.0; rotateCmd = 70.0; sideCmd = 0.0;
				lift->GiveGoalPosition(LIFT_PICKUP_POSITION);
				loopCount++;
			}
			else
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;
				autoStage++;
			}
			break;

		case 2: //Drive to side cube while grabbing
			if (distance->GetForwardsDistance() < DISTANCE_SAME_SIDE_SCALE_TO_SIDECUBE)
			{
				driveCmd = 0.35; rotateCmd = 0.0; sideCmd = 0.0;
				//Open Claw when we are within 10 inches
				if(DISTANCE_SAME_SIDE_SCALE_TO_SIDECUBE - distance->GetForwardsDistance() < 10.0)
				{
					claw->ReleaseCube();
				}
			}
			else
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;
				autoStage++;
			}
			break;

		case 3: //Check for at Goal
			if(lift->AtGoalPosition())
			{
				claw->GatherCube();
				driveCmd = 0.1; rotateCmd = 0.0; sideCmd = 0.0;
				loopCount = 0;
				autoStage++;
			}
			break;

		case 4: //Wait before Lifting
			if (loopCount < WAIT_TO_GRAB_CUBE)
			{
				loopCount++;
			}
			else
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;
				autoStage++;
			}
			break;

		case 5: //Keep grabbing until we have cube
			if (claw->HaveCube()) //Once we have the cube, move on
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				lift->GiveGoalPosition(LIFT_SWITCH_POSITION);
				loopCount = 0;
				autoStage++;
			}
			break;

		case 6: //Check for at height
			if(lift->AtGoalPosition())
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;
				autoStage++;
			}
			break;

		case 7: //Drive to switch
			if(distance->GetForwardsDistance() < SWITCH_DEPLOY_FORWARD)
			{
				driveCmd = 0.3; rotateCmd = 0.0; sideCmd = 0.0;
			}
			else
			{
				driveCmd = 0.3; rotateCmd = 0.0; sideCmd = 0.0;
				//distance->ResetEncoders();
				loopCount = 0;
				autoStage++;
			}
			break;

		case 8: //Wait before ejecting
			if(loopCount > AUTO_AT_GOAL_DELAY)
			{
				claw->EjectCube();
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;
				autoStage++;
			}
			else
			{
				loopCount++;
			}
			break;
	}

	sideCmd *= autoTurnMultiplier;
	rotateCmd *= autoTurnMultiplier;
}

// MODE 12
void TalonXIX::SideMultiCubeScaleMode()
{

	switch(autoStage)
	{
		case 0: //Reset
			driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
			distance->ResetEncoders();
			loopCount = 0;
			autoStage++;
			break;

		case 1: //Turn 70 degrees to cube
			if (loopCount < AUTO_TURN_ONE_SEC)
			{
				driveCmd = 0.0; rotateCmd = 70.0; sideCmd = 0.0;
				lift->GiveGoalPosition(LIFT_PICKUP_POSITION);
				loopCount++;
			}
			else
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;
				autoStage++;
			}
			break;

		case 2: //Drive to side cube while grabbing
			if (distance->GetForwardsDistance() < DISTANCE_SAME_SIDE_SCALE_TO_SIDECUBE_TO_SCALE)
			{
				driveCmd = 0.35; rotateCmd = 0.0; sideCmd = 0.0;
				//Open Claw when we are within 10 inches
				if(DISTANCE_SAME_SIDE_SCALE_TO_SIDECUBE_TO_SCALE - distance->GetForwardsDistance() < 20.0)
				{
					claw->ReleaseCube();
				}
			}
			else
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;
				autoStage++;
			}
			break;

		case 3: //Check for height
			if(lift->AtGoalPosition())
			{
				claw->GatherCube();
				driveCmd = 0.1; rotateCmd = 0.0; sideCmd = 0.0;
				loopCount = 0;
				autoStage++;
			}
			break;

		case 4:
			if (loopCount < WAIT_TO_GRAB_CUBE)
			{
				loopCount++;
			}
			else
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;
				autoStage++;
			}
			break;

		case 5: //Keep grabbing until we have cube
			if (claw->HaveCube()) //Once we have the cube, move on
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				lift->GiveGoalPosition(LIFT_HIGHEST_POSITION);
				loopCount = 0;
				autoStage++;
			}
			break;

		case 6: //Back from side cube
			if((distance->GetForwardsDistance() * -1.0) < REVERSE_TO_SCALE_FROM_SWITCH)
			{
				driveCmd = -0.5; rotateCmd = 0.0; sideCmd = 0.0;
				lift->GiveGoalPosition(LIFT_HIGHEST_POSITION);
			}
			else
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;
				autoStage++;
			}
			break;

		case 7: //Turn around 95 degrees towards scale
			if (loopCount < AUTO_TURN_ONE_SEC)
			{
				driveCmd = 0.0; rotateCmd = -100.0; sideCmd = 0.0; //-95.0
				loopCount++;
			}
			else
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;
				autoStage++;
			}
			break;

		case 8: //Check for height
			if(lift->AtGoalPosition())
			{
				distance->ResetEncoders();
				loopCount = 0;
				autoStage++;
			}
			break;

		case 9: //Small forward and eject
			if((distance->GetForwardsDistance()) < SMALL_FORWARD_TO_SCALE)
			{
				driveCmd = 0.3; rotateCmd = 0.0; sideCmd = 0.0;
			}
			else
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				claw->EjectCube();
				distance->ResetEncoders();
				loopCount = 0;
				autoStage++;
			}
			break;

		case 10: //Wait(0.5)
			if(loopCount > WAIT_BEFORE_BACK_FROM_SCALE)
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				autoStage++;
				loopCount = 0;
			}
			else
			{
				loopCount++;
			}
			break;

		case 11: //Back from Scale
			if((distance->GetForwardsDistance() * -1.0) < SMALL_FORWARD_TO_SCALE)
			{
				driveCmd = -0.3; rotateCmd = 0.0; sideCmd = 0.0;
			}
			else
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;
				autoStage++;
			}
			break;

		case 12: //Reset
			driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
			distance->ResetEncoders();
			loopCount = 0;
			autoStage++;
			break;

	}

	sideCmd *= autoTurnMultiplier;
	rotateCmd *= autoTurnMultiplier;
}

// MODE 13
void TalonXIX::FarSideScaleThenSwitch()
{
	switch(autoStage)
	{
		case 0: //Reset
			driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
			distance->ResetEncoders();
			loopCount = 0;
			autoStage++;
			break;

		case 1: //Drive a bit against switch
			if(lift->AtGoalPosition())
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;
				autoStage++;
			}
			break;

		case 2:
			if(distance->GetForwardsDistance() < SWITCH_DEPLOY_FORWARD)
			{
				driveCmd = 0.2; rotateCmd = 0.0; sideCmd = 0.0;
			}
			else
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;
				autoStage++;
			}
			break;

		case 3:
			if(loopCount > AUTO_AT_GOAL_DELAY)
			{
				claw->EjectCube();
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;
				autoStage++;
			}
			else
			{
				loopCount++;
			}
			break;
	}

		sideCmd *= autoTurnMultiplier;
		rotateCmd *= autoTurnMultiplier;
}

// MODE 14
void TalonXIX::LeftCenterSwitchThenExchangeMode()
{
	switch(autoStage)
	{
		case 0: //Reset
			if(targetPositionData[0] == LEFT_POS)
			{
				centerRotateCmd = 55.0;
			}
			else
			{
				centerRotateCmd = 50.0;
			}
			driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
			distance->ResetEncoders();
			loopCount = 0;
			autoStage++;
			break;

		case 1: //Back up from switch
			if ((distance->GetForwardsDistance() * -1.0) < DISTANCE_SWITCH_TO_PYRAMID)
			{
				driveCmd = -0.4; rotateCmd = 0.0; sideCmd = 0.0;
				lift->GiveGoalPosition(LIFT_PICKUP_POSITION);
			}
			else
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;
				autoStage++;
			}
			break;

		case 2: //Rotate 90 degrees towards cube pyramid
			if (loopCount < AUTO_TURN_ONE_SEC)
			{
				driveCmd = 0.0; rotateCmd = centerRotateCmd; sideCmd = 0.0;
				loopCount++;
			}
			else
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				claw->EjectCube();
				loopCount = 0;
				autoStage++;
			}
			break;

		case 3: //Drive to cube pyramid
			if (distance->GetForwardsDistance() < DISTANCE_TO_PYRAMID_CENTER_CUBE)
			{
				driveCmd = 0.4; rotateCmd = 0.0; sideCmd = 0.0;
			}
			else
			{
				driveCmd = 0.1; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;
				claw->GatherCube();
				autoStage++;
			}
			break;

		case 4: //Keep grabbing cube until claw has cube
			if (claw->HaveCube()) //Once we have cube we move on
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;
				autoStage++;
			}
			break;

		case 5: //Drive back to switch
			if ((distance->GetForwardsDistance() * -1) < DISTANCE_TO_PYRAMID_CENTER_CUBE)
			{
				driveCmd = -0.4; rotateCmd = 0.0; sideCmd = 0.0;
			}
			else
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;
				autoStage++;
			}
			break;

		case 6: //Turn to exchange
			if (loopCount < AUTO_TURN_1p5_SEC)
			{
				driveCmd = 0.0; rotateCmd = 103.0; sideCmd = 0.0;
				loopCount++;
			}
			else
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;
				autoStage++;
			}
			break;

		case 7: //Drive to Exchange
			if (distance->GetForwardsDistance() < DISTANCE_PYRAMID_TO_EXCHANGE)
			{
				driveCmd = 0.4; rotateCmd = 0.0; sideCmd = 0.0;
				lift->GiveGoalPosition(2.5);
			}
			else
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				claw->EjectCube();
				loopCount = 0;
				autoStage++;
			}
			break;

		case 8: //Wait to load cube in exchange
			if (loopCount < CUBE_LOAD_WAIT)
			{
				loopCount++;
			}
			else
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;
				autoStage++;
			}
			break;

		case 9: //Drive away from exchange
			if ((distance->GetForwardsDistance() * -1) < (DISTANCE_PYRAMID_TO_EXCHANGE/2.0))
			{
				driveCmd = -0.4; rotateCmd = 0.0; sideCmd = 0.0;
			}
			else
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;
				autoStage++;
			}
			break;

		case 10: //Rotate back to pyramid
			if (loopCount < AUTO_TURN_1p5_SEC)
			{
				driveCmd = 0.0; rotateCmd = -103.0; sideCmd = 0.0;
				lift->GiveGoalPosition(LIFT_PICKUP_POSITION);
				loopCount++;
			}
			else
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;
				autoStage++;
			}
			break;
	}

	rotateCmd *= autoTurnMultiplier;
}

// MODE 14
void TalonXIX::RightCenterSwitchThenExchangeMode()
{
	switch(autoStage)
	{
		case 0: //Reset
			if(targetPositionData[0] == LEFT_POS)
			{
				centerRotateCmd = 55.0;
			}
			else
			{
				centerRotateCmd = 50.0;
			}
			driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
			distance->ResetEncoders();
			loopCount = 0;
			autoStage++;
			break;

		case 1: //Back up from switch
			if ((distance->GetForwardsDistance() * -1.0) < DISTANCE_SWITCH_TO_PYRAMID)
			{
				driveCmd = -0.4; rotateCmd = 0.0; sideCmd = 0.0;
				lift->GiveGoalPosition(LIFT_PICKUP_POSITION);
			}
			else
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;
				autoStage++;
			}
			break;

		case 2: //Rotate 90 degrees towards cube pyramid
			if (loopCount < AUTO_TURN_ONE_SEC)
			{
				driveCmd = 0.0; rotateCmd = centerRotateCmd; sideCmd = 0.0;
				loopCount++;
			}
			else
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				claw->EjectCube();
				loopCount = 0;
				autoStage++;
			}
			break;

		case 3: //Drive to cube pyramid
			if (distance->GetForwardsDistance() < RIGHT_DISTANCE_TO_PYRAMID_CENTER_CUBE)
			{
				driveCmd = 0.4; rotateCmd = 0.0; sideCmd = 0.0;
			}
			else
			{
				driveCmd = 0.1; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;
				claw->GatherCube();
				autoStage++;
			}
			break;

		case 4: //Keep grabbing cube until claw has cube
			if (claw->HaveCube()) //Once we have cube we move on
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;
				autoStage++;
			}
			break;

		case 5: //Drive back to switch
			if ((distance->GetForwardsDistance() * -1) < RIGHT_DISTANCE_TO_PYRAMID_CENTER_CUBE)
			{
				driveCmd = -0.4; rotateCmd = 0.0; sideCmd = 0.0;
			}
			else
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;
				autoStage++;
			}
			break;

		case 6: //Turn to exchange
			if (loopCount < AUTO_TURN_ONE_SEC)
			{
				driveCmd = 0.0; rotateCmd = 80.0; sideCmd = 0.0;
				loopCount++;
			}
			else
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;
				autoStage++;
			}
			break;

		case 7: //Drive to exchange
			if (distance->GetForwardsDistance() < RIGHT_DISTANCE_PYRAMID_TO_EXCHANGE)
			{
				driveCmd = 0.4; rotateCmd = 0.0; sideCmd = 0.0;
			}
			else
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;
				autoStage++;
			}
			break;

		case 8: //Turn to Lineup with Exchange
			if (loopCount < AUTO_TURN_ONE_SEC)
			{
				driveCmd = 0.0; rotateCmd = 70.0; sideCmd = 0.0;
				loopCount++;
				lift->GiveGoalPosition(2.5);
			}
			else
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;
				autoStage++;
			}
			break;

		case 9: //Drive a bit forward into exchange
			if (distance->GetForwardsDistance() < RIGHT_DISTANCE_INCH_TO_EXCHANGE)
			{
				driveCmd = 0.2; rotateCmd = 0.0; sideCmd = 0.0;
			}
			else
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				claw->EjectCube();
				loopCount = 0;
				autoStage++;
			}
			break;

		case 10: //Wait to load cube in exchange
			if (loopCount < CUBE_LOAD_WAIT)
			{
				loopCount++;
			}
			else
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;
				autoStage++;
			}
			break;

		case 11: //Drive away from exchange
			if ((distance->GetForwardsDistance() * -1) < (RIGHT_DISTANCE_PYRAMID_TO_EXCHANGE/3.0))
			{
				driveCmd = -0.4; rotateCmd = 0.0; sideCmd = 0.0;
			}
			else
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;
				autoStage++;
			}
			break;

		case 12: //Rotate back to pyramid
			if (loopCount < AUTO_TURN_ONE_SEC)
			{
				driveCmd = 0.0; rotateCmd = -70.0; sideCmd = 0.0;
				loopCount++;
				lift->GiveGoalPosition(LIFT_PICKUP_POSITION);
			}
			else
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;
				autoStage++;
			}
			break;

	}

	rotateCmd *= autoTurnMultiplier;
}

// MODE 15
void TalonXIX::SameSideSwitchThenFarScale()
{
	switch(autoStage)
	{
		case 0: //Reset
			driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
			distance->ResetEncoders();
			loopCount = 0;
			autoStage++;
			break;

		case 1: //Turn 90 degrees to scale
			if (loopCount < AUTO_TURN_ONE_SEC)
			{
				driveCmd = 0.0; rotateCmd = -90.0; sideCmd = 0.0;
				loopCount++;
			}
			else
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;
				autoStage++;
			}
			break;

		case 2: //Forward past corner cube
			if ((distance->GetForwardsDistance()) < FORWARD_TO_CORNER_OF_SWITCH)
			{
				driveCmd = 0.3; rotateCmd = 0.0; sideCmd = 0.0;
				lift->GiveGoalPosition(LIFT_PICKUP_POSITION);
			}
			else
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;
				autoStage++;
			}
			break;

		case 3: //Rotate to corner cube
			if (loopCount < AUTO_TURN_ONE_SEC)
			{
				driveCmd = 0.0; rotateCmd = 130.0; sideCmd = 0.0;
				loopCount++;
			}
			else
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;
				autoStage++;
			}
			break;

		case 4: //Drive to corner cube
			if ((distance->GetForwardsDistance()) < FORWARD_TO_CORNER_CUBE)
			{
				driveCmd = 0.3; rotateCmd = 0.0; sideCmd = 0.0;
				claw->ReleaseCube();
			}
			else
			{
				driveCmd = 0.1; rotateCmd = 0.0; sideCmd = 0.0;
				claw->GatherCube();
				distance->ResetEncoders();
				loopCount = 0;
				autoStage++;
			}
			break;

		case 5: //Check if have cube
			if (claw->HaveCube()) //Once we have the cube, move on
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;
				autoStage++;
			}
			break;

		case 6: //Back from corner cube
			if ((distance->GetForwardsDistance() * -1.0) < BACK_FROM_CORNER_CUBE)
			{
				driveCmd = -0.35; rotateCmd = 0.0; sideCmd = 0.0;
			}
			else
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;
				autoStage++;
			}
			break;

		case 7: //Turn to cross platform
			if (loopCount < TURN_TIME_BACK_FROM_CORNER_CUBE)
			{
				driveCmd = 0.0; rotateCmd = -84.0; sideCmd = 0.0;
				loopCount++;
			}
			else
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;
				autoStage++;
			}
			break;

		case 8: //Cross platform zone
			if ((distance->GetForwardsDistance()) < DISTANCE_CROSS_PLATFORM)
			{
				driveCmd = 0.5; rotateCmd = 0.0; sideCmd = 0.0;
				lift->GiveGoalPosition(LIFT_HIGHEST_POSITION);
			}
			else
			{
				driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;
				autoStage++;
			}
			break;

		case 9: //Reset
			driveCmd = 0.0; rotateCmd = 0.0; sideCmd = 0.0;
			distance->ResetEncoders();
			loopCount = 0;
			break;
	}

	rotateCmd *= autoTurnMultiplier;
}
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////  MODE SELECTION  /////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void TalonXIX::UpdateModeSelections(void)
{
	bool modeChange = false;

	int tempLL1 = LL_Level1; int tempLL2 = LL_Level2;
	int tempLR1 = LR_Level1; int tempLR2 = LR_Level2;
	int tempRL1 = RL_Level1; int tempRL2 = RL_Level2;
	int tempRR1 = RR_Level1; int tempRR2 = RR_Level2;

	// Read current values from the driverstation screen
	autoPosition = *((char*)AutoPositionChooser->GetSelected());
	autoPositionString[0] = autoPosition;
	SmartDashboard::PutString("Auto Pos: ", autoPositionString);

	autoSetupChoice = *((int*)AutoSetupChooser->GetSelected());
	SmartDashboard::PutNumber("Setup Mode: ", autoSetupChoice);

	// assign values based on position if default is selected
	if (autoSetupChoice == defaultSetup)
	{
		if (autoPosition == CENTER_POS)
		{
			LL_Level1 = centerDefaults[0]; LL_Level2 = centerDefaults[1];    LR_Level1 = centerDefaults[2]; LR_Level2 = centerDefaults[3];
			RL_Level1 = centerDefaults[4]; RL_Level2 = centerDefaults[5];    RR_Level1 = centerDefaults[6]; RR_Level2 = centerDefaults[7];
		}
		else if (autoPosition == LEFT_POS)
		{
			LL_Level1 = leftDefaults[0]; LL_Level2 = leftDefaults[1];        LR_Level1 = leftDefaults[2]; LR_Level2 = leftDefaults[3];
			RL_Level1 = leftDefaults[4]; RL_Level2 = leftDefaults[5];	     RR_Level1 = leftDefaults[6]; RR_Level2 = leftDefaults[7];
		}
		else
		{
			LL_Level1 = rightDefaults[0]; LL_Level2 = rightDefaults[1];      LR_Level1 = rightDefaults[2]; LR_Level2 = rightDefaults[3];
			RL_Level1 = rightDefaults[4]; RL_Level2 = rightDefaults[5];      RR_Level1 = rightDefaults[6]; RR_Level2 = rightDefaults[7];
		}

		// Update the settings on the screen (so the user can see them)
		SmartDashboard::PutNumber("LL Level 1", LL_Level1);    SmartDashboard::PutNumber("LL Level 2", LL_Level2);
		SmartDashboard::PutNumber("LR Level 1", LR_Level1);    SmartDashboard::PutNumber("LR Level 2", LR_Level2);
		SmartDashboard::PutNumber("RL Level 1", RL_Level1);    SmartDashboard::PutNumber("RL Level 2", RL_Level2);
		SmartDashboard::PutNumber("RR Level 1", RR_Level1);    SmartDashboard::PutNumber("RR Level 2", RR_Level2);

	}
	else
	{
		// Assign values based on custom choice
		LL_Level1 = (int)(SmartDashboard::GetNumber("LL Level 1", 0));    LL_Level2 = (int)(SmartDashboard::GetNumber("LL Level 2", 0));
		LR_Level1 = (int)(SmartDashboard::GetNumber("LR Level 1", 0));    LR_Level2 = (int)(SmartDashboard::GetNumber("LR Level 2", 0));
		RL_Level1 = (int)(SmartDashboard::GetNumber("RL Level 1", 0));    RL_Level2 = (int)(SmartDashboard::GetNumber("RL Level 2", 0));
		RR_Level1 = (int)(SmartDashboard::GetNumber("RR Level 1", 0));    RR_Level2 = (int)(SmartDashboard::GetNumber("RR Level 2", 0));
	}

	//Check to see if any of the selections changed
	if((LL_Level1 != tempLL1) || (LL_Level2 != tempLL2))
	{
		modeChange = true;
	}
	if((LR_Level1 != tempLR1) || (LR_Level2 != tempLR2))
	{
		modeChange = true;
	}
	if((RL_Level1 != tempRL1) || (RL_Level2 != tempRL2))
	{
		modeChange = true;
	}
	if((RR_Level1 != tempRR1) || (RR_Level2 != tempRR2))
	{
		modeChange = true;
	}

	//if any mode changed, print out setup for verification
	if (modeChange)
	{
		printf("\nMODE SETUP:\n");
		printf("LL 1: %d %s", LL_Level1, PrintMode(LL_Level1));
		printf("  2: %d %s\n", LL_Level2, PrintMode(LL_Level2));

		printf("LR 1: %d %s", LR_Level1, PrintMode(LR_Level1));
		printf("  2: %d %s\n", LR_Level2, PrintMode(LR_Level2));

		printf("RL 1: %d %s", RL_Level1, PrintMode(RL_Level1));
		printf("  2: %d %s\n", RL_Level2, PrintMode(RL_Level2));

		printf("RR 1: %d %s", RR_Level1, PrintMode(RR_Level1));
		printf("  2: %d %s\n", RR_Level2, PrintMode(RR_Level2));
	}
}


bool TalonXIX::MatchModeAssignments(void)
{
	targetPositionData = frc::DriverStation::GetInstance().GetGameSpecificMessage();

	// Based on where the Scale/Switch targets ended up being, assign both auto mode levels
	if ((targetPositionData[0] == LEFT_POS) && (targetPositionData[1] == LEFT_POS))
	{
		autoMode   = LL_Level1;
		autoLevel2 = LL_Level2;
	}
	else if ((targetPositionData[0] == LEFT_POS) && (targetPositionData[1] == RIGHT_POS))
	{
		autoMode   = LR_Level1;
		autoLevel2 = LR_Level2;
	}
	else if ((targetPositionData[0] == RIGHT_POS) && (targetPositionData[1] == LEFT_POS))
	{
		autoMode   = RL_Level1;
		autoLevel2 = RL_Level2;
	}
	else if((targetPositionData[0] == RIGHT_POS) && (targetPositionData[1] == RIGHT_POS))
	{
		autoMode   = RR_Level1;
		autoLevel2 = RR_Level2;
	}

	// If the input was negative, use as auto delay time
	if (autoMode < 0)
	{
		autoDelay = autoMode * -0.1;
	}
	else
	{
		autoDelay = 0.0;
	}

	// Assign rotation multiplier, Based on switch assignment if in center, or based on your position if on side
	if (autoPosition == CENTER_POS)
	{
		if (targetPositionData[0] == LEFT_POS)
		{
			autoTurnMultiplier = 1.0;
		}
		else
		{
			autoTurnMultiplier = -1.0;
		}
	}
	else
	{
		if (autoPosition == LEFT_POS)
		{
			autoTurnMultiplier = 1.0;
		}
		else
		{
			autoTurnMultiplier = -1.0;
		}
	}


	//Print out Level 1 mode
	printf("Mode 1: %s\n", PrintMode(autoMode));

	//Print out Level 2 Mode
	printf("Mode 2: %s\n", PrintMode(autoLevel2));

	if ((targetPositionData[0] != 'L' && targetPositionData[0] != 'R') && (targetPositionData[1] != 'L' && targetPositionData[1] != 'R'))
	{
		return false;
	}
	return true;
}


const char* TalonXIX::PrintMode(int mode)
{
	if (mode >= 0)
	{
		switch(mode)
		{
			case 1:
				return("ReachBaseline");
				break;

			case 2:
				return("CenterSwitchBase");
				break;

			case 3:
				return("SideSwitchBase");
				break;

			case 4:
				return("SideScaleBase");
				break;

			case 5:
				return("FarScaleBase");
				break;

			case 10:
				return("Center2CubeSwitch");
				break;

			case 11:
				return("Scale&Switch");
				break;

			case 12:
				return("Side2CubeScale");
				break;

			case 13:
				return("FarScale&Switch");
				break;

			case 14:
				return("CtrSwitch&Exchange");
				break;

			case 15:
				return("SideSwitch&FarScale");
				break;

			default:
			case 0:
				return("Do nothing");
				break;
		}
	}
	else
	{
		return("Delay * 0.1");
	}
	return ("");	// code will not come here
}

