/*
 * TalonXIX_main.h
 *
 *  Created on: Jan 13, 2018
 *      Author: Gaelhawks
 */

#ifndef SRC_TALONXIX_MAIN_H_
#define SRC_TALONXIX_MAIN_H_

#include "iostream"
#include "string"
#include "SampleRobot.h"
#include "SmartDashboard/SendableChooser.h"
#include "SmartDashboard/SmartDashboard.h"
#include "WPILib.h"

#include "AutonomousModes.h"
#include "Claw.h"
#include "Climber.h"
#include "Common.h"
#include "CubeLift.h"
#include "DriveDistance.h"
#include "DriveTrain.h"
#include "LightController.h"
#ifdef USE_PIXY
#include "PixyCameraI2C.h"
#endif


class TalonXIX: public frc::SampleRobot
{
public:
	Joystick *joystick;
	bool isJoystickCmd;
	Joystick *gamepad;
    PowerDistributionPanel *pdp;

	Claw *claw;
	Climber *climber;
	CubeLift *lift;
	DriveTrain *drive;
	DriveDistance *distance;
	LightController *lights;


#ifdef USE_PIXY
	PixyJunk *cubeCamera;
	int testBlock;
#endif


	int dashCounter;
	int loopCount;
	double loopTime;
	double startTime;
	double curTime;
	double waitTime;
	bool loopOverTime;
	bool isBlueAlliance;
	int NumLoopsOverTime;
	bool isAuto;
	bool firstTime;

	double rotateCmd;
	double driveCmd;
	double sideCmd;
	double centerRotateCmd;
	double centerRotateCount;
	double centerDistance;

	/*----------------- vv AUTONOMOUS CHOOSERS vv -----------------*/
	frc::SendableChooser<char*> *AutoPositionChooser;
	frc::SendableChooser<int*> *AutoSetupChooser;

	std::string targetPositionData;
	char autoPositionString[2];
	char autoPosition;
	int autoSetupChoice;
	double autoTurnMultiplier;
	int autoMode;
	int autoLevel2;
	double autoDelay;
	int autoStage;
	bool validRead;

	int LL_Level1;
	int LL_Level2;
	int LR_Level1;
	int LR_Level2;
	int RL_Level1;
	int RL_Level2;
	int RR_Level1;
	int RR_Level2;
						   // LL1  LL2  LR1  LR2  RL1  RL2  RR1  RR2
	int centerDefaults[8] = {   2,  14,   2,  14,   2,  14,   2,  14};
	int leftDefaults[8]   = {   4,  11,   3,  15,   4,  12,   5,  13};
	int rightDefaults[8]  = {   5,  13,   4,  12,   3,  15,   4,  11};

private:
	char LEFT_POS = 'L';
	char CENTER_POS = 'C';
	char RIGHT_POS = 'R';

	int defaultSetup = 0;
	int customSetup = 1;
	/*----------------- ^^ AUTONOMOUS CHOOSERS ^^ -----------------*/


public:
	TalonXIX();
	void RobotInit(void);
	void InitializeAlliance(void);
	void Autonomous(void);
	void OperatorControl(void);
	void Disabled(void);
	void Test(void);
	static double Limit(double min, double max, double curValue);
	void RobotStartConfig(void);
	void Omnicide(void);
	void CommunityService(void);
	void ServiceDash(void);
	void StopAll(void);


	// Autonomous Decider Functions: (see AutonomousModes.cpp)
	void UpdateModeSelections(void);
	bool MatchModeAssignments(void);
	const char* PrintMode(int mode);

	// Autonomous Level 1 Functions: (see AutonomousModes.cpp)
	void WaitDelayOrDoNothingMode(void);
	void ReachBaselineMode(void);
	void CenterSwitchBaseMode(void);
	void SameSideSwitchBaseMode(void);
	void SameSideScaleBaseMode(void);
	void FarSideScaleBaseMode(void);

	// Autonomous Level 2 Functions: (see AutonomousModes.cpp)
	void CenterMultiCubeSwitchMode(void);
	void SameSideScaleThenSwitchMode(void);
	void SideMultiCubeScaleMode(void);
	void FarSideScaleThenSwitch(void);
	void LeftCenterSwitchThenExchangeMode(void);
	void RightCenterSwitchThenExchangeMode(void);
	void SameSideSwitchThenFarScale(void);

};


#endif /* SRC_TALONXIX_MAIN_H_ */
