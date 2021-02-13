#ifndef CLIMBER_H_
#define CLIMBER_H_
/*
 * Climber.h
 *
 *  Created on: Jan 7, 2015
 *      Author: Gaelhawks
 */
#include "Common.h"
#include "TalonXIX_main.h"

class TalonXIX;

#define CLIMBER_DISTANCE_PER_PULSE			(2.0 * PI / CLIMBER_PULSES_PER_REVOLUTION)
#define CLIMBER_PULSES_PER_REVOLUTION		(256.0)

#define CLIMBER_SPEED						(10.0)
#define CLIMBER_MAX_DELTA_POS				(CLIMBER_SPEED * LOOPTIME)

//#ifdef PRACTICE_BOT
#define MAX_CLIMBER_EXTENSION				(45.0)
//#else
//#define MAX_CLIMBER_EXTENSION				(41.0)
//#endif

#define MIN_CLIMBER_EXTENSION				(1.0)
#define CLIMBER_TOP_STOP_HEIGHT				(MAX_CLIMBER_EXTENSION - CLIMBER_ALLOWANCE)
#define CLIMBER_BOTTOM_STOP_HEIGHT			(MIN_CLIMBER_EXTENSION + CLIMBER_ALLOWANCE)
#define CLIMBER_ALLOWANCE					(0.1)
#define CLIMBER_RELEASE_CONSTANT			(0.5)
#define CLIMBER_KP							(5.0) //(2.0)
#define CLIMBER_KFF							(0.0)
#define CLIMBER_MIN_MOTOR_CMD				(-1.0)
#define CLIMBER_MAX_MOTOR_CMD				(1.0)
#define CLIMBER_K_DEADBAND					(10.0)
#define CLIMBER_K							(18.0)
#define CLIMBER_TAU							(0.01)
// #define CLIMBER_MIN_VEL_ERROR_INT			(-1.0 * CLIMBER_K_DEADBAND/CLIMBER_K)
// #define CLIMBER_MAX_VEL_ERROR_INT			(CLIMBER_K_DEADBAND/CLIMBER_K)
#define CLIMBER_MIN_VEL_ERROR_INT			(-1.0 * CLIMBER_K/CLIMBER_K_DEADBAND)
#define CLIMBER_MAX_VEL_ERROR_INT			(CLIMBER_K/CLIMBER_K_DEADBAND)
#define CLIMBER_MANUAL_DELTA				(0.95 * CLIMBER_MAX_DELTA_POS)

#define CLIMBER_CALIBRATION_CMD				(-0.25)

#define CLIMBER_MIN_POS_ERROR				(-80.0)
#define CLIMBER_MAX_POS_ERROR				(80.0)
#define CLIMBER_MIN_VELOCITY				(-35.0)
#define CLIMBER_MAX_VELOCITY				(35.0)

// used in TalonXIX_main
#define CLIMBER_RELEASE_HEIGHT				MAX_CLIMBER_EXTENSION //(40.0)

class Climber
{
	private:
		VictorSP *climberMotor;
		Encoder *climbEncoder;

		TalonXIX *mainRobot;

		double motorSpeed;

		double positionCmd;
		double goalPosition;
		double currentPosition;
		double posError;
		double oldPosition;
		double curVelocity;
		double velocityError;
		double velocityCmd;
		double velocityErrorInt;
		bool atClGoal;
		bool isInitialized;
		int initStage;
		int curCount;
		int lastCount;
		int loopCount;
		bool isManualMove;
		double oldPCmd;
		bool isReadyToClimb;

   	public:
		Climber(TalonXIX* pRobot);
		void LocalReset(void);
		void StopAll(void);
		double ReadEncoder(void);
		bool IsAtTop(void);
		bool IsAtBottom(void);
		bool IsAtGoal(void);
		bool IsClimbing(void);

//#ifdef TEST_MODE	//SHOULD BE IFDEF
		void PracManualMove(double);
//#else
		void GiveGoalPosition(double);
		void ManualMove(double);
//#endif
		void UpdateDash(void);
		void Service(void);
		void StartAtBottom(void);
		void ControlSystemInit(void);
		//void ResetEncoder(void);
		void StopWhereIAm(void);
		void ReCalibrate(void);

   private:
#ifndef TEST_MODE
		void ControlRelease();
#endif
};



#endif /* SAMPLE_H_ */
