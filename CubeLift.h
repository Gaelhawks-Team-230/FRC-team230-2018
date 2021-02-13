#ifndef CUBELIFT_H_
#define CUBELIFT_H_
/*
 * CubeLift.h
 *
 *  Created on: Jan 7, 2015
 *     Author: Siddharth Jain
 */
#include "Common.h"

class TalonXIX;

#define LIFT_DISTANCE_PER_PULSE			(0.01384 * 2.0)							//2/15/18
//#define LIFT_PULSES_PER_REVOLUTION	  (256.0)
#define LIFT_CONSTANT					(0.5)//(1.0)//(0.2)
#ifdef PRACTICE_BOT
#define MAX_LIFT_HEIGHT					(70.5)//(73)//(38.22 * 2.0)
#else
#define MAX_LIFT_HEIGHT					(77.5) //(70.8)//(73)//(38.22 * 2.0)
#endif
#define MIN_LIFT_HEIGHT					(0.0)
#define LIFT_ALLOWANCE					(0.1)
#define LIFT_AT_GOAL_ALLOWANCE			(0.01)
#define LIFT_TOP_STOP_HEIGHT 			(MAX_LIFT_HEIGHT - LIFT_ALLOWANCE)
#define LIFT_BOTTOM_STOP_HEIGHT			(MIN_LIFT_HEIGHT + LIFT_ALLOWANCE)
#define LIFT_MIN_POSITION_ERROR			(-80.0)
#define LIFT_MAX_POSITION_ERROR			(80.0)
#define LIFT_MIN_VELOCITY_ERROR			(-10.0)
#define LIFT_MAX_VELOCITY_ERROR			(10.0)
#define LIFT_MIN_VELOCITY_ERROR_INT		(-10.0 * 2.0)									//2/15/18
#define LIFT_MAX_VELOCITY_ERROR_INT		(10.0 * 2.0)									//2/15/18

//#define LIFT_MIN_VELOCITY_ERROR_INT		(-1.0 * LIFT_K_BANDWITH/LIFT_K * 1.5)									//2/15/18
//#define LIFT_MAX_VELOCITY_ERROR_INT		(LIFT_K_BANDWITH/LIFT_K * 1.5)									//2/15/18

#define LIFT_KP							(3.0)//(5.0)									//2/15/18
#define LIFT_KFF						(0.0)//(0.1)		//0.1							//2/15/18
#define LIFT_K_BANDWITH					(8.0)//(10.0)									//2/15/18
#define LIFT_K							(22.0 * 2.0)									//2/15/18
#define	LIFT_TAU						(0.1)									//2/12/18
//#define LIFT_CONTROL_POS_ALLOWANCE		(0.1)									//2/12/18
#define LIFT_SPEED						(25.0)//(35.0)//(15.0 * 2.0)									//2/15/18			//inches per second
#define LIFT_MAX_DELTA_POS				(LIFT_SPEED * LOOPTIME)
#define LIFT_MIN_MOTOR_CMD				(-1.0)
#define LIFT_MAX_MOTOR_CMD				(1.0)
#define LIFT_CALIBRATION_CMD			(-0.1)
#define LIFT_MANUAL_DEADBAND			(0.5)
#define MANUAL_DELTA					(0.95 * LIFT_MAX_DELTA_POS)

// used in TalonXIX_main
#define LIFT_PICKUP_POSITION			(0.0)
#define LIFT_SWITCH_POSITION			(22.5)
#define LIFT_NEUTRAL_POSITION			(65.5)
#define LIFT_HIGHEST_POSITION			(MAX_LIFT_HEIGHT)


class CubeLift
{
	private:
		// Create objects needed by this class
		// example: VictorSP *sampleMotor;
		Encoder *encoder;
		VictorSP *cubelift;

		TalonXIX *mainRobot;

		float motorCmd;

		double currentPosition;
		double oldPosition;
		double positionError;
		double curVelocity;
		bool atGoal;
		double positionCmd;
		double velocityCmd;
		double velocityError;
		double velocityErrorInt;
		double goalPosition;
		bool isManualMove;
		double oldPCmd;

   	public:
		CubeLift(TalonXIX* pRobot);
		void LocalReset(void);
		void StopAll(void);
#ifdef TEST_MODE
		void PracManualMove(double);
#else
		void ManualMove(double);
		void GiveGoalPosition(double);
#endif
		void UpdateDash(void);
		void Service(void);
		void StartAtBottom(void);
		bool AtGoalPosition(void);
		void ControlSystemInit(void);
		void LiftStop(void);

   	private:
		double GetHeight(void);
	#ifndef TEST_MODE
		void ControlMove(void);
	#endif
		bool AtFloor(void);
		bool AtMax(void);
};



#endif /* CUBELIFT_H_ */
