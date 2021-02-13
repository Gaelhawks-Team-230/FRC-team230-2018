#ifndef CLAW_H_
#define CLAW_H_

/*
 * Claw.h
 *
 *  Created on: Jan 14, 2018
 *      Author: JessW
 */
class TalonXIX;

#define CLAW_RELEASE_TIME				((int)(1.0 * (N1SEC)))
#define GRAB_PAUSE_WAIT 				((int)(0.25 * (N1SEC)))
#define EJECT_UNTIL_STOP_WAIT			((int)(1.0 * (N1SEC)))
#define TARGET_CURRENT_STALL  			((int)(0.5 * (N1SEC)))
#define SQUEEZE_TIME					((int)(0.75 * (N1SEC)))	//1.5 //0.5
#define RELEASE_TIME					((int)(0.5 * (N1SEC)))
#define GATHER_SPIN_TIMER				((int)(0.25 * (N1SEC)))
#define EJECT_SPIN_TIMER				((int)(0.75 * (N1SEC)))
#define ADJUST_SPIN_OUT_TIME			((int)(0.03 * (N1SEC)))

#define SQUEEZE_COMMAND		( 1.0)
#define RELEASE_COMMAND		(-1.0)

#ifdef PRACTICE_BOT
#define ADJUST_COMMAND		(-0.7)
#define GATHER_COMMAND		( 1.0)
#define EJECT_COMMAND		(-1.0)
#else
#define ADJUST_COMMAND		(-0.7)
#define GATHER_COMMAND		( 0.9)
#define EJECT_COMMAND		(-0.7)
#endif


#define IDLE_MODE			0
#define SPIN_WHEELS_MODE	1
#define SPIN_OUT_MODE		1
#define SQUEEZE_CUBE_MODE	2
#define RELEASE_CUBE_MODE	2
#define SPIN_IN_MODE		2

class Claw
{
	private:
		VictorSP *wheels;
		VictorSP *clawSqueeze;
		Relay *clawDrop;
		DigitalInput * cubeSensor;


		TalonXIX *mainRobot;

		// Create objects needed by this class
		// example: VictorSP *sampleMotor;

		// Declare local variables that need to be saved for future processing
		bool hasCube;
		bool holdingCube;
		double clawMotorCmd;
		bool dropMode;
		int dropCount;
		bool movingClaw;
		double squeezeMotorCmd;
		int waitToPauseCount;
		int ejectCount;
		bool clawIsOpen;
		int squeezeTimeCount;
		int moveClawTime;

		bool isGatheringCube;
		bool isEjectingCube;
		bool isAdjustingCube;

		int gatherState;
		int ejectState;
		int adjustState;

		int gatherCubeCount;
		int ejectCubeCount;
		int adjustCubeCount;



		// example: bool isEnabled;
		// example: int serviceMode;

   	public:
		Claw(TalonXIX *pRobot);
		void LocalReset(void);
		void StopAll(void);
		// Define functions for needed operations, as an example
		// example: void ManualMove(float cmd);
		void StartWheels(bool wheelsGrab);
		void StopWheels(void);
		void PauseWheels(void);
		void ReleaseCube(void);
		bool HaveCube(void);
		double GetSqueezeCurrent(void);
		void SqueezeCube(void);
		void DropClaw(void);
		void StopSqueeze(void);
		void StartingConfig(void);
		void GatherCube(void);
		void EjectCube(void);
		void AdjustCube(void);
		void AdvancedCubeOperations(void);



		void UpdateDash(void);
		void Service(void);

};



#endif /* CLAW_H_ */

