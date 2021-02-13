#ifndef LIGHT_CONTROLLER_H_
#define LIGHT_CONTROLLER_H_
/*
 * LightController.h
 *
 *  Created on: Jan 30, 2018
 *      Author: Suhaas Nadella
 */

#include "WPILib.h"
#include "Common.h"

class TalonXIX;

class LightController
{
	private:
		DigitalOutput *light1;
		DigitalOutput *light2;

		TalonXIX *mainRobot;


   	public:
		LightController(TalonXIX *pRobot);
		void LocalReset(void);
		void StopAll(void);
		void StartingConfig(void);
		void UpdateDash(void);
		void Service(void);
		void DisabledSetup(void);
		//4 light displays
		void LightShow1(void);
		void LightShow2(void);
		void LightShow3(void);
		void LightShow4(void);
};



#endif /* LightControllerrrrrrrrrrrrrrrrrrrrrrrrrr*/
