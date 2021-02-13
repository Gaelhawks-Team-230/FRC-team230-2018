#include "WPILib.h"
#include "Common.h"
#include "LightController.h"


// CONSTRUCTOR for LightController Class
// controls light patterns of the robot's(clyde's) lights
LightController::LightController(TalonXIX *pRobot)
{

	light1 = new DigitalOutput(LED_LIGHT_OUTPUT_1);
	light2 = new DigitalOutput(LED_LIGHT_OUTPUT_2);

	mainRobot = pRobot;

	LocalReset();
}


// Set initial values for ALL objects and variables
void LightController::LocalReset()
{
	//new objects
	light1->Set(false);
	light2->Set(false);
}


void LightController::StopAll()
{
	LocalReset();
}

void LightController::StartingConfig()
{
	LocalReset();
}
void LightController::UpdateDash()
{

}

void LightController::DisabledSetup()
{
	light1->Set(false);
	light2->Set(false);
}


// All classes need a Service function
void LightController::Service()
{
	if(mainRobot->climber->IsClimbing())
	{
		light1->Set(true);//22
		light2->Set(true);//23
		//printf("Looking up\n");
	}
	else if(!mainRobot->isAuto)
	{
		//printf("Driving");
		if(mainRobot->isBlueAlliance)
		{
			light1->Set(false);//22
			light2->Set(true);//23
			//printf(" Blue\n");
		}
		else
		{
			light1->Set(true);//22
			light2->Set(false);//23
			//printf(" Red\n");
		}
	}
	else
	{
		light1->Set(false);//22
		light2->Set(false);//23
		//printf("Standing Still\n");
	}
}
