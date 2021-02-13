
#include "WPILib.h"
#include "Common.h"
#include "TalonXIX_main.h"
#include "DriveDistance.h"


// CONSTRUCTOR for DriveDistance
// - measures distance the robot drives: forwards, backwards, strafing right, strafing left, left or right diagonally
DriveDistance::DriveDistance()
{
	distancePerPulse = (WHEEL_CIRCUMFERENCE/ENCODER_PULSE_COUNT);
	encoderFL = new Encoder(FRONT_LEFT_ENCODER_ONE, FRONT_LEFT_ENCODER_TWO, false);
	encoderFR = new Encoder(FRONT_RIGHT_ENCODER_ONE, FRONT_RIGHT_ENCODER_TWO, false);
	encoderBL = new Encoder(BACK_LEFT_ENCODER_ONE, BACK_LEFT_ENCODER_TWO, false);
	encoderBR = new Encoder(BACK_RIGHT_ENCODER_ONE, BACK_RIGHT_ENCODER_TWO, false);

	encoderFL->SetDistancePerPulse(-1.0 * distancePerPulse);
	encoderFR->SetDistancePerPulse(distancePerPulse);
	encoderBL->SetDistancePerPulse(-1.0 * distancePerPulse);
	encoderBR->SetDistancePerPulse(distancePerPulse);
	LocalReset();
}


// Set initial values for ALL objects and variables
void DriveDistance::LocalReset()
{
	ResetEncoders();

//	backDis = 0.0;
//	frontDis = 0.0;
	fwdDis = 0.0;
	leftDis = 0.0;
	rightDis  = 0.0;
	diagDis = 0.0;
	LsideDis = 0.0;
	RsideDis = 0.0;
	averageDis = 0.0;
	frontDis = 0.0;
	backDis = 0.0;

}

void DriveDistance::ResetEncoders()
{
	encoderFL->Reset();
	encoderFR->Reset();
	encoderBL->Reset();
	encoderBR->Reset();
}

//no motors
void DriveDistance::StopAll()
{
	ResetEncoders();
}

//double DriveDistance::GetLargerDistance()
//{
//	GetForwardsDistance();

//	if(fabs(frontDis) > fabs(backDis))w
//	{
//		return frontDis;
//	}
//	else
//	{
//		return backDis;
//	}
//}

//updates dash with new info every 20 loops(1 second)
void DriveDistance::UpdateDash()
{
	//SmartDashboard::PutNumber("Average Distance", GetAverageDistance());
	//SmartDashboard::PutNumber("Distance left", GetLeftDistance());
	//SmartDashboard::PutNumber("Distance right", GetRightDistance());
//#ifdef TEST_MODE
	SmartDashboard::PutNumber("FL Distance", encoderFL->GetDistance());
	SmartDashboard::PutNumber("FR Distance", encoderFR->GetDistance());
	SmartDashboard::PutNumber("BL Distance", encoderBL->GetDistance());
	SmartDashboard::PutNumber("BR Distance", encoderBR->GetDistance());
//#endif
	//SmartDashboard::PutNumber("FL encoder", encoderFL->Get());
	SmartDashboard::PutNumber("Strafe Distance", GetStrafingDistance());
	SmartDashboard::PutNumber("Forwards Distance", GetForwardsDistance());
	SmartDashboard::PutBoolean("Encoder Status", EncoderStatus());
}

//no need
void DriveDistance::Service()
{

}

//checks encoders if they are 100 pulses away
bool DriveDistance::CheckEncoders(int encoderCase1, int encoderCase2)
{

	if (abs(encoderCase1 - encoderCase2) >= ENCODER_PULSE_CHECK)
		return false;
	else
		return true;
}

//gets distance of wheels on right side
double DriveDistance::GetRightDistance()
{
	rightDis = (encoderFR->GetDistance() + encoderBR->GetDistance())/2;
	return rightDis;
}
//gets report from CheckEncoders function for frontDis and backDis and updates the encoder status
bool DriveDistance::EncoderStatus()
{
	GetForwardsDistance();
	if(CheckEncoders(frontDis, backDis) == false)
		//light red
		return false;
	else
		//light green
		return true;
}

//gets distance of wheels on left side
double DriveDistance::GetLeftDistance()
{
	leftDis = (encoderFL->GetDistance() + encoderBL->GetDistance())/2;
	return leftDis;
}
/*
//gets distance average distance between right and left
double DriveDistance::GetAverageDistance()
{
	//averageDis = GetLargerDistance();
	averageDis = (GetRightDistance() + GetLeftDistance())/2;
	return averageDis;
}
*/

//gets straight distance and checks encoders if they are broken
double DriveDistance::GetForwardsDistance()
{
	// note left side must be inverted
	frontDis = (encoderFR->Get() + (-1*encoderFL->Get()))/2;
	backDis = (encoderBR->Get() + (-1*encoderBL->Get()))/2;

	fwdDis = (frontDis + backDis)/2;

	//fwdDis = CheckEncoders(frontDis, backDis, average);
	if (CheckEncoders(frontDis, backDis))
		return (fwdDis * distancePerPulse);
	else if (fabs(frontDis) > fabs(backDis))
		return (frontDis * distancePerPulse);
	else
		return (backDis * distancePerPulse);
}


//gets distance traveled when strafing using check encoder function. combines left and right negative means left, positive means right
double DriveDistance::GetStrafingDistance()
{
	double RsideDis1;
	double RsideDis2;
	double Rvalue;

	RsideDis1 = (encoderFL->Get() - encoderBL->Get());
	RsideDis2 = (encoderBR->Get() - encoderFR->Get());

	RsideDis = (RsideDis1/RsideDis2)/2;

	double LsideDis1;
	double LsideDis2;
	double Lvalue;

	LsideDis1 = (encoderFR->Get() - encoderBR->Get());
	LsideDis2 = (encoderBL->Get() - encoderFL->Get());

	LsideDis = (LsideDis1 + LsideDis2)/2;

	if (CheckEncoders(RsideDis1, RsideDis2))
		Rvalue = (RsideDis * distancePerPulse);
	else if (RsideDis1 > RsideDis2)
		Rvalue = (RsideDis1 * distancePerPulse);
	else
		Rvalue = (RsideDis2 * distancePerPulse);

	if (CheckEncoders(LsideDis1, LsideDis2))
		Lvalue = (LsideDis * distancePerPulse);
	else if (LsideDis1 > LsideDis2)
		Lvalue = (LsideDis1 * distancePerPulse);
	else
		Lvalue = (LsideDis2 * distancePerPulse);

	return (Lvalue + Rvalue);
	return Lvalue;
	return Rvalue;
}


//gets distance when strafing diagonally, left or right
double DriveDistance::GetDiagonalDistance()
{
	double diagDis1;
	double diagDis2;

	if(GetRightDistance() > GetLeftDistance())
	{
		diagDis1 = (encoderFL->Get() - encoderBR->Get());
		diagDis2 = (encoderBL->Get() - encoderFR->Get());

		diagDis = (diagDis1 + diagDis2)/2;

		if(CheckEncoders(diagDis1, diagDis2))
			return (diagDis * distancePerPulse);
		else if (diagDis1 > diagDis2)
			return (diagDis1 * distancePerPulse);
		else
			return (diagDis2 * distancePerPulse);
	}
	else
	{
		diagDis1 = (encoderBR->Get() - encoderFL->Get());
		diagDis2 = (encoderFR->Get() - encoderFL->Get());

		diagDis = (diagDis1 + diagDis2)/2;

		if(CheckEncoders(diagDis1, diagDis2))
			return (diagDis * distancePerPulse);
		else if (diagDis1 > diagDis2)
			return (diagDis1 * distancePerPulse);
		else
			return (diagDis2 * distancePerPulse);
	}
	return diagDis;
}

//add more encoders to classes and average them out for more reliable data

//used Get instead of GetDistance to get the pulse count of encoders and compared it with constant numbers
//returned it by multiplying the pulses counted by encoders by distancePerPulse to get actual distance
