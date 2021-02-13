#ifndef SRC_DRIVEDISTANCE_H_
#define SRC_DRIVEDISTANCE_H_


#include "WPILib.h"
#include "Common.h"

/*
 * DriveDistance.h
 *
 *  Created on: Jan 14, 2018
 *      Author: Suhaas
 */

#define WHEEL_CIRCUMFERENCE    (18.653)  // inches
#define ENCODER_PULSE_COUNT    (360.0)


class DriveDistance
{
private:
	Encoder *encoderFL;
	Encoder *encoderFR;
	Encoder *encoderBL;
	Encoder *encoderBR;


	//double backDis;
	//double frontDis;
	double fwdDis;
	double leftDis;
	double rightDis;
	double diagDis;
	double LsideDis;
	double RsideDis;
	double averageDis;
	double frontDis;
	double backDis;

	double distancePerPulse;
	const double ENCODER_PULSE_CHECK = 80;
	//const double ENCODER_PULSE_CHECK_NEG = -4;



 public:
	DriveDistance(void);
	void LocalReset(void);
	void ResetEncoders(void);
	void StopAll(void);
	void UpdateDash(void);
	void Service(void);
	void SetDistancePerPulse(double distancePerPulse);
	double GetLeftDistance(void);
	double GetRightDistance(void);
	double GetStrafingDistance(void);
	//double GetAverageDistance(void);
	//double GetLargerDistance(void);
	double GetDiagonalDistance(void);
	double GetForwardsDistance(void);
	bool CheckEncoders(int encoderCase1, int encoderCase2);
	bool EncoderStatus(void);
	//double GetBackwardsDistance(void);


};



#endif /* dRiVeDiStAnCe*/
