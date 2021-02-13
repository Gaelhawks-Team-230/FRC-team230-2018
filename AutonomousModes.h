/*
 * AutonomousModes.h
 *
 *  Created on: Jan 14, 2018
 *      Author: jacob
 */

#ifndef SRC_AUTONOMOUSMODES_H_
#define SRC_AUTONOMOUSMODES_H_

//BASE MODE INDEXES
#define DO_NOTHING				(0)
#define REACH_BASELINE_AUTO		(1)
#define CENTER_SWITCH_AUTO		(2)
#define SIDE_SWITCH_AUTO	 	(3)
#define SIDE_SCALE_AUTO			(4)
#define FAR_SIDE_SCALE_AUTO		(5)

//SUBROUTINE INDEXES
#define CENTER_MULTI_CUBE_SWITCH_AUTO		(6)
#define CENTER_SWITCH_THEN_SCALE_AUTO		(7)
#define SIDE_MULTI_CUBE_SWITCH_AUTO			(8)
#define FAR_SIDE_SWITCH_THEN_SCALE_AUTO		(9)
#define SAME_SIDE_SWITCH_THEN_SCALE_AUTO	(10)
#define SCALE_THEN_SWITCH_AUTO				(11)
#define SIDE_MULTI_CUBE_SCALE_AUTO			(12)


//Center Switch Distances
#define LEFT_DISTANCE_TO_SWITCH_AT_ANGLE		(93.0)//(95.0)//(105.0)//(110.0) //When at a 25 degree angle, distance is 155 inches
#define RIGHT_DISTANCE_TO_SWITCH_AT_ANGLE		(88.0)//(105.0)//(110.0) //When at a 25 degree angle, distance is 155 inches
#define DISTANCE_TO_SWITCH						(70.0)	//140.0
#define SMALL_BACK_UP							(5.0)
#define DISTANCE_SWITCH_TO_PYRAMID				(38.0)
#define DISTANCE_PYRAMID_TO_SWITCH				(58.0)
#define DISTANCE_STRAFE_FROM_SWITCH				(41.0)
#define DISTANCE_SWITCH_TO_SIDE_CUBE			(88.5)
#define DISTANCE_SIDECUBE_LINEUP_TO_CUBE		(15.0)
#define DISTANCE_SIDECUBE_TO_SCALE				(70.0)
#define AUTO_TURN_HALF_SEC						((int)(0.5 * (N1SEC)))
#define DISTANCE_TO_PYRAMID_CENTER_CUBE			(15.0)
#define RIGHT_DISTANCE_TO_PYRAMID_CENTER_CUBE	(23.0)//25.0
#define DISTANCE_PYRAMID_TO_EXCHANGE			(34.5)//(32.0)//(37.0)//(40.0)
#define RIGHT_DISTANCE_PYRAMID_TO_EXCHANGE		(40.0)//(45.0)//(58.5)//(57.0)//(60.0)
#define RIGHT_DISTANCE_EXCHANGE_LINEUP			(43.0)//(50.0)
#define RIGHT_DISTANCE_INCH_TO_EXCHANGE			(7.0)



//Side Switch Distances
#define DISTANCE_WALL_TO_SWITCH_SIDE		(145.0) //Ends up centered at switch
#define DISTANCE_SIDE_LINEDUP_TO_SWITCH		(15.0)
#define DISTANCE_STRAFE_SWITCH_TO_SIDECUBE	(56.0)
#define DISTANCE_STRAFE_FROM_SIDECUBE		(20.0)
#define DISTANCE_CROSS_PZONE				(155.0)
#define DISTANCE_PZONE_TO_SCALE				(66.0)
#define AUTO_TURN_ONE_SEC					((int)(1.0 * (N1SEC)))


//Side Scale Distances
#define DISTANCE_WALL_TO_SCALE					(260.0)//(270.0)
#define DISTANCE_WALL_TO_MID_PLATFORM_ZONE		(197.0)//(207.0)
#define DISTANCE_CROSS_PLATFORM_ZONE			(185.0)//(195.0)
#define DISTANCE_MIDPLATFORM_ZONE_TO_SCALE		(30.0)
#define DISTANCE_SAME_SIDE_SCALE_TO_SIDECUBE	(88.0)//(83.0)//(88.0)//(92.0)
#define DISTANCE_FAR_SIDE_SCALE_TO_SIDECUBE 	(30.0)
#define SMALL_MOVE								(10.0)
#define SCALE_BACKUP							(15.0)
#define SCALE_BACKUP_SAME_SIDE					(12.0)
#define AUTO_TURN_1p5_SEC						((int)(1.5 * (N1SEC)))
#define AUTO_CLAW_OPEN_DELAY					((int)(0.5 * (N1SEC)))
#define AUTO_AT_GOAL_DELAY						((int)(0.2 * (N1SEC)))
#define SWITCH_DEPLOY_FORWARD					(8.0)//(17.0)//(12.0)//(15.0)//(7.0)
#define REVERSE_TO_SCALE_FROM_SWITCH			(42.0) //(40.0)
#define DISTANCE_SAME_SIDE_SCALE_TO_SIDECUBE_TO_SCALE	(90.0)//(88.0)
#define SMALL_FORWARD_TO_SCALE					(15.0)
#define WAIT_BEFORE_BACK_FROM_SCALE				((int)(0.5 * (N1SEC)))

#define FORWARD_TO_CORNER_OF_SWITCH				(50.0)
#define FORWARD_TO_CORNER_CUBE					(22.0)
#define BACK_FROM_CORNER_CUBE					(20.0)
#define TURN_TIME_BACK_FROM_CORNER_CUBE			((int)(0.5 * (N1SEC)))
#define DISTANCE_CROSS_PLATFORM					(196.0)
#define WAIT_TO_GRAB_CUBE						((int)(0.5 * (N1SEC)))

//Mechanism Wait Times
#define CUBE_LOAD_WAIT			((int)(0.5 * N1SEC))




#endif /* SRC_AUTONOMOUSMODES_H_ */
