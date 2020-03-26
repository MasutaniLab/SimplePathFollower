
// #define DEBUG

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/idl/InterfaceDataTypesSkel.h>


#define _USE_MATH_DEFINES
#include <math.h>

#include "SimpleFollower.h"

using namespace RTC;

#define DEBUG
#define DEBUG_GOAL

#ifdef DEBUG
#include <iostream>
#include <fstream>
#include <coil/Time.h>
#endif

SimpleFollower::SimpleFollower(float maxTranslationVelocity,
	float minTranslationVelocity, 
	float maxRotationVelocity,
	float minRotationVelocity,
	float distanceToTranslationGain, 
	float directionToTranslationGain,
	float distanceToRotationGain,
	float directionToRotationGain,
	float approachDistanceGain,
	float approachDirectionGain)
{
	setGain(maxTranslationVelocity, minTranslationVelocity, maxRotationVelocity, minRotationVelocity, distanceToTranslationGain, directionToTranslationGain,
		distanceToRotationGain, directionToRotationGain, approachDistanceGain, approachDirectionGain);
}

void SimpleFollower::setGain(float maxTranslationVelocity, float minTranslationVelocity, 
	float maxRotationVelocity, float minRotationVelocity, 
	float distanceToTranslationGain, float directionToTranslationGain, float distanceToRotationGain, float directionToRotationGain, float approachDistanceGain, float approachDirectionGain) {
		m_MaxTranslationVelocity = maxTranslationVelocity;
		m_MinTranslationVelocity = minTranslationVelocity;
		m_MaxRotationVelocity = maxRotationVelocity;
		m_MinRotationVelocity = minRotationVelocity;
		m_distanceToTranslationGain = distanceToTranslationGain;
		m_directionToTranslationGain = directionToTranslationGain;
		m_distanceToRotationGain = distanceToRotationGain;
		m_directionToRotationGain = directionToRotationGain; 
		m_approachDistanceGain = approachDistanceGain;
		m_approachDirectionGain = approachDirectionGain;

}

SimpleFollower::~SimpleFollower()
{

}

#ifdef DEBUG
static std::ofstream fout, fout2;
coil::TimeValue startTime;
#endif

void SimpleFollower::startFollow(Path2D& path)
{ 
#ifdef DEBUG
  fout2.open("path.txt");
  size_t n = path.waypoints.length();
  for (size_t i = 0; i < n; i++) {
    std::cout << i << ": ";
    std::cout << path.waypoints[i].target.position.x << ",";
    std::cout << path.waypoints[i].target.position.y << ",";
    std::cout << path.waypoints[i].target.heading << ",";
    std::cout << path.waypoints[i].distanceTolerance << std::endl;
    fout2 << i << " "
      << path.waypoints[i].target.position.x << " "
      << path.waypoints[i].target.position.y << " "
      << path.waypoints[i].target.heading << " "
      << path.waypoints[i].distanceTolerance << std::endl;
  }
  fout2.close();
#endif
	m_goal = false;
	m_approaching = false;
	m_targetPath = path;
	m_following = true;
	m_StartPointIndex = 0;
#ifdef DEBUG
    startTime = coil::gettimeofday();
	fout.open("log.txt");
#endif
}

void SimpleFollower::stopFollow()
{
	m_targetVelocity.vx = m_targetVelocity.vy = m_targetVelocity.va = 0;
	m_following = false;

}

static double getDistance(const Pose2D& pose1, const Pose2D& pose2)
{
	double dx = pose1.position.x - pose2.position.x;
	double dy = pose1.position.y - pose2.position.y;
	return sqrt( dx*dx + dy*dy );
}


static int getNearestIndex(const RTC::Pose2D& pose, const RTC::Path2D& path) {
	int index = -1;
	double distance = 100000;

	for(int i = 0;i < path.waypoints.length();i++) {
		double d = getDistance(pose, path.waypoints[i].target);
		if (d < distance) {
			index = i;
			distance = d;
		}
	}
	return index;
}

static double getDistanceFromPath(const RTC::Waypoint2D& startPoint, 
	const RTC::Waypoint2D& stopPoint, 
	const RTC::Pose2D& pose) {
		double Va[2] = {stopPoint.target.position.x - startPoint.target.position.x,
			stopPoint.target.position.y - startPoint.target.position.y};
		double Vb[2] = {pose.position.x - startPoint.target.position.x,
			pose.position.y - startPoint.target.position.y};

		double distance = getDistance(stopPoint.target, startPoint.target);

		double distance_from_path = (Va[0]*Vb[1] - Va[1]*Vb[0]) / distance;
		return distance_from_path;
}

static double getAngularDistanceFromPath(const RTC::Waypoint2D& startPoint, 
	const RTC::Waypoint2D& stopPoint, 
	const RTC::Pose2D& pose) {
		double V[2] = {stopPoint.target.position.x - startPoint.target.position.x,
			stopPoint.target.position.y - startPoint.target.position.y};

		double pathDirection = atan2(V[1], V[0]);
		double robotDirection = pose.heading;

		double delta_direction = robotDirection - pathDirection;

		if (delta_direction > M_PI) {delta_direction -= 2*M_PI;}
		else if(delta_direction < -M_PI) {delta_direction += 2*M_PI;}

		return delta_direction;
}

static bool isArrived(const RTC::Waypoint2D& stopPoint,
	const RTC::Pose2D& pose) {
		double distance = getDistance(pose, stopPoint.target);
		return (distance < stopPoint.distanceTolerance);
}

static bool isOverRun(const RTC::Waypoint2D& startPoint, 
	const RTC::Waypoint2D& stopPoint, 
	const RTC::Pose2D& pose) {
		double A[2] = {stopPoint.target.position.x - startPoint.target.position.x,
			stopPoint.target.position.y - startPoint.target.position.y};
		double B[2] = {stopPoint.target.position.x - pose.position.x,
			stopPoint.target.position.y - pose.position.y};
		double dot = A[0] * B[0] + A[1] * B[1];

#ifdef DEBUG
		std::cout << "[SimpleFollower] dot = " << dot << std::endl;
#endif
		return (dot < 0);
}

void SimpleFollower::updateReferenceLine() {

	RTC::Path2D& path = m_targetPath;
	RTC::Pose2D& pose = m_currentPose;

	int index = getStopPointIndex();

	int startIndex;
	int stopIndex;

	if(index == 0) {
		startIndex = index;
		stopIndex = index+1;
	} else if (index == path.waypoints.length()-1) {
		startIndex = index-1;
		stopIndex = index;
	} else {
		if(isArrived(path.waypoints[index], pose)) {
			startIndex = index;
			stopIndex = index+1;
		} else {
			startIndex = index -1;
			stopIndex = index;
			if(isOverRun(path.waypoints[index-1],
				path.waypoints[index],
				pose)) { 
					/// TODO: OVERRUN ERROR
					startIndex = index;
					stopIndex = index+1;
			} else { 
				startIndex = index-1;
				stopIndex = index;
			}
		}
		m_StartPointIndex = startIndex;
	}

#if 0
	RTC::Waypoint2D nearestPoint = path.waypoints[index];
	RTC::Waypoint2D startPoint = path.waypoints[startIndex];
	RTC::Waypoint2D stopPoint = path.waypoints[stopIndex];

	fout << nearestPoint.target.position.x << ", "
		<< nearestPoint.target.position.y << ", "
		<< nearestPoint.target.heading << ", " 
		<< startPoint.target.position.x << ", "
		<< startPoint.target.position.y << ", "
		<< startPoint.target.heading << ", " 
		<< stopPoint.target.position.x << ", "
		<< stopPoint.target.position.y << ", "
		<< stopPoint.target.heading << ", " 
		<< pose.position.x << ", "
		<< pose.position.y << ", "
		<< pose.heading << std::endl;

#endif
}



FOLLOW_RESULT SimpleFollower::follow()
{
	if(!isFollowing()) {
		m_targetVelocity.vx = m_targetVelocity.vy = m_targetVelocity.va = 0;
		return FOLLOW_NOT_IN_FOLLOW;
	}

	updateReferenceLine();
	int startIndex = getStartPointIndex();
	int stopIndex  = getStopPointIndex();
	RTC::Waypoint2D startPoint = m_targetPath.waypoints[startIndex];
	RTC::Waypoint2D stopPoint = m_targetPath.waypoints[stopIndex];


	double maxTranslationVelocity = stopPoint.maxSpeed.vx;
	double maxRotationVelocity = stopPoint.maxSpeed.va;

	int nearestIndex = getNearestIndex(m_currentPose, m_targetPath);
	if (nearestIndex == (m_targetPath.waypoints.length() -1 ) || m_approaching) {
#ifdef DEBUG
		std::cout << "[SimpleFollower] Approaching Goal." << std::endl;
#endif
		m_approaching = true;
		return approachGoal(m_currentPose, m_targetPath.waypoints[m_targetPath.waypoints.length()-1]);
	}

	double distanceFromPath = getDistanceFromPath(startPoint, stopPoint, m_currentPose);
	if(fabs(distanceFromPath) > stopPoint.distanceTolerance) { // Out of Range
#ifdef DEBUG
		std::cout << "[SimpleFollower] Distance Out Of Range (now=" << fabs(distanceFromPath) << " range=" << stopPoint.distanceTolerance << ")" << std::endl;
#endif
		//throw OutOfRangeException();
		return FOLLOW_DISTANCEOUTOFRANGE;
	}

	double angularDistanceFromPath = getAngularDistanceFromPath(startPoint, stopPoint, m_currentPose);

	//  distanceFromPath = 0.1;
	m_targetVelocity.vx = 0;

	m_targetVelocity.va = -(m_distanceToRotationGain * distanceFromPath + m_directionToRotationGain * angularDistanceFromPath);
	if(m_targetVelocity.va > maxRotationVelocity) {
		m_targetVelocity.va = maxRotationVelocity;
	} else if(m_targetVelocity.va < -maxRotationVelocity) {
		m_targetVelocity.va = -maxRotationVelocity;
	}  

	// 角度誤差が許容差以上だったら，旋回速度計算が終了時点で例外を出す．並進速度はゼロ
	if(fabs(angularDistanceFromPath) > stopPoint.headingTolerance) {
#ifdef DEBUG
		std::cout << "[SimpleFollower] Heading Out Of Range (now=" << fabs(angularDistanceFromPath) << " range=" << stopPoint.headingTolerance << ")" << std::endl;
#endif
		if (fabs(m_targetVelocity.va) < m_MinRotationVelocity) {
			if (m_targetVelocity.va < 0) m_targetVelocity.va = -m_MinRotationVelocity;
			else if(m_targetVelocity.va > 0) m_targetVelocity.va = m_MinRotationVelocity;
		}
		//throw HeadingOutOfRangeException();
		return FOLLOW_HEADINGOUTOFRANGE;
	}



	m_targetVelocity.vx = m_MaxTranslationVelocity - m_distanceToTranslationGain * fabs(distanceFromPath)
		- m_directionToTranslationGain * fabs(angularDistanceFromPath);
	if(m_targetVelocity.vx < m_MinTranslationVelocity) {
		m_targetVelocity.vx = m_MinTranslationVelocity;
	}

#ifdef DEBUG
	std::cout << "[SimpleFollower] Target Point Index = [" << startIndex << ", " << stopIndex << "]" << std::endl;

	std::cout << "[SimpleFollower] Current/Start/Stop = [" 
		<< m_currentPose.position.x << "," << m_currentPose.position.y << "]/["
		<< startPoint.target.position.x << ", " << startPoint.target.position.y << "]/[" 
		<< stopPoint.target.position.x << ", " << stopPoint.target.position.y << "]"  << std::endl;
	std::cout << "[SimpleFollower] Distance = [" << distanceFromPath << ", " << angularDistanceFromPath << "]" << std::endl;
	std::cout << "[SimpleFollower] TargetVelocity = ["
		<< m_targetVelocity.vx << ", " << m_targetVelocity.va << "]" << std::endl;

    double elapsedTime = coil::gettimeofday() - startTime;
    fout << elapsedTime << " N "
      << startIndex << " " << stopIndex << " "
      << m_currentPose.position.x << " " << m_currentPose.position.y << " " << m_currentPose.heading << " "
      << distanceFromPath << " " << angularDistanceFromPath << " "
      << m_targetVelocity.vx << " " << m_targetVelocity.va << std::endl;
#endif

	return FOLLOW_OK;
}


FOLLOW_RESULT SimpleFollower::approachGoal(RTC::Pose2D& currentPose, RTC::Waypoint2D& goal)
{

	RTC::Path2D& path = m_targetPath;
	int index = getStopPointIndex();
	//  RTC::Waypoint2D& goal = path.waypoints[index];

	double transVelocity = 0;
	double rotVelocity = 0;
	int length = m_targetPath.waypoints.length();
	double dx = goal.target.position.x - currentPose.position.x;
	double dy = goal.target.position.y - currentPose.position.y;
	double distance = sqrt(dx*dx + dy*dy);
	double deltaPose = goal.target.heading - currentPose.heading;
	while(deltaPose > +M_PI) deltaPose -= 2 * M_PI;
	while(deltaPose < -M_PI) deltaPose += 2 * M_PI;

	FOLLOW_RESULT ret = FOLLOW_APPROACHINGTOGOAL;
#ifdef DEBUG_GOAL
	std::cout << "[SimpleFollower] Goal = " << goal.target.position.x << ", " << goal.target.position.y << ", " << goal.target.heading << std::endl;
	std::cout << "[SimpleFollower] Curr = " << currentPose.position.x << ", " << currentPose.position.y << ", " << currentPose.heading << std::endl;
	std::cout << "[SimpleFollower] Distance = " << distance << ", Direction = " << deltaPose <<  std::endl;

#endif
	if(distance <= goal.distanceTolerance) {

		transVelocity = 0;
		if(fabs(deltaPose) < goal.headingTolerance) {
			rotVelocity = 0;
			m_goal = true;
                        std::cout << "[SimpleFollower] m_goal = true" << std::endl;
#ifdef DEBUG
			fout.close();
#endif
			ret = FOLLOW_GOAL;
		} else {
			//m_Settled = true;

			rotVelocity = m_approachDirectionGain * (deltaPose/(M_PI/2));

			if(rotVelocity > m_MaxRotationVelocity) {
			  rotVelocity = m_MaxRotationVelocity;
                        } else if (rotVelocity < -m_MaxRotationVelocity) {
                          rotVelocity = -m_MaxRotationVelocity;
                        }

			ret = FOLLOW_TURNTOGOALPOSE;
		}

	} else {
		double theta = atan2(dy, dx);
		double dtheta = theta - currentPose.heading;
		while(dtheta >  +M_PI) dtheta -= 2*M_PI;
		while(dtheta <= -M_PI) dtheta += 2*M_PI;

		transVelocity = m_approachDistanceGain * distance;
		
		if(transVelocity > m_MaxTranslationVelocity) {
		  transVelocity = m_MaxTranslationVelocity;
		}
		
		//if(dtheta > PI/6 || dtheta < -PI/6) transVelocity = 0;

		if (dtheta < -goal.headingTolerance) {
			transVelocity = 0;//m_MinTranslationVelocity;
		} else if(dtheta > goal.headingTolerance) {
			transVelocity = 0;//m_MinTranslationVelocity;
		}


		rotVelocity = m_approachDirectionGain * (dtheta/(M_PI/2));
		
		if(rotVelocity > m_MaxRotationVelocity) {
		  rotVelocity = m_MaxRotationVelocity;
		} else if(rotVelocity <= -m_MaxRotationVelocity) {
		  rotVelocity = -m_MaxRotationVelocity;
		}
		
		/*
		double ratio = (m_MaxRotationSpeed - fabs(rotVelocity)) / m_MaxRotationSpeed;
		transVelocity = ratio * transVelocity;
		*/
		/*
		if(m_Mode == PATH_TURN_TO_POINT ) {
		transVelocity = 0;
		if(fabs(dtheta) < PI/ 18) {
		m_Settled = true;
		}
		}
		*/
	}

	m_targetVelocity.vx = transVelocity;
	m_targetVelocity.vy = 0;
	m_targetVelocity.va = rotVelocity;

#ifdef DEBUG_GOAL
	std::cout << "[SimpleFollower] Vel  = " << m_targetVelocity.vx << ", " << m_targetVelocity.vy << ", " << m_targetVelocity.va << std::endl;
    double elapsedTime = coil::gettimeofday() - startTime;
    fout << elapsedTime << " G "
      << index-1 << " " << index << " "
      << m_currentPose.position.x << " " << m_currentPose.position.y << " " << m_currentPose.heading << " "
      << 0 << " " << 0 << " "
      << m_targetVelocity.vx << " " << m_targetVelocity.va << std::endl;
#endif
	return ret;
}
