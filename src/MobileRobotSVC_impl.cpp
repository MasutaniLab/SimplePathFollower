// -*-C++-*-
/*!
 * @file  MobileRobotSVC_impl.cpp
 * @brief Service implementation code of MobileRobot.idl
 *
 */

#include "MobileRobotSVC_impl.h"

#include "SimplePathFollower.h"

/*
 * Example implementational code for IDL interface RTC::PathFollower
 */
RTC_PathFollowerSVC_impl::RTC_PathFollowerSVC_impl()
{
  // Please add extra constructor code here.
}


RTC_PathFollowerSVC_impl::~RTC_PathFollowerSVC_impl()
{
  // Please add extra destructor code here.
}


/*
 * Methods corresponding to IDL attributes and operations
 */
RTC::RETURN_VALUE RTC_PathFollowerSVC_impl::followPath(const RTC::Path2D& path)
{
	RTC::RETURN_VALUE result = RETVAL_OK;
	// Please insert your code here and remove the following warning pragma


	if(path.waypoints.length() == 0){ 
		m_pRTC->stopFollow();
		std::cout << "[RTC::SimplePathFollower] Stop Following" << std::endl;

	} else {
		m_pRTC->setPath(path);
		m_pRTC->startFollow();
		std::cout << "[RTC::SimplePathFollower] Start Following" << std::endl;

		while(!m_pRTC->isGoal()) {
			coil::usleep(1000*100);

			if (m_pRTC->getMode() == MODE_TIMEOUT) {
				std::cout << "[RTC::SimplePathFollower] Current Pose Timeout" << std::endl;
				return RTC::RETVAL_ODOMETRY_TIME_OUT;
			} else if (m_pRTC->getMode() == MODE_OUTOFRANGE) {
				std::cout << "[RTC::SimplePathFollower] OutOfRange" << std::endl;
				return RTC::RETVAL_OUTOF_RANGE;
			}

		}

		std::cout << "[RTC::SimplePathFollower] Goal Reached." << std::endl;
	}

	return result;
}

RTC::RETURN_VALUE RTC_PathFollowerSVC_impl::getState(RTC::FOLLOWER_STATE& state)
{
	RTC::RETURN_VALUE result = RETVAL_OK;


	return result;
}

RTC::RETURN_VALUE RTC_PathFollowerSVC_impl::followPathNonBlock(const RTC::Path2D& path)
{
	RTC::RETURN_VALUE result = RETVAL_OK;

	if(path.waypoints.length() == 0){ 
		m_pRTC->stopFollow();
	} else {
		m_pRTC->setPath(path);
		m_pRTC->startFollow();
		std::cout << "[RTC::SimplePathFollower] Start Following" << std::endl;
	}

	return result;
}



// End of example implementational code



