// -*- C++ -*-
/*!
 * @file  SimplePathFollower.cpp
 * @brief Simple Algorithm Path Follower
 * @date $Date$
 *
 * $Id$
 */

#include <stdio.h>
#include "SimplePathFollower.h"

// Module specification
// <rtc-template block="module_spec">
static const char* simplepathfollower_spec[] =
  {
    "implementation_id", "SimplePathFollower",
    "type_name",         "SimplePathFollower",
    "description",       "Simple Algorithm Path Follower",
    "version",           "1.0.0",
    "vendor",            "Sugar Sweet Robotics",
    "category",          "Navigation",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.debug", "0",
    "conf.default.poseTimeout", "3.0",
    "conf.default.approachDirectionGain", "1.0",
    "conf.default.directionToTranslationGain", "0.0",
    "conf.default.directionToRotationGain", "0.0",
    "conf.default.minVelocity", "0.2",
    "conf.default.distanceToTranslationGain", "0.0",
    "conf.default.approachDistanceGain", "0.5",
    "conf.default.maxVelocity", "0.8",
    "conf.default.distanceToRotationGain", "2.0",
    "conf.default.maxRotationVelocity", "2.0",
    "conf.default.minRotationVelocity", "0.5",

    // Widget
    "conf.__widget__.debug", "text",
    "conf.__widget__.poseTimeout", "text",
    "conf.__widget__.approachDirectionGain", "text",
    "conf.__widget__.directionToTranslationGain", "text",
    "conf.__widget__.directionToRotationGain", "text",
    "conf.__widget__.minVelocity", "text",
    "conf.__widget__.distanceToTranslationGain", "text",
    "conf.__widget__.approachDistanceGain", "text",
    "conf.__widget__.maxVelocity", "text",
    "conf.__widget__.distanceToRotationGain", "text",
    "conf.__widget__.maxRotationVelocity", "text",
    "conf.__widget__.minRotationVelocity", "text",
    // Constraints

    "conf.__type__.debug", "int",
    "conf.__type__.poseTimeout", "float",
    "conf.__type__.approachDirectionGain", "float",
    "conf.__type__.directionToTranslationGain", "float",
    "conf.__type__.directionToRotationGain", "float",
    "conf.__type__.minVelocity", "float",
    "conf.__type__.distanceToTranslationGain", "float",
    "conf.__type__.approachDistanceGain", "float",
    "conf.__type__.maxVelocity", "float",
    "conf.__type__.distanceToRotationGain", "float",
    "conf.__type__.maxRotationVelocity", "float",
    "conf.__type__.minRotationVelocity", "float",

    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
SimplePathFollower::SimplePathFollower(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_currentPoseIn("currentPose", m_currentPose),
    m_velocityOut("velocity", m_velocity),
    m_PathFollowerPort("PathFollower")

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
SimplePathFollower::~SimplePathFollower()
{
}



RTC::ReturnCode_t SimplePathFollower::onInitialize()
{
  RTC_INFO(("onInitialize()"))
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("currentPose", m_currentPoseIn);
  
  // Set OutPort buffer
  addOutPort("velocity", m_velocityOut);
  
  // Set service provider to Ports
  m_PathFollowerPort.registerProvider("PathFollower", "RTC::PathFollower", m_pathFollower);
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  addPort(m_PathFollowerPort);
  m_pathFollower.setRTC(this);
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("debug", m_debug, "0");
  bindParameter("poseTimeout", m_poseTimeout, "3.0");
  bindParameter("approachDirectionGain", m_approachDirectionGain, "1.0");
  bindParameter("directionToTranslationGain", m_directionToTranslationGain, "0.0");
  bindParameter("directionToRotationGain", m_directionToRotationGain, "0.0");
  bindParameter("minVelocity", m_minVelocity, "0.2");
  bindParameter("distanceToTranslationGain", m_distanceToTranslationGain, "0.0");
  bindParameter("approachDistanceGain", m_approachDistanceGain, "0.5");
  bindParameter("maxVelocity", m_maxVelocity, "0.8");
  bindParameter("distanceToRotationGain", m_distanceToRotationGain, "2.0");
  bindParameter("maxRotationVelocity", m_maxRotationVelocity, "2.0");
  bindParameter("minRotationVelocity", m_minRotationVelocity, "0.5");
  // </rtc-template>
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t SimplePathFollower::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SimplePathFollower::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SimplePathFollower::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t SimplePathFollower::onActivated(RTC::UniqueId ec_id)
{
    RTC_INFO(("onActivated()"))
	m_poseUpdated = false;
	m_Mode = MODE_NORMAL;
	m_pathFollowerObj.stopFollow();
	m_lastReceivedTime = coil::gettimeofday();
    m_outputRequired = true;
    return RTC::RTC_OK;
}


RTC::ReturnCode_t SimplePathFollower::onDeactivated(RTC::UniqueId ec_id)
{
  RTC_INFO(("onDeactivated()"))
    return RTC::RTC_OK;
}


RTC::ReturnCode_t SimplePathFollower::onExecute(RTC::UniqueId ec_id)
{

	coil::TimeValue currentTime = coil::gettimeofday();
	if (m_currentPoseIn.isNew()) {
		m_currentPoseIn.read();
		m_poseUpdated = true;

		m_lastReceivedTime = currentTime;
	} else {
		double duration = currentTime - m_lastReceivedTime;
		if (duration > m_poseTimeout && m_poseTimeout > 0) {
			m_Mode = MODE_TIMEOUT;
		}
	}

	//if (m_debug) {

	if(m_pathFollowerObj.isFollowing()) {


		if (m_poseUpdated) {
			m_pathFollowerObj.setGain(m_maxVelocity, m_minVelocity,
                                m_maxRotationVelocity, m_minRotationVelocity,
				m_distanceToTranslationGain, m_directionToTranslationGain,
				m_distanceToRotationGain, m_directionToRotationGain,
				m_approachDistanceGain, m_approachDirectionGain);

			if (false) {
				static int i = 0;
				if(i++ % 30 == 0) {
					std::cout << "[RTC::SimplePathFollower] Now Following." << std::endl;
					std::cout << "[RTC::SimplePathFollower] Current Pose (" << m_currentPose.data.position.x << ", "
						<< m_currentPose.data.position.y << ", "
						<< m_currentPose.data.heading << ")" << std::endl;

				}
			}


			m_pathFollowerObj.setCurrentPose(m_currentPose.data);
			FOLLOW_RESULT ret = m_pathFollowerObj.follow();

			if (ret == FOLLOW_DISTANCEOUTOFRANGE) {
				m_Mode = MODE_OUTOFRANGE;
				m_pathFollowerObj.stopFollow();
			} else if(m_pathFollowerObj.isGoal()) {
                          std::cout << "[RTC::SimplePathFollower] isGoal()" << std::endl;
				m_pathFollowerObj.stopFollow();
                                m_Mode = MODE_GOALED;
                        } else {
                          m_Mode = MODE_NORMAL;
                        }


			m_pathFollowerObj.getTargetVelocity(m_velocity.data);
			
			if (false) {
				static int j = 0;
				if(++j % 30 == 0) {
					std::cout << "[RTC::SimplePathFollower] Target Velocity (" << m_velocity.data.vx << ", "
						<< m_velocity.data.vy << ", "
						<< m_velocity.data.va << ")" << std::endl;

				}
			}
			setTimestamp(m_velocity);
            if (m_outputRequired) {
                m_velocityOut.write();
            }
			m_poseUpdated = false;
		}
	}

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t SimplePathFollower::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SimplePathFollower::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SimplePathFollower::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SimplePathFollower::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SimplePathFollower::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void SimplePathFollowerInit(RTC::Manager* manager)
  {
    coil::Properties profile(simplepathfollower_spec);
    manager->registerFactory(profile,
                             RTC::Create<SimplePathFollower>,
                             RTC::Delete<SimplePathFollower>);
  }
  
};


