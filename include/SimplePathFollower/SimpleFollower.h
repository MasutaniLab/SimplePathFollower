#pragma once


enum FOLLOW_RESULT {
	FOLLOW_OK,
	FOLLOW_DISTANCEOUTOFRANGE,
	FOLLOW_HEADINGOUTOFRANGE,
	FOLLOW_NOT_IN_FOLLOW,

	FOLLOW_APPROACHINGTOGOAL,
	FOLLOW_TURNTOGOALPOSE,
	FOLLOW_GOAL,
};

class SimpleFollower {
 private:
  float m_distanceToTranslationGain;
  float m_directionToTranslationGain;
  float m_distanceToRotationGain;
  float m_directionToRotationGain;
  float m_approachDirectionGain;
  float m_approachDistanceGain;

  float m_MaxTranslationVelocity;
  float m_MinTranslationVelocity;
  float m_MaxRotationVelocity;
  float m_MinRotationVelocity;
  bool m_following;
  bool m_approaching;
  bool m_goal;
  
  int m_StartPointIndex;
  RTC::Path2D m_targetPath;
  RTC::Pose2D m_currentPose;
  RTC::Velocity2D m_targetVelocity;

 public:
  int getStartPointIndex() {return m_StartPointIndex;}
  int getStopPointIndex() {return m_StartPointIndex+1;}

 public:
  SimpleFollower(float maxTranslationVelocity=0.5, float minTranslationVelocity=0.2, 
	  float maxRotationVelocity=0.5, float minRotationVelocity=0.2,
	  float distanceToTranslationGain=2.0, float directionToTranslationGain=2.0, 
          float distanceToRotationGain=2.5, float directionToRotationGain=2.5, 
          float approachDistanceGain=0.5, float approachDirectionGain=0.5);

  void setGain(float maxTranslationVelocity=0.5, float minTranslationVelocity=0.2,
	  float maxRotationVelocity=0.5, float minRotationVelocity=0.2,
	  float distanceToTranslationGain=2.0, float directionToTranslationGain=2.0, 
          float distanceToRotationGain=2.5, float directionToRotationGain=2.5, 
          float approachDistanceGain=0.5, float approachDirectionGain=0.5);

  ~SimpleFollower();
  
  void startFollow(RTC::Path2D& path);

  void stopFollow();

  bool isFollowing() {return m_following;};

  void setCurrentPose(RTC::Pose2D& pose) { m_currentPose = pose; }
  
  FOLLOW_RESULT follow();

  FOLLOW_RESULT approachGoal(RTC::Pose2D& currentPose, RTC::Waypoint2D& goal);
  void getTargetVelocity(RTC::Velocity2D& velocity) {
    velocity = m_targetVelocity;
  }
  
  bool isGoal() {
    return m_goal;
  }

 private:
  void updateReferenceLine();


};
