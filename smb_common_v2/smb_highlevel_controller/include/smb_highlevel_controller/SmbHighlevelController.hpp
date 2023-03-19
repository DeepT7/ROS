#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <string>
#include <cmath>
#include <vector>
namespace smb_highlevel_controller {

/*!
 * Class containing the Husky Highlevel Controller
 */
class SmbHighlevelController {
public:
	/*!
	 * Constructor.
	 */
	SmbHighlevelController(ros::NodeHandle& nh);
	SmbHighlevelController(const SmbHighlevelController &) = delete;
	SmbHighlevelController& operator = (const SmbHighlevelController &) = delete;
	// Destructor;
	~SmbHighlevelController() = default;

	/* 
	set robot's linear & angular velocity
	*/
	void setVel(const float &vel, const std::string &dof);
	/*
	publish a message to topic /cmd_vel to send a Twist to the robot
	*/
	void DriveRobot();
	/*
	adjust speed using saturated P control
	*/
	void adjustSpeed(const float &dist);

	/*
	adjust heading using P control
	*/
	void adjustHeading(const float &ang);

	/*
	visualize pillar with marker in RViz
	*/
	void vizPillar();

private:
	ros::NodeHandle nodeHandle;
	ros::Subscriber subscriber;
	ros::Publisher vel_pub, viz_pub;
	geometry_msgs::Twist msg;
	visualization_msgs::Marker marker;
	float p_ang, p_vel;
	float pillar_pos[2];
  void callbackFunc(const sensor_msgs::LaserScan &msg);

  // initialize pillar marker in RViz
  void initPillarMarker();
};

} /* namespace */
