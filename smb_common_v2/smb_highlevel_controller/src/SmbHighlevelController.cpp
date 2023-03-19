#include <smb_highlevel_controller/SmbHighlevelController.hpp>

namespace smb_highlevel_controller {

SmbHighlevelController::SmbHighlevelController(ros::NodeHandle& nh)
    :nodeHandle(nh), subscriber(), vel_pub(), viz_pub(), marker(),
     msg(), p_ang(-1.0), p_vel(0.1)
{
  std::string topic;
  int queue_size;
//  if ( !nodeHandle.getParam("subscriber_topic", topic)
//      || !nodeHandle.getParam("queue_size", queue_size) )
//  {
//    ROS_ERROR("Could not find subscriber params!");
//    ros::requestShutdown();
//  }
  // create subcriber
  subscriber = nodeHandle.subscribe("/scan", 100, &SmbHighlevelController::callbackFunc, this);
  // create publishers on topic /cmd_vel
  vel_pub = nodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  viz_pub = nodeHandle.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  // pillar marker in RViz
  initPillarMarker();
  ROS_INFO("smb highlevel controller node launched");
  
  // initialize speed
  setVel(2.0, "forward");
}
 /*
  publish a message to topic /cmd_vel to send a Twist to the robot
 */
void SmbHighlevelController::DriveRobot()
{
  ROS_INFO_STREAM("Angular velocity is " << msg.angular.z);
  ROS_INFO_STREAM("Linear velocity is " << msg.linear.x);
  vel_pub.publish(msg);
}

/*
  set robot's linear & angular velocity
*/

void SmbHighlevelController::setVel(const float &vel, const std::string &dof)
{
  if (dof == "ang")
  {
    msg.angular.z = vel;
  }
  else if(dof == "forward")
  {
    msg.linear.x = vel;
  }
  // if (dof == "ang")
  // {
  //   msg.angular.z = vel;
  // }
}

/*
  adjust speed using P control
*/
void SmbHighlevelController::adjustSpeed(const float &dist)
{
  float vel = p_vel * (dist - 1.0); // stop at 0.2m away
  if (vel > 5.0){
    vel = 5.0;
  }
  else if(vel < 0.05){
    vel = .0;
    setVel(.0,"ang"); // do not turn either
  }

  setVel(vel, "forward");
}

/*
  adjust heading using P control
*/
void SmbHighlevelController::adjustHeading(const float &ang)
{
  float diff = -ang;
  setVel(p_ang*diff, "ang");
}

/*
  visualize pillar with marker in RViz
*/
void SmbHighlevelController::vizPillar()
{
  marker.pose.position.x = pillar_pos[0];
  marker.pose.position.y = pillar_pos[1];
  marker.pose.position.z = -1.0;
  viz_pub.publish(marker);
}

/*
  initialize pillar marker in RViz
*/
void SmbHighlevelController::initPillarMarker()
{
  marker.header.frame_id = "base_laser";
  marker.header.stamp = ros::Time();
  marker.ns = "pillar";
  marker.id = 1;
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = pillar_pos[0];
  marker.pose.position.y = pillar_pos[1];
  marker.pose.position.z = -1.0;
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 2;
  marker.color.a = 1.0;// alpha
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0; 
}

/*
 * Print out the position of the pillar with respect to the smb_robot
 * and adjust the robot heading towards the pillar
 */

void SmbHighlevelController::callbackFunc(const sensor_msgs::LaserScan &msg)
{
  // first get the distance
  // typeof(msg.ranges) vector<float>(720)
  auto dist = std::min_element(msg.ranges.cbegin(),msg.ranges.cend());
  // second get the sensor angle [-135 deg, 135 deg]
  int count = dist - msg.ranges.cbegin();
  auto ang = msg.angle_min + msg.angle_increment * count;
  ROS_INFO_STREAM("Pillar is " << *dist << "m away at "
                  << ang / M_PI * 180.0 << "degrees");

  
  // calculate the coorodinate 
  pillar_pos[0] = *dist * std::cos(ang);
  pillar_pos[1] = *dist * std::sin(ang);
  ROS_INFO_STREAM("Pillar's coordinate to the robot is [" << pillar_pos[0] << ", " << pillar_pos[1] << "]");
  // adjust heading and drives robot
  adjustHeading(ang);
  adjustSpeed(*dist);
  DriveRobot();

  // viz pillar
  vizPillar();
}

} /* namespace */
