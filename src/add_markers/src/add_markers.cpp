#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
//#include <move_base_msgs/MoveBaseActionResult.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <geometry_msgs/PoseStamped.h>

class markerAction
{
   public:
       markerAction(ros::NodeHandle n)
       {
          sub1 = n.subscribe("/move_base/status", 10, &markerAction::goal_status_callback, this);
          sub2 = n.subscribe("/move_base/current_goal", 10, &markerAction::current_goal_callback, this);
       }
       ~markerAction() {};
       void goal_status_callback(const actionlib_msgs::GoalStatusArray& status);
       void current_goal_callback(const geometry_msgs::PoseStamped& pose);

       uint8_t goalStatus() { return goalStatus_;}
       void setGoalStatus(uint8_t status) { goalStatus_ = status;}
       uint8_t getAction() { return action_;}
       void setAction(uint8_t action) { action_ = action;}
      ros::Subscriber sub1;
      ros::Subscriber sub2;

   private:
      uint8_t goalStatus_ = actionlib_msgs::GoalStatus::PENDING;
      geometry_msgs::Pose current_goal;
      uint8_t action_ = visualization_msgs::Marker::ADD;
 };


void markerAction::current_goal_callback(const geometry_msgs::PoseStamped& pose)
{
}

void markerAction::goal_status_callback(const actionlib_msgs::GoalStatusArray& status)
{
  //actionlib_msgs::GoalStatus *statusList = status.status_list;
  //ROS_INFO("%d", status.status_list[0].status);
  goalStatus_ = status.status_list[0].status;
}


int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(10);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  markerAction A(n);

  //ros::Subscriber sub = n.subscribe("/move_base/status", 10, goal_status_callback);
  //ros::spin();
  //ROS_INFO("Number of publishers to %d", (int)sub.getNumPublishers());

  // // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;
  bool prev_goalStatus = actionlib_msgs::GoalStatus::PENDING;
  int success_count = 0;
  while (ros::ok())
  {
    if (A.sub1.getNumPublishers() < 1)
    {
      A.setGoalStatus(actionlib_msgs::GoalStatus::PENDING);
    }
    
    if (success_count == 1)
    {
      A.setAction(visualization_msgs::Marker::DELETE);
    }
    else if (success_count == 2)
    {
      A.setAction(visualization_msgs::Marker::ADD);    
    }
    else
    {
      A.setAction(visualization_msgs::Marker::ADD);       
    }

    if (A.goalStatus() == actionlib_msgs::GoalStatus::SUCCEEDED && prev_goalStatus != actionlib_msgs::GoalStatus::SUCCEEDED)
    {
      success_count += 1;
    }

    // if (A.goalStatus() != actionlib_msgs::GoalStatus::SUCCEEDED && prev_goalStatus == actionlib_msgs::GoalStatus::SUCCEEDED)
    // {
    //   ROS_INFO("Marker deleted");
    //   success_count += 1;
    // }

    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    //ROS_INFO("test");
    ROS_INFO("%d",(int)A.goalStatus());
    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "add_markers";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = A.getAction();

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = 1;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    // while (marker_pub.getNumSubscribers() < 1)
    // {
    //   if (!ros::ok())
    //   {
    //     return 0;
    //   }
    //   ROS_WARN_ONCE("Please create a subscriber to the marker");
    //   sleep(1);
    // }
    marker_pub.publish(marker);

    prev_goalStatus = A.goalStatus();

    ros::spinOnce();
    r.sleep();
  }
}