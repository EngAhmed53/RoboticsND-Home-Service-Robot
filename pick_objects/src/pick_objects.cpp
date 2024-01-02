#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/String.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

actionlib::SimpleClientGoalState navigateToGoal(MoveBaseClient &ac, move_base_msgs::MoveBaseGoal &goal)
{
    // Send the goal position and orientation for the robot to reach
    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    // Wait an infinite time for the results
    ac.waitForResult();

    return ac.getState();
}

int main(int argc, char **argv)
{
    // Initialize the pick_objects node
    ros::init(argc, argv, "pick_objects");

    ros::NodeHandle nh;

    // Create a publisher for the operation message
    ros::Publisher operation_pub = nh.advertise<std_msgs::String>("robot_operation", 10);

    std_msgs::String operation_msg;

    // tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    // Wait 5 sec for move_base action server to come up
    while (!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal pick_up_pose;
    pick_up_pose.target_pose.header.frame_id = "map";
    pick_up_pose.target_pose.header.stamp = ros::Time::now();
    pick_up_pose.target_pose.pose.position.x = -2.57;
    pick_up_pose.target_pose.pose.position.y = -1.47;
    pick_up_pose.target_pose.pose.orientation.w = 0.824;

    move_base_msgs::MoveBaseGoal drop_pose;
    drop_pose.target_pose.header.frame_id = "map";
    drop_pose.target_pose.header.stamp = ros::Time::now();
    drop_pose.target_pose.pose.position.x = 4.16;
    drop_pose.target_pose.pose.position.y = -1.74;
    drop_pose.target_pose.pose.orientation.w = 0.716;

    auto pick_up_goal_state = navigateToGoal(ac, pick_up_pose);

    if (pick_up_goal_state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("The robot reached the pick up pose");

        operation_msg.data = "CARRY";
        operation_pub.publish(operation_msg);
        ros::spinOnce();

        ros::Duration(5).sleep();

        auto drop_goal_state = navigateToGoal(ac, drop_pose);

        if (drop_goal_state == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("The robot reached the drop pose");

            operation_msg.data = "DROP";
            operation_pub.publish(operation_msg);
            ros::spinOnce();
        }
        else
            ROS_INFO("The robot failed to move to drop pose");
    }
    else
        ROS_INFO("The robot failed to move to pick up pose");

    return 0;
}
