#include <ros/ros.h>
#include "std_msgs/String.h"
#include <visualization_msgs/Marker.h>

enum Operation
{
    PICK_UP,
    CARRY,
    DROP
};

auto operation = PICK_UP;

void operationCallback(const std_msgs::String::ConstPtr &msg)
{
    ROS_INFO("Received String: %s", msg->data.c_str());

    if (msg->data == "CARRY")
        operation = CARRY;

    else if (msg->data == "DROP")
        operation = DROP;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "add_markers");

    ros::NodeHandle n;

    ros::Rate r(4);

    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("object_marker", 1);
    ros::Subscriber operation_sub = n.subscribe("robot_operation", 10, operationCallback);

    while (ros::ok())
    {
        visualization_msgs::Marker marker;

        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();

        marker.ns = "object_shape";
        marker.id = 0;

        marker.type = visualization_msgs::Marker::CUBE;
        marker.lifetime = ros::Duration();

        marker.scale.x = .25;
        marker.scale.y = .25;
        marker.scale.z = .25;

        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.pose.position.z = .15;

        ros::spinOnce();

        switch (operation)
        {
        case PICK_UP:
            ROS_INFO("Going to pick up goal");
            marker.pose.position.x = -2.7;
            marker.pose.position.y = -1.47;
            marker.action = visualization_msgs::Marker::ADD;
            break;

        case CARRY:
            ROS_INFO("Reached pick up pose and carried the object");
            marker.action = visualization_msgs::Marker::DELETE;
            break;
        case DROP:
            ROS_INFO("Reached drop pose and droped the object");
            marker.pose.position.x = 4.4;
            marker.pose.position.y = -1.74;
            marker.action = visualization_msgs::Marker::ADD;
            break;
        }

        marker_pub.publish(marker);

        r.sleep();
    }

    return 0;
}