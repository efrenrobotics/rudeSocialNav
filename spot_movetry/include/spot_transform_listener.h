#ifndef TRANSFORM_LISTENER_H
#define TRANSFORM_LISTENER_H

#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <amrl_msgs/Localization2DMsg.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

class SpotTransformListener {
    public:
        
        SpotTransformListener(ros::NodeHandle &nh);
        ~SpotTransformListener();

        geometry_msgs::TransformStamped getRobotLocation();

        void localizationCallback(const amrl_msgs::Localization2DMsg::ConstPtr& msg);

    private:
        
        ros::Subscriber _loc_sub;
        geometry_msgs::TransformStamped _base_tf;
        tf2_ros::TransformBroadcaster br;
};



#endif