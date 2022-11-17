#include <spot_leash/spot_leash_transform_listener.h>


SpotLeashTransformListener::SpotLeashTransformListener(ros::NodeHandle &nh){
    _loc_sub = nh.subscribe("/localization", 1000, &SpotLeashTransformListener::localizationCallback, this);
}

SpotLeashTransformListener::~SpotLeashTransformListener(){}

geometry_msgs::TransformStamped SpotLeashTransformListener::getRobotLocation(){
    return _base_tf;
}

geometry_msgs::TransformStamped getKinectTf() {
    geometry_msgs::TransformStamped kinect_tf;
    kinect_tf.header.stamp = ros::Time::now();
    kinect_tf.header.frame_id = "spot_location";   
    kinect_tf.child_frame_id = "nav_kinect_link";
    kinect_tf.transform.translation.x = 0;
    kinect_tf.transform.translation.y = 0;
    kinect_tf.transform.translation.z = 0;
    kinect_tf.transform.rotation.x = 0;
    kinect_tf.transform.rotation.y = 0;
    kinect_tf.transform.rotation.z = 0;
    kinect_tf.transform.rotation.w = 1;

    return kinect_tf;
}

void SpotLeashTransformListener::localizationCallback(const amrl_msgs::Localization2DMsg::ConstPtr& msg){
    tf2::Quaternion quaternion;
    quaternion.setRPY( 0, 0, msg->pose.theta );
    ROS_INFO("Publishing tf");

    _base_tf.header.stamp = ros::Time::now();
    // Static publisher map -> level_mux_map should exist on launch
    _base_tf.header.frame_id = "level_mux_map";   
    _base_tf.child_frame_id = "spot_location";
    _base_tf.transform.translation.x = msg->pose.x;
    _base_tf.transform.translation.y = msg->pose.y;
    _base_tf.transform.translation.z = 0;
    _base_tf.transform.rotation.x = quaternion.x();
    _base_tf.transform.rotation.y = quaternion.y();
    _base_tf.transform.rotation.z = quaternion.z();
    _base_tf.transform.rotation.w = quaternion.w();

    br.sendTransform(_base_tf);
    geometry_msgs::TransformStamped _kinect_link_tf = _base_tf;
    br.sendTransform(getKinectTf());
}
