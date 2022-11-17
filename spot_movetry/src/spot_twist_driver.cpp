#include <algorithm>
#include <iostream>
#include <math.h>
#include <tf/tf.h>
#include <vector>
#include "spot_leash/amrl_viz_tools.h"
#include "spot_leash/driver.h"
#include "spot_leash/hallway_lanes.h"
#include "spot_leash/viz_tools.h"

namespace spot_leash {



    // TODO: Make UTILS class 
    // TODO: Make this accept templates!
    inline double distance(geometry_msgs::Point p1, geometry_msgs::Point p2){
        return sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2));
    }

    int current_step = 0;

    geometry_msgs::Point Driver::getGoalPoint(geometry_msgs::Point base_point) {
        return _hallway_lanes.closestOffsetPointLane2(base_point, _north_star, 1.5);
    }

    move_base_msgs::MoveBaseGoal presetMoveBaseGoal(){
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "level_mux_map";
        goal.target_pose.header.stamp = ros::Time::now();
        return goal;
    }


    // Get the angular offset of current position to facing goal point
    float angular_difference(geometry_msgs::TransformStamped base_tf, 
		             move_base_msgs::MoveBaseGoal move_base_goal) {
        double base_roll, base_pitch, base_yaw;

        // Geometry_msgs quaternion not accepted by tf matrix - need to manually load a tf one
        tf::Quaternion q(base_tf.transform.rotation.x,
                         base_tf.transform.rotation.y,
                         base_tf.transform.rotation.z,
                         base_tf.transform.rotation.w);

        tf::Matrix3x3(q).getRPY(base_roll, base_pitch, base_yaw);

        double xdiff = move_base_goal.target_pose.pose.position.x - base_tf.transform.translation.x;
        double ydiff = move_base_goal.target_pose.pose.position.y - base_tf.transform.translation.y;
        double base_to_point_yaw = atan2(ydiff, xdiff);
	return (base_to_point_yaw - base_yaw);
    }

    /* Converts move base goal message to a similar twist message */
    geometry_msgs::Twist move_base_to_twist(geometry_msgs::TransformStamped base_tf, 
                                            move_base_msgs::MoveBaseGoal move_base_goal){
        geometry_msgs::Twist twist_msg;
        geometry_msgs::Point base_point = getGeometryPoint(base_tf.transform.translation.x, 
                                                           base_tf.transform.translation.y, 
                                                           base_tf.transform.translation.z);
        double dist = distance(base_point, move_base_goal.target_pose.pose.position);


        //ROS_INFO("DIST: %f", dist);


	float angular_diff = angular_difference(base_tf, move_base_goal);
	int angular_direction = angular_diff > 0 ? -1 : 1;
	//ROS_INFO("ANGULAR DIFF BEFORE %f", angular_diff);
	if (fabs(angular_diff) > M_PI) {
	    angular_diff = ((2 * M_PI) - fabs(angular_diff)) * angular_direction;
	}
	//ROS_INFO("ANGULAR AFTER BEFORE %f", angular_diff);

        double gain = 1.2;
        twist_msg.angular.z = angular_diff * gain;
	//ROS_INFO("TWIST ANGULAR %f", twist_msg.angular.z);
        double angular_scale = std::abs(twist_msg.angular.z);
        twist_msg.linear.x = 0.35/std::max(angular_scale, 0.35); //0.4 moves a little too fast
        //twist_msg.linear.x = 0.5;
        if(dist < 1) {
            twist_msg.linear.x = 0.3;
        }
        //ROS_INFO("------------------------------------------------------");
        //ROS_INFO("TWIST ANGULAR SPEED: %f", twist_msg.angular.z);
        //ROS_INFO("TWIST LINEAR SPEED: %f", twist_msg.linear.x);
        //ROS_INFO("------------------------------------------------------");
        return twist_msg;
    }

    int cooldown = 0;
    // Test snippet
    int current_lane = 0;


    bool Driver::resetSpin() {
	// Get base information
        geometry_msgs::TransformStamped base_tf = tfBuffer.lookupTransform("level_mux_map", "spot_location", ros::Time(0));
	// Set opposite of start point as goal
        geometry_msgs::Point HallwayLanes::_createPoint(double x, double y){
            return _createPoint(x, y, 0);
        }

        


        geometry_msgs::Point origin_point = _hallway_lanes.getLane2().bottom_point;

        move_base_msgs::MoveBaseGoal move_base_goal = presetMoveBaseGoal();
        move_base_goal.target_pose.pose.position.x = origin_point.x;
        move_base_goal.target_pose.pose.position.y = origin_point.y;
        move_base_goal.target_pose.pose.position.z = origin_point.z;
	    // Clip twist message
        geometry_msgs::Twist goal_twist_msg = move_base_to_twist(base_tf, move_base_goal);
	    goal_twist_msg.linear.x = 0;
        //ROS_INFO("ANGULAR SPIN %f", goal_twist_msg.angular.z);
        //goal_twist_msg.angular.z *= 0.15;
	if(fabs(goal_twist_msg.angular.z) < M_PI/10) {
		//ROS_INFO("FINISHED");
		goal_twist_msg.angular.z = 0;
		return true;
	}


        //FIXME: Ignores above and defaults to just a constant spin regardless of direction - not fully automatic!
        goal_twist_msg.angular.z = M_PI/6;
        _twist_pub.publish(goal_twist_msg);
        return true;
    }

    bool Driver::step(){
        // Display hallway in amrl vizualization
        addHallwayLanesToVizMessage(_hallway_lanes, VIZ_COLOR_YELLOW, _amrl_viz_msg);
	    addHallwayDetectionLanesToVizMessage(_hallway_lanes, VIZ_COLOR_MAGENTA, _amrl_viz_msg);
        addHallwayPointsToVizMessage(_hallway_lanes, _amrl_viz_msg);


	    cooldown = std::max(cooldown - 1, 0);
        //ROS_INFO("Driver Step: ");

        move_base_msgs::MoveBaseGoal move_base_goal = presetMoveBaseGoal();

        geometry_msgs::TransformStamped base_tf;
        geometry_msgs::TransformStamped person_tf;

        geometry_msgs::Point person_point;
        geometry_msgs::Point base_point;

        geometry_msgs::Point goal_point;

        bool person_in_frame = false;
        DetectionLane closest_lane_person;
        Lane closest_lane_robot;


        //TODO: Requesting the TFs can be done in a shared util function across drivers */
        /* ------------------------------- Request TF information ------------ */
        try {
            //ROS_INFO("Getting transform between kinect and person now");
            person_tf = tfBuffer.lookupTransform("level_mux_map", "person", ros::Time(0));
            if(abs((int)person_tf.header.stamp.sec - (int)ros::Time::now().sec) < 5.0){
                person_point = getGeometryPoint(person_tf.transform.translation.x, 
                        person_tf.transform.translation.y, 
                        person_tf.transform.translation.z);
                // Publish rviz visualization
                publishArrow(_vis_pub, getGeometryPoint(0, 0, 0), person_point);

                // Push an arrow-like figure to amrl visualization
                addLineToVizMessage(getGeometryPoint(0, 0, 0), person_point, VIZ_COLOR_BLACK, _amrl_viz_msg);
                addPointToVizMessage(person_point, VIZ_COLOR_GREEN, _amrl_viz_msg, 0.6);


                //ROS_INFO("Person Detected at: (%f, %f, %f)", person_point.x, person_point.y, person_point.z);

                closest_lane_person = _hallway_lanes.getClosestDetectionLaneToPoint(person_point);
                //publishGoalHallwayLaneLine(_vis_pub, _hallway_lanes.getDetectionLaneLine(closest_lane_person));

                //AMRL Viz Message for obstructed lane line
                addObstructedHallwayLaneToVizMessage(_hallway_lanes.getDetectionLaneLine(closest_lane_person), _amrl_viz_msg);
                person_in_frame = true;

            }else{
                //ROS_WARN("Old timestamp from kinect transform - skipping");
            }
        } catch (tf2::TransformException &ex) {
            //ROS_WARN("Could NOT find between kinect and person");
        }

        //ROS_INFO("Getting transform between map and robot now");

        try {
            base_tf = tfBuffer.lookupTransform("level_mux_map", "spot_location", ros::Time(0));
            base_point = getGeometryPoint(base_tf.transform.translation.x, 
                    base_tf.transform.translation.y, 
                    base_tf.transform.translation.z);
            //ROS_INFO("base_point: (%f, %f, %f)", base_point.x, base_point.y, base_point.z);
            goal_point = getGoalPoint(base_point);
	    //std::cout  << "RECV POINT " <<  goal_point <<  " \n";
	    //std::cout << " RECV POINT " << goal_point << "\n";
            closest_lane_robot = _hallway_lanes.getClosestLaneToPoint(base_point);
        } catch (tf2::TransformException &ex) {
            //std::cout << ex.what() << std::endl;
            ROS_WARN("Could NOT find between map and baseline");
            return true; // try again - maybe it's not initialized yet
        }

        double base_to_person_dist = distance(base_point, person_point);
	    //ROS_INFO("PDIST: %f", base_to_person_dist);
	    static double inc = 0;

	static bool dodging = false;
	static bool finish_run = false;
        if(person_in_frame && base_to_person_dist < 8.5 && cooldown <= 0 && !finish_run){
	    dodging = true;
	    finish_run = true;
            double goal_offset = 2.75; /*+ (inc-=0.0005);*/
            ROS_INFO("LANE OFFSET: %f", goal_offset);
            Lane goal_lane = LANE_1;
            _next_point = _hallway_lanes.closestOffsetPointLane1(base_point, 
                    _north_star, goal_offset);
            if(closest_lane_person == LANE_LEFT){
                _next_point = _hallway_lanes.closestOffsetPointLane3(base_point, 
                        _north_star, goal_offset);
		goal_lane = LANE_3;
            }
	    ROS_INFO("PERSON DETECTED: CHANGING TO LANE %d", goal_lane);
	    cooldown = 240;
        }else if (distance(base_point, _next_point) < 0.75){
            _next_point = _north_star;
            dodging = false;
        } else if (!dodging && finish_run) {
            _next_point = _north_star;
        } else if (!dodging) {
            _next_point = goal_point;
	}
            			//_next_point = goal_point;

        publishArrow(_vis_pub, base_point, _next_point);

        // Push an arrow-like figure to amrl visualization
        addLineToVizMessage(base_point, _next_point, VIZ_COLOR_BLUE, _amrl_viz_msg);
        addPointToVizMessage(_next_point, VIZ_COLOR_BLUE, _amrl_viz_msg, 0.4);


        move_base_goal.target_pose.pose.position.x = _next_point.x;
        move_base_goal.target_pose.pose.position.y = _next_point.y;
        double yaw = atan((base_point.x - goal_point.x) / (base_point.y - goal_point.y));
        //ROS_INFO("yaw is %f", yaw);
        tf2::Quaternion goal_base_orientation;
        goal_base_orientation.setRPY(0.0, 0.0, yaw);
        move_base_goal.target_pose.pose.orientation.w = goal_base_orientation.getW();
        move_base_goal.target_pose.pose.orientation.x = goal_base_orientation.getX();
        move_base_goal.target_pose.pose.orientation.y = goal_base_orientation.getY();
        move_base_goal.target_pose.pose.orientation.z = goal_base_orientation.getZ();

        /*ROS_INFO("Moving to goal lane position: (%f, %f, %f)", 
                move_base_goal.target_pose.pose.position.x,
                move_base_goal.target_pose.pose.position.y,
                move_base_goal.target_pose.pose.position.z); */


        geometry_msgs::Twist goal_twist_msg = move_base_to_twist(base_tf, move_base_goal);

        double dist_to_end = distance(base_point, _north_star);
        static double dist_thresh = 0.5;
	if (dist_to_end < dist_thresh) {
       	    dist_thresh *= 2;
            // We are close enough out of the hallway to turn around
            return false;
	}
        _twist_pub.publish(goal_twist_msg);
	return true;
    }
}

