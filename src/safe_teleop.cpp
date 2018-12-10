/**
 * @file safe_teleop.cpp
 * @brief Safe teleoperation library implementation
 * Created by rakesh on 28/09/18.
 */
#include <limits>
#include <safe_teleop/safe_teleop.h>

namespace safe_teleop
{

SafeTeleop::SafeTeleop() :
  is_shutdown_(false),
  max_cmd_vel_age_(1.0),
  max_linear_vel_(1.0),
  max_angular_vel_(1.0),
  linear_vel_increment_(0.05),
  angular_vel_increment_(0.05),
  laser_safety_check_angle_(0.25),
  min_safety_impact_time_(0.5),
  min_safety_distance_(0.5),
  linear_vel_(0.0),
  angular_vel_(0.0),
  linear_speed_(0.0),
  angular_speed_(0.0),
  last_command_timestamp_(0.0)
{
  ros::NodeHandle global_nh;
  cmd_vel_pub_ = global_nh.advertise<geometry_msgs::Twist>("cmd_vel", 5);
  // The subscriber callback is set to the laserScanCallback method of the instantiated object of this class
  laser_scan_sub_ = global_nh.subscribe("scan", 5, &SafeTeleop::laserScanCallback, this);
  run_thread_ = boost::thread(&SafeTeleop::run, this);
  displayCurrentSpeeds();
}

SafeTeleop::~SafeTeleop()
{
  shutdown();
  // wait for the run thread to terminate
  run_thread_.join();

  geometry_msgs::Twist zero_cmd_vel;
  zero_cmd_vel.linear.x = 0;
  zero_cmd_vel.angular.z = 0;
  cmd_vel_pub_.publish(zero_cmd_vel);

}

void SafeTeleop::run()
{
  ros::Rate r(10);
  while (ros::ok() && !is_shutdown_)
  {
    // ROS_WARN("running\r");
    auto current_timestamp = ros::Time::now().toSec();

    auto last_cmd_vel_age = current_timestamp - last_command_timestamp_;

    if (last_cmd_vel_age > max_cmd_vel_age_)
    {
    	// ROS_WARN_THROTTLE(1.0, "Timeout not implemented\r");
    	ROS_WARN("Timeout\r");
    	geometry_msgs::Twist zero_cmd_vel;
			zero_cmd_vel.linear.x = 0;
			zero_cmd_vel.angular.z = 0;
			cmd_vel_pub_.publish(zero_cmd_vel);
    }
    else
    {
      auto is_safe = checkSafety(static_cast<double>(linear_vel_));
      double _linear_vel_;
      double _angular_vel_;

      if(is_safe == false){
      	_linear_vel_ = 0;
      	ROS_INFO_THROTTLE(1,"Unsafe, too close to obstacle\r");
      }else{
      	_linear_vel_ = linear_vel_.load(boost::memory_order_consume);
      }
      _angular_vel_ = angular_vel_.load(boost::memory_order_consume);
      geometry_msgs::Twist cmd_vel_;
		  cmd_vel_.linear.x = _linear_vel_;
		  cmd_vel_.angular.z = _angular_vel_;
		  cmd_vel_pub_.publish(cmd_vel_);
    }

    r.sleep();
  }
}

void SafeTeleop::moveForward()
{
  // ROS_WARN("Method not implemented\r");
  ROS_WARN("Move Forward\r");
  double _linear_speed_;
  _linear_speed_ = linear_speed_.load(boost::memory_order_consume);
  linear_vel_.store(_linear_speed_, boost::memory_order_release);
  angular_vel_.store(0, boost::memory_order_release);
  last_command_timestamp_ = ros::Time::now().toSec();
  // displayCurrentVelocities();
}

void SafeTeleop::moveBackward()
{
  // ROS_WARN("Method not implemented\r");
  ROS_WARN("Move Backward\r");
  double _linear_speed_;
  _linear_speed_ = -linear_speed_.load(boost::memory_order_consume);
  linear_vel_.store(_linear_speed_, boost::memory_order_release);
  angular_vel_.store(0, boost::memory_order_release);
  last_command_timestamp_ = ros::Time::now().toSec();
  // displayCurrentVelocities();
}

void SafeTeleop::rotateClockwise()
{
  // ROS_WARN("Method not implemented\r");
  ROS_WARN("Rotate Clockwise\r");
  double _angular_speed_;
  _angular_speed_ = -angular_speed_.load(boost::memory_order_consume);
  angular_vel_.store(_angular_speed_, boost::memory_order_release);
  linear_vel_.store(0, boost::memory_order_release);
  last_command_timestamp_ = ros::Time::now().toSec();
  // displayCurrentVelocities();
}

void SafeTeleop::rotateCounterClockwise()
{
  // ROS_WARN("Method not implemented\r");
  ROS_WARN("Rotate CounterClockwise\r");
  double _angular_speed_;
  _angular_speed_ = angular_speed_.load(boost::memory_order_consume);
  angular_vel_.store(_angular_speed_, boost::memory_order_release);
  linear_vel_.store(0, boost::memory_order_release);
  last_command_timestamp_ = ros::Time::now().toSec();
  // displayCurrentVelocities();
}

void SafeTeleop::stop()
{
  // ROS_WARN("Method not implemented\r");
  angular_vel_.store(0, boost::memory_order_release);
  linear_vel_.store(0, boost::memory_order_release);
  last_command_timestamp_ = ros::Time::now().toSec();
  displayCurrentSpeeds();
}


void SafeTeleop::increaseLinearSpeed()
{
  // ROS_WARN("Method not implemented\r");
  double _linear_speed_;
  _linear_speed_ = linear_speed_.load(boost::memory_order_consume);
  _linear_speed_ += linear_vel_increment_;
  if(_linear_speed_ > max_linear_vel_){
  	_linear_speed_ = max_linear_vel_;
  }
  linear_speed_.store(_linear_speed_, boost::memory_order_release);
  displayCurrentSpeeds();
}

void SafeTeleop::decreaseLinearSpeed()
{
  // ROS_WARN("Method not implemented\r");
  double _linear_speed_;
  _linear_speed_ = linear_speed_.load(boost::memory_order_consume);
  _linear_speed_ -= linear_vel_increment_;
  if(_linear_speed_ < 0){
  	_linear_speed_ = 0;
  }
  linear_speed_.store(_linear_speed_, boost::memory_order_release);
  displayCurrentSpeeds();
}

void SafeTeleop::increaseAngularSpeed()
{
  // ROS_WARN("Method not implemented\r");
  double _angular_speed_;
  _angular_speed_ = angular_speed_.load(boost::memory_order_consume);
  _angular_speed_ += angular_vel_increment_;
  if(_angular_speed_ > max_angular_vel_){
  	_angular_speed_ = max_angular_vel_;
  }
  angular_speed_.store(_angular_speed_, boost::memory_order_release);
  displayCurrentSpeeds();
}

void SafeTeleop::decreaseAngularSpeed()
{
  // ROS_WARN("Method not implemented\r");
  double _angular_speed_;
   _angular_speed_ = angular_speed_.load(boost::memory_order_consume);
  _angular_speed_ -= angular_vel_increment_;
  if(_angular_speed_ < 0){
  	_angular_speed_ = 0;
  }
  angular_speed_.store(_angular_speed_, boost::memory_order_release);
  displayCurrentSpeeds();
}

bool SafeTeleop::checkSafety(double linear_vel)
{
  // ROS_WARN("checkSafety Method not implemented\r");
  auto laser_scan = getLaserScan();
  double _linear_vel_;
  _linear_vel_ = linear_vel_.load(boost::memory_order_consume);
  
	/*for stimulator*/
	if(laser_scan.ranges.size() == 128){
	  double current_angle = (laser_scan.angle_min/(2*M_PI))*360;
	  double max_angle = (laser_scan.angle_max/(2*M_PI))*360;
	  for(int idx = 0; current_angle <= max_angle; idx++){

	    if(_linear_vel_>0){
	    	double total_distance_ = min_safety_distance_ + (_linear_vel_*min_safety_impact_time_);
	      if(current_angle>=-15&&current_angle<=15){
	        if(laser_scan.ranges[idx]<total_distance_){
	          return false;
	        }
	      }
	    }else{
	    	double total_distance_ = min_safety_distance_ - (_linear_vel_*min_safety_impact_time_);
	      if(current_angle>=-180&&current_angle<=-165){
	        if(laser_scan.ranges[idx]<total_distance_){
	          return false;
	        }
	      }
	      if(current_angle>=165&&current_angle<=180){
	        if(laser_scan.ranges[idx]<total_distance_){
	          return false;
	        }
	      }
	    }
	    current_angle+=(laser_scan.angle_increment/(2*M_PI))*360;
	    if(idx > laser_scan.ranges.size()){
	      return true;
	    }
	  }
	  return true;
	}else if(laser_scan.ranges.size() == 360){
		/*for real robot*/
	  double current_angle = (laser_scan.angle_min/(2*M_PI))*360;
	  double max_angle = (laser_scan.angle_max/(2*M_PI))*360;
	  double min_angle = (laser_scan.angle_min/(2*M_PI))*360;

	  for(int idx = 1; current_angle <= max_angle; idx++){
	    if(laser_scan.ranges[idx] == 0)
	        continue;
	    if(_linear_vel_>0){
	    	double total_distance_ = min_safety_distance_ + (_linear_vel_*min_safety_impact_time_);
	      // ROS_INFO_THROTTLE(1,"current_angle:%f, ranges:%f\r", current_angle, laser_scan.ranges[idx]);
	      // ROS_WARN("current_angle:%f, ranges:%f\r", current_angle, laser_scan.ranges[idx]);
	      if(current_angle>=0&&current_angle<=15){
	        
	        if(laser_scan.ranges[idx]<total_distance_){
	          return false;
	        }
	      }
	      if(current_angle>=345&&current_angle<=360){
	        if(laser_scan.ranges[idx]<total_distance_){
	          return false;
	        }
	      }
	    }else{
	    	double total_distance_ = min_safety_distance_ - (_linear_vel_*min_safety_impact_time_);
	      // ROS_INFO_THROTTLE(1,"current_angle:%f, ranges:%f\r", current_angle, laser_scan.ranges[idx]);
	      // ROS_WARN("current_angle:%f, ranges:%f\r", current_angle, laser_scan.ranges[idx]);

	      if(current_angle>=165&&current_angle<=195){
	        if(laser_scan.ranges[idx]<total_distance_){
	          return false;
	        }
	      }
	    }
	    current_angle+=(laser_scan.angle_increment/(2*M_PI))*360;
	    if(idx > laser_scan.ranges.size()){
	      return true;
	    }
	  }
	  return true;
	}else{
		return false;
	}


}

} // namespace safe_teleop_node
