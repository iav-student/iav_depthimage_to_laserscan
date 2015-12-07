/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2015, IAV Automotive Engineering
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 *    this list of conditions and the following disclaimer in the documentation 
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR 
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/*
 *  IAV Team
 *  http://www.iav.com
 *
 */


#include "../include/DTL_ROS.h"

using namespace iav_depthimage_to_laserscan;

DTL_ROS::DTL_ROS(ros::NodeHandle& n, CParam& Param_1, CParam& Param_2, LSParam& laser_param): it_(n){
  boost::mutex::scoped_lock lock(connect_mutex_);

  // Initialize the parameters
  Param1 = Param_1;
  Param2 = Param_2;
  lsparam = laser_param;
  sensor_msgs::LaserScanPtr scan_msg(new sensor_msgs::LaserScan());
  scan_msg_united = scan_msg;
  	  
  // Calculate and fill the ranges for the scan_msg_united
  uint32_t ranges_size = lsparam.laser_rays - 1;
  scan_msg_united->ranges.assign(ranges_size, std::numeric_limits<float>::quiet_NaN());

  scan_msg_united->angle_min = lsparam.laser_min_angle;
  scan_msg_united->angle_max = lsparam.laser_max_angle;
  scan_msg_united->angle_increment = (scan_msg_united->angle_max - scan_msg_united->angle_min) / (scan_msg_united->ranges.size() - 1);;
  scan_msg_united->time_increment = 0.0;
  scan_msg_united->scan_time = 0;
  scan_msg_united->range_min = lsparam.laser_min_range;
  scan_msg_united->range_max = lsparam.laser_max_range;

  pub_ = n.advertise<sensor_msgs::LaserScan>("Laser_Scan", 10);

  image_transport::TransportHints hints("raw", ros::TransportHints(), n);
  sub_ = it_.subscribeCamera("image", 10, &DTL_ROS::image_callback, this, hints); //Subscribe to a synchronized image & camera info topic pair.
  sub_2 = it_.subscribeCamera("image2", 10, &DTL_ROS::image_callback_2, this, hints); //Subscribe to a synchronized image & camera info topic pair.
  
}

DTL_ROS::~DTL_ROS() {
  sub_.shutdown();
  sub_2.shutdown();
}

/*
* Callbacks for image_transport
*
* Synchronized callback for depth image and camera info (parameters of the camera). Publishes laserscan at the end.
*
* @param depth_msg Image provided by image_transport.
* @param info_msg CameraInfo provided by image_transport.
*/

void DTL_ROS::image_callback(const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::CameraInfoConstPtr& info_msg) {
  // Called the function to convert the depth_msg in laser_msg
  sensor_msgs::LaserScanPtr scan_msg = itl.convert_msg(depth_msg, Param1, lsparam); 
 
  scan_msg_united->header = depth_msg->header; // Associates the general laser_msg header with depth_msg header of the first camera
  scan_msg_united->header.frame_id = lsparam.laser_output_frame_id; // Add the real frame of the laser 
  
  int index; // Index for the correct depth points

  // Fill the ranges for the scan_msg_united with the laser of the first camera
  for(int i=0; i < Param1.cam_image_width; i++) { 
     index = Param1.cam_index[i];
     scan_msg_united->ranges[index] = scan_msg->ranges[index];
  }

  // Set current time and publish over TransformBroadcaster
  ros::Time time = ros::Time::now();
  Param1.m_tf->header.stamp = time;
  m_TB.sendTransform(*Param1.m_tf);

  // Publishes the general laser scan (with informations from the laser(s))
  pub_.publish(scan_msg_united); 
}

void DTL_ROS::image_callback_2(const sensor_msgs::ImageConstPtr& depth_msg2, const sensor_msgs::CameraInfoConstPtr& info_msg2) {
  // Called the function to convert the depth_msg in laser_msg
  sensor_msgs::LaserScanPtr scan_msg_2 = itl.convert_msg(depth_msg2, Param2, lsparam); 
  
  int index; // index for the correct depth points

  // Fill the ranges for the scan_msg_united with the laser of the second camera
  for(int i=0; i < Param2.cam_image_width; i++) { 
     index = Param2.cam_index[i];
     scan_msg_united->ranges[index] = scan_msg_2->ranges[index];
  }

  // Set current time and publish over TransformBroadcaster
  ros::Time time = ros::Time::now();
  Param2.m_tf->header.stamp = time;
  m_TB.sendTransform(*Param2.m_tf);

}
