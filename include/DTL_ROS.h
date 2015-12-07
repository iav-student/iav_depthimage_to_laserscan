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


#include <image_transport/image_transport.h>
#include <boost/thread/mutex.hpp>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "../include/DepthImage_to_Laserscan.h"

namespace iav_depthimage_to_laserscan
{
  /*
  * These class is like a interface with ROS. Here are all the publishes and subscribers, this means that these
  * Class is the Receiver and Sender the daten of the camera(s) to the virtual Laserscan respectively. 
  *
  * Furthermore, it creates a "general" Laserscan joining the daten from the cameras (when there are two cameras) or 
  * just use the results from the first camera. 
  *
  * The class updates the TFs to the actual angles as well. 
  */
  class DTL_ROS 
  {

  public:	

	  DTL_ROS(ros::NodeHandle& n, CParam& Param_1, CParam& Param_2, LSParam& laser_param);
	  ~DTL_ROS();

  private:

  	  void image_callback(const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);
	
	  void image_callback_2(const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);
	
	  iav_depthimage_to_laserscan::DepthImage_to_Laserscan itl; // Instance of the Depthimage_to_Laserscan conversion class

	  iav_depthimage_to_laserscan::CParam Param1; // Parameters of the first camera
	  iav_depthimage_to_laserscan::CParam Param2; // Parameters of the second camera
	  iav_depthimage_to_laserscan::LSParam lsparam; // Parameters of the laser
	
	  // First camera
   	  image_transport::ImageTransport it_; // Subscribes to synchronized Image CameraInfo pairs. Subscribe to image topics.
    	  image_transport::CameraSubscriber sub_; // Subscriber for image_transport
	
	  // Second camera
    	  image_transport::CameraSubscriber sub_2; ///< Subscriber for image_transport

      	  ros::Publisher pub_; // Publisher for output LaserScan messages
    
  	  boost::mutex connect_mutex_; // Prevents the publish and subscriber from being called until everything is initialized.

	  tf::TransformBroadcaster m_TB; //< Broadcaster to send TFs

	  sensor_msgs::LaserScanPtr scan_msg_united; // General Laser Scan (here the combination between the lasers from the first and second camera)
	
  };

}; //iav_depthimage_to_laserscan

