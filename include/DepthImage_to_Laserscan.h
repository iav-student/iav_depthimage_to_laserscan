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
 *  IAV VI-A3
 *  http://www.iav.com
 *
 */


#ifndef DEPTHIMAGE_TO_LASERSCAN
#define DEPTHIMAGE_TO_LASERSCAN

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/image_encodings.h>

#include "../include/CParam.h"

namespace iav_depthimage_to_laserscan
{

  /*
  * These Class creates a new parameter LaserScan (scan_msg) associated with the camera image and converts
  * the information of the depth image, that it is encoded, into a sensor_msgs::Laserscan.
  *
  * Then fill the ranges of the scan_msg with the points of the depth image msg using a method to project 
  * each pixel into a LaserScan angular increment. And when multiple points coorespond to a specific angular 
  * measurement, then the shortest range is used.
  */
  class DepthImage_to_Laserscan
  {

  public:
	
	  DepthImage_to_Laserscan();
	  ~DepthImage_to_Laserscan();

  	  sensor_msgs::LaserScanPtr convert_msg(const sensor_msgs::ImageConstPtr& depth_msg, const CParam& Param, const LSParam& lsparam);

  private:

	  bool use_point(float new_value, float old_value, float range_min, float range_max);

	  template<typename T>
	  void convert_dtl(const sensor_msgs::ImageConstPtr& depth_msg, const CParam& Param, const LSParam& lsparam, const sensor_msgs::LaserScanPtr& scan_msg);

  };

}; //iav_depthimage_to_laserscan

#endif
