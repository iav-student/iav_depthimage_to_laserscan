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


#include "../include/DepthImage_to_Laserscan.h"

using namespace iav_depthimage_to_laserscan;

  
DepthImage_to_Laserscan::DepthImage_to_Laserscan() {
}

DepthImage_to_Laserscan::~DepthImage_to_Laserscan() {
}

/*
* Determines whether or not new_value should replace old_value in the LaserScan.
*
* Uses the values of range_min, and range_max to determine if new_value is a valid point.  Then it determines if
* new_value is 'more ideal' (currently shorter range) than old_value.
*
* @param new_value The current calculated range.
* @param old_value The current range in the output LaserScan.
* @param range_min The minimum acceptable range for the output LaserScan.
* @param range_max The maximum acceptable range for the output LaserScan.
* @return If true, insert new_value into the output LaserScan. 
*/

bool DepthImage_to_Laserscan::use_point(float new_value, float old_value, float range_min, float range_max) {  
  // Check for NaNs and Infs, a real number within our limits is more desirable than these.
  bool new_finite = std::isfinite(new_value);
  bool old_finite = std::isfinite(old_value);
  
  // Infs are preferable over NaNs (more information)
  if(!new_finite && !old_finite){ // Both are not NaN or Inf.
    if(!isnan(new_value)){ // new is not NaN, so use it's +-Inf value.
      return true;
    }
    return false; // Do not replace old_value
  }
  
  // If not in range, don't bother
  bool range_check = range_min <= new_value && new_value <= range_max;
  if(!range_check){
    return false;
  }
  
  if(!old_finite){ // New value is in range and finite, use it.
    return true;
  }
  
  // Finally, if they are both numerical and new_value is closer than old_value, use new_value.
  bool shorter_check = new_value < old_value;
  return shorter_check;
}

/*
* Create and initiatilize a new parameter LaserScan associated with the image.
*
* This function converts the information in the depth encoded image (UInt16 or Float32 encoding) into
* a sensor_msgs::LaserScan. 
*
* @param depth_msg UInt16 or Float32 encoded depth image.
* @param Param Parameters of the camera(s).
* @return sensor_msgs::LaserScanPtr for the area of the depth image bounded by height_max and height_min. 
*/ 

sensor_msgs::LaserScanPtr DepthImage_to_Laserscan::convert_msg(const sensor_msgs::ImageConstPtr& depth_msg, const CParam& Param, const LSParam& lsparam) 
{
  // Create a new parameter LaserScan associated with the header of Image
  sensor_msgs::LaserScanPtr scan_msg(new sensor_msgs::LaserScan());
  scan_msg->header = depth_msg->header;

  // Add a frame 
  if(lsparam.laser_output_frame_id.length() > 0){
     scan_msg->header.frame_id = lsparam.laser_output_frame_id;
  }

  // Calculate and fill the ranges
  uint32_t ranges_size = lsparam.laser_rays - 1;
  scan_msg->ranges.assign(ranges_size, std::numeric_limits<float>::quiet_NaN());

  scan_msg->angle_min = lsparam.laser_min_angle;
  scan_msg->angle_max = lsparam.laser_max_angle;
  scan_msg->angle_increment = (scan_msg->angle_max - scan_msg->angle_min) / (scan_msg->ranges.size() - 1);;
  scan_msg->time_increment = 0.0;
  scan_msg->scan_time = 0;
  scan_msg->range_min = lsparam.laser_min_range;
  scan_msg->range_max = lsparam.laser_max_range;
  
  if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
  {
     convert_dtl<uint16_t>(depth_msg, Param, lsparam, scan_msg);
  }
  else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
     convert_dtl<float>(depth_msg, Param, lsparam, scan_msg);
  }	
  else
  {
    std::stringstream ss;
    ss << "Depth image has unsupported encoding: " << depth_msg->encoding;
    throw std::runtime_error(ss.str());
  }
  
  return scan_msg;

}

/*
* Fill the ranges with points of the depth image msg.
*
* This uses a method to project each pixel into a LaserScan angular increment. When multiple points coorespond to
* a specific angular measurement, then the shortest range is used.
*
* @param depth_msg The UInt16 or Float32 encoded depth message.
* @param Param Parameters of the camera(s).
* @param scan_msg The output LaserScan.
*/

template<typename T>
void DepthImage_to_Laserscan::convert_dtl(const sensor_msgs::ImageConstPtr& depth_msg, const CParam& Param, const LSParam& lsparam, const sensor_msgs::LaserScanPtr& scan_msg) 
{

  double r; // range
  T depth; // depth point
  double d; // depth in meters
  int index; // index for the correct depth points

  // Auxiliaries for the Resolution Mapping
  int i_start = 0;
  int i_stop = 0;

  // Calculate the resolution of the camera and laser
  float cam_resolution = Param.cam_image_width/Param.cam_AOV_h;
  float laser_resolution = lsparam.laser_rays/(lsparam.laser_max_angle - lsparam.laser_min_angle);

  // Means to acess every depth point of the image
  const T* depth_row = reinterpret_cast<const T*>(&depth_msg->data[0]);
  int row_step = depth_msg->step / sizeof(T);

  // Initialize the ranges
  for (int i=0; i < scan_msg->ranges.size(); i++) scan_msg->ranges[i] = scan_msg->range_max;

  // Loop over all depth image data
  for(int row = 0; row < Param.cam_image_height; row++, depth_row += row_step) {
     
     for(int col = 0; col < Param.cam_image_width; col++) {
	depth = depth_row[col];
	d = depth * 0.001f - Param.cam_offset; // originally mm
        index = Param.cam_index[col];
   
	// Checks if the distance is between the distances limits (height_max and height_min)
	if((Param.cam_min_distances[row] <= d) && (d <= Param.cam_max_distances [row])) {
	   r = d/Param.cam_cos_h[col];
	   
	   // Determines whether or not new_value should replace old_value in the LaserScan.
	   if(use_point(r, scan_msg->ranges[index], lsparam.laser_min_range, lsparam.laser_max_range)) { 
	
	      // Resolution Mapping
	      if ((cam_resolution < laser_resolution) && (col < Param.cam_image_width - 1))
    	      {
        	 // loop over all fitting idx in laserscan i_start = f(col) and i_stop = f(col)
		 i_stop = Param.cam_index[col];
		 i_start = Param.cam_index[col+1];
		 
		 for (int i=i_start; i <= i_stop; i++) scan_msg->ranges[i] = r;
     	      }
	      else
     	      {
		 // look for matching idx in laserscan i = f(col)
		 scan_msg->ranges[index] = r;
     	      }
	   }
	}
     }
  }
}
