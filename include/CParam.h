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


#ifndef CPARAM
#define CPARAM

#include <stdlib.h>
#include <sstream>
#include <math.h>
#include <limits.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "../include/LSParam.h"

namespace iav_depthimage_to_laserscan
{

  /*
  * These Class contains all the parameters of the camera(s), that must to be initialized. 
  * 
  * Furthermore had the method init_parameter, that is used to stablish the geometrical limits of the image through
  * a max and min height.
  *
  * The class had methods and some parameters for the frames of every camera, from the creation to the
  * updating of a frame. 
  */
  class CParam
  {
  public:

  	  CParam();
	  ~CParam();

	  void init_parameter(CParam& Param, LSParam& laserParam, float height_max, float height_min);

          void createTF(std::string header, std::string child);

	  // Camera Variables
	  double cam_AOV_h;    // Horizontal angle of view
  	  double cam_AOV_v;    // Vertical angle of view
	  double cam_yaw;      // Yaw is positive leftwards. 
	  double cam_pitch;    // Pitch is positive downward. 
	  double cam_offset;   // Offset (short distance) between camera and laser

	  int cam_image_width; // Image width
	  int cam_image_height; // Image height
	  int* cam_index; // Index for the correct depth points (pre calculated)
	  double cam_focal_length_x; // Focal length in x
	  double cam_focal_length_y; // Focal length in y
	  std::string cam_frame_id; // Camera header.frame_id

	  // Auxiliaries to limit the "angle of view" of the laser
	  float* cam_max_distances; // Max. distances limits within the area bounded by height_max and height_min
	  float* cam_min_distances; // Min. distances limits within the area bounded by height_max and height_min
	  float* cam_cos_h; // Calculating a cosine for each point in horizontal
          float* cam_cos_v; // Calculating a cosine for each point in vertical	
          float* cam_sin_v; // Calculating a sine for each point in vertical

       	  geometry_msgs::TransformStamped* m_tf; //< Contains the TF

  };

}; //iav_depthimage_to_laserscan

#endif
