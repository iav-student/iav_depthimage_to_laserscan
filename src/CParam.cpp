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


#include "../include/CParam.h"

using namespace iav_depthimage_to_laserscan;

CParam::CParam() {
}

CParam::~CParam() {
}

/*
* Init a TF and set names
*
* @param header header.frame_id
* @param child child_frame_id
*/

void CParam::createTF(std::string header, std::string child) {
    // Create new TF
    geometry_msgs::TransformStamped* tf = new geometry_msgs::TransformStamped();
    
    // Set header and child frame id
    tf->header.frame_id = header;
    tf->child_frame_id = child;

    // Set Transform Translations of the camera (x y z)
    // apply translation from Laserscan's coordinates
    tf->transform.translation.x = cam_offset*cos(cam_yaw);  // cam_offset*cos(cam_yaw)
    tf->transform.translation.y = cam_offset*sin(cam_yaw);  // cam_offset*sin(cam_yaw)
    tf->transform.translation.z = 0.0;  // 0.0

    // Set Transform Rotations of the camera (roll pitch yaw)
    // apply rotation from Laserscan's angles
    tf->transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0.0, cam_pitch, cam_yaw);

    // Add a TF to m_tf
    m_tf = tf;
}

/*
* This function is used to stablish the geometrical limits of the image. It is used just once.
* @param Param Parameters of the camera
* @param laserParam Parameters of the laser
* @param height_max value > 0 (considering 0 the laser_height), wich limits the height above the laser scan
* @param height_min value < 0 (considering 0 the laser_height), wich limits the height below the laser scan
*/
void CParam::init_parameter(CParam& Param, LSParam& laserParam, float height_max, float height_min) {
  
  // Allocate memory for the auxiliaries
  Param.cam_max_distances = (float*) malloc (Param.cam_image_height*sizeof(float));
  Param.cam_min_distances = (float*) malloc (Param.cam_image_height*sizeof(float));

  Param.cam_sin_v = (float*) malloc (Param.cam_image_height*sizeof(float)); 
  Param.cam_cos_v = (float*) malloc (Param.cam_image_height*sizeof(float)); 
  Param.cam_cos_h = (float*) malloc (Param.cam_image_width*sizeof(float));
  Param.cam_index = (int*) malloc (Param.cam_image_width*sizeof(int)); 

  // Auxiliares for the calculation of the distances limits
  float increment_v = Param.cam_AOV_v / Param.cam_image_height;
  float distance_a; 
  float distance_b;
  float longer;
  float shorter; 
  float angle;

  // Define the laser angle CParam 
  double th;
  double angle_min = laserParam.laser_min_angle;
  double angle_increment = (laserParam.laser_max_angle - laserParam.laser_min_angle)/(laserParam.laser_rays - 1);

  // Use correct principal point from calibration
  float center_x = Param.cam_image_width/2;
  float center_y = Param.cam_image_height/2;

  // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
  double unit_scaling = 0.001f;
  float constant_x = unit_scaling / Param.cam_focal_length_x;
  float constant_y = unit_scaling / Param.cam_focal_length_y;

  // Pre calculation of the index for the correct depth points
  for(int u = 0; u <  Param.cam_image_width; u++) {
    th = atan2((double)(center_x - u) * constant_x, unit_scaling); // Atan2(x, z), but depth divides out

    if((th - angle_min + cam_yaw) / angle_increment > laserParam.laser_rays - 1) cam_index[u] = laserParam.laser_rays - 1;
    else if((th - angle_min + cam_yaw) / angle_increment < 0) cam_index[u] = 0;
    else cam_index[u] = (th - angle_min + cam_yaw) / angle_increment;
	
    Param.cam_cos_h[u] =  cos (th);
  }

  // Calculate the distances limits based on height_max and height_min args
  for (int j = 0; j < Param.cam_image_height; j++) {	
    angle = atan2((double)(center_y - j) * constant_y, unit_scaling) - Param.cam_pitch;
   
    Param.cam_sin_v[j] =  sin (angle);
    Param.cam_cos_v[j] =  cos (angle);
    
            
    if (Param.cam_sin_v [j] != 0.0) {
       distance_a = (height_max) / Param.cam_sin_v[j]; 
       distance_b = (height_min) / Param.cam_sin_v[j]; 
		 
       if (distance_a < 0.0) {
          distance_a =  laserParam.laser_min_range;
       }

       if (distance_b < 0.0){
          distance_b = laserParam.laser_min_range; 
       }                             
    
       longer  = std::max ( distance_a, distance_b );
       shorter = std::min ( distance_a, distance_b );

       Param.cam_max_distances[j] = std::min(longer,(float) laserParam.laser_max_range);
       Param.cam_min_distances[j] = std::max(shorter,(float) laserParam.laser_min_range);
    }
                       
    else
    {
       Param.cam_max_distances[j] = laserParam.laser_max_range;
       Param.cam_min_distances[j] = laserParam.laser_min_range;
    }                                                    
  } 
} 
