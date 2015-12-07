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

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Main program /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
    {			
        ros::init(argc, argv, "iav_depthimage_to_laserscan");
        ros::NodeHandle n;	

	iav_depthimage_to_laserscan::CParam CParam1;
	iav_depthimage_to_laserscan::CParam CParam2;
	iav_depthimage_to_laserscan::LSParam laser_param;

	double height_max; // value > 0 (considering 0 the laser_height), wich limits the height above the laser scan
	double height_min; // value < 0 (considering 0 the laser_height), wich limits the height below the laser scan

	n.param<double>("height_max", height_max, 0.20);
	n.param<double>("height_min", height_min, -0.65);

	n.param<double>("cam_1_AOV_h", CParam1.cam_AOV_h, 1.01229097);
	n.param<double>("cam_1_AOV_v", CParam1.cam_AOV_v, 0.785398163);
	n.param<double>("cam_1_yaw", CParam1.cam_yaw, 0.0);
	n.param<double>("cam_1_pitch", CParam1.cam_pitch, 0.0);
	n.param<double>("cam_1_offset", CParam1.cam_offset, 0.0);
	n.param<int>("cam_1_image_width", CParam1.cam_image_width, 640);
	n.param<int>("cam_1_image_height", CParam1.cam_image_height, 480);
	n.param<double>("cam_1_focal_length_x", CParam1.cam_focal_length_x, 570);
	n.param<double>("cam_1_focal_length_y", CParam1.cam_focal_length_y, 570);
	n.param<std::string>("cam_1_frame_id", CParam1.cam_frame_id, "camera389_link");

	n.param<double>("cam_2_AOV_h", CParam2.cam_AOV_h, 1.01229097);
	n.param<double>("cam_2_AOV_v", CParam2.cam_AOV_v, 0.785398163);
	n.param<double>("cam_2_yaw", CParam2.cam_yaw, 1.009744261); // It is lagged by 57° (default value) to the left in relation to the first camera
	n.param<double>("cam_2_pitch", CParam2.cam_pitch, 0.0);
	n.param<double>("cam_2_offset", CParam2.cam_offset, 0.0);
	n.param<int>("cam_2_image_width", CParam2.cam_image_width, 640);
	n.param<int>("cam_2_image_height", CParam2.cam_image_height, 480);
	n.param<double>("cam_2_focal_length_x", CParam2.cam_focal_length_x, 570);
	n.param<double>("cam_2_focal_length_y", CParam2.cam_focal_length_y, 570);
	n.param<std::string>("cam_2_frame_id", CParam2.cam_frame_id, "camera568_link");

	n.param<double>("laser_max_range", laser_param.laser_max_range, 5);
	n.param<double>("laser_min_range", laser_param.laser_min_range, 0.5);
	n.param<double>("laser_max_angle", laser_param.laser_max_angle, 3.14);
	n.param<double>("laser_min_angle", laser_param.laser_min_angle, -3.14);
	n.param<int>("laser_rays", laser_param.laser_rays, 640);
	n.param<std::string>("laser_output_frame_id", laser_param.laser_output_frame_id, "laser_link");

	// create and adding tf to m_tf
	CParam1.createTF(laser_param.laser_output_frame_id, CParam1.cam_frame_id);   
	CParam2.createTF(laser_param.laser_output_frame_id, CParam2.cam_frame_id);   

	// Initializes the parameters of each camera
	CParam1.init_parameter(CParam1, laser_param, height_max, height_min);
	CParam2.init_parameter(CParam2, laser_param, height_max, height_min);

	iav_depthimage_to_laserscan::DTL_ROS dtl(n, CParam1, CParam2, laser_param);
	
	ros::spin();	

	return 0;
}
