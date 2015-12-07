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


#ifndef LSPARAM
#define LSPARAM

#include <stdlib.h>
#include <sstream>
#include <math.h>
#include <limits.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

namespace iav_depthimage_to_laserscan
{
  
  // These Class contains all the parameters of the laser scanner, that must to be initialized.  
  class LSParam
  {
  public:

  	  LSParam(){
	  }

	  ~LSParam(){
	  }

	  // Laser Variables
  	  std::string laser_output_frame_id; // Laser header.frame_id

	  double laser_max_range; // Max. ranges of the laser
	  double laser_min_range; // Min. ranges of the laser
          
	  // Angle of View of the laser
  	  double laser_max_angle; 
	  double laser_min_angle;
	
  	  int laser_rays; // amount of rays (for the resolution)

  };

}; //iav_depthimage_to_laserscan

#endif
