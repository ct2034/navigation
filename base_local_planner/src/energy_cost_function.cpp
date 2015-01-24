/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: TKruse
 *********************************************************************/

#include <base_local_planner/energy_cost_function.h>
#include <cmath>
#include <Eigen/Core>
#include <ros/console.h>

#include <iostream>
#include <fstream>

#define PI 3.14159265359

namespace base_local_planner {

EnergyCostFunction::EnergyCostFunction(costmap_2d::Costmap2D* costmap) 
    : costmap_(costmap), sum_scores_(false) {
  if (costmap != NULL) {
    world_model_ = new base_local_planner::CostmapModel(*costmap_);
  }
  new_set_ = false;
  ROS_INFO(">>> EnergyCostFunction Created");
}

EnergyCostFunction::~EnergyCostFunction() {
  if (world_model_ != NULL) {
    delete world_model_;
  }
}

void EnergyCostFunction::setParams(double max_trans_vel, double max_scaling_factor, double scaling_speed) {
  // TODO: move this to prepare if possible
  max_trans_vel_ = max_trans_vel;
  max_scaling_factor_ = max_scaling_factor;
  scaling_speed_ = scaling_speed;
}

bool EnergyCostFunction::prepare() {
  new_set_ = true;
  return true;
}

double EnergyCostFunction::scoreTrajectory(Trajectory &traj) {
  double cost = 0;
  double x, y, th;
  double ox, oy, oth;
  double length = 0;
  double rot = 0;
  double acc[3];

  if (new_set_) {
  
    last_speeds_[0] = traj.xv_;
    last_speeds_[1] = traj.yv_;
    last_speeds_[2] = traj.thetav_;

    new_set_ = false;
  }

  acc[0] = traj.xv_ - last_speeds_[0];
  acc[1] = traj.yv_ - last_speeds_[1];
  acc[2] = traj.thetav_ - last_speeds_[2];
  
  for (unsigned int i = 0; i < traj.getPointsSize(); ++i) {
    traj.getPoint(i, x, y, th);
    if(i > 0)
    {
      length += hypot((ox - x), (oy - y));
      rot += fmod(std::abs(oth - th), (2*PI));
    }
    ox = x;
    oy = y;
    oth = th;
  }

  ROS_INFO(">>> scoreTrajectory s:%d, l:%.2f, r:%.2f, v:%.3f, a:%.3f", \
    traj.getPointsSize(), length, rot, traj.xv_, acc[0]);

  return cost;
}

void EnergyCostFunction::setLastSpeeds(double x, double y, double th) {
  last_speeds_[0] = x;
  last_speeds_[1] = y;
  last_speeds_[2] = th;
}

} /* namespace base_local_planner */
