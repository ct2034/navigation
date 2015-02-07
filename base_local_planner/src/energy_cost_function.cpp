/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redisribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redisributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redisributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the disribution.
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
#define DOF 3
#define N_MAX 50
#define TRAJ_SCALE 1

namespace base_local_planner {

EnergyCostFunction::EnergyCostFunction(costmap_2d::Costmap2D* costmap) 
    : costmap_(costmap), sum_scores_(false) {
  if (costmap != NULL) {
    world_model_ = new base_local_planner::CostmapModel(*costmap_);
  }
  new_set_ = false;
  ROS_INFO(">>> EnergyCostFunction Created");

  theta_[0] = .3;
  theta_[1] = .2;
  theta_[2] = .2;
  theta_[3] = .1;
  theta_[4] = .3;
  theta_[5] = .3;
  theta_[6] = .1;
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
  double n = traj.getPointsSize();
  double dis [3][N_MAX];
  double vel [3][N_MAX];
  double vel_mean [3];
  double acc [3][N_MAX];
  double acc_mean [3];
  double t = traj.time_delta_;
  double E_traj, E_route;

  for (int j = 0; j < DOF; j++) {
    vel_mean[j] = 0; 
    acc_mean[j] = 0; 
  }

  for (int i = 0; i < n; ++i) {
    traj.getPoint(i, x, y, th);
    dis [0][i] = x;
    dis [1][i] = y;
    dis [2][i] = th;
    
    if (i > 0) // velocity
    {
      // x
      vel[0][i-1] = ( sin(dis[2][i-1])*(dis[0][i-1] - dis[0][i]) + cos(dis[2][i-1])*(dis[1][i-1] - dis[1][i]) ) / t;
      // y
      vel[1][i-1] = ( cos(dis[2][i-1])*(dis[0][i-1] - dis[0][i]) - sin(dis[2][i-1])*(dis[1][i-1] - dis[1][i]) ) / t;
      // theta
      vel[2][i-1] = fmod(abs(dis[2][i-1] - dis[2][i]), (2*PI)) / t;

      // mean
      for (int j = 0; j < DOF; j++) {
        vel_mean[j] += abs(vel[j][i-1]);
      }

      // lengths
      length += hypot((dis[0][i-1] - dis[0][i]), (dis[1][i-1] - dis[1][i]));
      rot += fmod(abs(dis[2][i-1] - dis[2][i]), (2*PI));
    }
    if (i > 1) // acceleration
    {
      // x
      acc[0][i-2] = ( sin(dis[2][i-2])*(vel[0][i-2] - vel[0][i-1]) + cos(dis[2][i-2])*(vel[1][i-2] - vel[1][i-1]) ) / t;
      // y
      acc[1][i-2] = ( cos(dis[2][i-2])*(vel[0][i-2] - vel[0][i-1]) - sin(dis[2][i-2])*(vel[1][i-2] - vel[1][i-1]) ) / t;
      // theta
      acc[2][i-2] = fmod(abs(vel[2][i-2] - vel[2][i-1]), (2*PI)) / t;

      // mean
      for (int j = 0; j < DOF; j++) {
        acc_mean[j] += acc[j][i-2];
      }
    }
  }

  for (int j = 0; j < DOF; j++) {
    vel_mean[j] /= n-1;
    acc_mean[j] /= n-2; 
  }
    
  E_traj = 
   (theta_[0] +                 //  th_1
    theta_[1] * vel_mean[0] +   //  v_x
    theta_[2] * vel_mean[1] +   //  v_y
    theta_[3] * vel_mean[2] +   //  v_th
    theta_[4] * acc_mean[0] +   //  a_x
    theta_[5] * acc_mean[1] +   //  a_y
    theta_[6] * acc_mean[2] ) * //  a_th
    n * t /                     //  * duration
    hypot(length, rot);         //  / distance

  E_route = 0;

  // ROS_INFO(">>> scoreTrajectory s:%d, l:%.2f, r:%.2f, v:%.3f, a:%.3f, e:%.3f", \
  //   traj.getPointsSize(), length, rot, vel[0][0], acc[0][0], E_traj);

  cost = E_traj * TRAJ_SCALE + E_route * (1-TRAJ_SCALE) ;
  return cost;
}

void EnergyCostFunction::setLastSpeeds(double x, double y, double th) {
  last_speeds_[0] = x;
  last_speeds_[1] = y;
  last_speeds_[2] = th;
}

void EnergyCostFunction::thetaCallback(const auckbot_analysis::ModelTheta msg) {
  ROS_INFO("------> thetaCallback <----");
}

} /* namespace base_local_planner */
