/*
Copyright (C) 2022 Hongkai Ye (kyle_yeh@163.com)
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/
#ifndef _BIAS_SAMPLER_
#define _BIAS_SAMPLER_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <random>
#include <cmath>
#define inf 1000000000
class BiasSampler
{
public:
  BiasSampler()
  {
    std::random_device rd;
    gen_ = std::mt19937_64(rd());
    uniform_rand_ = std::uniform_real_distribution<double>(0.0, 1.0);
    normal_rand_ = std::normal_distribution<double>(0.0, 1.0);
    range_.setZero();
    origin_.setZero();
    origin_informed_.setZero();
  };

  void setSamplingRange(const Eigen::Vector3d origin, const Eigen::Vector3d range)
  {
    origin_ = origin;
    range_ = range;
  };
  
  void enableInformed(bool informed_en)
  {
            informed_en_ = informed_en;
  };

  void setInformedParam(Eigen::Vector3d x_start_,Eigen::Vector3d x_goal_)
  {
         Eigen::Vector3d vector_local(1,0,0);
         Eigen::Vector3d vector_global ;
         vector_global= x_goal_-x_start_;
         vector_global.normalize();
         rotMatrix_informed_ = Eigen::Quaterniond::FromTwoVectors(vector_local,vector_global).toRotationMatrix();
         origin_informed_ = (x_start_+x_goal_)/2;
         c_min_ = (x_start_-x_goal_).norm();
  }
  void samplingOnce(Eigen::Vector3d &sample,double &c_best_)
  {
    if(c_best_<inf && informed_en_)
    {

     //在单位球体内均匀采样；
     double radius = pow(uniform_rand_(gen_),1.0/3);
     double theta = acos(1.0-2.0*uniform_rand_(gen_));
     double psi = 2.0*M_PI*uniform_rand_(gen_);
    //转直角坐标并完成伸缩变换；
        double r0 = c_best_;
        double r1 = sqrt(pow(c_best_,2)-pow(c_min_,2));
        double r2 = r1;
        sample[0]=r0*radius*sin(theta)*cos(psi);
        sample[1]=r1*radius*sin(theta)*sin(psi);
        sample[2]=r2*radius*cos(theta);
    //完成旋转变换；
        sample = rotMatrix_informed_*sample;
        sample +=origin_informed_;

    }else{

        sample[0] = uniform_rand_(gen_);
        sample[1] = uniform_rand_(gen_);
        sample[2] = uniform_rand_(gen_);
        sample.array() *= range_.array();
        sample += origin_;

    }
  };

  // (0.0 - 1.0)
  double getUniRandNum()
  {
    return uniform_rand_(gen_);
  }

private:
  Eigen::Vector3d range_, origin_,origin_informed_;
  Eigen::Matrix3d rotMatrix_informed_;
  double c_min_;
  bool informed_en_;
  std::mt19937_64 gen_;
  std::uniform_real_distribution<double> uniform_rand_;
  std::normal_distribution<double> normal_rand_;
};

#endif
