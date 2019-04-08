#ifndef _GATE_EST_FRAME_HPP_
#define _GATE_EST_FRAME_HPP_

#include <ros/ros.h>
#include <sophus/se3.hpp>
#include <opencv/cv.h>
#include "common.hpp"

#include <memory>

namespace gate_est {

  struct Frame {
    ros::Time timestamp;
    Landmark2D landmarks_2d;
    Landmark3D landmarks_3d;
    Sophus::SE3d t_c_w;

    typedef std::shared_ptr<Frame> Ptr;

    inline static Ptr MakePtr() {
      return std::make_shared<Frame>();
    }
  };

}

#endif
