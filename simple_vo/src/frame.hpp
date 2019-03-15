#ifndef _SIMPLE_VO_FRAME_H_
#define _SIMPLE_VO_FRAME_H_

#include <sophus/se3.hpp>
#include "common.hpp"

namespace simple_vo {

  struct Frame {
    double timestamp;
    Landmark2D landmarks_2d;
    Landmark3D landmarks_3d;
    Sophus::SE3d t_c_w;
  };

}

#endif
