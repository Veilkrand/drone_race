#ifndef _GATE_EST_COMMON_H_
#define _GATE_EST_COMMON_H_

#include <cstddef>
#include <cstring>
#include <map>
#include <Eigen/Core>
#include <string>
#include <opencv2/opencv.hpp>

namespace gate_est {

  typedef std::map<std::size_t, Eigen::Vector3d> Landmark3D;
  typedef std::map<std::size_t, Eigen::Vector2d> Landmark2D;
  typedef std::shared_ptr<Landmark3D> Landmark3DPtr;
  typedef std::shared_ptr<Landmark2D> Landmark2DPtr;

  inline std::size_t GetLandmarkKey(std::size_t gate_idx, std::size_t corner_idx) {
    return 100000 + gate_idx * 100 + corner_idx;
  }

  inline std::size_t GetGateIndex(std::size_t key) {
    return (key - 100000) / 100;
  }

  inline std::size_t GetCornerIndex(std::size_t key) {
    return key % 100;
  }
  
  template <typename T>
  inline void StoreLandmark(std::map<std::size_t, T>& map, std::size_t key, const T& position) {
      map[key] = position;
  }

  template <typename T>
  inline void StoreLandmark(std::map<std::size_t, T>& map, std::size_t gate_idx, std::size_t corner_idx, const T& position) {
      map[GetLandmarkKey(gate_idx, corner_idx)] = position;
  }

  template <typename T>
  inline void StoreLandmark(std::map<std::size_t, T>& map, const char* gate_name, const char* corner_name, const T& position) {
    std::size_t gate_idx = std::atoi(gate_name + 4);
    std::size_t corner_idx = std::atoi(corner_name);
    map[GetLandmarkKey(gate_idx, corner_idx)] = position;    
  }

  template <typename T>
  inline void StoreLandmark(std::map<std::size_t, T>& map, const std::string& gate_name, const std::string& corner_name, const T& position) {
    StoreLandmark(map, gate_name.c_str(), corner_name.c_str(), position);
  }

  template <typename T>  
  inline T& GetLandmark(std::map<std::size_t, T>& map, std::size_t key) {
    return map[key];
  }

  template <typename T>  
  inline T& GetLandmark(std::map<std::size_t, T>& map, std::size_t gate_idx, std::size_t corner_idx) {
    return map[GetLandmarkKey(gate_idx, corner_idx)];
  }

  template <typename T>    
  inline T& GetLandmark(std::map<std::size_t, T>& map, const char* gate_name, const char* corner_name) {
    std::size_t gate_idx = std::atoi(gate_name + 4);
    std::size_t corner_idx = std::atoi(corner_name);
    return map[GetLandmarkKey(gate_idx, corner_idx)];
  }

  template <typename T>
  inline T& GetLandmark(std::map<std::size_t, T>& map, const std::string& gate_name, const std::string& corner_name) {
    return GetLandmark(map, gate_name.c_str(), corner_name.c_str());
  }

  template <typename T>
  inline bool HasLandmark(const std::map<std::size_t, T>& map, std::size_t key) {
    return map.find(key) != map.end();
  }

  template <typename T>
  inline bool HasLandmark(const std::map<std::size_t, T>& map, std::size_t gate_idx, std::size_t corner_idx) {
    return map.find(GetLandmarkKey(gate_idx, corner_idx)) != map.end();
  }

  template <typename T>
  inline bool HasLandmark(const std::map<std::size_t, T>& map,  const char* gate_name, const char* corner_name) {
    std::size_t gate_idx = std::atoi(gate_name + 4);
    std::size_t corner_idx = std::atoi(corner_name);
    return HasLandmark(map, gate_idx, corner_idx);
  }

  template <typename T>
  inline bool HasLandmark(const std::map<std::size_t, T>& map, const std::string& gate_name, const std::string& corner_name) {
    return HasLandmark(map, gate_name.c_str(), corner_name.c_str());
  }

  inline void EigenVector2CvPoint(const Eigen::Vector2d& vec, cv::Point2d& pt) {
    pt.x = vec[0];
    pt.y = vec[1];
  }

  inline void EigenVector2CvPoint(const Eigen::Vector3d& vec, cv::Point3d& pt) {
    pt.x = vec[0];
    pt.y = vec[1];
    pt.z = vec[2];
  }

  inline Eigen::MatrixXd CvMat2EigenMatrix(const cv::Mat& mat) {
    Eigen::MatrixXd ret(mat.rows, mat.cols);
    for (std::size_t r = 0; r < mat.rows; ++r) {
      for (std::size_t c = 0; c < mat.cols; ++c) {
        ret(r, c) = mat.at<double>(r, c);
      }
    }
    return ret;
  }
}

#endif
