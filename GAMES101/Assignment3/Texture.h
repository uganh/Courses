#pragma once

#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>

#include <string>

class Texture {
  cv::Mat image;

public:
  Texture(const std::string &filename);

  int getWidth(void) const {
    return image.cols;
  }

  int getHeight(void) const {
    return image.rows;
  }

  Eigen::Vector3f getColor(float u, float v) const;
};
