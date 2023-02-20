#include "Texture.h"

Texture::Texture(const std::string &filename)
  : image(cv::imread(filename)) {
  cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
}

Eigen::Vector3f Texture::getColor(float u, float v) const {
  int u_img = static_cast<int>(u * getWidth());
  int v_img = static_cast<int>((1.0f - v) * getHeight());
  auto color = image.at<cv::Vec3b>(v_img, u_img);
  return Eigen::Vector3f(color[0], color[1], color[2]);
}
