#define _USE_MATH_DEFINES

#include "Rasterizer.h"

#include <opencv2/opencv.hpp>

#include <cmath>
#include <vector>

int main(int argc, const char* argv[]) {
  Rasterizer r(700, 700);

  r.clearColorBuffer();
  r.clearDepthBuffer();

  r.setRotation(0.0f, 0.0f, 0.0f);
  r.setCamera({0.0f, 0.0f, 5.0f});
  r.setPerspectiveProjection(45.0f / 180.0f * M_PI, 1.0f, 0.1f, 50.0f);

  std::vector<Eigen::Vector3f> vertices = {
    { 2.0f,  0.0f, -2.0f},
    { 0.0f,  2.0f, -2.0f},
    {-2.0f,  0.0f, -2.0f},
    { 3.5f, -1.0f, -5.0f},
    { 2.5f,  1.5f, -5.0f},
    {-1.0f,  0.5f, -5.0f}
  };

  std::vector<Eigen::Vector3f> colors = {
    {217.0f, 238.0f, 185.0f},
    {217.0f, 238.0f, 185.0f},
    {217.0f, 238.0f, 185.0f},
    {185.0f, 217.0f, 238.0f},
    {185.0f, 217.0f, 238.0f},
    {185.0f, 217.0f, 238.0f}
  };

  std::vector<Eigen::Vector3i> indices = {
    {0, 1, 2},
    {3, 4, 5}
  };

  r.drawTriangle(vertices, colors, indices);

  cv::Mat image(700, 700, CV_32FC3, r.getFrameBuffer().data());
  image.convertTo(image, CV_8UC3, 1.0f);
  cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

  cv::imwrite("Output.png", image);

  return 0;
}
