#pragma once

#include <eigen3/Eigen/Eigen>

#include <vector>

class Rasterizer {
  unsigned int width;
  unsigned int height;

  std::vector<Eigen::Vector3f> frameBuffer;
  std::vector<float> depthBuffer;

public:
  Rasterizer(unsigned int width, unsigned int height);

  void clearColorBuffer(void);
  void clearDepthBuffer(void);

  void drawLine(int x0, int y0, int x1, int y1, const Eigen::Vector3f &color);
  void drawTriangle(const Eigen::Vector3f &a, const Eigen::Vector3f &b, const Eigen::Vector3f &c, const Eigen::Vector3f &color);

  void setPixel(int x, int y, const Eigen::Vector3f &color);

  std::vector<Eigen::Vector3f>& getFrameBuffer(void) {
    return frameBuffer;
  }
};

