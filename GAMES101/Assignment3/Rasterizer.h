#pragma once

#include <Eigen/Eigen>

#include <vector>

class Rasterizer {
  int width;
  int height;

  std::vector<Eigen::Vector3f> frameBuffer;
  std::vector<float> depthBuffer;

  Eigen::Matrix4f modelMatrix;
  Eigen::Matrix4f viewMatrix;
  Eigen::Matrix4f projectionMatrix;

public:
  Rasterizer(int width, int height);

  void clearBuffer(void);

  void setRotation(float yaw, float pitch, float roll);

  void setCamera(const Eigen::Vector3f &eye);

  void setPerspectiveProjection(float fov, float aspectRatio, float zNear, float zFar);

  void draw(const std::vector<Triangle> &triangles);

  std::vector<Eigen::Vector3f> &getFrameBuffer(void) {
    return frameBuffer;
  }

  const std::vector<Eigen::Vector3f> &getFrameBuffer(void) const {
    return frameBuffer;
  }

private:
  void rasterizeTriangle(const std::array<Eigen::Vector4f, 3> &vertices);

  void setPixel(int x, int y, const Eigen::Vector3f &color);
};

