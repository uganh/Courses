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

  void clearColorBuffer(void);
  void clearDepthBuffer(void);

  void setRotation(float yaw, float pitch, float roll);

  void setCamera(const Eigen::Vector3f &eye);

  void setPerspectiveProjection(float fov, float aspectRatio, float zNear, float zFar);

  void drawTriangle(const std::vector<Eigen::Vector3f> &vertices,
                    const std::vector<Eigen::Vector3f> &colors,
                    const std::vector<Eigen::Vector3i> &indices);

  std::vector<Eigen::Vector3f> &getFrameBuffer(void) {
    return frameBuffer;
  }

private:
  std::vector<Eigen::Vector3f> processVertices(const std::vector<Eigen::Vector3f> &vertices) const;

  void rasterizeTriangle(const std::array<Eigen::Vector3f, 3> &vertices, const std::array<Eigen::Vector3f, 3> &colors);

  void drawLine(int x0, int y0, int x1, int y1, const Eigen::Vector3f &color);

  void setPixel(int x, int y, const Eigen::Vector3f &color);
};

