#pragma once

#include "Shader.h"

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

  std::function<fragment_shader_signature> fragmentShader;

public:
  Rasterizer(int width, int height);

  void clearBuffer(void);

  void setRotation(float yaw, float pitch, float roll);

  void setCamera(const Eigen::Vector3f &eye);

  void setPerspectiveProjection(float fov, float aspect_ratio, float zNear, float zFar);

  void setFragmentShader(std::function<fragment_shader_signature> fragShader) {
    fragmentShader = fragShader;
  }

  void draw(const std::vector<Triangle> &triangles, const class Texture *texture = nullptr);

  std::vector<Eigen::Vector3f> &getFrameBuffer(void) {
    return frameBuffer;
  }

  const std::vector<Eigen::Vector3f> &getFrameBuffer(void) const {
    return frameBuffer;
  }

private:
  void rasterizeTriangle(const Triangle& t, const std::array<Eigen::Vector3f, 3>& viewspacePos, const class Texture *texture);

  void setPixel(int x, int y, const Eigen::Vector3f &color);
};

