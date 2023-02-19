#include "Mesh.h"
#include "Rasterizer.h"

#include <cmath>

static bool insideTriangle(float x, float y, const std::array<Eigen::Vector4f, 3> &t) {
  return (t[1].x() - t[0].x()) * (y - t[0].y()) - (t[1].y() - t[0].y()) * (x - t[0].x()) > 0.0f &&
         (t[2].x() - t[1].x()) * (y - t[1].y()) - (t[2].y() - t[1].y()) * (x - t[1].x()) > 0.0f &&
         (t[0].x() - t[2].x()) * (y - t[2].y()) - (t[0].y() - t[2].y()) * (x - t[2].x()) > 0.0f;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const std::array<Eigen::Vector4f, 3> &t) {
  float d = t[0].x() * (t[1].y() - t[2].y()) + t[1].x() * (t[2].y() - t[0].y())  + t[2].x() * (t[0].y() - t[1].y());
  return {
    (x * (t[1].y() - t[2].y()) + t[1].x() * (t[2].y() - y)  + t[2].x() * (y - t[1].y())) / d,
    (t[0].x() * (y - t[2].y()) + x * (t[2].y() - t[0].y())  + t[2].x() * (t[0].y() - y)) / d,
    (t[0].x() * (t[1].y() - y) + t[1].x() * (y - t[0].y())  + x * (t[0].y() - t[1].y())) / d
  };
}

Rasterizer::Rasterizer(int width, int height)
    : width(width)
    , height(height)
    , modelMatrix(Eigen::Matrix4f::Identity())
    , viewMatrix(Eigen::Matrix4f::Identity())
    , projectionMatrix(Eigen::Matrix4f::Identity()) {
  frameBuffer.resize(width * height);
  depthBuffer.resize(width * height);
}

void Rasterizer::clearBuffer(void) {
  std::fill(frameBuffer.begin(), frameBuffer.end(), Eigen::Vector3f{0.0, 0.0, 0.0});
  std::fill(depthBuffer.begin(), depthBuffer.end(), std::numeric_limits<float>::infinity());
}

void Rasterizer::setRotation(float yaw, float pitch, float roll) {
  /* Tait–Bryan angles (y-x-z) */

  Eigen::Matrix4f R_y, R_x, R_z;

  R_y <<  std::cos(yaw), 0.0f, std::sin(yaw), 0.0f,
                   0.0f, 1.0f,          0.0f, 0.0f,
         -std::sin(yaw), 0.0f, std::cos(yaw), 0.0f,
                   0.0f, 0.0f,          0.0f, 1.0f;
  
  R_x <<  1.0f,            0.0f,             0.0f, 0.0f,
          0.0f, std::cos(pitch), -std::sin(pitch), 0.0f,
          0.0f, std::sin(pitch),  std::cos(pitch), 0.0f,
          0.0f,            0.0f,             0.0f, 1.0f;
  
  R_z << std::cos(roll), -std::sin(roll), 0.0f, 0.0f,
         std::sin(roll),  std::cos(roll), 0.0f, 0.0f,
                   0.0f,            0.0f, 1.0f, 0.0f,
                   0.0f,            0.0f, 0.0f, 1.0f;

  modelMatrix = R_z * R_x * R_y;
}

void Rasterizer::setCamera(const Eigen::Vector3f &eye) {
  Eigen::Matrix4f T_view;
  T_view << 1.0f, 0.0f, 0.0f, -eye.x(),
            0.0f, 1.0f, 0.0f, -eye.y(),
            0.0f, 0.0f, 1.0f, -eye.z(),
            0.0f, 0.0f, 0.0f,     1.0f;
  Eigen::Matrix4f R_view = Eigen::Matrix4f::Identity();
  
  viewMatrix = R_view * T_view;
}

void Rasterizer::setPerspectiveProjection(float fov, float aspectRatio, float zNear, float zFar) {
  /* 
   * 2.0 * zNear / (r - l),                   0.0,               (l + r) / (l - r),                                 0.0,
   *                   0.0, 2.0 * zNear / (t - b),               (b + t) / (b - t),                                 0.0,
   *                   0.0,                   0.0, (zFar + zNear) / (zFar - zNear), 2.0 * zNear * zFar / (zFar - zNear),
   *                   0.0,                   0.0,                            -1.0,                                 0.0;
   * 
   * t = zNear * tan(fov / 2.0);
   * r = aspectRatio * t;
   */
  projectionMatrix << 1.0f / (aspectRatio * std::tan(fov / 2.0f)),                        0.0f,                            0.0f,                                 0.0f,
                                                             0.0f, 1.0f / std::tan(fov / 2.0f),                            0.0f,                                 0.0f,
                                                             0.0f,                        0.0f, (zFar + zNear) / (zFar - zNear), 2.0f * zNear * zFar / (zFar - zNear),
                                                             0.0f,                        0.0f,                           -1.0f,                                 0.0f;
}

void Rasterizer::draw(const std::vector<Triangle> &triangles) {
  // Eigen::Matrix4f mv = viewMatrix * modelMatrix;

  Eigen::Matrix4f mvp = projectionMatrix * viewMatrix * modelMatrix;

  for (const auto &t : triangles) {
    std::array<Eigen::Vector4f, 3> vertices = {
      Eigen::Vector4f(t.vertices[0].x(), t.vertices[0].y(), t.vertices[0].z(), 1.0f),
      Eigen::Vector4f(t.vertices[1].x(), t.vertices[1].y(), t.vertices[1].z(), 1.0f),
      Eigen::Vector4f(t.vertices[2].x(), t.vertices[2].y(), t.vertices[2].z(), 1.0f),
    };

    for (auto &v : vertices) {
      v = mvp * v;

      // Homogeneous division
      v.x() /= v.w();
      v.y() /= v.w();
      v.z() /= v.w();

      // Viewport transformation
      v.x() = 0.5f * width  * (v.x() + 1.0f);
      v.y() = 0.5f * height * (v.y() + 1.0f);
    }

    // @todo Transform normals

    rasterizeTriangle(vertices);
  }
}

void Rasterizer::rasterizeTriangle(const std::array<Eigen::Vector4f, 3> &vertices) {
  float x_min_f = std::min(std::min(vertices[0].x(), vertices[1].x()), vertices[2].x());
  float x_max_f = std::max(std::max(vertices[0].x(), vertices[1].x()), vertices[2].x());
  float y_min_f = std::min(std::min(vertices[0].y(), vertices[1].y()), vertices[2].y());
  float y_max_f = std::max(std::max(vertices[0].y(), vertices[1].y()), vertices[2].y());

  int x_min = std::max(static_cast<int>(x_min_f), 0);
  int x_max = std::min(static_cast<int>(x_max_f + 1.0), width);
  int y_min = std::max(static_cast<int>(y_min_f), 0);
  int y_max = std::min(static_cast<int>(y_max_f + 1.0), height);

  for (int x = x_min; x < x_max; ++x) {
    for (int y = y_min; y < y_max; ++y) {
      float px = x + 0.5;
      float py = y + 0.5;

      if (insideTriangle(px, py, vertices)) {
        auto [alpha, beta, gamma] = computeBarycentric2D(px, py, vertices);

        /*
         * vertices[i].w() is the vertex view space depth value z.
         * interpolated_z is interpolated view space depth for the current pixel
         */
        float interpolated_z = 1.0f / (alpha / vertices[0].w() + beta / vertices[1].w() + gamma / vertices[2].w());

        int index = (height - y - 1) * width + x;
        if (interpolated_z < depthBuffer[index]) {
          setPixel(x, y, { 148.0f, 121.0f, 92.0f }); // @todo

          // @todo Interpolate the attributes
          // interpolated_color
          // interpolated_normal
          // interpolated_texcoords
          // interpolated_shadingcoords

          depthBuffer[index] = interpolated_z;
        }
      }
    }
  }
}

void Rasterizer::setPixel(int x, int y, const Eigen::Vector3f &color) {
  if (0 < x && x < width && 0 < y && y < height) {
    frameBuffer[(height - y - 1) * width + x] = color;
  }
}
