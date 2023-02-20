#include "Mesh.h"
#include "Rasterizer.h"
#include "Shader.h"

#include <cmath>

static bool insideTriangle(float x, float y, const Eigen::Vector3f *t) {
  return (t[1].x() - t[0].x()) * (y - t[0].y()) - (t[1].y() - t[0].y()) * (x - t[0].x()) > 0.0f &&
         (t[2].x() - t[1].x()) * (y - t[1].y()) - (t[2].y() - t[1].y()) * (x - t[1].x()) > 0.0f &&
         (t[0].x() - t[2].x()) * (y - t[2].y()) - (t[0].y() - t[2].y()) * (x - t[2].x()) > 0.0f;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Eigen::Vector3f *t) {
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

void Rasterizer::setPerspectiveProjection(float fov, float aspect_ratio, float zNear, float zFar) {
  /* 
   * 2.0 * zNear / (r - l),                   0.0,               (l + r) / (l - r),                                 0.0,
   *                   0.0, 2.0 * zNear / (t - b),               (b + t) / (b - t),                                 0.0,
   *                   0.0,                   0.0, (zFar + zNear) / (zFar - zNear), 2.0 * zNear * zFar / (zFar - zNear),
   *                   0.0,                   0.0,                            -1.0,                                 0.0;
   * 
   * t = zNear * tan(fov / 2.0);
   * r = aspect_ratio * t;
   */
  projectionMatrix << 1.0f / (aspect_ratio * std::tan(fov / 2.0f)),                        0.0f,                            0.0f,                                 0.0f,
                                                             0.0f, 1.0f / std::tan(fov / 2.0f),                            0.0f,                                 0.0f,
                                                             0.0f,                        0.0f, (zFar + zNear) / (zFar - zNear), 2.0f * zNear * zFar / (zFar - zNear),
                                                             0.0f,                        0.0f,                           -1.0f,                                 0.0f;
}

void Rasterizer::draw(const std::vector<Triangle> &triangles, const Texture *texture) {
  Eigen::Matrix4f mv = viewMatrix * modelMatrix;
  Eigen::Matrix4f mvp = projectionMatrix * mv;

  for (const auto &triangle : triangles) {
    std::array<Eigen::Vector4f, 3> vertices = {
      Eigen::Vector4f(triangle.vertices[0].x(), triangle.vertices[0].y(), triangle.vertices[0].z(), 1.0f),
      Eigen::Vector4f(triangle.vertices[1].x(), triangle.vertices[1].y(), triangle.vertices[1].z(), 1.0f),
      Eigen::Vector4f(triangle.vertices[2].x(), triangle.vertices[2].y(), triangle.vertices[2].z(), 1.0f),
    };

    std::array<Eigen::Vector4f, 3> normals = {
      Eigen::Vector4f(triangle.normals[0].x(), triangle.normals[0].y(), triangle.normals[0].z(), 0.0f),
      Eigen::Vector4f(triangle.normals[1].x(), triangle.normals[1].y(), triangle.normals[1].z(), 0.0f),
      Eigen::Vector4f(triangle.normals[2].x(), triangle.normals[2].y(), triangle.normals[2].z(), 0.0f),
    };

    Triangle t;
    std::array<Eigen::Vector3f, 3> viewspacePos;

    for (int i = 0; i < 3; ++i) {
      viewspacePos[i] = (mv * vertices[i]).head<3>();

      vertices[i] = mvp * vertices[i];

      // Homogeneous division
      vertices[i].x() /= vertices[i].w();
      vertices[i].y() /= vertices[i].w();

      // Viewport transformation
      t.vertices[i] = {
        0.5f * width  * (vertices[i].x() + 1.0f),
        0.5f * height * (vertices[i].y() + 1.0f),
        vertices[i].w(),
      };

      t.texcoords[i] = triangle.texcoords[i];

      t.normals[i] = (mv * normals[i]).head<3>();
    }

    rasterizeTriangle(t, viewspacePos, texture);
  }
}

void Rasterizer::rasterizeTriangle(const Triangle &t, const std::array<Eigen::Vector3f, 3> &viewspacePos, const Texture *texture) {
  float x_min_f = std::min(std::min(t.vertices[0].x(), t.vertices[1].x()), t.vertices[2].x());
  float x_max_f = std::max(std::max(t.vertices[0].x(), t.vertices[1].x()), t.vertices[2].x());
  float y_min_f = std::min(std::min(t.vertices[0].y(), t.vertices[1].y()), t.vertices[2].y());
  float y_max_f = std::max(std::max(t.vertices[0].y(), t.vertices[1].y()), t.vertices[2].y());

  int x_min = std::max(static_cast<int>(x_min_f), 0);
  int x_max = std::min(static_cast<int>(x_max_f + 1.0), width);
  int y_min = std::max(static_cast<int>(y_min_f), 0);
  int y_max = std::min(static_cast<int>(y_max_f + 1.0), height);

  for (int x = x_min; x < x_max; ++x) {
    for (int y = y_min; y < y_max; ++y) {
      float px = x + 0.5;
      float py = y + 0.5;

      if (insideTriangle(px, py, t.vertices)) {
        auto [alpha, beta, gamma] = computeBarycentric2D(px, py, t.vertices);

        /*
         * t.vertices[i].z() is the vertex view space depth value z.
         * interpolated_z is interpolated view space depth for the current pixel
         */
        float interpolated_z = 1.0f / (alpha / t.vertices[0].z() + beta / t.vertices[1].z() + gamma / t.vertices[2].z());

        int index = (height - y - 1) * width + x;
        if (interpolated_z < depthBuffer[index]) {
          /*
           * Interpolate the attributes: interpolated_? = (alpha * ?[0] / vertices[0].w() + beta * ?[1] / vertices[1].w() + gamma * ?[2] / vertices[2].w()) * interpolated_z;
           */
          Eigen::Vector3f interpolated_shadingpoint = (alpha * viewspacePos[0] / t.vertices[0].z() + beta * viewspacePos[1] / t.vertices[1].z() + gamma * viewspacePos[2] / t.vertices[2].z()) * interpolated_z;
          Eigen::Vector3f interpolated_normal = (alpha * t.normals[0] / t.vertices[0].z() + beta * t.normals[1] / t.vertices[1].z() + gamma * t.normals[2] / t.vertices[2].z()) * interpolated_z;
          Eigen::Vector2f interpolated_texcoords = (alpha * t.texcoords[0] / t.vertices[0].z() + beta * t.texcoords[1] / t.vertices[1].z() + gamma * t.texcoords[2] / t.vertices[2].z()) * interpolated_z;

          fragment_shader_payload payload;
          payload.shadingpoint = interpolated_shadingpoint;
          payload.normal = interpolated_normal;
          payload.color = { 0.5804f, 0.4745f, 0.3608f };
          payload.texcoords = interpolated_texcoords;
          payload.texture = texture;

          setPixel(x, y, fragmentShader(payload));

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
