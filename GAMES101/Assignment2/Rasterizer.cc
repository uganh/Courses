#include "Rasterizer.h"

#include <cmath>

static bool isInsideTriangle(float x, float y, const std::array<Eigen::Vector3f, 3> &t) {
  return (t[1].x() - t[0].x()) * (y - t[0].y()) - (t[1].y() - t[0].y()) * (x - t[0].x()) > 0.0f &&
         (t[2].x() - t[1].x()) * (y - t[1].y()) - (t[2].y() - t[1].y()) * (x - t[1].x()) > 0.0f &&
         (t[0].x() - t[2].x()) * (y - t[2].y()) - (t[0].y() - t[2].y()) * (x - t[2].x()) > 0.0f;
}

static Eigen::Vector3f computeBarycentric2D(float x, float y, const std::array<Eigen::Vector3f, 3> &t) {
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

void Rasterizer::clearColorBuffer(void) {
  std::fill(frameBuffer.begin(), frameBuffer.end(), Eigen::Vector3f{0.0, 0.0, 0.0});
}

void Rasterizer::clearDepthBuffer(void) {
  std::fill(depthBuffer.begin(), depthBuffer.end(), -std::numeric_limits<float>::infinity());
}

void Rasterizer::setRotation(float yaw, float pitch, float roll) {
  /**
   * Tait–Bryan angles (y-x-z)
   */

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

void Rasterizer::drawTriangle(const std::vector<Eigen::Vector3f> &vertices,
                              const std::vector<Eigen::Vector3f> &colors,
                              const std::vector<Eigen::Vector3i> &indices) {
  std::vector<Eigen::Vector3f> processed_vertices = processVertices(vertices);

  for (const Eigen::Vector3i &triangleIndices : indices) {
    std::array<Eigen::Vector3f, 3> triangleVertices = {
      processed_vertices[triangleIndices[0]],
      processed_vertices[triangleIndices[1]],
      processed_vertices[triangleIndices[2]]
    };

    std::array<Eigen::Vector3f, 3> triangleColors = {
      colors[triangleIndices[0]],
      colors[triangleIndices[1]],
      colors[triangleIndices[2]]
    };

    rasterizeTriangle(triangleVertices, triangleColors);
  }
}

std::vector<Eigen::Vector3f> Rasterizer::processVertices(const std::vector<Eigen::Vector3f> &vertices) const {
  Eigen::Matrix4f mvp = projectionMatrix * viewMatrix * modelMatrix;

  std::vector<Eigen::Vector3f> outVertices;

  for (const Eigen::Vector3f &origianl_v : vertices) {
    Eigen::Vector4f v = { origianl_v.x(), origianl_v.y(), origianl_v.z(), 1.0f };
    v = mvp * v;

    // Homogeneous division
    v /= v.w();

    // Viewport transformation
    outVertices.push_back({ 0.5f * width * (v.x() + 1.0f), 0.5f * height * (v.y() + 1.0f), v.z() });
  }

  return outVertices;
}

void Rasterizer::rasterizeTriangle(const std::array<Eigen::Vector3f, 3> &triangle, const std::array<Eigen::Vector3f, 3> &colors) {
  float x_min_f = std::min(std::min(triangle[0].x(), triangle[1].x()), triangle[2].x());
  float x_max_f = std::max(std::max(triangle[0].x(), triangle[1].x()), triangle[2].x());
  float y_min_f = std::min(std::min(triangle[0].y(), triangle[1].y()), triangle[2].y());
  float y_max_f = std::max(std::max(triangle[0].y(), triangle[1].y()), triangle[2].y());

  int x_min = std::max(static_cast<int>(x_min_f), 0);
  int x_max = std::min(static_cast<int>(x_max_f + 1.0), width);
  int y_min = std::max(static_cast<int>(y_min_f), 0);
  int y_max = std::min(static_cast<int>(y_max_f + 1.0), height);

  for (int x = x_min; x < x_max; ++x) {
    for (int y = y_min; y < y_max; ++y) {
      float px = x + 0.5f;
      float py = y + 0.5f;
      if (isInsideTriangle(px, py, triangle)) {
        Eigen::Vector3f w = computeBarycentric2D(px, py, triangle);
        float z_interpolated = w[0] * triangle[0].z() + w[1] * triangle[1].z() + w[2] * triangle[1].z();
        int index = (height - y - 1) * width + x;
        if (z_interpolated > depthBuffer[index]) {
          Eigen::Vector3f color = w[0] * colors[0] + w[1] * colors[1] + w[2] * colors[2];
          setPixel(x, y, color);
          depthBuffer[index] = z_interpolated;
        }
      }
    }
  }
}

void Rasterizer::drawLine(int x0, int y0, int x1, int y1, const Eigen::Vector3f &color) {
  int dx = x1 - x0;
  int dy = y1 - y0;

  int abs_dx = std::abs(dx);
  int abs_dy = std::abs(dy);

  int xi, yi, xe, ye, delta;

  if ((dx < 0 && dy < 0) || (dx > 0 && dy > 0)) {
    delta =  1;
  } else {
    delta = -1;
  }

  if (abs_dy <= abs_dx) {
    int pi = 2 * abs_dy - abs_dx;

    if (dx >= 0) {
      xi = x0;
      yi = y0;
      xe = x1;
    } else {
      xi = x1;
      yi = y1;
      xe = x0;
    }

    while (xi <= xe) {
      setPixel(xi, yi, color);

      xi = xi + 1;
      if (pi < 0) {
        pi = pi + 2 * abs_dy;
      } else {
        yi = yi + delta;
        pi = pi + 2 * (abs_dy - abs_dx);
      }
    }
  } else {
    int pi = 2 * abs_dx - abs_dy;

    if (dy >= 0) {
      xi = x0;
      yi = y0;
      ye = y1;
    } else {
      xi = x1;
      yi = y1;
      ye = y0;
    }

    while (yi <= ye) {
      setPixel(xi, yi, color);

      yi = yi + 1;
      if (pi < 0) {
        pi = pi + 2 * abs_dx;
      } else {
        xi = xi + delta;
        pi = pi + 2 * (abs_dx - abs_dy);
      }
    }
  }
}

void Rasterizer::setPixel(int x, int y, const Eigen::Vector3f &color) {
  if (0 < x && x < width && 0 < y && y < height) {
    frameBuffer[(height - y - 1) * width + x] = color;
  }
}
