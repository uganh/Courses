#include "Rasterizer.h"

#include <cmath>

Rasterizer::Rasterizer(int width, int height)
    : width(width), height(height) {
  frameBuffer.resize(width * height);
  depthBuffer.resize(width * height);
}

void Rasterizer::clearColorBuffer(void) {
  std::fill(frameBuffer.begin(), frameBuffer.end(), Eigen::Vector3f{0.0, 0.0, 0.0});
}

void Rasterizer::clearDepthBuffer(void) {
  std::fill(depthBuffer.begin(), depthBuffer.end(), std::numeric_limits<float>::infinity());
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

void Rasterizer::drawTriangle(const Eigen::Vector3f &a, const Eigen::Vector3f &b, const Eigen::Vector3f &c, const Eigen::Vector3f &color) {
  /* Viewport transformation */
  int ax = static_cast<int>(0.5f * width  * (a.x() + 1.0f));
  int ay = static_cast<int>(0.5f * height * (a.y() + 1.0f));
  int bx = static_cast<int>(0.5f * width  * (b.x() + 1.0f));
  int by = static_cast<int>(0.5f * height * (b.y() + 1.0f));
  int cx = static_cast<int>(0.5f * width  * (c.x() + 1.0f));
  int cy = static_cast<int>(0.5f * height * (c.y() + 1.0f));

  drawLine(ax, ay, bx, by, color);
  drawLine(bx, by, cx, cy, color);
  drawLine(cx, cy, ax, ay, color);
}

void Rasterizer::setPixel(int x, int y, const Eigen::Vector3f &color) {
  if (0 < x && x < width && 0 < y && y < height) {
    frameBuffer[(height - y - 1) * width + x] = color;
  }
}

