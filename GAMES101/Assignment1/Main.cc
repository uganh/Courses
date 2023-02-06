#define _USE_MATH_DEFINES

#include "Rasterizer.h"

#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>

#include <cmath>

int main(int argc, const char* argv[]) {
  Rasterizer r(700, 700);

  /* Configurations */

  float angle = 0.0f / 180.0f * M_PI;

  Eigen::Vector3f e = {0.0, 0.0, 5.0}; // Eye position

  float fov = 45.0f / 180.0f * M_PI;
  float aspectRatio = 1.0f;
  float zNear = 0.1f;
  float zFar = 50.0f;

  Eigen::Vector3f color = {255.0, 255.0, 255.0};

  Eigen::Vector4f a = { 2.0,  0.0, -2.0, 1.0};
  Eigen::Vector4f b = { 0.0,  2.0, -2.0, 1.0};
  Eigen::Vector4f c = {-2.0,  0.0, -2.0, 1.0};

  int frameCount = 0;
  int key = 0;

  while (key != 27) {
    std::cout << "Frame #" << frameCount++ << std::endl;

    r.clearColorBuffer();
    r.clearDepthBuffer();

    Eigen::Matrix4f model;
    model << std::cos(angle), -std::sin(angle), 0.0, 0.0,
             std::sin(angle),  std::cos(angle), 0.0, 0.0,
                         0.0,              0.0, 1.0, 0.0,
                         0.0,              0.0, 0.0, 1.0;

    Eigen::Matrix4f viewT;
    viewT << 1.0, 0.0, 0.0, -e.x(),
             0.0, 1.0, 0.0, -e.y(),
             0.0, 0.0, 1.0, -e.z(),
             0.0, 0.0, 0.0, 1.0;
    Eigen::Matrix4f viewR = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f view  = viewR * viewT;

    Eigen::Matrix4f projection;
    // projection << 2.0 * zNear / (r - l), 0.0, (l + r) / (l - r), 0.0,
    //               0.0, 2.0 * zNear / (t - b), (b + t) / (b - t), 0.0,
    //               0.0, 0.0, (zFar + zNear) / (zFar - zNear), 2.0 * zNear * zFar / (zNear - zFar),
    //               0.0, 0.0, 1.0, 0.0;
    // t = zNear * tan(fov / 2.0);
    // r = aspectRatio * t;
    projection << 1.0 / (aspectRatio * std::tan(fov / 2.0)), 0.0, 0.0, 0.0,
                  0.0, 1.0 / std::tan(fov / 2.0), 0.0, 0.0,
                  0.0, 0.0, (zFar + zNear) / (zFar - zNear), 2.0 * zNear * zFar / (zNear - zFar),
                  0.0, 0.0, 1.0, 0.0;

    Eigen::Matrix4f mvp = projection * view * model;

    Eigen::Vector4f Ma = mvp * a;
    Eigen::Vector4f Mb = mvp * b;
    Eigen::Vector4f Mc = mvp * c;

    r.drawTriangle(Ma.head<3>() / Ma.w(), Mb.head<3>() / Mb.w(), Mc.head<3>() / Mc.w(), color);

    cv::Mat image(700, 700, CV_32FC3, r.getFrameBuffer().data());
    image.convertTo(image, CV_8UC3, 1.0f);
    cv::imshow("image", image);

    key = cv::waitKey(10);

    switch (key) {
      case 'a':
        angle += 5.0f / 180.0f * M_PI;
        break;
      case 'd':
        angle -= 5.0f / 180.0f * M_PI;
        break;
    }
  }

  return 0;
}

