#include "Rasterizer.h"

#include <Eigen/Eigen>

#include <opencv2/opencv.hpp>

int main(int argc, const char* argv[]) {
  Rasterizer r(700, 700);

  /* Configurations */

  float angle = 45.0;

  Eigen::Vector3f e = {0.0, 0.0, 5.0}; // Eye position

  float fov = 45.0f; // Degree
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

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f mvp = projection * view * model;

    r.drawTriangle(
      (mvp * a).head<3>(),
      (mvp * b).head<3>(),
      (mvp * c).head<3>(),
      color);

    // r.drawLine(45, 39, 605, 100, Eigen::Vector3f(255.0, 255.0, 0.0));

    cv::Mat image(700, 700, CV_32FC3, r.getFrameBuffer().data());
    image.convertTo(image, CV_8UC3, 1.0f);
    cv::imshow("image", image);

    key = cv::waitKey(10);

    switch (key) {
      case 'a':
        break;
      case 'd':
        break;
    }

  }

  return 0;
}

