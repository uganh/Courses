#define _USE_MATH_DEFINES

#include "Mesh.h"
#include "Rasterizer.h"

#include <opencv2/opencv.hpp>

#include <cmath>
#include <iostream>

int main(int argc, const char* argv[]) {
  Mesh mesh;

  if (mesh.importObj("../models/spot/spot_triangulated_good.obj")) {
    std::cout << "Import model ok:" << std::endl;
    std::cout << " - " << mesh.vertexCount() << " vertices"  << std::endl;
    std::cout << " - " << mesh.triangleCount() << " triangles" << std::endl;

    Rasterizer r(700, 700);

    r.clearBuffer();
    
    r.setRotation(0.0f, 0.0f, 140.0f / 180.0f * M_PI);
    r.setCamera({0.0f, 0.0f, 10.0f});
    r.setPerspectiveProjection(45.0f / 180.0f * M_PI, 1.0f, 0.1f, 50.0f);

    r.draw(mesh.toTriangleList());

    cv::Mat image(700, 700, CV_32FC3, r.getFrameBuffer().data());
    image.convertTo(image, CV_8UC3, 1.0f);
    cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

    cv::imwrite("output.png", image);
  } else {
    std::cout << "Import model failed" << std::endl;
  }

  return 0;
}
