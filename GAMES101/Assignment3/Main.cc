#define _USE_MATH_DEFINES

#include "Mesh.h"
#include "Rasterizer.h"

#include <opencv2/opencv.hpp>

#include <cmath>
#include <iostream>
#include <vector>

struct Light {
  Eigen::Vector3f position;
  Eigen::Vector3f intensity;
};

Eigen::Vector3f normal_fragment_shader(const fragment_shader_payload& payload) {
  return (payload.normal.normalized() + Eigen::Vector3f(1.0f, 1.0f, 1.0f)) * 255.0f / 2.0f;
}

class PhongFragmentShader {
  Eigen::Vector3f ka;
  Eigen::Vector3f kd;
  Eigen::Vector3f ks;

  float p;

  Eigen::Vector3f I_a;

  Eigen::Vector3f eyePos;

  std::vector<Light> lights;

public:

  Eigen::Vector3f operator()(const fragment_shader_payload &payload) {
    Eigen::Vector3f color;

    for (auto &light : lights) {
      
    }
  }

};

Eigen::Vector3f texture_fragment_shader(const fragment_shader_payload& payload) {
  

  return Eigen::Vector3f(255.0f, 255.0f, 255.0f);
}

Eigen::Vector3f phong_fragment_shader(const fragment_shader_payload& payload) {
  Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
  // Eigen::Vector3f kd = payload.color;
  Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

  return Eigen::Vector3f(255.0f, 255.0f, 255.0f);
}

int main(int argc, const char* argv[]) {
  Mesh mesh;

  if (mesh.importObj("../models/spot/spot_triangulated_good.obj")) {
    std::cout << "Import model ok:" << std::endl;
    std::cout << " - " << mesh.vertexCount() << " vertices"  << std::endl;
    std::cout << " - " << mesh.triangleCount() << " triangles" << std::endl;

    Rasterizer r(700, 700);

    r.setFragmentShader(normal_fragment_shader);

    r.clearBuffer();
    
    r.setRotation(140.0f / 180.0f * M_PI, 0.0f, 0.0f);
    r.setCamera({ 0.0f, 0.0f, 10.0f });
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
