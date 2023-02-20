#define _USE_MATH_DEFINES

#include "Mesh.h"
#include "Rasterizer.h"
#include "Texture.h"

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
  Eigen::Vector3f k_a;
  Eigen::Vector3f k_s;

  /**
   * Ambient light intensity
   */
  Eigen::Vector3f I_a;

  Eigen::Vector3f eye_pos;

  float p;

  std::vector<Light> lights;

public:
  PhongFragmentShader(const Eigen::Vector3f &eye_pos)
    : k_a{ 0.0050f,  0.0050f,  0.0050f}
    , k_s{ 0.7937f,  0.7937f,  0.7937f}
    , I_a{10.0000f, 10.0000f, 10.0000f}
    , eye_pos(eye_pos)
    , p(150.0f) {

  }

  void addLightSource(const Light &light) {
    lights.push_back(light);
  }

  Eigen::Vector3f operator()(const fragment_shader_payload &payload) const {
    Eigen::Vector3f texture_color;
    if (payload.texture) {
      // Get the texture value at the texture coordinates of the current fragment
      texture_color = payload.texture->getColor(payload.texcoords.x(), payload.texcoords.y()) / 255.0f;
    } else {
      texture_color = payload.color;
    }

    Eigen::Vector3f k_d = texture_color;

    // Viewer direction
    Eigen::Vector3f v = (eye_pos - payload.shadingpoint).normalized();
    // Surface normal
    Eigen::Vector3f n = payload.normal.normalized();

    Eigen::Vector3f result_color = { 0.0f, 0.0f, 0.0f };

    for (auto &light : lights) {
      float squared_r = (light.position - payload.shadingpoint).squaredNorm();

      // Light direction
      Eigen::Vector3f l = (light.position - payload.shadingpoint).normalized();
    
      // Halfway vector
      Eigen::Vector3f h = (v + l).normalized();

      Eigen::Vector3f L_d = k_d.cwiseProduct(light.intensity / squared_r) * std::max(0.0f, n.dot(l));
      Eigen::Vector3f L_s = k_s.cwiseProduct(light.intensity / squared_r) * std::pow(std::max(0.0f, n.dot(h)), p);
      Eigen::Vector3f L_a = k_a.cwiseProduct(I_a);

      result_color += L_d + L_s + L_a;
    }

    return result_color * 255.0f;
  }
};

int main(int argc, const char* argv[]) {
  Mesh mesh;

  if (mesh.importObj("../models/spot/spot_triangulated_good.obj")) {
    std::cout << "Import model ok:" << std::endl;
    std::cout << " - " << mesh.vertexCount() << " vertices"  << std::endl;
    std::cout << " - " << mesh.triangleCount() << " triangles" << std::endl;

    Rasterizer r(700, 700);

    Texture texture("../models/spot/spot_texture.png");

    Eigen::Vector3f eye_pos = { 0.0f, 0.0f, 10.0f };

    PhongFragmentShader phong_shader(eye_pos);
    phong_shader.addLightSource({ {  20.0f, 20.0f, 20.0f }, { 500.0f, 500.0f, 500.0f } });
    phong_shader.addLightSource({ { -20.0f, 20.0f, 20.0f }, { 500.0f, 500.0f, 500.0f } });

    r.setFragmentShader(phong_shader);

    // r.setFragmentShader(normal_fragment_shader);

    r.clearBuffer();
    
    r.setRotation(140.0f / 180.0f * M_PI, 0.0f, 0.0f);
    r.setCamera(eye_pos);
    r.setPerspectiveProjection(45.0f / 180.0f * M_PI, 1.0f, 0.1f, 50.0f);

    r.draw(mesh.toTriangleList(), &texture);

    cv::Mat image(700, 700, CV_32FC3, r.getFrameBuffer().data());
    image.convertTo(image, CV_8UC3, 1.0f);
    cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

    cv::imwrite("output.png", image);
  } else {
    std::cout << "Import model failed" << std::endl;
  }

  return 0;
}
