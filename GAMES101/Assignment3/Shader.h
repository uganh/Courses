#pragma once

#include <Eigen/Eigen>

struct fragment_shader_payload {
  Eigen::Vector3f shadingpoint;
  Eigen::Vector3f normal;

  Eigen::Vector2f texcoords;

  class Texture* texture;

  fragment_shader_payload(void) : texture(nullptr) {}
};

using fragment_shader_signature = Eigen::Vector3f(const fragment_shader_payload &);
