#pragma once

#include <Eigen/Core>

#include <string>
#include <vector>

struct Vertex {
  Eigen::Vector3f p;

  /**
   * Texture coordinates
   */
  float u, v;

  /**
   * Normal vector
   */
  Eigen::Vector3f normal;
};

struct Material {
  
};

struct Mesh {

  bool importObj(const std::string &path);
};
