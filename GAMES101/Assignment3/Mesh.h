#pragma once

#include <Eigen/Core>

#include <string>
#include <vector>

struct Vertex {
  /**
   * Position
   */
  Eigen::Vector3f p;

  /**
   * Texture coordinate
   */
  Eigen::Vector2f uv;

  /**
   * Normal vector
   */
  Eigen::Vector3f n;
};

class Mesh {
  std::vector<Vertex> vertices;
  std::vector<unsigned int> indices;

public:
  size_t vCount(void) const {
    return vertices.size();
  }

  size_t tCount(void) const {
    return indices.size() / 3;
  }

  bool importObj(const std::string &path);
};
