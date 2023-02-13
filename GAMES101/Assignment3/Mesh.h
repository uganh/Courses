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

struct Triangle {
  Eigen::Vector3f vertices[3];

  /**
   * Color at each vertex
   */
  Eigen::Vector3f colors[3];

  /**
   * Texture coordinates
   */
  Eigen::Vector2f texCoords[3];

  /**
   * Normal vector for each vertex
   */
  Eigen::Vector3f normals[3];

  Eigen::Vector4f a(void) const {
    return { vertices[0], 1.0f };
  }

  Eigen::Vector4f b(void) const {
    return { vertices[1], 1.0f };
  }

  Eigen::Vector4f c(void) const {
    return { vertices[2], 1.0f };
  }
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
