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
  /**
   * Vertex positions
   */
  Eigen::Vector3f vertices[3];

  /**
   * Texture coordinates
   */
  Eigen::Vector2f texcoords[3];

  /**
   * Normal vector for each vertex
   */
  Eigen::Vector3f normals[3];

  std::array<Eigen::Vector4f, 3> toHomogeneousCoordinates(void) const;
};

class Mesh {
  std::vector<Vertex> vertices;
  std::vector<unsigned int> vertexIndices;

public:
  size_t vertexCount(void) const {
    return vertices.size();
  }

  size_t triangleCount(void) const {
    return vertexIndices.size() / 3;
  }

  std::vector<Triangle> toTriangleList(void) const;

  bool importObj(const std::string &path);
};
