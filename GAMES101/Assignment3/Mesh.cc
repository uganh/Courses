#include "Mesh.h"

#include <fstream>
#include <sstream>

bool Mesh::importObj(const std::string &path) {
  std::ifstream file;
  file.open(path, std::ios::in);

  if (!file.is_open())
    return false;

  std::string line;
  
  std::string t;

  float x, y, z;
  float u, v;

  std::vector<Eigen::Vector3f> positions;
  std::vector<Eigen::Vector2f> texCoords;
  std::vector<Eigen::Vector3f> normals;

  while (std::getline(file, line)) {
    std::istringstream iss(line);

    if (iss >> t) {
      if (t.empty() || t[0] == '#') {
        continue;
      } else if (t == "v") {
        // Vertex position
        iss >> x >> y >> z;
        positions.push_back({ x, y, z });
      } else if (t == "vt") {
        // Vertex texture coordinate
        iss >> u >> v;
        texCoords.push_back({ u, v });
      } else if (t == "vn") {
        // Vertex normal
        iss >> x >> y >> z;
        normals.push_back({ x, y, z });
      } else if (t == "f") {
        // Face
        std::string part;

        Vertex vertex;
        int verticesCount = 0;
        unsigned int lastVerticesCount = vertices.size();

        while (iss >> part) {
          int vi = std::stoi(part);
          if (vi < 0)
            vertex.p = positions[positions.size() - vi];
          else
            vertex.p = positions[vi - 1];

          size_t sep1 = part.find('/', sep1 + 1);

          if (sep1 != std::string::npos) {
            size_t sep2 = part.find('/', sep1 + 1);

            if (sep1 + 1 != sep2) {
              int vti = std::stoi(part.substr(sep1 + 1));
              if (vti < 0)
                vertex.uv = texCoords[texCoords.size() - vti];
              else
                vertex.uv = texCoords[vti - 1];
            } else {
              vertex.uv = { 0.0f, 0.0f };
            }

            if (sep2 != std::string::npos) {
              int vni = std::stoi(part.substr(sep2 + 1));
              if (vni < 0)
                vertex.n = normals[normals.size() - vni];
              else
                vertex.n = normals[vni - 1];
            }
          }

          vertices.push_back(vertex);
          ++verticesCount;
        }

        if (verticesCount >= 3) {
          for (int i = 2; i < verticesCount; ++i) {
            indices.push_back(lastVerticesCount + 0);
            indices.push_back(lastVerticesCount + i - 1);
            indices.push_back(lastVerticesCount + i);
          }
        }
      } else if (t == "usemtl") {
        // Material name
      } else if (t == "mtllib") {
        // Material library
      } else {
        goto failure;
      }
    }
  }

  return true;
  
failure:
  vertices.clear();
  indices.clear();

  return false;
}

