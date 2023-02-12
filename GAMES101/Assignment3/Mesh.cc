#include "Mesh.h"

#include <fstream>

bool Mesh::importObj(const std::string &path) {
  std::ifstream file;
  file.open(path, std::ios::in);

  if (!file.is_open())
    return false;

  std::string line;

  while (std::getline(file, line)) {
    size_t off = 0, len = line.size();

    // Skip whitespaces
    while (off < std::isspace(isline[off])) {
      off++;
    }

    
  }
  
  return false;
}

