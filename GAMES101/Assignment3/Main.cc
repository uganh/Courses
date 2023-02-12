#include "Mesh.h"

#include <iostream>

int main(int argc, const char* argv[]) {
  Mesh mesh;

  if (mesh.importObj("../models/spot/spot_triangulated_good.obj")) {
    std::cout << "Import model ok:" << std::endl;
    std::cout << " - " << mesh.vCount() << " vertices"  << std::endl;
    std::cout << " - " << mesh.tCount() << " triangles" << std::endl;
  } else {
    std::cout << "Import model failed" << std::endl;
  }

  return 0;
}
