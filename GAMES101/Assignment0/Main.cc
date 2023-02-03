#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"

#include <cmath>
#include <iostream>

int main(void) {
  // Basic example of cpp
  std::cout << "Example of cpp" << std::endl;
  float a = 1.0, b = 2.0;
  std::cout << a << std::endl;
  std::cout << a / b << std::endl;
  std::cout << std::sqrt(b) << std::endl;
  std::cout << std::acos(-1.0) << std::endl;
  std::cout << std::sin(30.0 / 180.0 * std::acos(-1.0)) << std::endl;

  // Example of vector
  std::cout << "Example of vector" << std::endl;
  // Vector definition
  Eigen::Vector3f v(1.0f, 2.0f, 3.0f);
  Eigen::Vector3f w(1.0f, 0.0f, 0.0f);
  // Vector output
  std::cout << "Vector output" << std::endl;
  std::cout << v << std::endl;
  // Vector addition
  std::cout << "Vector addition" << std::endl;
  std::cout << v + w << std::endl;
  // Vector scalar multiplication
  std::cout << "Vector scalar multiplication" << std::endl;
  std::cout << v * 3.0f << std::endl;
  std::cout << 2.0f * v << std::endl;

  // Example of matrix
  std::cout << "Example of matrix" << std::endl;
  // Matrix definition
  Eigen::Matrix3f i, j;
  i << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;
  j << 2.0, 3.0, 1.0, 4.0, 6.0, 5.0, 9.0, 7.0, 8.0;
  // Matrix output
  std::cout << "Matrix output" << std::endl;
  std::cout << i << std::endl;
  // Matrix addition
  std::cout << "Matrix addition" << std::endl;
  std::cout << i + j << std::endl;
  // Matrix scalar multiplication
  std::cout << "Matrix scalar multiplication" << std::endl;
  std::cout << i * 2.0f << std::endl;
  // Matrix multiplication
  std::cout << "Matrix multiplication" << std::endl;
  std::cout << i * j << std::endl;
  // Matrix vector multiplication
  std::cout << "Matrix vector multiplication" << std::endl;
  std::cout << i * v << std::endl;

  /**
   * PA 0
   */
  Eigen::Vector3f P(2.0f, 1.0f, 1.0f);

  Eigen::Matrix3f R, T;
  float alpha = 45.0f / 180.0f * std::acos(-1.0);
  R << std::cos(alpha), -std::sin(alpha), 0.0, std::sin(alpha), std::cos(alpha), 0.0, 0.0, 0.0, 1.0;
  T << 1.0, 0.0, 1.0, 0.0, 1.0, 2.0, 0.0, 0.0, 1.0;

  Eigen::Vector3f Q = T * R * P;

  std::cout << "After transformation" << std::endl;
  std::cout << Q << std::endl;

  return 0;
}

