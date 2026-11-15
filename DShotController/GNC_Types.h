#pragma once

struct Vec3D {
  float x, y, z;

  Vec3D operator+(const Vec3D& other) const {
    // Retunrs the addition of two vectors
    return Vec3D(x + other.x, y + other.y, z+other.z);
  }

  Vec3D operator-(const Vec3D& other) const {
    // Returns the subtraction of two vectors
    return Vec3D(x - other.x, y - other.y, z - other.z);
  }


  /**
  *@brief Calculate the cross product of two 3x1 Vectors
  *
  *@param other Vec3D& of the second vector to cross
  *@return Vec3D of the resultant X, Y, Z components
  */
  Vec3D cross(const Vec3D& other) const {
    // Returns a cross product of two vectors as another vector
    return Vec3D((y * other.z) - (z * other.y),
                 (z * other.x) - (x * other.z),
                 (x * other.y) - (y * other.x));
  }
};


/*
struct sensorData {
  // Define the struct fields
  Vec3D last_omega;
  Vec3D alpha;

  void inline loadData(float omegaX, float omegaY, float omegaZ, uint32_t t0) {

    // Compute the numerical derivative with respect to time
    uint32_t time_diff = micros() - t0;
    alpha.x = (omegaX - last_omega.x) / time_diff;
    alpha.y = (omegaY - last_omega.y) / time_diff;
    alpha.z = (omegaZ - last_omega.z) / time_diff;

    // Set the ccurrent omegas to last_omega;
    last_omega.x = omegaX;
    last_omega.y = omegaY;
    last_omega.z = omegaZ;
  }
};
*/
