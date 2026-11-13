/**
 * @file Vec3D.h
 * @brief Three-dimensional vector support for C++.
 *
 * @author Charles Waldron
 * @date 2026-11-01
 */
#pragma once
#include <cmath> 

namespace GNC
{
  namespace Math
  {
    /**
     * @brief Represents a three-dimensional vector.
     *
     * Stores x, y, and z components as floating-point values.
     */
    struct Vec3D {
      float x, y, z;

      // Constructor 
      Vec3D(float x_value = 0.0f,
          float y_value = 0.0f,
          float z_value = 0.0f)
        : x{x_value}, y{y_value}, z{z_value} {}

      Vec3D operator+(const Vec3D& other) const {
        return Vec3D(x + other.x, y + other.y, z+other.z);
      }

      Vec3D operator-(const Vec3D& other) const {
        return Vec3D(x - other.x, y - other.y, z - other.z);
      }

      // Returns the element wise multiplication of two vectors
      Vec3D operator*(const Vec3D& other) const {
        return Vec3D(x * other.x, y * other.y, z * other.z);
      }

      // Returns the element wise division of two vectors
      Vec3D operator/(const Vec3D& other) const {
        return Vec3D(x / other.x, y / other.y, z / other.z);
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


      /**
      *@brief Calculate the dot product of two 3x1 Vectors
      *
      *@param other Vec3D& of the second vector to cross
      *@return float of the resultant X, Y, Z components
      */
      float dot(const Vec3D& other) const {
        // Returns the dot product of two vectors as another vector
        return (x * other.x) + (y * other.y) + (z * other.z);
      }

      /**
       * @brief Take the magnitude of the vector
       * 
       * @return float The magnitude
       */
      inline float magnitude() const {
        return sqrtf(x * x + y * y + z * z);
      }
    };
  } // namespace Math
  
} // namespace GNC
