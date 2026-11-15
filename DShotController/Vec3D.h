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
     */
    struct Vec3D {
      float x; /**< X value as a float*/
      float y; /**< Y value as a float*/
      float z; /**< Z value as a float*/

      /**
       * @brief Vec3D constructor method
       * 
       * @param x_value Inital x value
       * @param y_value Inital y value
       * @param z_value Inital z value
       * 
       * @return Vec3D
       */
      inline Vec3D(float x_value = 0.0f,
          float y_value = 0.0f,
          float z_value = 0.0f)
        : x{x_value}, y{y_value}, z{z_value} {}

      
      /**
       * @brief Adds two 3D vectors together using element wise addition
       * 
       * @param other The right hand size of the operand
       * @return A Vec3D of the resultant vector
       */
      inline Vec3D operator+(const Vec3D& other) const {
        return Vec3D(x + other.x, y + other.y, z+other.z);
      }

      /**
       * @brief Subtracts two 3D vectors together using element wise subtraction
       * 
       * @param other The right hand size of the operand
       * @return A Vec3D of the resultant vector 
       */
      inline Vec3D operator-(const Vec3D& other) const {
        return Vec3D(x - other.x, y - other.y, z - other.z);
      }

      /**
       * @brief Multiplies two 3D vectors together using element wise multiplication
       * 
       * @note The operation is element wise, for a dot product, us the dot member function
       * 
       * @param other The right hand size of the operand
       * @return A Vec3D of the resultant vector 
       */
      inline Vec3D operator*(const Vec3D& other) const {
        return Vec3D(x * other.x, y * other.y, z * other.z);
      }

      /**
       * @brief Divides two 3D vectors together using element wise division
       * 
       * @note The operation is element wise
       * 
       * @param other The right hand size of the operand
       * @return A Vec3D of the resultant vector 
       */
      inline Vec3D operator/(const Vec3D& other) const {
        return Vec3D(x / other.x, y / other.y, z / other.z);
      }


      /**
      *@brief Calculate the cross product of two 3D Vectors
      *
      *@param other The right side of the cross operand
      *@return Vec3D of the resultant X, Y, Z components
      */
      inline Vec3D cross(const Vec3D& other) const {
        return Vec3D((y * other.z) - (z * other.y),
                    (z * other.x) - (x * other.z),
                    (x * other.y) - (y * other.x));
      }


      /**
      *@brief Calculate the dot product of two 3D Vectors
      *
      *@param other The right side of the operand
      *@return float of the resultant X, Y, Z components
      */
      inline float dot(const Vec3D& other) const {
        return (x * other.x) + (y * other.y) + (z * other.z);
      }

      /**
       * @brief Take the magnitude of the vector
       * 
       * @return Magnitude as a float
       */
      inline float magnitude() const {
        return sqrtf(x * x + y * y + z * z);
      }
    };
  } // namespace Math
  
} // namespace GNC
