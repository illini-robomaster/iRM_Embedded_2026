#pragma once

#include "angle2d.h"
#include "rotation3d.h"

class Vector3d{
public:
    Vector3d();
    Vector3d(float x, float y, float z);
    Vector3d(const Vector3d& v);
    ~Vector3d();

    float _x;
    float _y;
    float _z;

    Vector3d operator+(const Vector3d& v) const;
    Vector3d operator-(const Vector3d& v) const;
    Vector3d operator*(float scalar) const;
    Vector3d operator/(float scalar) const;
    Vector3d& operator=(const Vector3d& v);
    Vector3d& operator+=(const Vector3d& v);
    Vector3d& operator-=(const Vector3d& v);
    Vector3d& operator*=(float scalar);
    Vector3d& operator/=(float scalar);
    bool operator==(const Vector3d& v) const;
    bool operator!=(const Vector3d& v) const;

    float dot(const Vector3d& v) const;
    Vector3d cross(const Vector3d& v) const;
    float length() const;
    float lengthSquared() const;
    Vector3d normalized() const;
    void normalize();

    /// @brief the angle between this vector and the given vector, between 0 and PI
    /// @param v 
    /// @return 
    Angle2d angleBetween(const Vector3d& v) const;

    /// @brief the vector rotated by the given rotation
    /// @param r 
    /// @return 
    Vector3d rotateBy(const Rotation3d& r) const;

    
    /// @brief Get the Rotation3d object that represents the rotation that would rotate this vector to the given vector in parameter
    /// @param v the vector that this vector would be rotated to if applyed the returned rotation
    /// @return 
    Rotation3d getRotation3d(const Vector3d& v) const;

    std::string toString() const;
};