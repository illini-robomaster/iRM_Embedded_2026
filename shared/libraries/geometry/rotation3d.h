#pragma once
#include "angle2d.h"
#include <string>

struct Quaternion {
    float w;
    float x;
    float y;
    float z;
};

struct AxisAngle {
    float axis_x;
    float axis_y;
    float axis_z;
    Angle2d angle;
};

class Rotation3d {
// use quaternion to represent rotation
private:

    // quaternion representation
    float _x; // i
    float _y; // j 
    float _z; // k
    float _w; // real part
    
public:
    Rotation3d();
    // quaternion
    Rotation3d(Quaternion q);

    // quaternion, but not in struct
    Rotation3d(float x, float y, float z, float w); 
    /// @brief Represents a 3d rotation using euler angles, an object first rotates around x axis by roll, then around y axis by pitch, and finally around z axis by yaw
    /// @param roll 
    /// @param pitch 
    /// @param yaw 
    Rotation3d(float roll, float pitch, float yaw); 

    /// @brief Angle-axis representation
    /// @param axis 
    /// @param  
    Rotation3d(float axis_x, float axis_y, float axis_z, Angle2d angle);
    // TODO: axis-angle and rotation matrix

    Rotation3d operator*(const Rotation3d& r) const;
    Rotation3d operator/(float scalar) const;
    bool operator==(const Rotation3d& r) const;
    bool operator!=(const Rotation3d& r) const;

    float getX() const;
    float getY() const;
    float getZ() const;
    float getW() const;

    float getYaw() const;
    float getPitch() const;
    float getRoll() const;

    Rotation3d normalized() const;

    Rotation3d conjugate() const;

    Rotation3d inverse() const;

    float dot(const Rotation3d& other) const;

    AxisAngle getAxisAngle() const;

    Angle2d angleBetween(const Rotation3d& r) const;

    /** @brief this quaternion minus another quaternion, the result satisfies:
    * `result * this quaternion = other quaternion`
    * @param r 
    * @return this quaternion multiples the inverse of the other quaternion
    */ 
    Rotation3d minus(const Rotation3d& r) const;

    std::string toString() const;
};