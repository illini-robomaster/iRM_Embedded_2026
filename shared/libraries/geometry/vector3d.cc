#include "vector3d.h"
// implement Vector3d class


Vector3d::~Vector3d()
{
}

Vector3d::Vector3d()
{
     _x = 0;
     _y = 0;
     _z = 0;
}

Vector3d::Vector3d( float x,  float y,  float z)
{
     _x = x;
     _y = y;
     _z = z;
}

Vector3d::Vector3d(const Vector3d& v)
{
    _x = v._x;
    _y = v._y;
    _z = v._z;
}

Vector3d Vector3d::operator+(const Vector3d& v) const
{
    return Vector3d(_x + v._x, _y + v._y, _z + v._z);
}

Vector3d Vector3d::operator-(const Vector3d& v) const
{
    return Vector3d(_x - v._x, _y - v._y, _z - v._z);
}

Vector3d Vector3d::operator*( float scalar) const
{
    return Vector3d(_x * scalar, _y * scalar, _z * scalar);
}

Vector3d Vector3d::operator/( float scalar) const
{
    return Vector3d(_x / scalar, _y / scalar, _z / scalar);
}

Vector3d& Vector3d::operator=(const Vector3d& v)
{
    _x = v._x;
    _y = v._y;
    _z = v._z;
    return *this;
}

Vector3d& Vector3d::operator+=(const Vector3d& v)
{
    _x += v._x;
    _y += v._y;
    _z += v._z;
    return *this;
}

Vector3d& Vector3d::operator-=(const Vector3d& v)
{
    _x -= v._x;
    _y -= v._y;
    _z -= v._z;
    return *this;
}

Vector3d& Vector3d::operator*=( float scalar)
{
    _x *= scalar;
    _y *= scalar;
    _z *= scalar;
    return *this;
}

Vector3d& Vector3d::operator/=( float scalar)
{
    _x /= scalar;
    _y /= scalar;
    _z /= scalar;
    return *this;
}

bool Vector3d::operator==(const Vector3d& v) const
{
    return _x == v._x && _y == v._y && _z == v._z;
}

bool Vector3d::operator!=(const Vector3d& v) const
{
    return !(*this == v);
}

float Vector3d::dot(const Vector3d& v) const
{
    return _x * v._x + _y * v._y + _z * v._z;
}

Angle2d Vector3d::angleBetween(const Vector3d& v) const
{
    float dot = this->dot(v);
    float len1 = this->length();
    float len2 = v.length();
    float cosTheta = dot / (len1 * len2);
    return Angle2d(acos(cosTheta));
}

Vector3d Vector3d::rotateBy(const Rotation3d& r) const
{
    // rotate this vector by the Rotation represented by r
    // r is a quaternion

    Quaternion q = {r.getW(), r.getX(), r.getY(), r.getZ()}; // 将Rotation3d转换为四元数
    Quaternion qConjugate = {r.getW(), -r.getX(), -r.getY(), -r.getZ()}; // 计算四元数的共轭
    Quaternion qv = {0, _x, _y, _z}; // 将向量扩展为四元数

    // 计算四元数乘法
    Quaternion result = {
        q.w * qv.w - q.x * qv.x - q.y * qv.y - q.z * qv.z,
        q.w * qv.x + q.x * qv.w + q.y * qv.z - q.z * qv.y,
        q.w * qv.y - q.x * qv.z + q.y * qv.w + q.z * qv.x,
        q.w * qv.z + q.x * qv.y - q.y * qv.x + q.z * qv.w
    };

    // 再次进行四元数乘法，这次是与共轭
    Vector3d rotatedVec = {
        result.x * qConjugate.w + result.w * qConjugate.x + result.y * qConjugate.z - result.z * qConjugate.y,
        result.y * qConjugate.w - result.x * qConjugate.z + result.w * qConjugate.y + result.z * qConjugate.x,
        result.z * qConjugate.w + result.x * qConjugate.y - result.y * qConjugate.x + result.w * qConjugate.z
    };

    return rotatedVec;
}


Rotation3d Vector3d::getRotation3d(const Vector3d& v) const
{
    // rotation axis
    Vector3d n = this->cross(v).normalized(); 

    // special case when the two vectors are parallel
    
    if (this->dot(v) > 0.99999)
    {
        // if same direction, return identity rotation3d
        return Rotation3d(0, 0, 0);
    }
    else if(this->dot(v) < -0.99999)
    {
        // if opposite direction, return 180 degree rotation around any axis perpendicular to the two vectors
        // choose x axis
        return Rotation3d(0, 0, M_PI);
    }
    

    // use vector angle algorithm to find quaternion

    float cos_theta = _x * v._x + _y * v._y + _z * v._z;
    // float half_theta = acos(cos_theta)/2.0;
    // float sin_half_theta = sin(half_theta);
    // float cos_half_theta = cos(half_theta);
    float sin_half_theta = sqrt((1.0f - cos_theta) / 2.0f);
    float cos_half_theta = sqrt((1.0f + cos_theta) / 2.0f);


    Quaternion q;
    q.w = cos_half_theta;
    q.x = sin_half_theta * n._x;
    q.y = sin_half_theta * n._y;
    q.z = sin_half_theta * n._z;

    return Rotation3d(q);
}

std::string Vector3d::toString() const {
  return "Vector3d(" + std::to_string(_x) + ", " + std::to_string(_y) + ", " + std::to_string(_z) + ")";
}

Vector3d Vector3d::cross(const Vector3d& v) const
{
    return Vector3d(
        _y * v._z - _z * v._y,
        _z * v._x - _x * v._z,
        _x * v._y - _y * v._x
    );
}

float Vector3d::length() const
{
    return sqrt(_x * _x + _y * _y + _z * _z);
}

float Vector3d::lengthSquared() const
{
    return _x * _x + _y * _y + _z * _z;
}

Vector3d Vector3d::normalized() const
{
    float len = length();
    if (len < 1e-9)
    {
        return Vector3d(0, 0, 0);
    }
    return *this / len;
}

void Vector3d::normalize()
{
    float len = length();
    if (len < 1e-9)
    {
        _x = 0;
        _y = 0;
        _z = 0;
    }
    else
    {
        _x /= len;
        _y /= len;
        _z /= len;
    }
}

