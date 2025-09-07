
#include "angle2d.h"

Angle2d::~Angle2d()
{
}

Angle2d::Angle2d(float radians)
{
     _radians = radians;
     _cos_value = cos(radians);
     _sin_value = sin(radians);

}

Angle2d::Angle2d()
{
     _radians = 0;
     _cos_value = 1;
     _sin_value = 0;
}

Angle2d Angle2d::createFromDegrees( float degrees)
{
    return Angle2d(degrees/180.0*M_PI);
}

Angle2d Angle2d::createFromRotations( float rotations)
{
    return Angle2d(rotations*2*M_PI);
}

Angle2d::Angle2d( float x,  float y)
{
     float magnitude = sqrt(x*x+y*y);

    //to prevent chaos near 0
    if(magnitude > 1e-6){
         _sin_value = y / magnitude;
         _cos_value = x / magnitude;
    }else{
         _sin_value = 0;
         _cos_value = 1;
    }
     _radians = atan2(_sin_value, _cos_value);
}

Angle2d Angle2d::plus(Angle2d other)
{
    return Angle2d( _radians + other.getRadians());
}

Angle2d Angle2d::unaryMinus()
{
    return Angle2d( _radians*-1);
}

Angle2d Angle2d::minus(Angle2d other)
{
    return  plus(other.unaryMinus());
}

Angle2d Angle2d::oppositeAngle()
{
    return Angle2d( _radians+M_PI); //current radian + PI
}

Angle2d Angle2d::times( float scalar)
{
    return Angle2d(scalar *  _radians);
}

Angle2d Angle2d::getAngleIn1Rotation()
{
    return Angle2d(getRadians0to2PI());
}

Angle2d Angle2d::getAngleIn1RotationSymmetricAbout0()
{
    return Angle2d(getRadiansNegPItoPI());
}

 float Angle2d::getRadians0to2PI()
{
     float temp_angle = atan2(_sin_value, _cos_value); // [-PI, PI)
    return temp_angle < 0?temp_angle+2*M_PI:temp_angle;
}

 float Angle2d::getRadiansNegPItoPI()
{
    return atan2(_sin_value, _cos_value);
}

 float Angle2d::getDegrees0to360()
{
    return  getRadians0to2PI()/M_PI*180;
}

 float Angle2d::getDegreesN180to180()
{
    return getRadiansNegPItoPI()/M_PI*180;
}

 float Angle2d::getRadians()
{
    return  _radians;
}

 float Angle2d::getDegrees()
{
    return  _radians / M_PI * 180;
}

 float Angle2d::getRotations()
{
    return  _radians / 2 / M_PI;
}

 float Angle2d::getCos()
{
    return  _cos_value;
}

 float Angle2d::getSin()
{
    return  _sin_value;
}

std::string Angle2d::toString()
{
    return std::string("Angle2d with " + std::to_string(_radians) + " radians");
}

bool Angle2d::equalsTo(Angle2d other)
{
    return hypot(other._cos_value -  _cos_value, other._sin_value -  _sin_value) < 1e-9;
}