/****************************************************************************
 *                                                                          *
 *  Copyright (C) 2023 RoboMaster.                                          *
 *  Illini RoboMaster @ University of Illinois at Urbana-Champaign          *
 *                                                                          *
 *  This program is free software: you can redistribute it and/or modify    *
 *  it under the terms of the GNU General Public License as published by    *
 *  the Free Software Foundation, either version 3 of the License, or       *
 *  (at your option) any later version.                                     *
 *                                                                          *
 *  This program is distributed in the hope that it will be useful,         *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of          *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           *
 *  GNU General Public License for more details.                            *
 *                                                                          *
 *  You should have received a copy of the GNU General Public License       *
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.    *
 *                                                                          *
 ****************************************************************************/

#include <cmath>
#include <string>
#include "vector2d.h"


// ==========================================================================================
// Vector 2d
// =========================================================================================

Vector2d::~Vector2d()
{
}

Vector2d::Vector2d()
{
     _x = 0;
     _y = 0;
}

Vector2d::Vector2d( float x,  float y)
{
     _x = x;
     _y = y;
}

Vector2d::Vector2d( float r, Angle2d theta)
{
     _x = r * theta.getCos();
     _y = r * theta.getSin();
}

 float Vector2d::getX()
{
    return  _x;
}

 float Vector2d::getY()
{
    return  _y;
}

 float Vector2d::getMagnitude()
{
    return hypot(_x,  _y);
}

Angle2d Vector2d::getDirection()
{
    return Angle2d( _x,  _y);
}

Vector2d Vector2d::rotateBy(Angle2d other)
{
    return Vector2d(
        _x * other.getCos() - _y * other.getSin(),
        _x * other.getSin() + _y * other.getCos()
    );
}

Vector2d Vector2d::plus(Vector2d other)
{
    return Vector2d(_x + other.getX(), _y + other.getY());
}

Vector2d Vector2d::minus(Vector2d other)
{
    return plus(other.inverse());
}

Vector2d Vector2d::inverse()
{
    return Vector2d( _x * -1,  _y * -1);
}

 float Vector2d::dot(Vector2d other)
{
    return  _x * other.getX() +  _y * other.getY();
}

Vector2d Vector2d::times( float scalar)
{
    return Vector2d( _x * scalar,  _y * scalar);
}

std::string Vector2d::toString()
{
    return std::string("<" + std::to_string( _x) + "," + std::to_string( _y) + ">");
}

Vector2d Vector2d::normalize()
{
    if(getMagnitude() < 1e-9){
        return Vector2d(0, 0);
    }
    return times(1.0/getMagnitude());
}

bool Vector2d::equalsTo(Vector2d other)
{
    return  minus(other).getMagnitude() < 1e-9;
}