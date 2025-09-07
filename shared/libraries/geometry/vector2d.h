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

#pragma once
#include <string>
#include <cmath>
#include "angle2d.h"



class Vector2d {
    private:
        float  _x, _y;

    public:

        ~Vector2d();

        Vector2d();

        /**
         * Create an 2d vector with rectangular/Cartesian coordinates.
         * 
         * @param x the x coordinate of the vector.
         * @param y the y coordinate of the vector.
        */
        Vector2d(float  x, float  y);


        /**
         * Create an 2d vector with polar coordinates.
         * 
         * @param r the magnitude of the vector.
         * @param theta the angle/direction of the vector.
        */
        Vector2d(float  r, Angle2d theta);


        /**
         * @return The x coordinate of the vector.
        */
        float  getX();

        /**
         * @return The y coordinate of the vector.
        */
        float  getY();


        /**
         * @return The magnitude (r) of the vector.
        */
        float  getMagnitude();


        /** 
         * @return The angle(theta) of the vector as a Angle2d instance.
        */
        Angle2d getDirection();


        /**
         * @return A new instance of Vector2d which is this current vector rotated by the angle represented by "other".
         * 
         * For example, the vector <1,0> rotated by 90 degrees will be <0,1>;
         * 
         * the vector <1,0> rotated by 45 degrees will be <(sqrt2)/2, (sqrt2)/2>
         * 
        */
        Vector2d rotateBy(Angle2d other);

        /**
         * @returns This vector plus another vector.
         * For example, <2,1> plus <1,0> will be <3,1>.
         * 
        */
        Vector2d plus(Vector2d other);


        /**
         * @returns This vector minus another vector.
         * For example, <2,1> minus <1,0> will be <1,1>.
        */
        Vector2d minus(Vector2d other);


        /**
         * @returns The inverse of this vector.
         * For example, the inverse of <1,1> is <-1,-1>
        */
        Vector2d inverse();

        /**
         * @return The dot product of this and the other vector.
         * 
        */
        float  dot(Vector2d other);


        /**
         * @param scalar the scalar that you want to times the vector by.
         * @returns The scalar product of scalar times the vector.
        */
        Vector2d times(float  scalar);

        /**
         * @returns A string format of the vector
        */
        std::string toString();

        /**
         * @returns The normalized vector (magnitude 1).
        */
        Vector2d normalize();

        /**
         * @returns Whether the two vectors are equal.
        */
        bool equalsTo(Vector2d other);
};