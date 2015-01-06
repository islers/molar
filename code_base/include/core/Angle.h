#pragma once

/*Copyright (c) 2014, Stefan Isler, islerstefan@bluewin.ch
 *
    This file is part of MOLAR (Multiple Object Localization And Recognition),
    which was originally developed as part of a Bachelor thesis at the
    Institute of Robotics and Intelligent Systems (IRIS) of ETH Zurich.

    MOLAR is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    MOLAR is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with MOLAR.  If not, see <http://www.gnu.org/licenses/>.

*/
#include <math.h>


/** Simple class providing some often used angle functionality, angles are usually saved in rad. */
class Angle
{
public:
	Angle(void);
	~Angle(void);
	Angle( double _angle );

	/** returns the angle in radian */
	double rad();
	/** returns the angle in degree */
	double deg();

	/** increments the angle with pi */
	Angle& operator++();
	Angle& operator++(int);
	/** decrements the angle with pi */
	Angle& operator--();
	Angle& operator--(int);

	Angle& operator+(double);
	Angle& operator-(double);
	Angle& operator*(double);
	Angle& operator/(double);

	bool operator>=(Angle);
	bool operator>(Angle);
	bool operator<=(Angle);
	bool operator<(Angle);

	/** Finds the closest equivalent angle to _compare, if any angle obtainable by rotations with pi is equivalent and returns it, the angle on which this is called is not altered */
	Angle closestHalfEquivalent( Angle _compare );
	/** Finds the closest equivalent angle to _compare (any angle obtainable by rotations with pi is equivalent) and sets the angles' angle to that value */
	void toClosestHalfEquivalent( Angle _compare );
	/** Finds the closest equivalent angle to _compare, if any angle obtainable by rotations with two pi is equivalent */
	Angle closestEquivalent( Angle _compare );
	/** Finds the closest equivalent angle to _compare, if any angle obtainable by rotations with two pi is equivalent */
	void toClosestEquivalent( Angle _compare );

	/** converts the angle to its equivalent between 0 and 2 Pi */
	void toZero2Pi();

private:
	double pVal; // angle in radian
public:
	static double two_pi, pi, pi_half, pi_quarter;
};

