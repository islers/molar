/*  Copyright (c) 2014, Stefan Isler, islerstefan@bluewin.ch
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


#include "Angle.h"
#include <iostream>

Angle::Angle(void)
{
	pVal = 0;
}


Angle::~Angle(void)
{
}

Angle::Angle( double _angle )
{
	pVal = _angle;
}

double Angle::rad()
{
	return pVal;
}

double Angle::deg()
{
	return pVal/pi*180;
}

Angle& Angle::operator++()
{
	pVal += pi;
	return *this;
}

Angle& Angle::operator++(int)
{
	pVal += pi;
	return *this;
}

Angle& Angle::operator--()
{
	pVal -= pi;
	return *this;
}

Angle& Angle::operator--(int)
{
	pVal -= pi;
	return *this;
}

Angle& Angle::operator+(double _toAdd)
{
	pVal += _toAdd;
	return *this;
}

Angle& Angle::operator-(double _val)
{
	pVal -= _val;
	return *this;
}

Angle& Angle::operator*(double _val)
{
	pVal*=_val;
	return *this;
}

Angle& Angle::operator/(double _val)
{
	pVal/=_val;
	return *this;
}

bool Angle::operator>=(Angle _inp)
{
	return pVal>=_inp.pVal;
}

bool Angle::operator>(Angle _inp)
{
	return pVal>_inp.pVal;
}

bool Angle::operator<=(Angle _inp)
{
	return pVal<=_inp.pVal;
}

bool Angle::operator<(Angle _inp)
{
	return pVal<_inp.pVal;
}

Angle Angle::closestHalfEquivalent( Angle _compare )
{
	Angle equi(pVal);
	double lowerBound = _compare.pVal-pi_half;
	double upperBound = _compare.pVal+pi_half;

	while( _compare>=equi )
	{
		if( lowerBound<=equi.pVal) break;
		else equi++;
	}
	while( _compare<=equi )
	{
		if( upperBound>=equi.pVal) break;
		else equi--;
	}
	return equi;
}


using namespace std;
void Angle::toClosestHalfEquivalent( Angle _compare )
{
	double lowerBound = _compare.pVal-pi_half;
	double upperBound = _compare.pVal+pi_half;

	while( _compare.pVal>=pVal )
	{
		if( lowerBound<=pVal) break;
		else pVal += pi;
	}
	while( _compare.pVal<=pVal )
	{
		if( upperBound>=pVal) break;
		else pVal -= pi;
	}
	return;
}

Angle Angle::closestEquivalent( Angle _compare )
{
	Angle equi(pVal);
	double lowerBound = _compare.pVal-pi;
	double upperBound = _compare.pVal+pi;

	while( _compare>=equi )
	{
		if( lowerBound<=equi.pVal) break;
		else equi++++;
	}
	while( _compare<=equi )
	{
		if( upperBound>=equi.pVal) break;
		else equi----;
	}
	return equi;
}

void Angle::toClosestEquivalent( Angle _compare )
{
	double lowerBound = _compare.pVal-pi;
	double upperBound = _compare.pVal+pi;

	while( _compare.pVal>=pVal )
	{
		if( lowerBound<=pVal) break;
		else pVal+=two_pi;
	}
	while( _compare.pVal<=pVal )
	{
		if( upperBound>=pVal) break;
		else pVal-=two_pi;
	}
	return;
}

void Angle::toZero2Pi()
{
	while( pVal<0 )
	{
		pVal+=two_pi;
	}
	while( pVal>two_pi )
	{
		pVal-=two_pi;
	}
}

double Angle::two_pi=2*acos(-1.0);
double Angle::pi=acos(-1.0);
double Angle::pi_half=acos(-1.0)/2;
double Angle::pi_quarter=acos(-1.0)/4;
