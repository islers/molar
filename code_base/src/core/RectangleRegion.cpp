#include "RectangleRegion.h"
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



RectangleRegion::RectangleRegion(): origin(0,0), wholeRegion(true), calculated(false){}


RectangleRegion::RectangleRegion( Point& _p1, Point& _p2, Point& _p3, Point& _p4 ): origin(0,0), wholeRegion(false), calculated(false)
{
	pt[0]=_p1;
	pt[1]=_p2;
	pt[2]=_p3;
	pt[3]=_p4;

	return;
}


RectangleRegion::RectangleRegion( Point2f& _p1, Point2f& _p2, Point2f& _p3, Point2f& _p4 ): origin(0,0), wholeRegion(false), calculated(false)
{
	pt[0]=_p1;
	pt[1]=_p2;
	pt[2]=_p3;
	pt[3]=_p4;

	return;
}

RectangleRegion::RectangleRegion( RotatedRect& _rect ): wholeRegion(false), calculated(false)
{
	Point2f rectanglePoints[4];
	_rect.points( rectanglePoints );
	pt[0]=rectanglePoints[0];
	pt[1]=rectanglePoints[1];
	pt[2]=rectanglePoints[2];
	pt[3]=rectanglePoints[3];
}


RectangleRegion::~RectangleRegion(void)
{
}


bool RectangleRegion::emptyInitialize()
{
	return wholeRegion;
}


bool RectangleRegion::contains( Point _pt )
{
	if( wholeRegion ) return true;
	else if(!calculated) calculate();

	Point2d pt = rotate(_pt);

	return pt.x>=leftLimit && pt.x<=rightLimit && pt.y>=lowerLimit && pt.y<=upperLimit;
}


void RectangleRegion::points( vector<Point>& _vertices )
{
	_vertices.push_back( pt[0] );
	_vertices.push_back( pt[1] );
	_vertices.push_back( pt[2] );
	_vertices.push_back( pt[3] );
	return;
}


void RectangleRegion::points( Point* _vertices )
{
	try{
		_vertices[0] = pt[0];
		_vertices[1] = pt[1];
		_vertices[2] = pt[2];
		_vertices[3] = pt[3];
	}catch(...)
	{
		cerr<<endl<<"RectangleRegion::points: An error occured when trying to write into the elements of the array. Is it too small or uninitialized?"<<endl;
	}
	return;
}


void RectangleRegion::shorterEdgePoints( vector<Point>& _edgePoints, int _i )
{
	if( _i!=0 && _i!=1 ) _i = 0;
	if( !calculated ) calculate();

	double candidate1 = upperLimit-lowerLimit;
	double candidate2 = rightLimit-leftLimit;

	if( candidate2>=candidate1 ) // rotated rectangle lies on side
	{
		if( _i==0 )
		{
			_edgePoints.push_back(pt[3]); // -> after calculation the points in the array are ordered...
			_edgePoints.push_back(pt[0]);
			return;
		}
		else
		{
			_edgePoints.push_back(pt[1]);
			_edgePoints.push_back(pt[2]);
			return;
		}
	}
	else // rotated rectangle "stands on shorter side"
	{
		if( _i==0 )
		{
			_edgePoints.push_back(pt[0]);
			_edgePoints.push_back(pt[1]);
			return;
		}
		else
		{
			_edgePoints.push_back(pt[2]);
			_edgePoints.push_back(pt[3]);
			return;
		}
	}
}


void RectangleRegion::shorterEdgePoints( vector<Point2f>& _edgePoints, int _i )
{
	vector<Point> temp;
	shorterEdgePoints( temp, _i );

    for(unsigned int i=0;i<temp.size();i++)
	{
        Point2f toAdd = temp[i];
        _edgePoints.push_back( toAdd );
	}
	return;
}


void RectangleRegion::pointArray( Point* _points )
{
	try{
		_points[0] = pt[0];
		_points[1] = pt[1];
		_points[2] = pt[2];
		_points[3] = pt[3];
	}catch(...)
	{
		cerr<<endl<<"RectangleRegion::pointArray: An error occured when trying to write into the elements of the array. Is it too small or uninitialized?"<<endl;
	}
	return;
}


void RectangleRegion::recalculate()
{
	calculated=false;
}


// origin relative to current origin
void RectangleRegion::setNewOrigin( Point _origin )
{
	for( int i=0; i<4; i++ )
	{
		pt[i].x -= _origin.x;
		pt[i].y -= _origin.y;
	}
	origin += _origin;
	recalculate();
	return;
}


double RectangleRegion::getAngle()
{
	if( wholeRegion ) return 0;
	else if(!calculated) calculate();

	return asin(sinAngle);
}


double RectangleRegion::area()
{
	if( !calculated ) calculate();
	return abs( (upperLimit-lowerLimit)*(rightLimit-leftLimit) );
}


double RectangleRegion::height()
{
	if( !calculated ) calculate();
	double candidate1 = upperLimit-lowerLimit;
	double candidate2 = rightLimit-leftLimit;
	return (candidate1<candidate2)?candidate1:candidate2;
}


double RectangleRegion::width()
{
	if( !calculated ) calculate();
	double candidate1 = upperLimit-lowerLimit;
	double candidate2 = rightLimit-leftLimit;
	return (candidate1>candidate2)?candidate1:candidate2;
}


void RectangleRegion::getBoundaries( vector<double>& _boundaries )
{
	if( !calculated ) calculate();
	double mostLeft, highest, mostRight, lowest;

	highest=max( pt[0].y, pt[1].y );
	lowest=min( pt[2].y, pt[3].y );

	mostRight=max( pt[1].x,pt[2].x );
	mostLeft=min( pt[0].x,pt[3].x );

	_boundaries.push_back( mostLeft );
	_boundaries.push_back( highest );
	_boundaries.push_back( mostRight );
	_boundaries.push_back( lowest );
	return;
}


void RectangleRegion::setLengths( double _height, double _width )
{
	if( !calculated ) calculate();

	double candidate1 = upperLimit-lowerLimit;
	double candidate2 = rightLimit-leftLimit;

	if( candidate1<=candidate2 ) //consider upper-lower as height
	{
		double heightDifference = (_height-candidate1)/2;
		upperLimit+=heightDifference;
		lowerLimit-=heightDifference;
		double widthDifference = (_width-candidate2)/2;
		rightLimit+=widthDifference;
		leftLimit-=widthDifference;
	}
	else // upper-lower as width
	{
		double heightDifference = (_height-candidate2)/2;
		rightLimit+=heightDifference;
		leftLimit-=heightDifference;
		double widthDifference = (_width-candidate1)/2;
		upperLimit+=widthDifference;
		lowerLimit-=widthDifference;
	}

	pt[0] = rotate( Point2d( leftLimit, upperLimit ), -sinAngle, cosAngle );
	pt[1] = rotate( Point2d( rightLimit, upperLimit ), -sinAngle, cosAngle );
	pt[2] = rotate( Point2d( rightLimit, lowerLimit ), -sinAngle, cosAngle );
	pt[3] = rotate( Point2d( leftLimit, lowerLimit ), -sinAngle, cosAngle );

	return;
}


void RectangleRegion::gridPoints( vector<Point2f>& _gridPoints, double _maxCellWidth )
{
	if( !calculated ) calculate();

	double length_1 = upperLimit-lowerLimit;
	double length_2 = rightLimit-leftLimit;
	double cellWidthX, cellWidthY;

	if( length_1> length_2 ) // length along y direction
	{
		int cellsAlongHeight = ceil(length_2/_maxCellWidth);
		int cellsAlongLength = ceil(length_1/_maxCellWidth);
		cellWidthX = length_2/cellsAlongHeight;
		cellWidthY = length_1/cellsAlongLength;
	}
	else // length along x direction
	{
		int cellsAlongHeight = ceil(length_1/_maxCellWidth);
		int cellsAlongLength = ceil(length_2/_maxCellWidth);
		cellWidthX = length_2/cellsAlongLength;
		cellWidthY = length_1/cellsAlongHeight;
	}

	for( int x=leftLimit;x<=rightLimit; x+=cellWidthX )
	{
		for( int y=lowerLimit;y<=upperLimit; y+=cellWidthY )
		{
			Point2f gridPoint( x,y ); // grid point in (upright) rectangle space
			_gridPoints.push_back( rotate(gridPoint,-sinAngle,cosAngle) ); // transform grid point back to image space
		}
	}
	return;
}


void RectangleRegion::splitAlongLength( unsigned int _nrOfNewParts, vector<RectangleRegion>& _splitParts )
{
	if( !calculated ) calculate();

	double length_1 = upperLimit-lowerLimit;
	double length_2 = rightLimit-leftLimit;

	if( length_1>length_2 )
	{
		double partLength=length_1/_nrOfNewParts;

        for( size_t i=0;i<_nrOfNewParts;i++ )
		{
			Point lU( leftLimit,lowerLimit+(i+1)*partLength );
			Point rU( rightLimit,lowerLimit+(i+1)*partLength );
			Point rL( rightLimit,lowerLimit+i*partLength );
			Point lL( leftLimit,lowerLimit+i*partLength );
			lU = rotate(lU,-sinAngle,cosAngle);
			rU = rotate(rU,-sinAngle,cosAngle);
			rL = rotate(rL,-sinAngle,cosAngle);
			lL = rotate(lL,-sinAngle,cosAngle);
			
			_splitParts.push_back( RectangleRegion(lU,rU,rL,lL) );
		}
	}
	else
	{
		double partLength=length_2/_nrOfNewParts;

        for( size_t i=0;i<_nrOfNewParts;i++ )
		{
			Point lU( leftLimit+i*partLength,upperLimit );
			Point rU( leftLimit+(i+1)*partLength,upperLimit );
			Point rL( leftLimit+(i+1)*partLength,lowerLimit );
			Point lL( leftLimit+i*partLength,lowerLimit );
			lU = rotate(lU,-sinAngle,cosAngle);
			rU = rotate(rU,-sinAngle,cosAngle);
			rL = rotate(rL,-sinAngle,cosAngle);
			lL = rotate(lL,-sinAngle,cosAngle);
			
			_splitParts.push_back( RectangleRegion(lU,rU,rL,lL) );
		}
	}
	return;
}


void RectangleRegion::draw( Mat& _img, Scalar _color, double _width )
{
	line( _img, pt[0], pt[1], _color, _width, 8 );
	line( _img, pt[1], pt[2], _color, _width, 8 );
	line( _img, pt[2], pt[3], _color, _width, 8 );
	line( _img, pt[3], pt[0], _color, _width, 8 );
	return;
}

Point RectangleRegion::getOrigin()
{
	return origin;
}


Point2d RectangleRegion::rotate( Point _pt )
{
	return Point2d( cosAngle*_pt.x-sinAngle*_pt.y, sinAngle*_pt.x+cosAngle*_pt.y );
}


Point2d RectangleRegion::rotate( Point _pt, double _sinAngle, double _cosAngle )
{
	return Point2d( _cosAngle*_pt.x-_sinAngle*_pt.y, _sinAngle*_pt.x+_cosAngle*_pt.y );
}


void RectangleRegion::calculate()
{
	double rotationAngle = -atan( ((double)pt[1].y-pt[0].y)/(pt[1].x-pt[0].x) );

	sinAngle = sin(rotationAngle);
	cosAngle = cos(rotationAngle);

	Point2d pts[4];
	pts[0] = rotate( pt[0] );
	pts[1] = rotate( pt[1] );
	pts[2] = rotate( pt[2] );
	pts[3] = rotate( pt[3] );

	
	upperLimit = lowerLimit = pts[0].y;
	leftLimit = rightLimit = pts[0].x;

	for( int i=1;i<4;i++ )
	{
		if( pts[i].x<leftLimit ) leftLimit=pts[i].x;
		else if( pts[i].x>rightLimit ) rightLimit=pts[i].x;
		if( pts[i].y<lowerLimit ) lowerLimit=pts[i].y;
		else if( pts[i].y>upperLimit ) upperLimit=pts[i].y;
	}

	pt[0] = rotate( Point2d( leftLimit, upperLimit ), -sinAngle, cosAngle );
	pt[1] = rotate( Point2d( rightLimit, upperLimit ), -sinAngle, cosAngle );
	pt[2] = rotate( Point2d( rightLimit, lowerLimit ), -sinAngle, cosAngle );
	pt[3] = rotate( Point2d( leftLimit, lowerLimit ), -sinAngle, cosAngle );
	
	calculated=true;
}
