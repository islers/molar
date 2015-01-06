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
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include <math.h>
#include <iostream>

using namespace cv;
using namespace std;

class RectangleRegion
{
	public:
		RectangleRegion(); // indicates the whole image
		RectangleRegion( Point& _p1, Point& _p2, Point& _p3, Point& _p4 );
		RectangleRegion( Point2f& _p1, Point2f& _p2, Point2f& _p3, Point2f& _p4 );
		RectangleRegion( RotatedRect& _rect );
		~RectangleRegion(void);

		// returns true if the rectangle was initialized empty, setting wholeRegion=true
		bool emptyInitialize();

		// calculates if the rectangle region contains the point
		bool contains( Point _pt );

		// returns the vertices of the rectangle
		void points( vector<Point>& _vertices );
		void points( Point* _vertices );

		/** returns the points of the shorter edge i - for i=0 it returns the first pair, for i=1 the second pair */
		void shorterEdgePoints( vector<Point>& _edgePoints, int i=0 );
		void shorterEdgePoints( vector<Point2f>& _edgePoints, int i=0 );

		// returns the vertices of the rectangle, _points must be an array with 4 elements
		void pointArray( Point* _points );

		// tells the RectangleRegion that the intern values (for the contains function) will have to be recalculated if used
		void recalculate();

		// recalculates the edge coordinates, relative to the new origin (whose coordinates must be relative to the old origin)
		void setNewOrigin( Point _origin );

		//returns the angle
		double getAngle();

		// returns the rectangle area
		double area();

		// returns the height of the rectangle (shorter side)
		double height();

		// returns the width of the rectangle (longer side)
		double width();

		// returns the limits of the rectangle (limit coordinates in original coordinates), can be used to create a bounding upright rectangle [leftLimit | upperLimit | rightLimit | lowerLimit]
		void getBoundaries( vector<double>& _boundaries );

		// set height and width of rectangle (shorter side and longer side)
		void setLengths( double _height, double _width );


		/** creates a grid of points that cover the surface of the rectangle */
		void gridPoints( vector<Point2f>& _gridPoints, double _maxCellWidth );

		/** splits the rectangle along the length in _nrOfNewParts parts and returns those */
		void splitAlongLength( unsigned int _nrOfNewParts, vector<RectangleRegion>& _splitParts );

		// draws the rectangle
		void draw( Mat& _img, Scalar _color, double _width=1 );


		// returns the rectangles origin (which is the absolute shift its origin has undergone since the rectangle was created)
		Point getOrigin();
		
		/** rotates a point around the origin */
		static Point2d rotate( Point _pt, double _sinAngle, double _cosAngle );

	private:
		Point origin;
		Point pt[4];// after calculation they're ordered: leftUpper, rightUpper, rightLower, leftLower
		double sinAngle;
		double cosAngle;

		double upperLimit, lowerLimit, leftLimit, rightLimit; 
		bool wholeRegion;
		bool calculated;

		Point2d rotate( Point _pt );

		void calculate();
};

