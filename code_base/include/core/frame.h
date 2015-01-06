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

/* class to store frames of a movie in an edited and original version, together with their time stamp */
#pragma once

#include <list>
#include "opencv2/core/core.hpp"

using namespace cv;

class Frame
{
public:
	Frame(void);
	Frame(Mat _original, double _time);
	//assumes that _edited is already an independent version
	Frame(Mat _original, Mat _edited, double _time);

	~Frame(void);

	const Mat & original() const;
	Mat& edited();
	Mat& grey();
	double time() const;
	

	/** returns the used memory in bytes
	*/
	int memUsage() const;

private:
	Mat pOriginal;
	Mat pEdited;
	Mat pGreyscale;

	double pTime; //ms
};

