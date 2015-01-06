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

#include "frame.h"


Frame::Frame(void)
{
	pTime=0;
}

Frame::Frame(Mat _original, double _time)
{
	pOriginal = _original;
	pEdited = _original.clone();
	pTime = _time;

	Mat channels[3];
	split( _original, channels );
	pGreyscale = channels[0];
}

Frame::Frame(Mat _original, Mat _edited, double _time)
{
	pOriginal = _original;
	pEdited = _edited;
	pTime = _time;

	Mat channels[3];
	split( _edited, channels );
	pGreyscale = channels[0];
}

Frame::~Frame(void)
{
}


const Mat & Frame::original() const
{
	return pOriginal;
}

Mat& Frame::edited()
{
	return pEdited;
}

Mat& Frame::grey()
{
	return pGreyscale;
}

double Frame::time() const
{
	return pTime;
}

int Frame::memUsage() const
{
	int origSize = pOriginal.total()*pOriginal.elemSize();
	int editSize = pEdited.total()*pEdited.elemSize();
	return origSize + editSize + sizeof(double);
}
