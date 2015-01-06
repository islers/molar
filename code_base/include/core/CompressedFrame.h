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
#include "opencv2/core/core.hpp"
#include "opencv2/opencv.hpp"
#include "frame.h"
using namespace cv;


class CompressedFrame
{
	friend class VideoBuffer;
public:
	CompressedFrame(void);
	CompressedFrame( Frame& _uncompressed );
	CompressedFrame(Mat _original, double& _time);
	~CompressedFrame(void);

	/** returns the original image as Mat
	*/
	Mat original() const;

	/** saves and encodes a new orignal image
	*/
	void original( Mat& _image );

	/** returns the edited image as Mat
	*/
	Mat edited() const;

	/** saves and encodes a new edited image
	*/
	void edited( Mat& _image );

	double time() const;

	/** returns the used memory in bytes
	*/
	int memUsage() const;

private:
    vector<uchar> pOriginal; //compressed original image
    vector<uchar> pEdited; //compressed edited image
	double pTime; //ms

	/** encodes the image and saves it in the target memory using the standard settings
	*/
    bool encodeImage( vector<uchar>* _targetMemory, Mat& _image );
	Mat decodeImage( vector<uchar>* _targetMemory ) const;
};

