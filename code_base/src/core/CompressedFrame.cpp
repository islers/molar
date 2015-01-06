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

#include "CompressedFrame.h"
#include "Options.h"


CompressedFrame::CompressedFrame(void)
{
}

CompressedFrame::CompressedFrame( Frame& _uncompressed )
{
	Mat original = _uncompressed.original();

	encodeImage( &pOriginal, original );
	encodeImage( &pEdited, _uncompressed.edited() );
	pTime = _uncompressed.time();
}

CompressedFrame::CompressedFrame(Mat _original, double& _time)
{
	encodeImage( &pOriginal, _original );
    Mat clonedOriginal = _original.clone();
    encodeImage( &pEdited, clonedOriginal );
	pTime = _time;
}

CompressedFrame::~CompressedFrame(void)
{
}

Mat CompressedFrame::original() const
{
	return decodeImage( const_cast<std::vector<uchar>*>(&pOriginal) );
}

void CompressedFrame::original( Mat& _image )
{
	encodeImage( &pOriginal, _image );
	return;
}

Mat CompressedFrame::edited() const
{
	return decodeImage( const_cast<std::vector<uchar>*>(&pEdited) );
}

void CompressedFrame::edited( Mat& _image )
{
	encodeImage( &pEdited, _image );
	return;
}

double CompressedFrame::time() const
{
	return pTime;
}

bool CompressedFrame::encodeImage( vector<uchar>* _targetMemory, Mat& _image )
{
	vector<int> params(2);
	bool success=true;

	if( (*Options::General)["runtime"]["compression"]["format"].as<string>()==".PNG" )
	{
		params[0]=CV_IMWRITE_PNG_COMPRESSION;
		params[1]=(*Options::General)["runtime"]["compression"]["png_compression_level"].as<int>();
		success = success && imencode( ".PNG",_image,*_targetMemory, params );
	}
	else if( (*Options::General)["runtime"]["compression"]["format"].as<string>()==".JPEG" )
	{
		params[0]=CV_IMWRITE_JPEG_QUALITY;
		params[1]=(*Options::General)["runtime"]["compression"]["jpeg_quality"].as<int>();
		success = success &&  imencode( ".JPEG",_image,*_targetMemory, params );
	}
	else
	{
		std::cerr<<endl<<"Specified format for compression '"<<(*Options::General)["runtime"]["compression"]["format"].as<string>()<<"' is not supported."<<endl;
		return false;
	}
	
	if( !success )
	{
		std::cerr<<endl<<"Compression failed."<<endl;
		return false;
	}

	return true;

}

Mat CompressedFrame::decodeImage( vector<uchar>* _targetMemory ) const
{
	return imdecode( *_targetMemory,-1 ); //return the image as is: -1
}

int CompressedFrame::memUsage() const
{
	int origSize = pOriginal.size();
	int editSize = pEdited.size();

	return sizeof(double)+2*sizeof(vector<uchar>)+origSize*sizeof(uchar)+editSize*sizeof(uchar);
}
