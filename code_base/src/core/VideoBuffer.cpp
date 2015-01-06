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

#include "VideoBuffer.h"


VideoBuffer::VideoBuffer(void):tmpFilesO(),tmpFilesE(),pFrameCnt(0),buffCnt(0)
{
	tempVideoFileName="";
	tempVideoFileFolder="";
	setupOptions();
}


VideoBuffer::~VideoBuffer(void)
{
	//delete temporary files on hard disk
	while( tmpFilesO.size()> 0 )
	{
        if( !boost::filesystem::remove( tempVideoFileFolder+"/"+tmpFilesO.top()+"o.avi" ) )
		{
			cerr<<endl<<"Temporary video file "<<tmpFilesO.top() << "o.avi" << " could not be removed."<<endl;
		}
		else tmpFilesO.pop();
	}
	while( tmpFilesE.size()> 0 )
	{
        if( !boost::filesystem::remove( tempVideoFileFolder+"/"+tmpFilesE.top()+"e.avi"  ) )
		{
			cerr<<endl<<"Temporary video file " << tmpFilesE.top() <<"e.avi"<<" could not be removed." << endl ;
		}
		else tmpFilesE.pop();
	}
}


Frame& VideoBuffer::operator<<( Frame& _frame )
{
	//uncompressed memory first
	pFrameCnt++;

	pUBuffer.push_front( _frame );

	unsigned int pUBufferSize = pUBuffer.size()*pUBuffer.back().memUsage(); // [Byte]
	
	if( pUBufferSize <= max_video_ram_usage ) return _frame;
	
	if( pUBuffer.size()==1 )
	{
		cerr << endl << "Frame& VideoBuffer::operator<<( Frame& _frame ):: Warning: Not enough RAM assigned to hold even 1 frame. Ignoring RAM size assignment." << endl;
		return _frame;
	}

	if( !record_input )
	{
		pUBuffer.pop_back();
		return _frame;
	}
	
	boost::filesystem::create_directory( tempFileFolder() );

	stringstream tName;
	tName << tempFileName() << buffCnt++ ;
	string tmpName;
	tName >> tmpName;

	tmpFilesO.push( tmpName );
	tmpFilesE.push( tmpName );

	// HFYU : Files equivalent 17.9 MB huffman lossless codec
	// with loss: MPEG

    VideoWriter vOriginal( tempVideoFileFolder+"/"+tmpFilesO.top()+"o.avi", CV_FOURCC('H','F','Y','U'), 30, Size(640,480),true );
    VideoWriter vEdited( tempVideoFileFolder+"/"+tmpFilesE.top()+"e.avi", CV_FOURCC('H','F','Y','U'), 30, Size(640,480),true );
	
	if( !vOriginal.isOpened() || !vEdited.isOpened() )
	{
			cerr<<endl<<"Frame& VideoBuffer::operator<<( Frame& _frame ):: Temporary video files couldn't be written. Data is lost."<<endl;
			tmpFilesO.pop();
			tmpFilesE.pop();
			pUBuffer.pop_back();
	}

	// dumps 3/4 of the buffer to hard drive
	while( pUBuffer.size()*pUBuffer.back().memUsage() > max_video_ram_usage*3/4 )
	{
		Frame buffFrame = pUBuffer.back();
		pUBuffer.pop_back();
		
		vOriginal << buffFrame.original();
		vEdited << buffFrame.edited();
	}
	
	/* saving files using png compression for each frame -> writing process wasn't finished since the compression
	* stage takes too much time for real time application */
	/*
	//compress memory
	CompressedFrame cFrame( pUBuffer.back() );

	pCBuffer.push_front( cFrame );
	pUBuffer.pop_back();

	int maxCompressedBufferSize = 1024*1024*Options::General["runtime"]["memory"]["max_video_ram_usage"].as<int>();
	int cBufferSizeEstimate = pCBuffer.front().memUsage()*pCBuffer.size();//using size of first frame in buffer as estimate for size of the other frames

	if( cBufferSizeEstimate <= maxCompressedBufferSize ) return _frame;

	//safe 50% of compressed buffer content to hard drive
	boost::filesystem::create_directory( tempFileFolder() );

	string tmpFileName = tempFileName();
	tmpFilesO.push( tmpFileName );

	std::string filePath = Options::General["general_settings"]["program_folder"].as<string>()+"\\"+tempVideoFileFolder + "\\" + tmpFileName+"o.avi";
	
	ofstream tempVideoFile( filePath, ios::out | ios::app | ios::binary ); //appending data to file
	

	while( pCBuffer.front().memUsage()*pCBuffer.size() > 0.5*maxCompressedBufferSize )
	{
		CompressedFrame frameToSave = pCBuffer.back();
		pCBuffer.pop_back();

		if( !tempVideoFile.is_open() ) std::cerr<<endl<<"Temporary video file couldn't be accessed or written."<<endl;
		else
		{
			tempVideoFile.write( reinterpret_cast<char*>( frameToSave.pOriginal.data() ), frameToSave.pOriginal.size()*sizeof(uchar) );
			tempVideoFile.write( reinterpret_cast<char*>( frameToSave.pEdited.data() ), frameToSave.pEdited.size()*sizeof(uchar) );
		}
	}
	tempVideoFile.close();
	*/
	return _frame;
}


Mat& VideoBuffer::operator<<( Mat& /*_image*/ )
{
	return pUBuffer.front().edited();
}


bool VideoBuffer::operator>>( Frame& _frame )
{
	if( pUBuffer.size()==0 )
	{
		cerr << endl << "bool VideoBuffer::operator>>( Frame& _frame ):: Attempting to load frame from empty buffer." << endl;
		return false;
	}

	_frame = pUBuffer.front();
	pUBuffer.pop_front();
	return true;
}


bool VideoBuffer::operator>>( Mat& _image )
{
	if( pUBuffer.size()==0 )
	{
		cerr << endl << "bool VideoBuffer::operator>>( Mat& _image ):: Attempting to load image from empty buffer." << endl;
		return false;
	}

	_image = pUBuffer.front().edited();
	return true;
}

Mat& VideoBuffer::grey()
{
	return pUBuffer.front().grey();
}


Frame VideoBuffer::loadOldest()
{
	if( pUBuffer.size()==0 )
	{
		cerr << endl << "Frame& VideoBuffer::loadOldest():: Attempting to load oldest frame from empty buffer. Returned empty frame is not writable and attempting to do say may lead to memory faults."<<endl;
		throw 1;
	}

	Frame _oldest = pUBuffer.back();
	pUBuffer.pop_back();
	return _oldest;
}


Frame& VideoBuffer::load( unsigned int _i )
{
	if( _i>=pUBuffer.size() )
	{
		cerr << endl << "Frame& VideoBuffer::load( unsigned int _i ):: Attempting to load frame "<<_i<<" from buffer which isn't accessible. Returned empty frame is not writable and attempting to do say may lead to memory faults."<<endl;
		throw 1;
	}

	return pUBuffer[_i];
}

Frame& VideoBuffer::operator[]( unsigned int _i )
{
	return load(_i);
}


Mat& VideoBuffer::loadMat( unsigned int _i )
{
	if( _i>=pUBuffer.size() )
	{
		cerr << endl << "Mat& VideoBuffer::loadMat( unsigned int _i ):: Attempting to load Mat "<<_i<<" from buffer which isn't accessible. Returned empty frame is not writable and attempting to do so may lead to memory faults."<<endl;
		throw 1;
	}

	return pUBuffer[_i].edited();
}


Mat const& VideoBuffer::loadOriginal( unsigned int _i ) const
{
	if( _i>=pUBuffer.size() )
	{
		cerr << endl << "Mat& VideoBuffer::loadOriginal( unsigned int _i ):: Attempting to load original Mat "<<_i<<" from buffer which isn't accessible. "<<endl;
		throw 1;
	}

	return pUBuffer[_i].original();
}


unsigned int VideoBuffer::buffSize() const
{
	return pFrameCnt;
}


std::string VideoBuffer::tempFileName()
{
	time_t currentTime;
	time(&currentTime);

	stringstream fileName;
	fileName << "~tmp" << currentTime;
	fileName >> tempVideoFileName;

	return tempVideoFileName;
}


std::string VideoBuffer::tempFileFolder()
{
	if( tempVideoFileFolder!="" ) return tempVideoFileFolder;

	tempVideoFileFolder = temporary_folder_path;
	return tempVideoFileFolder;
}

bool VideoBuffer::setupOptions()
{
	Options::load_options();
	max_video_ram_usage = 1024*1024*(*Options::General)["runtime"]["memory"]["max_video_ram_usage"].as<int>(); // [Byte]
	record_input = (*Options::General)["runtime"]["buffer"]["record_input"].as<bool>();
	temporary_folder_path = (*Options::General)["runtime"]["memory"]["temporary_folder_path"].as<string>();
	return true;
}

unsigned int VideoBuffer::max_video_ram_usage; // [Byte]
bool VideoBuffer::record_input;
std::string VideoBuffer::temporary_folder_path;
