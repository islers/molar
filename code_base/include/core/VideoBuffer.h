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


#include "CompressedFrame.h"
#include "opencv2/core/core.hpp"
#include <deque>
#include "Options.h"
#include "boost/filesystem.hpp"
#include <ctime>
#include <stack>

class VideoBuffer
{
public:
	VideoBuffer(void);
	~VideoBuffer(void);

	//safe frames into buffer ***************************************************
	/** safes new frame in buffer */
	Frame& operator<<( Frame& _frame );
	/** safes image as edited version into last frame in the buffer */
	Mat& operator<<( Mat& _image );

	//load frames from buffer ***************************************************

	/** extracts the newest frame from the buffer (this means it gets removed) */
	bool operator>>( Frame& _frame );
	/** loads edited image from last frame in the buffer */
	bool operator>>( Mat& _image );
	/** loads greyscale image */
	Mat& grey();
	/** load oldest frame from buffer (and extract it) - currently only from buffer, not the temporary files */
	Frame loadOldest();
	/** load frame i from buffer, i=0: newest, i=1: second newest
	* @return	the frame, throws an exception if the number was invalid or the frame not accessible in the buffer
	*/
	Frame& load( unsigned int _i=0 );
	/** variant of load */
	Frame& operator[]( unsigned int _i );
	/** load edited mat i from buffer, see load for details
	*/
	Mat& loadMat( unsigned int _i=0 );
	/** load original mat i from buffer, see load for details
	*/
	Mat const & loadOriginal( unsigned int _i ) const;

	/** returns the number of buffered frames */
	unsigned int buffSize() const;

	// option setup
	static bool setupOptions();

private:
	std::deque<Frame> pUBuffer; //uncompressed frames
	std::stack<std::string> tmpFilesO;
	std::stack<std::string> tmpFilesE;

	unsigned int pFrameCnt;
	std::list<CompressedFrame> pCBuffer; //compressed frames
	std::string pMemoryBufferPath; //temp filename and path in which additional frames are stored on hard disk

	unsigned int buffCnt;
	std::string tempVideoFileName;
	std::string tempVideoFileFolder; //protection against a possible temp folder change in the options during runtime

	std::string tempFileName(); //creates and/or returns new tempFileName
	std::string tempFileFolder(); // returns temp file folder

	//option variables
	static unsigned int max_video_ram_usage;
	static bool record_input;
	static std::string temporary_folder_path;
};

