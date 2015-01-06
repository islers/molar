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


#include <iostream>
#include <string>
#include <list>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "SceneHandler.h"


using namespace std;
using namespace cv;


int main( int argc, char ** argv )
{
		
	string videoPath="../video_files/twoHelixTypes_original.avi";
	
	// to capture video streams the OpenCV VideoCapture object is used
	VideoCapture video(videoPath); //0: default camera
	if( !video.isOpened() )
	{
		cerr<<endl<<"Could not open camera or find the video"<<endl;
	}
		
	// creating a molar scene handler which is the interface MOLAR's functionality
	SceneHandler myMolarInstance(false);

	myMolarInstance.setFrameRate(video);
	
	//add a simple contrast/brightness-filter with custom options
	GenericMultiLevelMap<string> filterOptions;
	filterOptions["contrast_factor"].as<double>()=2.7;
	filterOptions["brightness_offset"].as<double>()=20;
	myMolarInstance.addInternPreProcessingAlgorithm("contrast brightness adjustment",filterOptions);
	
	
	// telling molar that these three types are expected to occur in the scene - they are defined in the provided files in the "generic classes" folder (which must reside in the same directory where the executable is called - it is created automatically if it doesn't exist yet)
	myMolarInstance.typesInScene("ThickHelix_KF","ThinHelix_KF","Unknown Type");
	
	
	cout<<endl<<"Be aware that the output of this example isn't timed, it runs as fast as possible..."<<endl;
	
	bool firstIt = true;
	
	while(true)
	{
		// push a frame from the stream to molar and process it
		if( !(myMolarInstance << video) )
		{
		  cout<<endl<<"Finished... Press any button to exit. (Focus must be on output window)"<<endl;
		  waitKey(0);
		  break;
		}
		
		/* off course you could also get informations about the detected objects
		 * e.g. through myMolarInstance.objects().activeIdList( vector<unsigned int>& info )"
		 * and then "myMolarInstance.objects().getObj( id ).pos()" to get a position
		 * 
		 * refer to the SceneHandler, ObjectHandler and SceneObject classes
		 */
		
		// show the output with openCV
		imshow("original video", myMolarInstance.getOriginalFrame() );
		imshow("processed video", myMolarInstance.last() );
		
		if(firstIt)
		{
		  cout<<endl<<"Press any button to start processing. (Focus must be on output window)"<<endl;
		  waitKey(0);
		  firstIt=false;
		}
		else waitKey(1);
	}
	
	return 0;

}