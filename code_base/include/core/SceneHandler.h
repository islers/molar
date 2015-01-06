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
#include "frame.h"
#include <list>
#include <cmath>

#include "opencv2/highgui/highgui.hpp"
#include "VideoBuffer.h"
#include "IPAlgorithm.h"

#include "average.h"
#include "objecthandler.h"

using namespace std;

class SceneHandler
{
	//general object options and members
	
	friend class Runner;

	public:
		/** @arg: initializeAllObjectClasses	if false then the initializer functions of the objects are not called and the typesInScene() function needs to be used later in order to initialize any classes
		*										if true then all classes are considered to occur in the object and are initialized (if possible that is, off course)
		*/
		SceneHandler( bool _initializeAllObjectClasses=true );
		SceneHandler( double _frameRate );
		~SceneHandler(void);

		static void setupOptions();

		/** saves the project with all settings, including object classes it contains but without object data (eg their paths etc), video frames and any data that is related only directly and only to the video input stream (eg time) */
		bool saveProject( string _filePath );
		/** load settings from file */
		bool loadFromFile( string _filePath );

	private:


	// [PROCESS CONTROL] load new image, get edited image and control of the overall process
	// ***************************************************************************************************************
	public:
		bool operator<<( VideoCapture& _vc); //copies frame from video capture object to SceneHandler
		void pushFrame( Mat _frame, double _time ); // pushes frame into buffer, starts processing it

		Mat last(); //returns last edited frame mat
		Mat grey(); //return last edited grey mat
		Mat resized( double _factor ); // returns a resized version of the image - very time consuming, no fit for real time applications
		Mat operator>>( Mat& _frame); //copies last edited frame Mat to output Mat
		double timeLeft(); // returns for real time applications how much time that is left [ms]
        static inline double msTime(){ return 1000*((double)clock())/CLOCKS_PER_SEC; }; // calculates the actual time in ms

	private:
		double pTime; // last time a frame was loaded


	// general video settings etc
	// ***************************************************************************************************************
	public:
		double getFrameRate() const;
		void setFrameRate( double& _newRate );
		void setFrameRate( VideoCapture& _vc ); //sets the SceneHandler frame rate to the same frame rate as _vc has


	// [BUFFER] video frame saving and loading to and from buffer functions and variables
	// ***************************************************************************************************************
	public:
		/** returns original frame: last:0, second last:1, an empty Mat if the frame number is invalid or the frame not accessible */
		Mat const & getOriginalFrame( unsigned int _frameNumber=0 ) const; //returns original frame
		Mat operator[]( unsigned int _frameNumber ); //returns edited frame
	
		static const int standardBufferSize = 20; //nr of frames, the width of the video buffer
		static const int maxBufferMemory = 50; //MB: if the buffer memory size is about to be exceeded, the buffer size is reduced, the settings options/general is privileged though
		static const int standardFrameRate = 33; //fps

	private:
		VideoBuffer pVideo;// video data
		double pFrameRate; // frames per second
		static bool buffering_activated;
		Mat pWorkingFrame; // only used if buffering is deactivated


	// [GENERAL IMAGE CONVERSION]
	// ***************************************************************************************************************
	/* currently not used: would affect original and edited version */
	private:
		bool initialProcessing( Mat& _image );


	// [PREPROCESSING] video frame preprocessing functions and variables
	// ***************************************************************************************************************
	private:
		/** applies all activated algorithms to the image in their order */
		bool preProcess( Mat& _image );

		/** applies all activated algorithms to the image in their order */
		bool preProcessIntern( Mat& _image );
	public:
		/** returns deque with the names of the algorithms currently activated on the preprocessing stack */
		list<string> preProcessStackInfo();

		/** returns deque with the names of the algorithms currently activated on the intern preprocessing stack */
		list<string> internPreProcessStackInfo();

		/** adds new algorithm to preprocess stack at the given position _i. These are applied to all but the original image data, thus affect the output.
		* If _pos is a position within the current stack, it is inserted, if _pos is higher or negative (no matter what number), it gets added at the end (this is the default behaviour)
		*
		* @ return	pointer to the algorithm object if the algorithm was found, NULL if no algorithm with the given name was found
		*/
		Ptr<IPAlgorithm> addPreProcessingAlgorithm( string _name, GenericMultiLevelMap<string> _options= GenericMultiLevelMap<string>(), int _pos=-1 );

		/** removes algorithm at position _idx from the preprocessing algorithm list */
		void removePreProcessingAlgorithm( int _idx );

		/** moves the preprocess algorithm at position _idx one position up the list if possible */
		void movePreProcessAlgorithmUp( int _idx );

		/** moves the preprocess algorithm at position _idx one position down the list if possible */
		void movePreProcessAlgorithmDown( int _idx );


		/** adds new algorithm to intern preprocess stack at the given position _i. These are applied to the image before attempting to extract the information about displayed objects. They are not applied to any image data in the output.
		* If _pos is a position within the current stack, it is inserted, if _pos is higher or negative (no matter what number), it gets added at the end (this is the default behaviour)
		*
		* @ return	pointer to the algorithm object if the algorithm was found, NULL if no algorithm with the given name was found
		*/
		Ptr<IPAlgorithm> addInternPreProcessingAlgorithm( string _name, GenericMultiLevelMap<string> _options= GenericMultiLevelMap<string>(), int _pos=-1 );
		
		/** removes algorithm at position _idx from the intern preprocessing algorithm list */
		void removeInternPreProcessingAlgorithm( int _idx );

		/** moves the intern preprocess algorithm at position _idx one position up the list if possible */
		void moveInternPreProcessAlgorithmUp( int _idx );

		/** moves the intern preprocess algorithm at position _idx one position down the list if possible */
		void moveInternPreProcessAlgorithmDown( int _idx );


		/** returns the preProcessImage */
		const Mat& preProcessImage();
		/** returns the preThresholdImage */
		const Mat& preThresholdImage();
	private:
		list< Ptr<IPAlgorithm> > pPreProcessStack;
		list< Ptr<IPAlgorithm> > pInternPreProcessStack;
		// image output buffers (for information purposes only) - they're set only if set so in options
		Mat pPreProcessImage, pPreThresholdImage;
		//temporary for preprocess image buffering options
		static bool create_preprocess_filter_image; // if true then the output after the first filter stage is buffered and can be accessed through preProcessImage() {affects: SceneHandler}
		static bool create_prethreshold_filter_image; // if true then the output after the second, internal filter stage (right before thresholding) is buffered and can be accessed through preThresholdImage() {affects: SceneHandler}

	// [OBJECT DETECTION] detecting objects in the scene
	// ***************************************************************************************************************
	public:
		/** returns the internal objecthandler object - giving access to the ObjectHandlers' public functions */
		ObjectHandler& objects();

		/** types indicated in the vector are considered to be in scene (names have to match the type names), all those types not mentioned are considered not to be in the scene */
		void typesInScene( vector<string>& _names );
		void typesInScene( string _name1, string _name2="", string _name3="", string _name4="", string _name5="", string _name6="", string _name7="", string _name8="" );

		/** types indicated in the vector are considered not to be in scene (names have to match the type names), all those types not mentioned are considered to be in the scene */
		void typesNotInScene( vector<string>& _names );
		void typesNotInScene( string _name1, string _name2="", string _name3="", string _name4="", string _name5="", string _name6="", string _name7="", string _name8="" );
		

		/** set the area inside the video frames in which objects shall be detected */
		void setObservationArea( int _upperLeftX, int _upperLeftY, int _width, int _height );

		const Rect& getObservationArea();

		/** Set the threshold which is used for binary creation. If the function isn't called, then the threshold is internally set to static_threshold (global option) */
		void setThreshold( double _threshold );

		/** gets the threshold value */
		double getThreshold();

	private:
		ObjectHandler pObjectSet;

		Rect pObservationArea; // area in the image in which objects shall be detected - if not set, then the whole area is being searched
		
		// temporary
		static bool draw_observation_area;
		static Scalar draw_observation_area_color;
};
