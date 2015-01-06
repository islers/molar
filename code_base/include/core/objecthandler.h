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
#include "sceneobject.h"
#include "RectangleRegion.h"
#include "VideoBuffer.h"
#include "DescriptorCreator.h"

class SceneHandler;
class GenericObject;

class ObjectHandler
{
	friend SceneHandler;
	friend SceneObject;
	friend GenericObject;

public:
	ObjectHandler( SceneHandler* _sceneLink, VideoBuffer* _videoLink );
	~ObjectHandler(void);

	/** saves any setting inside the object that is needed in order to operate a new video in the same fashion as the actual */
	void saveSettings( GenericMultiLevelMap<string>& _settings );

	/** loads settings as created with 'saveSettings' from GMLM */
	void loadFromMap( GenericMultiLevelMap<string>& _map );

private:
	/** creates a vector with iterators to all currently active objects (categorized, uncategorized and missing) */
    void buildObjList( vector< list<Ptr<SceneObject> >::iterator >& _objList );

	/** main function that starts and coordinates all object detection and classification
	*
	*	@param	Mat& _img			single channel greyscale image, used for calculatiosn
	*	@param	Mat& _outputImage	image that will be used for display and video
	*	@param	double _time		time stamp for the image
	*/
	void pushFrame( Mat& _img, Mat& _outputImage, double _time );

	/** types indicated in the vector are considered to be in scene (names have to match the type names), all those types not mentioned are considered not to be in the scene 
	*	The function also initializes the classes.
	*/
	void typesInScene( vector<string>& _names );

	/** types indicated in the vector are considered not to be in scene (names have to match the type names), all those types not mentioned are considered to be in the scene
	*	The function also initializes the classes.
	*/
	void typesNotInScene( vector<string>& _names );
public:
	/** Set the threshold which is used for binary creation. If the function isn't called, then the threshold is internally set to static_threshold (global option) */
	void setThreshold( double _threshold );

	/** creates a new descriptor creator and adds it directly */
    Ptr<DescriptorCreator> newDescriptorCreator( Ptr<vector<unsigned int> > _objectIds );
    Ptr<DescriptorCreator> newDescriptorCreator(int _id0, int _id1=-1, int _id2=-1, int _id3=-1, int _id4=-1, int _id5=-1, int _id6=-1, int _id7=-1, int _id8=-1, int _id9=-1 );

	/** adds a new descriptor creator to the set of descriptor creators, returns true if successful */
	bool addDescriptorCreator( Ptr<DescriptorCreator> _dC );
	bool addDescriptorCreator( DescriptorCreator* _dC );
	/** removes descriptor creator from set of descriptor creators returns true if successfully removed */
	bool removeDescriptorCreator( Ptr<DescriptorCreator> _dC );
	bool removeDescriptorCreator( DescriptorCreator* _dC );

	/** creates a list with the ids of all currently considered active objects in the scene (the vector _idList is cleared if it isn't empty yet)*/
	void activeIdList( vector<unsigned int>& _idList );
	/** returns the id of the currently considered active object with the lowest id */
	unsigned int lowestActiveId();
	/** returns the id of the currently considered active object with the highest id */
	unsigned int highestActiveId();

	/** returns a pointer to the object indicated by _objId */
	Ptr<SceneObject> getObj( unsigned int _objId );

private:
	/** searches an image frame for objects and returns their contours and bounding rectangles */
	void objRegions( Mat& _binaryImg, vector< vector<Point> >& _contours, vector<RectangleRegion>& _regions );

	/** calculates upright rectangles around the (possible rotated) RectangleRegions */
	void calculateROIs( vector<RectangleRegion>& _regions, vector<Rect>& _uprightRegions );

	/** calculates upright rectangle around the (possible rotated) Rectangle Region */
	void calculateROI( RectangleRegion& _region, Rect& _uprightRegion );

	/** constructs the Mat headers for the Rect regions, based on the image _img */
	void calculateROIMats( Mat& _img, vector<Rect>& _roiRegions, vector<Mat>& _roiMats );

	/** consturct the Mat header for the Rect region, based on the image _img */
	void calculateROIMat( Mat& _img, Rect& _roiRegion, Mat& _roiMat );

	/** transform the coordinates of the regions with the upper left corners of the ROI as new origins. Both vectors must have same size. */
	void transformToRelative( vector<Rect>& _roiRegions, vector<RectangleRegion>& _inputRegions, vector<RectangleRegion>& _transformedRegions );

	/** transform the coordinate of the region, with the upper left corner of the roi as new origin.*/
	void transformToRelative( Rect& _roiRegion, RectangleRegion& _region );

	/** calculates for each image in _roiImages the state of the region in it described by _objectRegions
	*
	*	@return		Mat&	states of the regions ( centroid.x | centroid.y | direction angle ) in the rows of the Mat
	*/
	void roiProperties( vector<Mat>& _roiImages, vector<RectangleRegion>& _objectRegions, vector<Rect>& _imgPositions, Mat& _states );

	/** calculates the state of the given region in the image
	*
	*	@return		Mat&		state as row Mat ( centroid.x | centroid.y | direction angle )
	*	@return		double&		area of the pixels in the image
	*/
	Mat calcROIstate( Mat& _roiImg, RectangleRegion& _region, Rect& _imgPosition, double& _area );

public:
	/** creates an image mask with the same size as _targetImage where the _unmaskedRegion is white (1), rest (0)
	*/
	static Mat imageMask( Mat& _targetImage, RectangleRegion& _unmaskedRegion );
	
private:
	/** function to weight states */
	Mat weightDimensions( Mat& _state );


	/** function to unweight states */
	Mat unweightDimensions( Mat& _state );


	/** calculates the predicted states for all active objects of the previous run */
    void predictProperties( vector< list<Ptr<SceneObject> >::iterator >& _objList, Mat& _predictedStates );


	/** function matches predicted states to the measured states and returns their mapping
	*/
	void matchObjects( Mat& _states, Mat& _predictedStates, vector< vector<Point> >& _contours, vector<int>& _objectMapping, vector<bool>& _foundObject );


	/** asks all objects that are missing if there is a potential match among the found areas in the current frame, all object matches to an area are returned as vectors, so basically for each area a group is built with objects mapped to it, unless the area represents only one object
	*  (the method leaves the details for each object to SceneObject class methods instead of calculating positions inside the method as it is done in "findMissingObjects")
	*
	*	@return		vector<vector<int>> _objectGroups	groups of objects that map to the region with the respective idx in the first level array
	*/
    void findPotentialAreaMatchesForMissingObjects( Mat& _predictedStates, Mat& _outputImage, vector< list<Ptr<SceneObject> >::iterator >& _objList, vector<RectangleRegion>& _regions, vector< vector<Point> >& _contours, vector<int>& _objectMapping, vector<bool>& _foundObjects, vector<vector<int> >& _objectGroups );


	/** calculates whether missing objects might be part of another found object and returns such groups
	*
	*	@return		vector<vector<int>> _objectGroups	groups of objects that map to the region with the respective idx in the first level array
	*/
    void findMissingObjects( Mat& _predictedStates, Mat& _outputImage, vector< list<Ptr<SceneObject> >::iterator >& _objList, vector<RectangleRegion>& _regions, vector< vector<Point> >& _contours, vector<int>& _objectMapping, vector<bool>& _foundObjects, vector<vector<int> >& _objectGroups );


	/** attempts to locate the missing objects that have been predicted to lie inside a found region in the image and calculates states for them which are added to the _objectStates object, together with the correct matching to the new state in _objectMapping */
    void locateMissingObjects( Mat& _invGreyImg, Mat& _outputImage, Mat& _objectStates, vector< list<Ptr<SceneObject> >::iterator >& _objList, vector<vector<int> >& _objectGroups, vector<int>& _objectMapping, vector<bool>& _foundObjects, vector<vector<Point> >& _contours, vector<RectangleRegion>& _regions, vector<Rect>& _roiRegions, vector<Mat>& _invROIs );

	
	/** update the object lists, based on object states and their mappings */
    void updateObjects( vector< list<Ptr<SceneObject> >::iterator >& _objList, Mat& _objectStates, Mat& _predictedStates, vector<int>& _objectMapping, vector<bool>& _foundObjects, vector<vector<Point> >& _contours, vector<RectangleRegion>& _regions, vector<Mat>& _invROIs );
	

	/** add object to missing object list and create a counter */
	void addToMissing( Ptr<SceneObject> _objectToAdd );

	/** removes the element from the missing object list and destroys its counter */
    bool removeFromMissing( list<Ptr<SceneObject> >::iterator& _object );


	/** returns the missing counter for the given object, -1 if it wasn't found */
	int& getMissingTime( Ptr<SceneObject> _object );


	/** increases the counter for all objects in the missing list by one and removes those that were in there too long, adding them to the lost object list */
	void updateMissing();


	/** classifies (or reclassifies) all objects if enough time is left */
	void updateClassifications( Mat _image );


	/** returns the next object up for classification */
	Ptr<SceneObject> getObjForClassification();


	/** fill an objects' descriptor set into the currently active descriptor creators (if any) */
	void fillDescriptorCreators( Mat& _containingImage, Mat& _descriptors, int _objId );


	/** calculates the average point for a point set (vector) */
	Point average( vector<Point>& _pointSet );

	/** calculates and returns a set of points that contains as a subset the local maxima for a given range (set includes more points since only local minima inside mesh boundaries are returned, there positions are not cross-checked for their distance) 
	*  @param	_range		half range in which the subset points should be a local maxima 
	*  @param	_minValue	minimal value the local maxima needs to have in order not to be discarded
	*  @return	_pointSet	each point in a row: ( x pos | y pos | value )
	*/
	void extendedLocalMaxima( Mat& _inputImage, Mat& _pointSet, unsigned int _range, double _minValue );

	/** calculates and returns the local maxima for a given range (set includes more points since only local minima inside mesh boundaries are returned, there positions are not cross-checked for their distance) 
	*  @param	_range		half range in which the subset points should be a local maxima 
	*  @param	_minValue	minimal value the local maxima needs to have in order not to be discarded
	*  @return	_pointSet	each point in a row: ( x pos | y pos | value )
	*/
	void calcLocalMaxima( Mat& _inputImage, Mat& _pointSet, unsigned int _range, double _minValue );

	/** calculates if the point is located close to the border (inside a range specified in the options as window_border_range)
	*/
	bool closeToWindowBorder( Point _point );

	/** draws results of calculations into the image, as set by user */
	void draw( Mat& _image );


	/** sets up the information vector about which object types might occur in the scene, _initValue: true - all types might occur, false - none of the types known occur */
	void setupObjectTypesInfo( bool _initValue );
	/** assures that the size of pObjectTypesInScene is the same as the number of registered classes (necessary if new classes are created) */
	void assureOTISSize();


    /** creates an image with the predicted positions, the currently found positions and all matches and shows it in a window called "_windowName"
	* blue: predicted positions and number in the predictedStates array
	* green: contour positions and number in the contourPositions array
	* white: matches between predicted positions and contour, the number indicates the id of the object at the position where it is believed to be
	*/
    void showMatchWindow( vector< list<Ptr<SceneObject> >::iterator >& _objList, Mat _predictedStates, Mat _contourPositions, vector<int>& _objectMapping, string _windowName );

public:

	// *ENVIRONMENT* INFORMATION //////
	unsigned int imageHeight();
	unsigned int imageWidth();

	// process image access
	const Mat& thresholdImage() const;
	
	// settings setup
	static void setupOptions();

	/** returns whether the type with the given id is set to be occuring in the scene or not */
	bool typeIsInScene( int _classId );

private:
	// pointer to video frames
    SceneHandler* pScene;
    VideoBuffer* pVideo;

	// environment information
	unsigned int pImageHeight;
	unsigned int pImageWidth;

	// process images
	Mat pThresholdImage;

	// temporary buffer for paths
	Ptr<Mat> pPathMat;
	unsigned int pFramesSinceLastFade; // help parameter for path drawing and fading

	// list of all objects found in last scene (and before)
	list< Ptr<SceneObject> > pCategorized;
	list< Ptr<SceneObject> > pUncategorized;
	list< Ptr<SceneObject> > pMissing;
	list< pair< Ptr<SceneObject>,int > > pMissingCount; // used to count the time objects were missing
	list< Ptr<SceneObject> > pLost;

	vector<bool> pObjectTypesInScene; // if true, then the object type is in the scene and will be checked during classification update, index corresponds to class id (standard is that all types are considered to be occuring in the scene)

	double pActualTime; // time stamp of last calculated state

	unsigned int pTimeToRecalculation;

	// for classification purposes
	Average<double> pEstimatedClassificationTime; // [ms] Estimate of how long one classification step takes per object. Its startvalue is updated during runtime.
	
	// list of descriptor creators
	list<DescriptorCreator*> pDescriptorCreators;
	Ptr<ObjectHandler> pDescriptorSharePointer; // the shared pointer is being copied to the descriptor creators and allows them to check if the objecthandler object still exists or not
	
public:
	// other temporaries
	static double two_pi;
	static double pi;
	static double pi_half;
	static double pi_quarter;

private:
	double pThreshold;

	// general options temporaries
	static unsigned int recalculation_interval;
	static double min_area; //objects in video with smaller size are are not considered
	static double max_area; //objects in video with larger size are are not considered
	static double contour_length_switch; //objects in video with larger size are are not considered
	static unsigned int max_object_missing_time; // time an object shall be maximally missing
	static double max_matching_distance_mismatch; // distance an objects center can be away from its predicted center before the match is refused
	static bool use_old_state_fallback; // if true then before calculating an overlap estimate between predicted object position and found contours, the old position will be checked if nothing was found at the predicted center position (this is one fast fallback option to handle possible prediction overshoots for the center point) {affects: ObjectHandler}
	static double max_object_gridpoint_distance; // [px] max distance between grid points in grid point generation over predicted object surface used to calculate surface overlap area estimation if using the center point position prediction (and old position) failed to produce a match for a known object {affects:ObjectHandler}
	static unsigned int static_threshold; // [color] determines which color value will be used in the object detection algorithm when thresholding the image to obtain a binary black and white
	static unsigned int window_border_range; // [px] defines the distance from the window border which is considered as the leaving area for objects: if an object is lost in this area, it is considered to have left the visible area
	static int path_length; // -2: draw all full paths into central image (faster than opt -1), -1: draw full paths per object (paths of vanished objects disappear with the object), 0: don't draw paths, >0: length of the drawn paths [frames back] (becomes slower with length, will only work if trace_states is true) {affects: ObjectHandler}
	static int path_fadeout_speed; // for options -1 and -2 a fadeout can be set: the higher path_fadeout_speed is, the slower the fadeout, negative values lead to no fadeout at all. No fadeout is computationally the fastest, if a fadeout is set then the slower the fadeout, the fewer computation time is needed {affects: ObjectHandler}
	static int missing_state_bridging; // indicates if "artificial" states shall be added to objects at instants in time when they are missing, possible values are 0:no bridging (leads to 'holes' in state list of objects whenever it was missing), 1: successively keep predicting next state, based on previous prediction and use these to fill the void , 2: use the last known (old) state for bridging {affects: ObjectHandler}
	static bool time_awareness; // If true, then the classification functions keeps track of the remaining time postpones further classifications if the estimated classification time exceeds the actually available time. If in a feature point creation procedure all descriptors of all states during a recording time span are to be recorded, this should be deactivated. {affects:ObjectHandler}
	static double time_overhead; // [ms] How much time should be left after the classification processes for further processing
	static bool create_threshold_detection_image; // if true then the output of the last threshold operation with contours of detected objects is created and can be accessed through thresholdImage() in ObjectHandler {affects: ObjectHandler}
	static bool draw_predicted_regions; // if true then the predicted region rectangles are drawn into the threshold detection image (if the latter is created, that is) {affects: ObjectHandler}
};
