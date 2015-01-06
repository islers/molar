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

#include <deque>
#include <math.h>
#include "average.h"
#include "Options.h"
#include "RectangleRegion.h"

using namespace cv;
using namespace std;

class ObjectHandler; // forward declaration (for pointer usage)

class SceneObject
{

public:
	class State;

	SceneObject( ObjectHandler* _environmentControl );
	SceneObject( Ptr<SceneObject> );
	~SceneObject(void);

	static Ptr<SceneObject> newObject();


	// INFORMATION ///////////////////////////////////////////////////////////////
	/** returns the id of the object */
	unsigned int id();

	/** returns the class name and color */
	virtual void classProperties( string& _className, Scalar& _classColor );
	virtual Scalar color();
	

	/** returns true if the object touches the border of the image in which it was detected */
	virtual bool touchesImageBorder();


	/** builds a new state of the objects' type - for the base class only the state vector is used, but for children the other arguments might be handy for additional computations
	*
	*	@param	Mat& _state					standard state information in a Mat row ( centroid.x | centroid.y | angle direction )
	*	@param	RectangleRegion& _region	region where the object state was calculated, relative to the image _invImg (consider case where it is actually empty)
	*	@param	vector<Point> _contour		calculated contour of the object, relative to the image _invImg (consider case where it is actually empty)
	*	@param	Mat& _invImg				single-channel greyscale image in which the object was found (consider case where it is actually empty)
	*/
	virtual Ptr<SceneObject::State> newState( Mat& _state, RectangleRegion& _region, vector<Point>& _contour, Mat& _invImg, double _time );

	/** adds new state to the object*/
	virtual bool addState( Ptr<SceneObject::State> _newState, RectangleRegion& _regionOfInterest, vector<Point>& _contour );
	
	/** is called inside addState to actually add the state: allows child class objects to perform further processings while in the SceneObject parent class further functions may be called */
	virtual bool addStateProcessing( Ptr<SceneObject::State> _newState, RectangleRegion& _regionOfInterest, vector<Point>& _contour );


	/** returns the index of the region or/and area that is a best match for the object to lie in, -1 if no suitable match is found
	*/
	virtual int findMatchingArea( vector<RectangleRegion>& _regions, vector< vector<Point> > _contours);


	// STATE ACCESS ////////////////////////////////////////////////////////////////


	/** prints all time data to the given file, using the given separators, returns true if no exception was thrown inside the function. If the file already exists, then old content is overwritten. */
	virtual bool exportData( string _filePath, string _dataPointSeparator=" ", string _timeStepSeparator="\n" );

	/** returns the current state of the object (where state is not necessarily raw data but the best 'guess' about the state the program has
	*/
	virtual Ptr<SceneObject::State> state();

	/** returns the object states history as Mat
	*/
	virtual Mat history();

	/** returns Mat of last state */
	virtual Mat currentState();

	/** returns actual position */
	virtual Point pos();

	/** returns current angle of object */
	virtual double angle();

	/** returns current direction vector of object */
	virtual Point direction();

	/** returns current area of object */
	virtual double area();

	/** returns unfiltered, raw x velocity of object centre: 0 if unknown (not enough data points) */
	virtual double velX();

	/** returns unfiltered, raw y velocity of object centre: 0 if unknown (not enough data points) */
	virtual double velY();

	/** return unfiltered, raw angle velocity of object: 0 if unknown (not enough data points) */
	virtual double velAng();

	/** returns unfiltered, raw x acceleration of object centre: 0 if unknown (not enough data points) */
	virtual double accX();

	/** returns unfiltered, raw y acceleration of object centre: 0 if unknown (not enough data points) */
	virtual double accY();

	/** return unfiltered, raw angle acceleration of object: 0 if unknown (not enough data points) */
	virtual double accAng();

	/** calculates the angular velocity, taking into account that the angles' range is 0 to 2 pi */
	static double calcAngleVelocity( double _angleBefore, double _angleAfter );

	/** returns the value that corresponds to the same angle as _angle, but lies inside the range 0 to 2 pi */
	static double moveAngleIntoRange( double _angle );

	/** transform _angleToAdjust with 2 pi additions and subtraction to the closest version to _angleToApproach */
	static double findClosestAngleEquivalent( double _angleToAdjust, double _angleToApproach );

	/** returns the last region of interest */
	virtual RectangleRegion lastROI();

	/** returns the last contour that has been found (for objects that weren't found in previous frames it's the contour the object had the last time it was found ) */
	virtual void lastContour( vector<Point>& _contour );


	// PREDICTION ///////////////////////////////////////////////////////////////////

	/** predicts the region where the object might be found
	*
	*	@argument/return	vector<Point>	empty vector
	*/
	virtual void predictROI( vector<Point>& _roi );

	//simple acceleration based prediction (assuming acceleration is the same)
	virtual Mat predictState();

	/** tells the prediction engine to discard any dependencies of the prediction from old state and treat the object as if nothing about its history was known after the next state is added (thus allows a reset if movements occur that lead to a complete prediction failure)
	*/
	virtual void resetPrediction();
	
	// predicts the position of a point of the abf (absolute position)
	virtual Point predictPosition( Point _pt );

	virtual Point predictCtrPosition();
	virtual Point2f predictCtrFPosition();

	virtual double predictXCtrVelocity(); //center velocity
	virtual double predictXCtrAcceleration(); //center acceleration

	virtual double predictYCtrVelocity(); //center velocity
	virtual double predictYCtrAcceleration(); //center acceleration

	virtual double predictAgl();
	virtual double predictAglVelocity();
	virtual double predictAglAcceleration();

	virtual double predictArea();


	// DRAW FUNCTIONS /////////////////////////////////////////////////////////////////
	/** draws (all) object informations into the image
	*/
	virtual void draw( Mat& _img, Scalar _color );
	/** draws the object informations into the image that correspond to the given layer. This function is called in a way that it is guaranteed that a figure belonging to a higher level of one object is always drawn over a figure belonging to a lower level of another object. Figures belonging to the same level may be drawn over each other, depending on the time when the draw function of the corresponding object is called.
	*	@return		bool		true if the drawn layer level is the highest the object has (or higher)
	*/
	virtual bool drawLayer( Mat& _img, unsigned int _layerLevel, Scalar _color );
	virtual void drawPath( Mat& _img, Scalar _color );
	virtual void drawDirection( Mat& _img, Scalar _color );
	virtual void drawName( Mat& _img );

	// CLASSIFICATION ////////////////////////////////////////////////////////////////
	/** classification function
	*	
	* @param	_img				image
	* @param	_boundingRectangle	ROI inside image, relative to image
	* @param	_objectId			the id of the object for which the type test is performed
	* @param	_objectClassId		the id of the object class which performs the test: this parameter is necessary for generic classes
	* @return						likelihood for the region in the image to be of the objects' type, negative if unknown
	*/
	static double isType( Mat& _img, Ptr<Point> _boundingRect, int _objectId, int _objectClassId );

	/** finds objects believed to be of the given class in the image (unused so far)
	*/
	static vector< Ptr<SceneObject> >  findObjects( Mat& _image, Ptr<Point> _boundingRectangle=NULL );


	/** adds a newly calculated likelihood for the indicated class to the object: likelihood values are averaged over time
	*/
	virtual void addNewLikelihood( unsigned int _classId, double _likelihood );

	/** recalculates the type of the object based on its class likelihood vector (called after new likelihoods were loaded)
	*
	*	@arg	Ptr<SceneObject> _oldPointer	is added in order to keep the smart pointer alive if the object stays the same
	*
	*	@return		pointer to the object itself -> new instance of child type if the objects' type has changed
	*/
	virtual Ptr<SceneObject> recalculateClass( Ptr<SceneObject> _oldPointer );

	/** returns the id of the class the object is believed to be, -1 if unknown type , -2 if not classified yet
	*/
	int type();


	/** returns the id of the class with name _className, -1 if the class wasn't found */
	static int getClassId( string _className );


	// OBJECT REGISTRATION ///////////////////////////////////////////////////////////
    /** setup object list object */
    static void setupObjectList();

	/** function to register an object with the "factory"
	*
	*	@return		class identifier
	*/
	static int registerObject( std::string _name, Ptr<SceneObject> (* _factoryFunction)( SceneObject* ), double (*_classTest)( Mat& _img, RectangleRegion& _boundingRect, Mat& _descriptors , int _id, int _classId ), bool (*_initializerFunction)( int _classId, ObjectHandler* _environment ), void (*_setOptionsFunction)( GenericMultiLevelMap<string>& _options, int _classId ), void (*_getOptionsFunction)( GenericMultiLevelMap<string>& _options, int _classId ) );

	/** creates the static class objects */
	static bool createClassVectors();

	// OPTIONS SETUP
	static bool setupOptions();


	// UTILITY FUNCTIONS ///////////////////////////////////////////////////////////
		
	/** returns a unit vector with the given direction
	*
	*	@param	_angle	angle in rad
	*/
	static vector<double> direction( double _angle );

	static void resetObjectCount();

protected:
	ObjectHandler* pEnvironmentControl;

	public:deque< Ptr<State> > pHistory; // state history: newest state is added at the back
	Point pROI[4]; // last region of interest (min rotated rectangle that contains the object)
	vector<Point> pLastContour; // contour in last frame

	unsigned int pObjectId; // unique id of the object
	static unsigned int objectCount;
	int pType; // -1 if unknown type , -2 if not classified yet
	
	vector< Average<double> > pClassLikelihood;
	void ensureClassLikelihoodSize();

	int pTimeSincePredictionReset; // -1: reset the prediction and treat the next added state as the very first, -2: if no reset occured or it is that far in the past that it has no effect anymore

	vector<double> pAccEMA_S_old; // exponential moving average filter for acceleration prediction: S_(t-1)
	static double pAccEMA_alpha; // weighting factor for exponential moving average filter for acceleration prediction
	vector<double> pVelEMA_S_old; // exponential moving average filter for velocity prediction: S_(t-1)
	static double pVelEMA_alpha;


	// setup helper
	static bool pClassVectorsAreSetup;

	// option temporary
	static bool trace_states;
	static unsigned int path_step_length;
	static bool use_old_state_fallback; // if true then before calculating an overlap estimate between predicted object position and found contours, the old position will be checked if nothing was found at the predicted center position (this is one fast fallback option to handle possible prediction overshoots for the center point) {affects: ObjectHandler}
	static double max_object_gridpoint_distance; // [px] max distance between grid points in grid point generation over predicted object surface used to calculate surface overlap area estimation if using the center point position prediction (and old position) failed to produce a match for a known object {affects:ObjectHandler}
	

	Ptr<Mat> pPathMat; // Mat serves as a temporary memory for the objects' path (in order to get a linear drawing time that does not depend on the path's length)
    int pFramesSinceLastFade;

	// object list
public:
	static vector< std::string >* objectList; // object class name set - class id corresponds to position of its entities in information vectors
	static vector< double (*)( Mat& _img, RectangleRegion& _boundingRect, Mat& _descriptors, int _id, int _classId ) >* classifierList; // object class classifier function set
	static vector< Ptr<SceneObject> (*)( SceneObject* _proto ) >* factoryList; // object class factory function set
	static vector< bool (*)(int _classId, ObjectHandler* _environment)>* initializerList; // initializer functions for all registered functions: returns true if the class was successfully initialized and thus is ready to be used - are called only when the class is actually expected to occur in the scenery - thus functions registered here can be used to initialize anything that isn't needed and only uses memory for classes
	static vector< void (*)( GenericMultiLevelMap<string>& _options, int _classId )>* setOptionsFunctionList; // list with functions to set class properties
	static vector< void (*)( GenericMultiLevelMap<string>& _options, int _classId )>* getOptionsFunctionList; // list with functions to get the current class properties
};



class SceneObject::State
{
	public:
		State();
		~State();
		State( double _x, double _y, double time, double _angle=0, double _area=0, int _type=-1 );
		State( Point _pos, double time, double _angle=0, double _area=0, int _type=-1 );

		double x; // stores the best known guess about the current state
		double y;
		double angle; // [rad]
		double area;
		double time;
		
		virtual Point2f pos();

		int type;
};
