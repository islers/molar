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
#include "GenericObject.h"
#include "objecthandler.h"

/** abstract base class to provide dynamic functionalities for generic objects - used for more flexibility, that is
	changing between different dynamic types
	*/

class GenericObject::Dynamics
{
protected:
	class DynamicsFacEntry;
public:

	// factory access function
	Dynamics( Ptr<GenericObject> _objectPointer );
	static Ptr<Dynamics> createDynamics( const string& _dynamicsType, Ptr<GenericObject> _objectPointer );
	// initializess standard settings
	static void setStandardSettings( const string& _dynamicsType, GenericMultiLevelMap<string>& _target );


	// INTERFACE TO GENERIC OBJECTS //////////////////////////////////////////////////////////////////////
	virtual void setOptions( GenericMultiLevelMap<string>& _dynamicsOptions ) =0;
	virtual void getOptions( GenericMultiLevelMap<string>& _optionLink ) =0;

	/** returns information about the Dynamics type implemented */
	//virtual string info() =0;


	virtual Ptr<SceneObject::State> newState( Mat& _state, RectangleRegion& _region, vector<Point>& _contour, Mat& _invImg, double _time );
	virtual bool addState( Ptr<SceneObject::State> _newState, RectangleRegion& _regionOfInterest, vector<Point>& _contour );

	// STATE ACCESS

	/** prints all time data to the given file, using the given separators, returns true if no exception was thrown inside the function. If the file already exists, then old content is overwritten. */
	virtual bool exportData( string _filePath, string _dataPointSeparator=" ", string _timeStepSeparator="\n" );

	virtual Ptr<SceneObject::State> state(); // see SceneObject class for function descriptions
	virtual Mat history();
	virtual Mat currentState();
	virtual Point pos();
	virtual double angle();
	virtual Point2f direction();
	virtual double area();
	virtual double velX();
	virtual double velY();
	virtual double velAng();
	virtual double accX();
	virtual double accY();
	virtual double accAng();
	double calcAngleVelocity( double _angleBefore, double _angleAfter );
	virtual RectangleRegion lastROI();
	virtual void lastContour( vector<Point>& _contour );

	// PREDICTION
	virtual void predictROI( vector<Point>& _roi ) =0;
	virtual Mat predictState() =0;
	virtual void resetPrediction() =0;
	virtual Point predictPosition( Point _pt ) =0;
	virtual Point predictCtrPosition() =0;
	virtual Point2f predictCtrFPosition() =0;
	virtual double predictXCtrVelocity() =0;
	virtual double predictXCtrAcceleration() =0;
	virtual double predictYCtrVelocity() =0;
	virtual double predictYCtrAcceleration() =0;
	virtual double predictAgl() =0;
	virtual double predictAglVelocity() =0;
	virtual double predictAglAcceleration() =0;
	virtual double predictArea() =0;

	// DRAWING
	virtual bool drawLayer( Mat& _img, unsigned int _layerLevel, Scalar _color );
	virtual void drawPath( Mat& _img, Scalar _color );
	virtual void drawDirection( Mat& _img, Scalar _color );
	virtual void drawName( Mat& _img );

	
	// UTILITY FUNCTIONS ///////////////////////////////////////////////////////////
		
	/** returns a unit vector with the given direction
	*
	*	@param	_angle	angle in rad
	*/
	static vector<double> direction( double _angle );


	//////////////////////////////////////////////////////////////////////////////////////////////////////

	// INTERFACE TO GENERIC OBJECT (AND SCENE OBJECT) PRIVATES FOR CHILD CLASSES (SINCE THOSE ARE NO GO CLASS MEMBERS ANYMORE)
protected:
	deque< Ptr<SceneObject::State> >& pHistory();
	bool& trace_states();
	Point* pROI();
	vector<Point>& pLastContour();
	int& pTimeSincePredictionReset();
	Ptr<Mat>& pPathMat();
    int& pFramesSinceLastFade();


	// FACTORY
public:
	/** returns a new, unique dynamics module id, unrelated to entries in the dynamics list
	*  -> useful for pure virtual classes that add some functionality for all derived classes using the unique id to identify e.g. states
	*/
	static int getUniqueId();
	static int registerDynamics( DynamicsFacEntry _newEntry );
	/** returns a list with the names of all available dynamics classes (clears the vector if it isn't empty) */
	static void availableDynamics( vector<string>& _availableDynamics );
	/** returns information for all available dynamic types */
	static void dynamicsInfo( vector<string>& _dynamicsInfo );

protected:
	Ptr<GenericObject> pObjectPointer;

	static map< string , DynamicsFacEntry >* dynamicsList;

private:		
    static unsigned int idCounter;

};


class GenericObject::Dynamics::DynamicsFacEntry
{
public:
	DynamicsFacEntry();
	DynamicsFacEntry( string _name, Ptr<GenericObject::Dynamics>(*_creatorFunc)( Ptr<GenericObject> _objectPointer ), string _info, GenericMultiLevelMap<string> _options=GenericMultiLevelMap<string>() );
	
	string name();
	string info();
	Ptr<GenericObject::Dynamics> create( Ptr<GenericObject> _objectPointer );
	GenericMultiLevelMap<string> options();

private:
	string pName;
	string pInfo;
	Ptr<GenericObject::Dynamics> (*pCreatorFunc) ( Ptr<GenericObject> _objectPointer );
	GenericMultiLevelMap<string> pOptions;
};
