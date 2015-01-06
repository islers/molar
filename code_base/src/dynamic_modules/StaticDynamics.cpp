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

#include "StaticDynamics.h"


StaticDynamics::StaticDynamics( Ptr<GenericObject> _objectPointer ):Dynamics(_objectPointer)
{
	pLastX = 0;
	pLastY = 0;
	pLastAngle = 0;
}


StaticDynamics::~StaticDynamics(void)
{
}


Ptr<GenericObject::Dynamics> StaticDynamics::factoryFunction( Ptr<GenericObject> _objectPointer )
{
	return new StaticDynamics(_objectPointer);
}



bool StaticDynamics::addState( Ptr<SceneObject::State> _newState, RectangleRegion& _regionOfInterest, vector<Point>& _contour )
{
	pLastX = _newState->x;
	pLastY = _newState->y;
	pLastAngle = _newState->angle;

	pHistory().push_back( _newState );

	if( !trace_states() )
	{
		while( pHistory().size()>3 ) pHistory().pop_front();
	}

	_regionOfInterest.points( pROI() );
	pLastContour() = _contour;
	
	return true;
}


void StaticDynamics::setOptions( GenericMultiLevelMap<string>& /*_dynamicsOptions*/ )
{
	// the class has currently no options that could be set
	return;
}


void StaticDynamics::getOptions( GenericMultiLevelMap<string>& /*_optionLink*/ )
{
	// empty since the class has no options
	return;
}


string StaticDynamics::info()
{
	return "Describes a static object.";
}


void StaticDynamics::predictROI( vector<Point>& _roi )
{
	_roi.push_back( predictPosition( pROI()[0] ) );
	_roi.push_back( predictPosition( pROI()[1] ) );
	_roi.push_back( predictPosition( pROI()[2] ) );
	_roi.push_back( predictPosition( pROI()[3] ) );
	return;
}

Mat StaticDynamics::predictState()
{
	if( pHistory().size()==0 ) return Mat();
	else
	{
		Mat prediction;
		prediction.create(1,3,CV_32FC1);
		prediction.at<float>(0)=pLastX;
		prediction.at<float>(1)=pLastY;
		prediction.at<float>(2)=pLastAngle;
		return prediction;
	}
}


void StaticDynamics::resetPrediction()
{
	pTimeSincePredictionReset()=0; // if 0: resetPrediction immediately affects all predictions, if -1: resetPrediction affects all predictions after the next SceneObject::State was added
	return;
}


Point StaticDynamics::predictPosition( Point _pt )
{
	double angleVel = predictAglVelocity();

	Point predictedCenter = predictCtrPosition();
	Point center( (int)pHistory().back()->x, (int)pHistory().back()->y );

	Point relPos = _pt-center;

	double cosEl = cos(angleVel);
	double sinEl = sin(angleVel);
	double newRelX = cosEl*relPos.x - sinEl*relPos.y;
	double newRelY = sinEl*relPos.x + cosEl*relPos.y;

	Point newRelPos( (int)newRelX, (int)newRelY );

	return predictedCenter+newRelPos;
}

Point StaticDynamics::predictCtrPosition()
{
	if( pHistory().size()==0 ) return Point(0,0);
	else return Point( (int)pLastX, (int)pLastY );
}

Point2f StaticDynamics::predictCtrFPosition()
{
	if( pHistory().size()==0 ) return Point(0,0);
	else return Point( pLastX, pLastY );
}

double StaticDynamics::predictXCtrVelocity()
{
	return 0;
}

double StaticDynamics::predictXCtrAcceleration()
{
	return 0;
}

double StaticDynamics::predictYCtrVelocity()
{
	return 0;
}

double StaticDynamics::predictYCtrAcceleration()
{
	return 0;
}

double StaticDynamics::predictAgl()
{
	return 0;
}

double StaticDynamics::predictAglVelocity()
{
	return 0;
}

double StaticDynamics::predictAglAcceleration()
{
	return 0;
}

double StaticDynamics::predictArea()
{
	if( pHistory().size()==0 ) return 0;
	else return pHistory().back()->area;
}




int StaticDynamics::registerFactoryFunction()
{
	string info = "Describes a static, non-moving object.";
	return Dynamics::registerDynamics( DynamicsFacEntry( "Static", &factoryFunction, info ) );
}



int StaticDynamics::classId = registerFactoryFunction();
