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

#include "FreeKalman.h"


FreeKalman::FreeKalman( Ptr<GenericObject> _objectPointer ):FilteredDynamics(_objectPointer)
{
	initializeKalman();
}


FreeKalman::~FreeKalman(void)
{
}


Ptr<GenericObject::Dynamics> FreeKalman::sfmCreator( Ptr<GenericObject> _objectPointer )
{
	return new FreeKalman(_objectPointer);
}


void FreeKalman::setOptions( GenericMultiLevelMap<string>& /*_dynamicsOptions*/ )
{
	// the class has currently no options that could be set
	return;
}


void FreeKalman::getOptions( GenericMultiLevelMap<string>& /*_optionLink*/ )
{
	// empty since the class has no options
	return;
}


Ptr<SceneObject::State> FreeKalman::newState( Mat& _state, RectangleRegion& /*_region*/, vector<Point>& /*_contour*/, Mat& /*_invImg*/, double _time )
{
	predict();
	Angle conv(_state.at<float>(2));
	conv.toClosestHalfEquivalent( pEstimator.statePre.at<float>(2) );
	conv.toZero2Pi();

	Ptr<SceneObject::State> newState = rawState( _state.at<float>(0), _state.at<float>(1), _time, conv.rad(), 0.0 );
	return newState;
}



bool FreeKalman::addState( Ptr<SceneObject::State> _newState, RectangleRegion& _regionOfInterest, vector<Point>& _contour )
{
	if( !isFiltered(_newState) ) return false;

	Ptr<FilteredDynamics::State> newState = _newState;

	// find closest angle equivalent of measured angle to predicted angle in order to get the correct prediction error of the Kalman filter ->already done when creating the state in newState(...)
	
	Mat measurement = ( Mat_<float>(3,1) << newState->raw_x, newState->raw_y, newState->raw_angle );
	
	// run prediction if not yet done
	predict();
	
	// measurement update
	pEstimator.correct( measurement );
	pHasAlreadyPredicted = false;
	
	// readjust angle into range (0...2Pi)
	Angle conv( pEstimator.statePost.at<float>(2) );
	conv.toZero2Pi();
	pEstimator.statePost.at<float>(2) = conv.rad();
	
	// save estimated state
	newState->x = pEstimator.statePost.at<float>(0);
	newState->y = pEstimator.statePost.at<float>(1);
	newState->angle = conv.rad();

	pHistory().push_back( newState );

	if( !trace_states() )
	{
		while( pHistory().size()>3 ) pHistory().pop_front();
	}

	_regionOfInterest.points( pROI() );
	pLastContour() = _contour;

	if( pTimeSincePredictionReset()>=-1 ) pTimeSincePredictionReset()++;
	if( pTimeSincePredictionReset() > 1 ) pTimeSincePredictionReset()=-1; // prediction reset has no effect anymore
	
	return true;
}

void FreeKalman::predictROI( vector<Point>& _roi )
{
	_roi.push_back( predictPosition( pROI()[0] ) );
	_roi.push_back( predictPosition( pROI()[1] ) );
	_roi.push_back( predictPosition( pROI()[2] ) );
	_roi.push_back( predictPosition( pROI()[3] ) );
	return;
}

Mat FreeKalman::predictState()
{
	if( pHistory().size()==0 ) return Mat();
    // let Kalman filter do prediction

	predict();

	Mat prediction;
	prediction.create(1,3,CV_32FC1);
	prediction.at<float>(0) = pEstimator.statePre.at<float>(0);
	prediction.at<float>(1) = pEstimator.statePre.at<float>(1);
	prediction.at<float>(2) = pEstimator.statePre.at<float>(2);
	return prediction;
}


void FreeKalman::resetPrediction()
{
	initializeKalman(); //reset Kalman filter
	return;
}


Point FreeKalman::predictPosition( Point _pt )
{
	predict();

	Point center( (int)pHistory().back()->x, (int)pHistory().back()->y );

	Point relPos = _pt-center;

	double cosEl = cos(pEstimator.statePre.at<float>(5));
	double sinEl = sin(pEstimator.statePre.at<float>(5));
	double newRelX = cosEl*relPos.x - sinEl*relPos.y;
	double newRelY = sinEl*relPos.x + cosEl*relPos.y;

	Point newRelPos( (int)newRelX, (int)newRelY );

	return Point( pEstimator.statePre.at<float>(0),pEstimator.statePre.at<float>(1) )+newRelPos;
}

Point FreeKalman::predictCtrPosition()
{
	if( pHistory().size()==0 ) return Point(0,0);
	
	predict();

	return Point( pEstimator.statePre.at<float>(0), pEstimator.statePre.at<float>(1) );
}

Point2f FreeKalman::predictCtrFPosition()
{
	if( pHistory().size()==0 ) return Point(0,0);
	
	predict();

	return Point2f( pEstimator.statePre.at<float>(0), pEstimator.statePre.at<float>(1) );
}

double FreeKalman::predictXCtrVelocity()
{
	if( pHistory().size()==0 ) return 0;
	
	predict();

	return pEstimator.statePre.at<float>(3);
}

double FreeKalman::predictXCtrAcceleration() // since these Dynamics assume constant velocities, this function should return 0. Instead however, the acceleration given by the smoothened states is calculated and returned, assuming constant acceleration
{
	return accX();
}

double FreeKalman::predictYCtrVelocity()
{
	if( pHistory().size()==0 ) return 0;
	
	predict();

	return pEstimator.statePre.at<float>(4);
}

double FreeKalman::predictYCtrAcceleration() // since these Dynamics assume constant velocities, this function should return 0. Instead however, the acceleration given by the smoothened states is calculated and returned, assuming constant acceleration
{
	return accY();
}

double FreeKalman::predictAgl()
{
	if( pHistory().size()==0 ) return 0;
	predict();

	return pEstimator.statePre.at<float>(2);
}

double FreeKalman::predictAglVelocity()
{
	if( pHistory().size()==0 ) return 0;
	
	predict();

	return pEstimator.statePre.at<float>(5);
}

double FreeKalman::predictAglAcceleration() // since these Dynamics assume constant velocities, this function should return 0. Instead however, the acceleration given by the smoothened states is calculated and returned, assuming constant acceleration
{
	return accAng();
}

double FreeKalman::predictArea()
{
	return area();
}


void FreeKalman::initializeKalman()
{
	pHasAlreadyPredicted = false;

	pEstimator.init( 6,3,0,CV_32F );
	pEstimator.transitionMatrix = ( Mat_<float>(6,6) << 1,0,0,1,0,0,  0,1,0,0,1,0,  0,0,1,0,0,1,  0,0,0,1,0,0,  0,0,0,0,1,0,  0,0,0,0,0,1 );
	pEstimator.measurementMatrix = ( Mat_<float>(3,6) << 1,0,0,0,0,0,  0,1,0,0,0,0,  0,0,1,0,0,0 );
	pEstimator.processNoiseCov = ( Mat_<float>(6,6) << 1.5,0,0,0,0,0, 0,1.5,0,0,0,0, 0,0,0.8,0,0,0, 0,0,0,5,0,0, 0,0,0,0,5,0, 0,0,0,0,0,3 );
	pEstimator.measurementNoiseCov = ( Mat_<float>(3,3) << 0.005,0,0, 0,0.005,0, 0,0,0.0001 );
	pEstimator.errorCovPost = ( Mat_<float>(6,6) << 0.005,0,0,0,0,0, 0,0.005,0,0,0,0, 0,0,0.0001,0,0,0, 0,0,0,3,0,0, 0,0,0,0,3,0, 0,0,0,0,0,1 );
	
	if( pHistory().size()!=0 )
	{
		float velX, velY, velA;
		if( pHistory().size()>=2 )
		{
			velX = pHistory().back()->x - pHistory()[ pHistory().size()-2 ]->x;
			velY = pHistory().back()->y - pHistory()[ pHistory().size()-2 ]->y;
			velA = calcAngleVelocity( pHistory()[ pHistory().size()-2 ]->angle, pHistory().back()->angle );
		}
		else
		{
			velX = 0;
			velY = 0;
			velA = 0;
		}
		pEstimator.statePost = ( Mat_<float>(6,1) << pHistory().back()->x, pHistory().back()->y, pHistory().back()->angle, velX, velY, velA );
	}


	return;
}


void FreeKalman::predict()
{
	if( pHasAlreadyPredicted ) return;

	pEstimator.predict();
	pHasAlreadyPredicted = true;
	return;
}




int FreeKalman::registerFactoryFunction()
{
	string info = "Filters and predicts states using a Kalman filter. The model assumes constant velocities, the state variables x, y and angle position as well as their velocities are uncorrelated.";
	return Dynamics::registerDynamics( DynamicsFacEntry( "FreeKalman", &sfmCreator, info ) );
}



int FreeKalman::classId = registerFactoryFunction();
