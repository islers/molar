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

#include "NonHoloKalman2D.h"


NonHoloKalman2D::NonHoloKalman2D( Ptr<GenericObject> _objectPointer ):FilteredDynamics(_objectPointer)
{
	pHasAlreadyPredicted = false;
	pEstimator.statePost.create( 5,1,CV_32FC1 );
	pEstimator.statePre.create( 5,1,CV_32FC1 );
	pIsInitialized = false;

	initializeKalman();
}


NonHoloKalman2D::~NonHoloKalman2D(void)
{
}


Ptr<GenericObject::Dynamics> NonHoloKalman2D::sfmCreator( Ptr<GenericObject> _objectPointer )
{
	return new NonHoloKalman2D(_objectPointer);
}


void NonHoloKalman2D::setOptions( GenericMultiLevelMap<string>& /*_dynamicsOptions*/ )
{
	// the class has currently no options that could be set
	return;
}


void NonHoloKalman2D::getOptions( GenericMultiLevelMap<string>& /*_optionLink*/ )
{
	// empty since the class has no options
	return;
}


Ptr<SceneObject::State> NonHoloKalman2D::newState( Mat& _state, RectangleRegion& /*_region*/, vector<Point>& /*_contour*/, Mat& /*_invImg*/, double _time )
{
	predict();
	Angle conv(_state.at<float>(2));
	conv.toClosestHalfEquivalent(pEstimator.statePost.at<float>(2) );// choosing last angle instead of predicted(otherwhise too high angle velocities can maintain themselves) ( pEstimator.statePre.at<float>(2) );
	conv.toZero2Pi();
	
	Ptr<SceneObject::State> newState = rawState( _state.at<float>(0), _state.at<float>(1), _time, conv.rad(), 0.0 );
	return newState;
}



bool NonHoloKalman2D::addState( Ptr<SceneObject::State> _newState, RectangleRegion& _regionOfInterest, vector<Point>& _contour )
{
	if( !isFiltered(_newState) ) return false;

	Ptr<FilteredDynamics::State> newState = _newState;

	// find closest angle equivalent of measured angle to predicted angle in order to get the correct prediction error of the Kalman filter ->already done when creating the state in newState(...)
	

	// first initialization phase
	if( !pIsInitialized )
	{
		if( pHistory().size()==0 )
		{
			pEstimator.statePost.at<float>(3) = 0;
			pEstimator.statePost.at<float>(4) = 0;
		}
		else // pHistory.size()==1
		{
			double velX = newState->raw_x - pHistory().back()->x;
			double velY = newState->raw_y - pHistory().back()->y;
			double velAbs = sqrt( velX*velX + velY*velY );
			pEstimator.statePost.at<float>(3) = velAbs;
			double angSpeed = SceneObject::calcAngleVelocity( pHistory().back()->angle, newState->raw_angle );
			pEstimator.statePost.at<float>(4) = angSpeed;

			pIsInitialized = true;
		}
		pEstimator.statePost.at<float>(0) = newState->raw_x;
		pEstimator.statePost.at<float>(1) = newState->raw_y;
		pEstimator.statePost.at<float>(2) = newState->raw_angle;

		newState->x = newState->raw_x;
		newState->y = newState->raw_y;
		newState->angle = newState->raw_angle;
	}
	else
	{
		Angle measAng( newState->raw_angle );
		Mat measurement = ( Mat_<float>(3,1) << newState->raw_x, newState->raw_y, measAng.closestEquivalent(pEstimator.statePre.at<float>(2)).rad() );
	
		// run prediction if not yet done
		predict();

		// calculate measurement prediction (nothing to be done for this class)
		Mat measPred = ( Mat_<float>(3,1) << pEstimator.statePre.at<float>(0), pEstimator.statePre.at<float>(1), pEstimator.statePre.at<float>(2) );
	
		// update measurement matrix H(k) and measurement noise transition matrix M(k) ->constant for this class

		// measurement update
		pEstimator.correct( measurement, measPred ); // noise model for measurement update is unchanged for this class since M(k) is constant
		
	
		Angle conv( pEstimator.statePost.at<float>(2) );
		// flip angle orientation if velocity is negative (which means that the orientation vector points in the wrong direction)
		if( pEstimator.statePost.at<float>(3)<0 )
		{
			conv++;
			pEstimator.statePost.at<float>(3) = -pEstimator.statePost.at<float>(3);
		}
		// readjust angle into range (0...2Pi)
		conv.toZero2Pi();
		pEstimator.statePost.at<float>(2) = conv.rad();
		
	
		// save estimated state
		newState->x = pEstimator.statePost.at<float>(0);
		newState->y = pEstimator.statePost.at<float>(1);
		newState->angle = conv.rad();
	}
	pHasAlreadyPredicted = false;

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

Mat NonHoloKalman2D::predictState()
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


void NonHoloKalman2D::resetPrediction()
{
	initializeKalman(); //reset Kalman filter
	return;
}


Point NonHoloKalman2D::predictPosition( Point _pt )
{
	predict();

	Point center( (int)pHistory().back()->x, (int)pHistory().back()->y );

	Point relPos = _pt-center;

	double cosEl = cos(pEstimator.statePre.at<float>(4));
	double sinEl = sin(pEstimator.statePre.at<float>(4));
	double newRelX = cosEl*relPos.x - sinEl*relPos.y;
	double newRelY = sinEl*relPos.x + cosEl*relPos.y;

	Point newRelPos( (int)newRelX, (int)newRelY );

	return Point( pEstimator.statePre.at<float>(0),pEstimator.statePre.at<float>(1) )+newRelPos;
}

Point NonHoloKalman2D::predictCtrPosition()
{
	if( pHistory().size()==0 ) return Point(0,0);
	
	predict();

	return Point( pEstimator.statePre.at<float>(0), pEstimator.statePre.at<float>(1) );
}

Point2f NonHoloKalman2D::predictCtrFPosition()
{
	if( pHistory().size()==0 ) return Point(0,0);
	
	predict();

	return Point2f( pEstimator.statePre.at<float>(0), pEstimator.statePre.at<float>(1) );
}

double NonHoloKalman2D::predictXCtrVelocity()
{
	if( pHistory().size()==0 ) return 0;
	
	predict();

	return cos(pEstimator.statePre.at<float>(2))*pEstimator.statePre.at<float>(3);
}

double NonHoloKalman2D::predictXCtrAcceleration() // since these Dynamics assume constant velocities, this function should return 0. Instead however, the acceleration given by the smoothened states is calculated and returned, assuming constant acceleration
{
	return accX();
}

double NonHoloKalman2D::predictYCtrVelocity()
{
	if( pHistory().size()==0 ) return 0;
	
	predict();

	return sin(pEstimator.statePre.at<float>(2))*pEstimator.statePre.at<float>(3);
}

double NonHoloKalman2D::predictYCtrAcceleration() // since these Dynamics assume constant velocities, this function should return 0. Instead however, the acceleration given by the smoothened states is calculated and returned, assuming constant acceleration
{
	return accY();
}

double NonHoloKalman2D::predictAgl()
{
	if( pHistory().size()==0 ) return 0;
	predict();

	return pEstimator.statePre.at<float>(2);
}

double NonHoloKalman2D::predictAglVelocity()
{
	if( pHistory().size()==0 ) return 0;
	
	predict();

	return pEstimator.statePre.at<float>(4);
}

double NonHoloKalman2D::predictAglAcceleration() // since these Dynamics assume constant velocities, this function should return 0. Instead however, the acceleration given by the smoothened states is calculated and returned, assuming constant acceleration
{
	return accAng();
}

double NonHoloKalman2D::predictArea()
{
	return area();
}


void NonHoloKalman2D::drawDirection( Mat& _img, Scalar _color )
{

	Ptr<SceneObject::State> current = pHistory().back();
	vector<double> myDirection = direction( current->angle );
	vector<double> myOrthDirection(2);
	myOrthDirection[0]=myDirection[1];
	myOrthDirection[1]=-myDirection[0];

	Point eins( current->x-30*myDirection[0], current->y-30*myDirection[1] );
	Point zwei( current->x+30*myDirection[0], current->y+30*myDirection[1] );
	Point drei( zwei.x-5*myOrthDirection[0]-8*myDirection[0], zwei.y-5*myOrthDirection[1]-8*myDirection[1] );
	Point vier( zwei.x+5*myOrthDirection[0]-8*myDirection[0], zwei.y+5*myOrthDirection[1]-8*myDirection[1] );

	line( _img, eins, zwei, _color, 2, 8 );
	line( _img, zwei, drei, _color, 2, 8 );
	line( _img, zwei, vier, _color, 2, 8 );

	return;
}


void NonHoloKalman2D::initializeKalman()
{
	pHasAlreadyPredicted = false;

	pEstimator.transitionMatrix = ( Mat_<float>(5,5) << 1,0,0,1,0,  0,1,0,0,0,  0,0,1,0,0,  0,0,0,1,0,  0,0,0,0,1 ); // changes at each step for the extended kalman filter, just to setup the matrix size
	pEstimator.measurementMatrix = ( Mat_<float>(3,5) << 1,0,0,0,0,  0,1,0,0,0,  0,0,1,0,0 );
	pEstimator.processNoiseCov = ( Mat_<float>(5,5) << 1.5,0,0,0,0, 0,1.5,0,0,0, 0,0,0.8,0,0, 0,0,0,4,0, 0,0,0,0,2 );
	pEstimator.measurementNoiseCov = ( Mat_<float>(3,3) << 2,0,0, 0,2,0, 0,0,0.6 );//2,0,0, 0,2,0, 0,0,0.0001 ); // 0.005,0,0, 0,0.005,0, 0,0,0.0001 );
	pEstimator.errorCovPost = ( Mat_<float>(5,5) << 0.005,0,0,0,0, 0,0.005,0,0,0, 0,0,0.0001,0,0, 0,0,0,3,0, 0,0,0,0,1 );
	pEstimator.errorCovPre.create(5,5, CV_32FC1);
	
	pEstimator.processNoiseTransMatrix = ( Mat_<float>(5,5) << 1,0,0,1,0, 0,1,0,0,0, 0,0,1,0,1, 0,0,0,1,0, 0,0,0,0,1 ); // changes every step
	pEstimator.measurementNoiseTransMatrix = (Mat_<float>(3,3) << 1,0,0, 0,1,0, 0,0,1 );
	
	if( pHistory().size()!=0 )
	{
		float velAng, velAbs;
		if( pHistory().size()>=2 )
		{
			double velX = pHistory().back()->x - pHistory()[ pHistory().size()-2 ]->x;
			double velY = pHistory().back()->y - pHistory()[ pHistory().size()-2 ]->y;
			velAbs = sqrt( velX*velX+velY*velY );
			velAng = calcAngleVelocity( pHistory()[ pHistory().size()-2 ]->angle, pHistory().back()->angle );
			pIsInitialized = true;
		}
		else
		{
			velAbs = 0;
			velAng = 0;
		}
		pEstimator.statePost = ( Mat_<float>(5,1) << pHistory().back()->x, pHistory().back()->y, pHistory().back()->angle, velAbs, velAng );
	}
	else
	{
		pEstimator.statePost = ( Mat_<float>(5,1) << 0,0,0,0,0 );
	}

	return;
}


void NonHoloKalman2D::predict()
{
	if( pHasAlreadyPredicted ) return;
	
	//extended kalman filter prediction step
	if( !pIsInitialized )
	{
		pEstimator.statePre.at<float>(0) = pEstimator.statePost.at<float>(0);
		pEstimator.statePre.at<float>(1) = pEstimator.statePost.at<float>(1);
		pEstimator.statePre.at<float>(2) = pEstimator.statePost.at<float>(2);
		pEstimator.statePre.at<float>(3) = pEstimator.statePost.at<float>(3);
		pEstimator.statePre.at<float>(4) = pEstimator.statePost.at<float>(4);
	}
	else
	{

		//state prediction
		pEstimator.statePre.at<float>(0) = pEstimator.statePost.at<float>(0) + pEstimator.statePost.at<float>(3)*cos( pEstimator.statePost.at<float>(2) );
		pEstimator.statePre.at<float>(1) = pEstimator.statePost.at<float>(1) + pEstimator.statePost.at<float>(3)*sin( pEstimator.statePost.at<float>(2) );
		Angle conv( pEstimator.statePost.at<float>(2) + pEstimator.statePost.at<float>(4) );

		if( pEstimator.statePost.at<float>(3) < 0 )
		{
			conv++;
			pEstimator.statePost.at<float>(3) *= -1;
		}
		conv.toZero2Pi();
		pEstimator.statePre.at<float>(2) = conv.rad();
		pEstimator.statePre.at<float>(3) = pEstimator.statePost.at<float>(3);
		pEstimator.statePre.at<float>(4) = pEstimator.statePost.at<float>(4);

		//calculate transition matrices
		setTransitionMatrices( pEstimator.statePost.at<float>(3), pEstimator.statePost.at<float>(2) );

		// state covariance prediction
		pEstimator.predict(false); // noise model changed
	}

	pHasAlreadyPredicted = true;
	return;
}


void NonHoloKalman2D::setTransitionMatrices( double _vel, double _angle)
{
	double sinPhi = sin(_angle);
	double cosPhi = cos(_angle);

	pEstimator.transitionMatrix = ( Mat_<float>(5,5) << 1,0,-_vel*sinPhi,cosPhi,0,  0,1,_vel*cosPhi,sinPhi,0,  0,0,1,0,1,  0,0,0,1,0,  0,0,0,0,1 ); // changes at each step for the extended kalman filter
	
	//pEstimator.processNoiseTransMatrix = ( Mat_<float>(5,5) << 1,0,0,cosPhi,0, 0,1,0,sinPhi,0, 0,0,1,0,1, 0,0,0,1,0, 0,0,0,0,1 ); // changes every step
	pEstimator.processNoiseTransMatrix = pEstimator.transitionMatrix;

	return;
}




int NonHoloKalman2D::registerFactoryFunction()
{
	string info = "Filters and predicts states using an Extended Kalman filter based on equations for non-holonomic motion for an object for which its axis and the movement direction are aligned. Assumes constant velocities for position and angle.";
	return Dynamics::registerDynamics( DynamicsFacEntry( "NonHoloKalman2D", &sfmCreator, info ) );
}



int NonHoloKalman2D::classId = registerFactoryFunction();
