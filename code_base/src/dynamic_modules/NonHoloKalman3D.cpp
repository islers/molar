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

#include "NonHoloKalman3D.h"


NonHoloKalman3D::NonHoloKalman3D( Ptr<GenericObject> _objectPointer ):NonHoloKalman2D(_objectPointer)
{
	pHasAlreadyPredicted = false;
	pEstimator.statePost.create( 7,1,CV_32FC1 );
	pEstimator.statePre.create( 7,1,CV_32FC1 );
	pIsInitialized = false;

	initializeKalman();
}


NonHoloKalman3D::~NonHoloKalman3D(void)
{
}


Ptr<SceneObject::State> NonHoloKalman3D::rawState( double _rawX, double _rawY, double _time, double _rawAngle, double _length, double /*_rawArea*/ ) const
{
	Ptr<NonHoloKalman3D::State> rawState = new State();

	rawState->raw_x = _rawX;
	rawState->raw_y = _rawY;
	rawState->raw_angle = _rawAngle;
	rawState->type = classId;
	rawState->time = _time;
	rawState->length = _length;	

	
	/*rawState->x = rawState->raw_x;
	rawState->y = rawState->raw_y;
	rawState->angle = rawState->raw_angle;*/

	return rawState;
}


Ptr<GenericObject::Dynamics> NonHoloKalman3D::sfmCreator( Ptr<GenericObject> _objectPointer )
{
	return new NonHoloKalman3D(_objectPointer);
}


void NonHoloKalman3D::setOptions( GenericMultiLevelMap<string>& /*_dynamicsOptions*/ )
{
	// the class has currently no options that could be set
	return;
}


void NonHoloKalman3D::getOptions( GenericMultiLevelMap<string>& /*_optionLink*/ )
{
	// empty since the class has no options
	return;
}


Ptr<SceneObject::State> NonHoloKalman3D::newState( Mat& _state, RectangleRegion& _region, vector<Point>& /*_contour*/, Mat& _invImg, double _time )
{
	
	predict();
	Angle conv(_state.at<float>(2));
	conv.toClosestHalfEquivalent(pEstimator.statePost.at<float>(2) );// choosing last angle instead of predicted(otherwhise too high angle velocities can maintain themselves) ( pEstimator.statePre.at<float>(2) );
	conv.toZero2Pi();

	
	bool imageBorderTouched = pObjectPointer->touchesImageBorder();

	double length = 0;
	// update rod geometry estimates - freeze properties if state infered from group image or from object at image border
	if( !_invImg.empty() && !imageBorderTouched ) // the inverse image is empty if the state was infered from a group
	{
		length = _region.width();
		double diameter = _region.height();
		if( length>pLengthMax ) pLengthMax = length;
		pWidth = diameter;

	}
	else // freeze geometric properties
	{
		if( pHistory().size()!=0 ) // if previous states exist
		{
			if( pHistory().back()->type==classId )
			{
				Ptr<State> lastState = pHistory().back();
				length = lastState->length;
			}
		}
		// else theta is unknown, assume robot in image plane (theta=0)
		
	}

	Ptr<SceneObject::State> newState = rawState( _state.at<float>(0), _state.at<float>(1), _time, conv.rad(), length, 0.0 );
	return newState;
}



bool NonHoloKalman3D::addState( Ptr<SceneObject::State> _newState, RectangleRegion& _regionOfInterest, vector<Point>& _contour )
{
	if( _newState->type != classId ) return false;

	Ptr<State> newState = _newState;

	// find closest angle equivalent of measured angle to predicted angle in order to get the correct prediction error of the Kalman filter ->already done when creating the state in newState(...)
	

	// first initialization phase or if geometry isn't initialized yet
	if( !pIsInitialized || pLengthMax<=0 || pWidth==(double)0 )
	{
		if( pHistory().size()==0 )
		{
			// initialize velocities with 0
			pEstimator.statePost.at<float>(4) = 0;
			pEstimator.statePost.at<float>(5) = 0;
			pEstimator.statePost.at<float>(6) = 0;
		}
		else // pHistory.size()==1
		{
			double velX = newState->raw_x - pHistory().back()->x;
			double velY = newState->raw_y - pHistory().back()->y;
			double velAbs = sqrt( velX*velX + velY*velY );
			pEstimator.statePost.at<float>(4) = velAbs;
			double angSpeed = SceneObject::calcAngleVelocity( pHistory().back()->angle, newState->raw_angle );
			pEstimator.statePost.at<float>(5) = angSpeed;
			pEstimator.statePost.at<float>(6) = 0;

			pIsInitialized = true;
		}
		pEstimator.statePost.at<float>(0) = newState->raw_x;
		pEstimator.statePost.at<float>(1) = newState->raw_y;
		pEstimator.statePost.at<float>(2) = newState->raw_angle;
		pEstimator.statePost.at<float>(3) = 0;

		newState->x = newState->raw_x;
		newState->y = newState->raw_y;
		newState->angle = newState->raw_angle;
	}
	else
	{
		Angle measAng( newState->raw_angle );
		Mat measurement = ( Mat_<float>(4,1) << newState->raw_x, newState->raw_y, measAng.closestEquivalent(pEstimator.statePre.at<float>(2)).rad(), newState->length );
	
		// run prediction if not yet done
		predict();
		
		// calculate measurement prediction
		double sinTheta = sin( pEstimator.statePre.at<float>(3) );
		double cosTheta = cos( pEstimator.statePre.at<float>(3) );

		double predictedLength = pLengthMax*cosTheta + pWidth*sinTheta;
		Mat measPred = ( Mat_<float>(4,1) << pEstimator.statePre.at<float>(0), pEstimator.statePre.at<float>(1), pEstimator.statePre.at<float>(2), predictedLength );
	
		// update measurement matrix H(k) and measurement noise transition matrix M(k) ->only one value changes for this class
		pEstimator.measurementMatrix.at<float>(3,3) = -pLengthMax*sinTheta + pWidth*cosTheta;
		
		// measurement update

		pEstimator.correct( measurement, measPred ); // noise model for measurement update is unchanged for this class since M(k) is constant

		
		double thetaPost = pEstimator.statePost.at<float>(3);
		bool directionInverse = false;

		// bound theta to [0...pi/2]
		if( thetaPost>Angle::pi_half )
		{
			thetaPost = Angle::pi-thetaPost;
			directionInverse = true;
			pEstimator.statePost.at<float>(6) = -pEstimator.statePost.at<float>(6);
		}
		else if( thetaPost<0 )
		{
			thetaPost = -thetaPost;
			pEstimator.statePost.at<float>(6) = -pEstimator.statePost.at<float>(6);
		}

		pEstimator.statePost.at<float>(3) = thetaPost;
		
	
		Angle conv( pEstimator.statePost.at<float>(2) );
		// flip angle orientation if velocity is negative (which means that the orientation vector points in the wrong direction) or a rotation through the image normal occured
		
		if( directionInverse )
		{
			conv++;
		}
		else if( pEstimator.statePost.at<float>(4)<0 )
		{
			conv++;
			pEstimator.statePost.at<float>(4) = -pEstimator.statePost.at<float>(4);
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

Mat NonHoloKalman3D::predictState()
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


void NonHoloKalman3D::resetPrediction()
{
	initializeKalman(); //reset Kalman filter
	return;
}


Point NonHoloKalman3D::predictPosition( Point _pt )
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

double NonHoloKalman3D::predictXCtrVelocity()
{
	if( pHistory().size()==0 ) return 0;
	
	predict();

	return cos(pEstimator.statePre.at<float>(2))*pEstimator.statePre.at<float>(4);
}

double NonHoloKalman3D::predictYCtrVelocity()
{
	if( pHistory().size()==0 ) return 0;
	
	predict();

	return sin(pEstimator.statePre.at<float>(2))*pEstimator.statePre.at<float>(4);
}

double NonHoloKalman3D::predictAgl()
{
	if( pHistory().size()==0 ) return 0;
	predict();

	return pEstimator.statePre.at<float>(2);
}

double NonHoloKalman3D::predictAglVelocity()
{
	if( pHistory().size()==0 ) return 0;
	
	predict();

	return pEstimator.statePre.at<float>(5);
}


void NonHoloKalman3D::initializeKalman()
{
	pHasAlreadyPredicted = false;

	pEstimator.transitionMatrix = ( Mat_<float>(7,7) << 1,0,0,0,0,0,0,  0,1,0,0,0,0,0,  0,0,1,0,0,1,0,  0,0,0,1,0,0,1,  0,0,0,0,1,0,0, 0,0,0,0,0,1,0, 0,0,0,0,0,0,1 ); // changes at each step for the extended kalman filter, just to setup the matrix size
	pEstimator.measurementMatrix = ( Mat_<float>(4,7) << 1,0,0,0,0,0,0,  0,1,0,0,0,0,0,  0,0,1,0,0,0,0, 0,0,0,1,0,0,0 ); // changes at each step
	pEstimator.processNoiseCov = ( Mat_<float>(7,7) << 1.5,0,0,0,0,0,0, 0,1.5,0,0,0,0,0, 0,0,0.35,0,0,0,0, 0,0,0,0.6,0,0,0, 0,0,0,0,4,0,0, 0,0,0,0,0,2,0, 0,0,0,0,0,0,2 );
	pEstimator.measurementNoiseCov = ( Mat_<float>(4,4) << 2,0,0,0, 0,2,0,0, 0,0,0.1,0, 0,0,0,9 ); // 0.005,0,0, 0,0.005,0, 0,0,0.0001 );
	pEstimator.errorCovPost = ( Mat_<float>(7,7) << 0.005,0,0,0,0,0,0, 0,0.005,0,0,0,0,0, 0,0,0.0001,0,0,0,0, 0,0,0,5,0,0,0, 0,0,0,0,10,0,0, 0,0,0,0,0,1,0, 0,0,0,0,0,0,1 );
	pEstimator.errorCovPre.create(7,7, CV_32FC1);
	
	pEstimator.processNoiseTransMatrix = ( Mat_<float>(7,7) << 1,0,0,0,1,0,0, 0,1,0,0,1,0,0, 0,0,1,0,0,1,0, 0,0,0,1,0,0,1, 0,0,0,0,1,0,0, 0,0,0,0,0,1,0, 0,0,0,0,0,0,1 ); // changes every step
	pEstimator.measurementNoiseTransMatrix = (Mat_<float>(4,4) << 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1 );
	
	if( pHistory().size()!=0 )
	{
		double theta = 0;
		double velTheta = 0;
		
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
		pEstimator.statePost = ( Mat_<float>(7,1) << pHistory().back()->x, pHistory().back()->y, pHistory().back()->angle, theta, velAbs, velAng, velTheta );
	}
	else
	{
		pEstimator.statePost = ( Mat_<float>(7,1) << 0,0,0,0,0,0,0 );
	}

	return;
}


void NonHoloKalman3D::predict()
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
		pEstimator.statePre.at<float>(5) = pEstimator.statePost.at<float>(5);
		pEstimator.statePre.at<float>(6) = pEstimator.statePost.at<float>(6);
	}
	else
	{

		//state prediction
		pEstimator.statePre.at<float>(0) = pEstimator.statePost.at<float>(0) + pEstimator.statePost.at<float>(4)*cos( pEstimator.statePost.at<float>(2) )*cos( pEstimator.statePost.at<float>(3) );
		pEstimator.statePre.at<float>(1) = pEstimator.statePost.at<float>(1) + pEstimator.statePost.at<float>(4)*sin( pEstimator.statePost.at<float>(2) )*cos( pEstimator.statePost.at<float>(3) );
		
		double predTheta = pEstimator.statePost.at<float>(3) + pEstimator.statePost.at<float>(6);

		bool directionInverse = false;
		// bound theta to [0...pi/2]
		if( predTheta>Angle::pi_half )
		{
			predTheta = Angle::pi-predTheta;
			directionInverse = true;
			pEstimator.statePre.at<float>(6) = -pEstimator.statePost.at<float>(6);
		}
		else if( predTheta<0 )
		{
			predTheta = -predTheta;
			pEstimator.statePre.at<float>(6) = -pEstimator.statePost.at<float>(6);
		}
		else
		{
			pEstimator.statePre.at<float>(6) = pEstimator.statePost.at<float>(6);
			//predTheta = predTheta;
		}
		pEstimator.statePre.at<float>(3) = predTheta;
		
		Angle conv( pEstimator.statePost.at<float>(2) + pEstimator.statePost.at<float>(5) );

		if( pEstimator.statePost.at<float>(4) < 0 || directionInverse )
		{
			conv++;
			pEstimator.statePost.at<float>(4) *= -1;
		}
		conv.toZero2Pi();
		pEstimator.statePre.at<float>(2) = conv.rad();
		pEstimator.statePre.at<float>(4) = pEstimator.statePost.at<float>(4);
		pEstimator.statePre.at<float>(5) = pEstimator.statePost.at<float>(5);

		//calculate transition matrices
		setTransitionMatrices( pEstimator.statePost.at<float>(4), pEstimator.statePost.at<float>(2), pEstimator.statePost.at<float>(3) );

		// state covariance prediction
		pEstimator.predict(false); // noise model changed
	}

	pHasAlreadyPredicted = true;
	return;
}


void NonHoloKalman3D::setTransitionMatrices( double _vel, double _angle, double _theta )
{
	double sinPhi = sin(_angle);
	double cosPhi = cos(_angle);
	double cosTheta = cos(_theta);
	double sinTheta = sin(_theta);

	// update only the non-constant values instead of overwriting the whole matrix at every time step
	pEstimator.transitionMatrix.at<float>(0,2) = -_vel*sinPhi*cosTheta;
	pEstimator.transitionMatrix.at<float>(0,3) = -_vel*cosPhi*sinTheta;
	pEstimator.transitionMatrix.at<float>(0,4) = cosPhi*cosTheta;
	pEstimator.transitionMatrix.at<float>(1,2) = _vel*cosPhi*cosTheta;
	pEstimator.transitionMatrix.at<float>(1,3) = -_vel*sinPhi*sinTheta;
	pEstimator.transitionMatrix.at<float>(1,4) = sinPhi*cosTheta;

	pEstimator.processNoiseTransMatrix = pEstimator.transitionMatrix;
	
	return;
}




int NonHoloKalman3D::registerFactoryFunction()
{
	string info = "Filters and predicts states using an Extended Kalman filter based on equations for non-holonomic motion for an object for which its axis and the movement direction are aligned. Assumes constant velocities for position and angle. The dynamics are targeted at a rod-like object and estimates its angle relative to the image plane.";
	return Dynamics::registerDynamics( DynamicsFacEntry( "NonHoloKalman3D", &sfmCreator, info ) );
}



int NonHoloKalman3D::classId = registerFactoryFunction();




NonHoloKalman3D::State::State()
{
	type = classId;
}


NonHoloKalman3D::State::~State()
{

}

NonHoloKalman3D::State::State( double _x, double _y, double time, double _angle, double _area ):FilteredDynamics::State( _x, _y, time, _angle, _area )
{
	type=classId;
	length = 0;
}

NonHoloKalman3D::State::State( Point _pos, double time, double _angle, double _area ):FilteredDynamics::State( _pos, time, _angle, _area )
{
	type=classId;
	length = 0;
}
