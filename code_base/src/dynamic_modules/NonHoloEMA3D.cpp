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

#include "NonHoloEMA3D.h"


NonHoloEMA3d::NonHoloEMA3d( Ptr<GenericObject> _objectPointer ):NonHoloEMA(_objectPointer)
{
	pPosAlpha = 0.4;
	pAngAlpha = 0.8;
	pSwitchAlpha = 0.05;
	pDirectionSwitch = 0.5; // chosen measure for certainty about current direction (0...1) switch performed if it falls under 0.5
	pHasPredicted = false;
	pReflectionOccured = 0;
	pLengthMax=0;
}


NonHoloEMA3d::~NonHoloEMA3d(void)
{
}


Ptr<SceneObject::State> NonHoloEMA3d::rawState( double _rawX, double _rawY, double _time, double _rawAngle, double _rawTheta, double /*_rawArea*/ ) const
{
	Ptr<NonHoloEMA3d::State> rawState = new State();

	rawState->raw_x = _rawX;
	rawState->raw_y = _rawY;
	rawState->raw_angle = _rawAngle;
	rawState->type = classId;
	rawState->time = _time;
	rawState->thetaEstimated_raw = _rawTheta;

		
	return rawState;
}


Ptr<GenericObject::Dynamics> NonHoloEMA3d::sfmCreator( Ptr<GenericObject> _objectPointer )
{
	return new NonHoloEMA3d(_objectPointer);
}


void NonHoloEMA3d::setOptions( GenericMultiLevelMap<string>& _dynamicsOptions )
{
	if( _dynamicsOptions.hasKey("position_alpha") ) pPosAlpha = _dynamicsOptions["position_alpha"].as<double>();
	if( _dynamicsOptions.hasKey("angle_alpha") ) pAngAlpha = _dynamicsOptions["angle_alpha"].as<double>();
	if( _dynamicsOptions.hasKey("switch_alpha") ) this->pSwitchAlpha = _dynamicsOptions["switch_alpha"].as<double>();
	return;
}


void NonHoloEMA3d::getOptions( GenericMultiLevelMap<string>& _optionLink )
{
	_optionLink["position_alpha"].as<double>() = pPosAlpha;
	_optionLink["angle_alpha"].as<double>() = pAngAlpha;
	_optionLink["switch_alpha"].as<double>() = pSwitchAlpha;
	return;
}



void NonHoloEMA3d::setStandardOptions( GenericMultiLevelMap<string>& _optLink )
{
	_optLink["position_alpha"].as<double>() = 0.4;
	_optLink["angle_alpha"].as<double>() = 0.8;
	_optLink["switch_alpha"].as<double>() = 0.05;
	return;
}


Ptr<SceneObject::State> NonHoloEMA3d::newState( Mat& _state, RectangleRegion& _region, vector<Point>& /*_contour*/, Mat& _invImg, double _time )
{
	double angle;
	if( pHistory().size()!=0 )
	{
		Angle conv(_state.at<float>(2));
		conv.toClosestHalfEquivalent( pHistory().back()->angle );
		conv.toZero2Pi();
		angle = conv.rad();
	}
	else angle = _state.at<float>(2);

	bool imageBorderTouched = pObjectPointer->touchesImageBorder();

	double thetaEstimation = 0;

	// update rod geometry estimates - freeze properties if state infered from group image or from object at image border
	if( !_invImg.empty() && !imageBorderTouched ) // the inverse image is empty if the state was infered from a group
	{
		double length = _region.width();
		double diameter = _region.height();
		if( length>pLengthMax ) pLengthMax = length;
		pWidth = diameter;

		// estimate theta
		if( pLengthMax!=0 && pWidth!=0 )
		{
			thetaEstimation = asin( length/sqrt(pLengthMax*pLengthMax+pWidth*pWidth) ) - atan(pLengthMax/pWidth); // from trigonometric formulas

			if( pHistory().size()!=0 )
			{
				if( pHistory().back()->type == classId ) // adjust theta
				{
					Angle thetaConv(thetaEstimation);
					Ptr<State> lastState = pHistory().back();
					thetaConv.toClosestHalfEquivalent( lastState->thetaEstimated );
					thetaConv.toZero2Pi();
					thetaEstimation = thetaConv.rad();
				}
			}
		}
		// else (shouldn't happen, but if) assume robot in plane (theta=0)
	}
	else // freeze geometric properties
	{
		if( pHistory().size()!=0 ) // if previous states exist
		{
			if( pHistory().back()->type==classId )
			{
				Ptr<State> lastState = pHistory().back();
				thetaEstimation = lastState->thetaEstimated;
			}
		}
		// else theta is unknown, assume robot in image plane (theta=0)
	}

	Ptr<SceneObject::State> newState = rawState( _state.at<float>(0), _state.at<float>(1), _time, angle, thetaEstimation, 0.0 );
	return newState;
}



bool NonHoloEMA3d::addState( Ptr<SceneObject::State> _newState, RectangleRegion& _regionOfInterest, vector<Point>& _contour )
{
	if( _newState->type!=classId ) return false;

	Ptr<NonHoloEMA3d::State> newState = _newState;

	// find closest angle equivalent of measured angle to predicted angle in order to get the correct prediction error of the Kalman filter ->already done when creating the state in newState(...)
	

	if( pHistory().size()==0 )
	{
		newState->x = newState->raw_x;
		newState->y = newState->raw_y;
		newState->angle = newState->raw_angle;
		newState->thetaEstimated = newState->thetaEstimated_raw;
	}
	else
	{
		// run prediction if not yet done (necessary to predict rotations through image plane normal)
		predict();

		Ptr<SceneObject::State> lastState = pHistory().back();

		// update positions, EMA-smoothed
		newState->x = pPosAlpha*newState->raw_x + (1-pPosAlpha)*lastState->x;
		newState->y = pPosAlpha*newState->raw_y + (1-pPosAlpha)*lastState->y;

		// check if still happy with chosen direction (using smoothed positions)
		vector<double> currentDirection = direction( newState->raw_angle );
		double scalProd = currentDirection[0]*( newState->x - lastState->x ) + currentDirection[1]*( newState->y - lastState->y );
		int correctDirection = (scalProd>=0)?1:0;
		// update certainty measure
		pDirectionSwitch = pSwitchAlpha*correctDirection + (1-pSwitchAlpha)*pDirectionSwitch;
		
		// EMA of angle - the old angle is moved to the closest value to raw_angle with 2*pi additions/subtractions to prevent slips over the border range between 2*pi and zero
		double smoothedAngle = pAngAlpha*newState->raw_angle + (1-pAngAlpha)*Angle(lastState->angle).closestEquivalent(newState->raw_angle).rad();

		double smoothedTheta = newState->thetaEstimated_raw;
		if( lastState->type==classId ) // EMA of theta
		{
			Ptr<State> convLS = lastState;
			smoothedTheta = pAngAlpha*newState->thetaEstimated_raw + (1-pAngAlpha)*Angle(convLS->thetaEstimated).closestEquivalent(newState->thetaEstimated_raw).rad();
		}

		Angle smooth(smoothedAngle);
		// switch direction if either average velocity vector indicates it's necessary or a rotation through the image normal was predicted
		if( pDirectionSwitch<0.5 || pReflectionOccured==1 )
		{
			Angle rawToSwitch( newState->raw_angle );
			smooth++; //increments the angle with pi
			rawToSwitch++;

			rawToSwitch.toZero2Pi(); //ensures it is still inside range

			newState->raw_angle = rawToSwitch.rad();
			pDirectionSwitch = 0.5;
		}
		smooth.toZero2Pi();

		newState->angle = smooth.rad();
		newState->thetaEstimated = smoothedTheta;
	}


	pHistory().push_back( newState );
	pHasPredicted = false;

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


void NonHoloEMA3d::resetPrediction()
{
	pDirectionSwitch = 0.5;
	pHasPredicted = false;
	return;
}


void NonHoloEMA3d::predict()
{
	if( pHasPredicted ) return;

	pHasPredicted = true;

	if( pHistory().size()==0 )
	{
		pXPre = 0;
		pYPre = 0;
		pAngPre = 0;
		return;
	}
	else if( pHistory().size()==1 )
	{
		pXPre = pHistory().back()->x;
		pYPre = pHistory().back()->y;
		pAngPre = pHistory().back()->angle;
		return;
	}
	else
	{
		double xVel = pHistory().back()->x - pHistory()[pHistory().size()-2]->x;
		double yVel = pHistory().back()->y - pHistory()[pHistory().size()-2]->y;
		double angVel = SceneObject::calcAngleVelocity( pHistory()[pHistory().size()-2]->angle, pHistory().back()->angle );

		double lastTheta=0;
		double thetaVel=0;
		if( pHistory().back()->type==classId )
		{
			Ptr<State> last = pHistory().back();
			lastTheta = last->thetaEstimated;

			if( pHistory()[ pHistory().size()-2 ]->type==classId )
			{
				Ptr<State> beforeLast = pHistory()[ pHistory().size()-2 ];
				// velocity depends on whether it was previously predicted that theta passes through the normal axis or image plane (which are not traceable directly, since theta is bound to [0...pi/2])
				if( pReflectionOccured==0 ) thetaVel = last->thetaEstimated - beforeLast->thetaEstimated; // nothing happened
				else if( pReflectionOccured==1 ) thetaVel = last->thetaEstimated + beforeLast->thetaEstimated - Angle::pi; // passed through normal axis
				else if( pReflectionOccured==2 ) thetaVel = last->thetaEstimated + beforeLast->thetaEstimated; // passed through image plane

				if( thetaVel>Angle::pi_half ) thetaVel = Angle::pi_half; // any higher velocity is untraceable and results in problems for the following calculations
			}
		}

		double absVel = sqrt( xVel*xVel + yVel*yVel );

		// predict values
		pXPre = pHistory().back()->x + absVel*cos( pHistory().back()->angle )*cos(lastTheta);
		pYPre = pHistory().back()->y + absVel*sin( pHistory().back()->angle )*cos(lastTheta);

		double predTheta = lastTheta+thetaVel;
		// theta is bounded in [0...pi/2]
		if( predTheta>Angle::pi_half )
		{
			//predTheta = Angle::pi-predTheta;
			pReflectionOccured = 1;
		}
		else if( predTheta<0 )
		{
			//predTheta = -predTheta;
			pReflectionOccured = 2;
		}
		else
		{
			//predTheta = predTheta;
			pReflectionOccured = 0;
		}
		Angle pred( pHistory().back()->angle + angVel );
		// orientation changes if reflection through pi/2 occured
		if( pReflectionOccured==1 ) pred++;
		pred.toZero2Pi();
		pAngPre = pred.rad();
		return;
	}
}


int NonHoloEMA3d::registerFactoryFunction()
{
	string info = "Nonholonomic movement where the direction of movement is aligned with the axis. Smoothed positions with an exponential average filter. Prediction based on constant velocity assumption. It is targeted at rod-like objects, their three-dimensional posture is estimated from 2d imagery.";
	GenericMultiLevelMap<string> options;
	setStandardOptions(options);
	int _classId =  Dynamics::registerDynamics( DynamicsFacEntry( "NonHoloEMA3d", &sfmCreator, info, options ) );
	return _classId;
}



int NonHoloEMA3d::classId = registerFactoryFunction();



NonHoloEMA3d::State::State()
{
	type = classId;
}


NonHoloEMA3d::State::~State()
{

}

NonHoloEMA3d::State::State( double _x, double _y, double time, double _angle, double _area ):FilteredDynamics::State( _x, _y, time, _angle, _area )
{
	type=classId;
	thetaEstimated = 0;
}

NonHoloEMA3d::State::State( Point _pos, double time, double _angle, double _area ):FilteredDynamics::State( _pos, time, _angle, _area )
{
	type=classId;
	thetaEstimated = 0;
}
