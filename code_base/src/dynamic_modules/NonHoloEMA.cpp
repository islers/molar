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

#include "NonHoloEMA.h"


NonHoloEMA::NonHoloEMA( Ptr<GenericObject> _objectPointer ):FilteredDynamics(_objectPointer)
{
	pPosAlpha = 0.4;
	pAngAlpha = 0.8;
	pSwitchAlpha = 0.05;
	pDirectionSwitch = 0.5; // chosen measure for certainty about current direction (0...1) switch performed if it falls under 0.5
	pHasPredicted = false;
}


NonHoloEMA::~NonHoloEMA(void)
{
}


Ptr<GenericObject::Dynamics> NonHoloEMA::sfmCreator( Ptr<GenericObject> _objectPointer )
{
	return new NonHoloEMA(_objectPointer);
}


void NonHoloEMA::setOptions( GenericMultiLevelMap<string>& _dynamicsOptions )
{
	if( _dynamicsOptions.hasKey("position_alpha") ) pPosAlpha = _dynamicsOptions["position_alpha"].as<double>();
	if( _dynamicsOptions.hasKey("angle_alpha") ) pAngAlpha = _dynamicsOptions["angle_alpha"].as<double>();
	if( _dynamicsOptions.hasKey("switch_alpha") ) this->pSwitchAlpha = _dynamicsOptions["switch_alpha"].as<double>();
	return;
}


void NonHoloEMA::getOptions( GenericMultiLevelMap<string>& _optionLink )
{
	_optionLink["position_alpha"].as<double>() = pPosAlpha;
	_optionLink["angle_alpha"].as<double>() = pAngAlpha;
	_optionLink["switch_alpha"].as<double>() = pSwitchAlpha;
	return;
}



void NonHoloEMA::setStandardOptions( GenericMultiLevelMap<string>& _optLink )
{
	_optLink["position_alpha"].as<double>() = 0.4;
	_optLink["angle_alpha"].as<double>() = 0.8;
	_optLink["switch_alpha"].as<double>() = 0.05;
	return;
}


Ptr<SceneObject::State> NonHoloEMA::newState( Mat& _state, RectangleRegion& /*_region*/, vector<Point>& /*_contour*/, Mat& /*_invImg*/, double _time )
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

	Ptr<SceneObject::State> newState = rawState( _state.at<float>(0), _state.at<float>(1), _time, angle, 0.0 );
	return newState;
}



bool NonHoloEMA::addState( Ptr<SceneObject::State> _newState, RectangleRegion& _regionOfInterest, vector<Point>& _contour )
{
	if( !isFiltered(_newState) ) return false;

	Ptr<FilteredDynamics::State> newState = _newState;

	// find closest angle equivalent of measured angle to predicted angle in order to get the correct prediction error of the Kalman filter ->already done when creating the state in newState(...)
	
	Mat measurement = ( Mat_<float>(3,1) << newState->raw_x, newState->raw_y, newState->raw_angle );
	
	if( pHistory().size()==0 )
	{
		newState->x = newState->raw_x;
		newState->y = newState->raw_y;
		newState->angle = newState->raw_angle;
	}
	else
	{
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
		Angle smooth(smoothedAngle);
		if( pDirectionSwitch<0.5 )
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

Mat NonHoloEMA::predictState()
{
	if( pHistory().size()==0 ) return Mat();
	
	predict();

	Mat prediction;
	prediction.create(1,3,CV_32FC1);
	prediction.at<float>(0) = pXPre;
	prediction.at<float>(1) = pYPre;
	prediction.at<float>(2) = pAngPre;
	return prediction;
}


void NonHoloEMA::resetPrediction()
{
	pDirectionSwitch = 0.5;
	pHasPredicted = false;
	return;
}


Point NonHoloEMA::predictPosition( Point _pt )
{
	predict();

	if( pHistory().size()<=1 ) return _pt;

	Point center( (int)pHistory().back()->x, (int)pHistory().back()->y );

	Point relPos = _pt-center;

	double predAngVel = SceneObject::calcAngleVelocity( pHistory().back()->angle, pAngPre );
	double cosEl = cos(predAngVel);
	double sinEl = sin(predAngVel);
	double newRelX = cosEl*relPos.x - sinEl*relPos.y;
	double newRelY = sinEl*relPos.x + cosEl*relPos.y;

	Point newRelPos( (int)newRelX, (int)newRelY );

	return Point( pXPre,pYPre )+newRelPos;
}

Point NonHoloEMA::predictCtrPosition()
{
	if( pHistory().size()==0 ) return Point(0,0);
	
	predict();

	return Point( pXPre, pYPre );
}

Point2f NonHoloEMA::predictCtrFPosition()
{
	if( pHistory().size()==0 ) return Point(0,0);
	
	predict();

	return Point2f( pXPre, pYPre );
}

double NonHoloEMA::predictXCtrVelocity()
{
	if( pHistory().size()==0 ) return 0;
	
	predict();

	return pXPre-pHistory().back()->x;
}

double NonHoloEMA::predictXCtrAcceleration() // since these Dynamics assume constant velocities, this function should return 0. Instead however, the acceleration given by the smoothened states is calculated and returned, assuming constant acceleration
{
	return accX();
}

double NonHoloEMA::predictYCtrVelocity()
{
	if( pHistory().size()==0 ) return 0;
	
	predict();

	return pYPre-pHistory().back()->y;
}

double NonHoloEMA::predictYCtrAcceleration() // since these Dynamics assume constant velocities, this function should return 0. Instead however, the acceleration given by the smoothened states is calculated and returned, assuming constant acceleration
{
	return accY();
}

double NonHoloEMA::predictAgl()
{
	if( pHistory().size()==0 ) return 0;
	predict();

	return pAngPre;
}

double NonHoloEMA::predictAglVelocity()
{
	if( pHistory().size()==0 ) return 0;
	
	predict();
	
	return SceneObject::calcAngleVelocity(pHistory().back()->angle,pAngPre);
}

double NonHoloEMA::predictAglAcceleration() // since these Dynamics assume constant velocities, this function should return 0. Instead however, the acceleration given by the smoothened states is calculated and returned, assuming constant acceleration
{
	return accAng();
}

double NonHoloEMA::predictArea()
{
	return area();
}


void NonHoloEMA::drawDirection( Mat& _img, Scalar _color )
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


void NonHoloEMA::predict()
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

		double absVel = sqrt( xVel*xVel + yVel*yVel );

		pXPre = pHistory().back()->x + absVel*cos( pHistory().back()->angle );
		pYPre = pHistory().back()->y + absVel*sin( pHistory().back()->angle );
		Angle pred( pHistory().back()->angle + angVel );
		pred.toZero2Pi();
		pAngPre = pred.rad();
		return;
	}
}


int NonHoloEMA::registerFactoryFunction()
{
	string info = "Nonholonomic movement where the direction of movement is aligned with the axis. Smoothed positions with an exponential average filter. Prediction based on constant velocity assumption.";
	GenericMultiLevelMap<string> options;
	setStandardOptions(options);
	int _classId =  Dynamics::registerDynamics( DynamicsFacEntry( "NonHoloEMA", &sfmCreator, info, options ) );
	return _classId;
}



int NonHoloEMA::classId = registerFactoryFunction();
