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

#include "NonHoloKalman2D_Orth.h"


NonHoloKalman2D_Orth::NonHoloKalman2D_Orth( Ptr<GenericObject> _objectPointer ):NonHoloKalman2D(_objectPointer)
{
	pIsInitialized = false;
}


NonHoloKalman2D_Orth::~NonHoloKalman2D_Orth(void)
{
}


Ptr<GenericObject::Dynamics> NonHoloKalman2D_Orth::sfmCreator( Ptr<GenericObject> _objectPointer )
{
	return new NonHoloKalman2D_Orth(_objectPointer);
}


void NonHoloKalman2D_Orth::setOptions( GenericMultiLevelMap<string>& /*_dynamicsOptions*/ )
{
	// the class has currently no options that could be set
	return;
}


void NonHoloKalman2D_Orth::getOptions( GenericMultiLevelMap<string>& /*_optionLink*/ )
{
	// empty since the class has no options
	return;
}


Ptr<SceneObject::State> NonHoloKalman2D_Orth::newState( Mat& _state, RectangleRegion& /*_region*/, vector<Point>& /*_contour*/, Mat& /*_invImg*/, double _time )
{
	predict();
	Angle conv(_state.at<float>(2));
	conv = conv + Angle::pi_half; // switch to orthogonal direction
	conv.toClosestHalfEquivalent(pEstimator.statePost.at<float>(2) );// choosing last angle instead of predicted(otherwhise too high angle velocities can maintain themselves) ( pEstimator.statePre.at<float>(2) );
	conv.toZero2Pi();
	
	Ptr<SceneObject::State> newState = rawState( _state.at<float>(0), _state.at<float>(1), _time, conv.rad(), 0.0 );
	return newState;
}




int NonHoloKalman2D_Orth::registerFactoryFunction()
{
	string info = "Filters and predicts states using an Extended Kalman filter based on equations for non-holonomic motion for an object for which its axis and the movement direction are orthogonal to each other. Assumes constant velocities for position and angle.";
	return Dynamics::registerDynamics( DynamicsFacEntry( "NonHoloKalman2D_Orth", &sfmCreator, info ) );
}



int NonHoloKalman2D_Orth::classId = registerFactoryFunction();
