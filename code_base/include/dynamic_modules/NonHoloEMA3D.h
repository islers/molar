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

#include "NonHoloEMA.h"

/* 2-dimensional non holonomic movement, constant velocity assumption, exponential moving average smoothed */

class NonHoloEMA3d: public NonHoloEMA
{
public:
	class State;

	NonHoloEMA3d( Ptr<GenericObject> _objectPointer );
	~NonHoloEMA3d(void);

	/** creates a state with raw values initialized - rawArea currently not implemented! */
	virtual Ptr<SceneObject::State> rawState( double _rawX, double _rawY, double _time, double _rawAngle, double _rawTheta, double _rawArea=0 ) const;

	static Ptr<Dynamics> sfmCreator( Ptr<GenericObject> _objectPointer );

	// realization
	virtual void setOptions( GenericMultiLevelMap<string>& _dynamicsOptions );
	virtual void getOptions( GenericMultiLevelMap<string>& _optionLink );
	static void setStandardOptions( GenericMultiLevelMap<string>& _optLink );

	virtual Ptr<SceneObject::State> newState( Mat& _state, RectangleRegion& _region, vector<Point>& _contour, Mat& _invImg, double _time );
	virtual bool addState( Ptr<SceneObject::State> _newState, RectangleRegion& _regionOfInterest, vector<Point>& _contour );
	
	virtual void resetPrediction();


protected:
	void predict(); //executes predictions step

	double pLengthMax; //maximal observed object length: outliers are currently not calculated
	Average<> pWidth; //average of the observed width

	double pThetaPre; // predicted theta angle
	int pReflectionOccured; // 0: nothing happened between previous two states, 1: theta passed through pi_half, 2: theta passed through zero

	static int classId;
	static int registerFactoryFunction();
};


class NonHoloEMA3d::State: public FilteredDynamics::State
{
	public:
		State();
		~State();
		State( double _x, double _y, double time, double _angle=0, double _area=0 );
		State( Point _pos, double time, double _angle=0, double _area=0 );

		double thetaEstimated; // estimated angle in "z-plane"
		double thetaEstimated_raw;
};
