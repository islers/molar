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

#include "FilteredDynamics.h"
#include "Angle.h"

/* 2-dimensional non holonomic movement, constant velocity assumption, exponential moving average smoothed */

class NonHoloEMA: public FilteredDynamics
{
public:
	NonHoloEMA( Ptr<GenericObject> _objectPointer );
	~NonHoloEMA(void);


	static Ptr<Dynamics> sfmCreator( Ptr<GenericObject> _objectPointer );

	// realization
	virtual void setOptions( GenericMultiLevelMap<string>& _dynamicsOptions );
	virtual void getOptions( GenericMultiLevelMap<string>& _optionLink );
	static void setStandardOptions( GenericMultiLevelMap<string>& _optLink );

	virtual Ptr<SceneObject::State> newState( Mat& _state, RectangleRegion& _region, vector<Point>& _contour, Mat& _invImg, double _time );
	virtual bool addState( Ptr<SceneObject::State> _newState, RectangleRegion& _regionOfInterest, vector<Point>& _contour );

	//virtual Ptr<SceneObject::State> state();
	//virtual Mat history();
	//virtual Mat currentState();
	//virtual Point pos();
	//virtual double angle();
	//virtual Point2f direction();
	//virtual double area();
	//virtual double velX();
	//virtual double velY();
	//virtual double velAng();
	//virtual double accX();
	//virtual double accY();
	//virtual double accAng();
	//virtual RectangleRegion lastROI();
	//virtual void lastContour( vector<Point>& _contour );

	virtual Mat predictState();
	virtual void resetPrediction();
	virtual Point predictPosition( Point _pt );
	virtual Point predictCtrPosition();
	virtual Point2f predictCtrFPosition();
	virtual double predictXCtrVelocity();
	virtual double predictXCtrAcceleration();
	virtual double predictYCtrVelocity();
	virtual double predictYCtrAcceleration();
	virtual double predictAgl();
	virtual double predictAglVelocity();
	virtual double predictAglAcceleration();
	virtual double predictArea(); // not implemented!

	//virtual bool drawLayer( Mat& _img, unsigned int _layerLevel, Scalar _color );
	virtual void drawDirection( Mat& _img, Scalar _color );


protected:
	double pPosAlpha, pAngAlpha, pSwitchAlpha;
	double pDirectionSwitch; // chosen as (un)certainty measure for current angle direction, based on scalar products (sp) of velocity and angle direction: EMA smoothed:add 1 if sp positive, 0 if negative

	void predict(); //executes predictions step

	bool pHasPredicted; // tells whether prediction step has been run or not
	double pXPre, pYPre, pAngPre;

	static int classId;
	static int registerFactoryFunction();
};
