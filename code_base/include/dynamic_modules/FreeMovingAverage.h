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

#include "SimpleFreeMovement.h"

class FreeMovingAverage: public SimpleFreeMovement
{
public:
	class State;

	FreeMovingAverage( Ptr<GenericObject> _objectPointer );
	~FreeMovingAverage(void);
	
	static Ptr<Dynamics> fmaCreator( Ptr<GenericObject> _objectPointer );

	// realization
	virtual void setOptions( GenericMultiLevelMap<string>& _dynamicsOptions );
	virtual void getOptions( GenericMultiLevelMap<string>& _optionLink );
	static void setStandardOptions( GenericMultiLevelMap<string>& _optLink );

	virtual string info();

	virtual Ptr<SceneObject::State> newState( Mat& _state, RectangleRegion& _region, vector<Point>& _contour, Mat& _invImg, double _time );
	//virtual bool addState( Ptr<SceneObject::State> _newState, RectangleRegion& _regionOfInterest, vector<Point>& _contour );

	virtual bool exportData( string _filePath, string _dataPointSeparator=" ", string _timeStepSeparator="\n" );
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
	virtual Point predictCtrPosition();
	virtual Point2f predictCtrFPosition();
	virtual double predictXCtrVelocity();
	virtual double predictXCtrAcceleration();
	virtual double predictYCtrVelocity();
	virtual double predictYCtrAcceleration();
	virtual double predictAgl();
	virtual double predictAglVelocity();
	virtual double predictAglAcceleration();

	//virtual bool drawLayer( Mat& _img, unsigned int _layerLevel, Scalar _color );


protected:
	double pPosAlpha, pAngAlpha, pVelAlpha, pVelAngAlpha, pAccAlpha, pAccAngAlpha;

	static int classId;
	static int registerFactoryFunction();
};



class FreeMovingAverage::State: public SceneObject::State
{
	public:
		State();
		~State();
		State( double _x, double _y, double time, double _angle=0, double _area=0, int _type=-1 );
		State( Point _pos, double time, double _angle=0, double _area=0, int _type=-1 );

		double raw_x;
		double raw_y;
		double raw_angle;

		double vel_x, vel_y, vel_angle; // all filtered using exponential moving average
		double acc_x, acc_y, acc_angle;
};
