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

#include "FreeMovingAverage.h"


class DirectedRodEMA: public FreeMovingAverage
{
public:
	class State;

	DirectedRodEMA( Ptr<GenericObject> _objectPointer );
	~DirectedRodEMA(void);
	
	static Ptr<Dynamics> dremaCreator( Ptr<GenericObject> _objectPointer );

	// realization
	virtual void setOptions( GenericMultiLevelMap<string>& _dynamicsOptions );
	virtual void getOptions( GenericMultiLevelMap<string>& _optionLink );
	static void setStandardOptions( GenericMultiLevelMap<string>& _optLink );

	virtual string info();

	virtual Ptr<SceneObject::State> newState( Mat& _state, RectangleRegion& _region, vector<Point>& _contour, Mat& _invImg, double _time );
	virtual bool addState( Ptr<SceneObject::State> _newState, RectangleRegion& _regionOfInterest, vector<Point>& _contour );

	
	virtual double predictHalfLengthOffset();

	
	// DRAW FUNCTION ///////////////////////
	//virtual bool drawLayer( Mat& _img, unsigned int _layerLevel, Scalar _color );
	virtual void drawDirection( Mat& _img, Scalar _color );

protected:
	//double pPosAlpha, pAngAlpha, pVelAlpha, pVelAngAlpha, pAccAlpha, pAccAngAlpha;
	double pHLOAlpha, pVelHLOAlpha, pAccHLOAlpha;
	
	double pDirectionConfidenceLevel; // describes how sure we are about a certain direction of the angle to be correct
	unsigned int pUprightDirectionCount; // counts for how many time steps the ABF was considered to be upright (aligned to an axis perpendicular to the image plane)

	Average<double> pDiameter; // width: average of the observed ones
	double pLength; // max length observed

	double pEMAAngle; //for direction switches, faster
	double pEMAAngleFac; // factor for averaging
	bool pEMAAngleInit;

	static int classId;
	static int registerFactoryFunction();
};



class DirectedRodEMA::State: public FreeMovingAverage::State
{
	public:
		State();
		~State();
		State( double _x, double _y, double time, double _angle=0, double _area=0, int _type=-1 );
		State( Point _pos, double time, double _angle=0, double _area=0, int _type=-1 );

		double halfLengthOffset; // instead of an estimation of the angle perpendicular to the view plane, use only the visible length offset (zero when the abf is oriented perfectly perpendicular). This assumption will be very bad if the abf's direction lies inside the view plane, but quite accurate for the perpendicular case, which is more important since this aims at predicting direction switches
		double raw_halfLengthOffset;
		double vel_hlo, acc_hlo;

		//double raw_x;
		//double raw_y;
		//double raw_angle;

		//double vel_x, vel_y, vel_angle;
		//double acc_x, acc_y, acc_angle;
};
