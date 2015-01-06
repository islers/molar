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
#include "NonHoloKalman2D.h"

/** Two dimensional extended kalman state estimator for non-holonomic robots with their axis aligned to their movement direction
	5-dim state space: (x_pos, y_pos, angle, velocity, angle velocity )
*/

class NonHoloKalman3D: public NonHoloKalman2D
{
public:
	NonHoloKalman3D( Ptr<GenericObject> _objectPointer );
	~NonHoloKalman3D(void);

	class State;
	
	/** creates a state with raw values initialized - rawArea currently not implemented! */
	virtual Ptr<SceneObject::State> rawState( double _rawX, double _rawY, double _time, double _rawAngle, double _rawLength, double _rawArea=0 ) const;
	
	static Ptr<Dynamics> sfmCreator( Ptr<GenericObject> _objectPointer );

	// realization
	virtual void setOptions( GenericMultiLevelMap<string>& _dynamicsOptions );
	virtual void getOptions( GenericMultiLevelMap<string>& _optionLink );

	virtual Ptr<SceneObject::State> newState( Mat& _state, RectangleRegion& _region, vector<Point>& _contour, Mat& _invImg, double _time );
	virtual bool addState( Ptr<SceneObject::State> _newState, RectangleRegion& _regionOfInterest, vector<Point>& _contour );


	virtual Mat predictState();
	virtual void resetPrediction();
	virtual Point predictPosition( Point _pt );
	virtual double predictXCtrVelocity();
	virtual double predictYCtrVelocity();
	virtual double predictAgl();
	virtual double predictAglVelocity();



protected:
	ExtendedKalmanFilter pEstimator;
	bool pHasAlreadyPredicted; // filter does not allow a prediction step if no new measurement has been given before: If more than one prediction function is called without new measurements available, then the old predictions are repeated
	//(re-)initializes the Kalman filter
	virtual void initializeKalman();
	/** runs the prediction step (but only if pHasAlreadyPredicted!=true)*/
	virtual void predict();
	/** calculates the current linearized transition matrix A(k) and the current linearized process noise transition matrix L(k) */
	virtual void setTransitionMatrices( double _velocity, double _angle, double _theta );
	
	double pLengthMax; //maximal observed object length: outliers are currently not calculated
	Average<> pWidth; //average of the observed width

	bool pIsInitialized; // if the state has already been setup properly or not (requires at least two states to initialize the velocity


	static int classId;
	static int registerFactoryFunction();
};


class NonHoloKalman3D::State: public FilteredDynamics::State
{
	public:
		State();
		~State();
		State( double _x, double _y, double time, double _angle=0, double _area=0 );
		State( Point _pos, double time, double _angle=0, double _area=0 );

		double length; // length measurement
};
