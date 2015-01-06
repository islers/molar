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

class NonHoloKalman2D_Orth: public NonHoloKalman2D
{
public:
	NonHoloKalman2D_Orth( Ptr<GenericObject> _objectPointer );
	~NonHoloKalman2D_Orth(void);

	static Ptr<Dynamics> sfmCreator( Ptr<GenericObject> _objectPointer );

	// realization
	virtual void setOptions( GenericMultiLevelMap<string>& _dynamicsOptions );
	virtual void getOptions( GenericMultiLevelMap<string>& _optionLink );

	virtual Ptr<SceneObject::State> newState( Mat& _state, RectangleRegion& _region, vector<Point>& _contour, Mat& _invImg, double _time );
	

	//virtual bool drawLayer( Mat& _img, unsigned int _layerLevel, Scalar _color );
	//virtual void drawDirection( Mat& _img, Scalar _color );


protected:
	bool pIsInitialized; // if the state has already been setup properly or not (requires at least two states to initialize the velocity

	static int classId;
	static int registerFactoryFunction();
};
