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
#include "Dynamics.h"

/** abstract parent class for filter types that smooth the state, providing a new State member class with smoothed and raw data */
class FilteredDynamics: public GenericObject::Dynamics
{
public:	
	class State;

	FilteredDynamics( Ptr<GenericObject> _objectPointer );

	/** creates a state with raw values initialized - rawArea currently not implemented! */
	virtual Ptr<SceneObject::State> rawState( double _rawX, double _rawY, double _time, double _rawAngle, double _rawArea=0 ) const;

	virtual void predictROI( vector<Point>& _roi );

	virtual bool exportData( string _filePath, string _dataPointSeparator=" ", string _timeStepSeparator="\n" );

	bool isFiltered( Ptr<SceneObject::State> _checkState );
private:
	static int classId;
};



class FilteredDynamics::State: public SceneObject::State
{
	public:
		State();
		~State();
		State( double _x, double _y, double time, double _angle=0, double _area=0 );
		State( Point _pos, double time, double _angle=0, double _area=0 );

		double raw_x;
		double raw_y;
		double raw_angle;
};
