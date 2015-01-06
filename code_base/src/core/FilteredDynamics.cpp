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

#include "FilteredDynamics.h"

FilteredDynamics::FilteredDynamics(  Ptr<GenericObject> _objectPointer ):Dynamics(_objectPointer)
{

}


Ptr<SceneObject::State> FilteredDynamics::rawState( double _rawX, double _rawY, double _time, double _rawAngle, double /*_rawArea*/ ) const
{
	Ptr<FilteredDynamics::State> rawState = new State();

	rawState->raw_x = _rawX;
	rawState->raw_y = _rawY;
	rawState->raw_angle = _rawAngle;
	rawState->type = classId;
	rawState->time = _time;
	
	/*rawState->x = rawState->raw_x;
	rawState->y = rawState->raw_y;
	rawState->angle = rawState->raw_angle;*/

	return rawState;
}

void FilteredDynamics::predictROI( vector<Point>& _roi )
{
	_roi.push_back( predictPosition( pROI()[0] ) );
	_roi.push_back( predictPosition( pROI()[1] ) );
	_roi.push_back( predictPosition( pROI()[2] ) );
	_roi.push_back( predictPosition( pROI()[3] ) );
	return;
}


bool FilteredDynamics::exportData( string _filePath, string _dataPointSeparator, string _timeStepSeparator )
{
	try
	{
		std::ofstream file;
        file.open( _filePath.c_str(),std::ios_base::trunc ); // old file is overwritten

		file<<"time"<<_dataPointSeparator<<"x_pos"<<_dataPointSeparator<<"y_pos"<<_dataPointSeparator<<"angle"<<_dataPointSeparator<<"x_pos_raw"<<_dataPointSeparator<<"y_pos_raw"<<_dataPointSeparator<<"angle_raw"<<_timeStepSeparator;

        for( size_t i=0;i<pHistory().size(); i++ )
		{
			file<<pHistory()[i]->time<<_dataPointSeparator<<pHistory()[i]->x<<_dataPointSeparator<<pHistory()[i]->y<<_dataPointSeparator<<pHistory()[i]->angle;
			if( pHistory()[i]->type == classId ) file << _dataPointSeparator << Ptr<State>( pHistory()[i] )->raw_x << _dataPointSeparator << Ptr<State>( pHistory()[i] )->raw_y << _dataPointSeparator << Ptr<State>( pHistory()[i] )->raw_angle;
			file<<_timeStepSeparator;
		}

		file.close();
		return true;
	}
	catch(...)
	{
		cerr<<endl<<"SceneObject::exportData:: An error occured when trying to write to file "<<_filePath<<endl;
		return false;
	}
}

bool FilteredDynamics::isFiltered( Ptr<SceneObject::State> _checkState )
{
	return _checkState->type == classId;
}

int FilteredDynamics::classId = Dynamics::getUniqueId();


FilteredDynamics::State::State()
{
	type = classId;
}


FilteredDynamics::State::~State()
{

}


FilteredDynamics::State::State( double _x, double _y, double time, double _angle, double _area ):SceneObject::State( _x, _y, time, _angle, _area, classId ){}

FilteredDynamics::State::State( Point _pos, double time, double _angle, double _area ):SceneObject::State( _pos, time, _angle, _area, classId ){}
