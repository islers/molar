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


#include "IPAlgorithm.h"

map<string,Ptr<IPAlgorithm>(*)()>* IPAlgorithm::registeredAlgorithms=NULL;


IPAlgorithm::IPAlgorithm(void)
{
	active = true;
}


IPAlgorithm::~IPAlgorithm(void)
{
}


bool IPAlgorithm::registerAlgorithm( string _name, Ptr<IPAlgorithm>(*_createInstance)() )
{
	if( registeredAlgorithms==NULL ) registeredAlgorithms = new map<string,Ptr<IPAlgorithm>(*)()>; //not sure if this works in all building environments: it is based on the assumption that the pointer primitive is always NULL at the start, trying to ensure that the algorithm register is setup before the registration call is executed. The problem is that two static members depend on each other

	if( (*registeredAlgorithms).count(_name )!=0 )
	{
		cerr<<endl<<"Registration of algorithm with name "<<_name << "was attempted but an algorithm with this name already exists. Call is ignored."<<endl;
		return false;
	}
	(*registeredAlgorithms)[_name] = _createInstance;
	return true;
}


int IPAlgorithm::numberOfAlgorithms()
{
	return IPAlgorithm::registeredAlgorithms->size();
}


Ptr<IPAlgorithm> IPAlgorithm::createAlgorithm( string _name )
{
	if( (*registeredAlgorithms).count(_name )==0 )
	{
		return Ptr<IPAlgorithm>();
	}

	return (*registeredAlgorithms)[_name]();
}


Ptr<IPAlgorithm> IPAlgorithm::createAlgorithm( int _id )
{
    if( _id<0 || (unsigned int)_id>=registeredAlgorithms->size() ) return Ptr<IPAlgorithm>();

	int i=0;
	map< string, Ptr<IPAlgorithm>(*)() >::iterator it, end;
	it = registeredAlgorithms->begin();
	end = registeredAlgorithms->end();

	for( ; it!=end ; it++, i++ )
	{
		if( i==_id ) return (*it).second();
	}
	return Ptr<IPAlgorithm>();
}

bool IPAlgorithm::apply( Mat& _image )
{
	return process( _image );
}
