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

#include "DescriptorCreator.h"
#include "objecthandler.h"

DescriptorCreator::DescriptorCreator(void)
{
	pIds = NULL;
	pIgnoreMode = false;
	pRecord = false;
}


DescriptorCreator::DescriptorCreator( Ptr<vector<unsigned int> > _objectIds)
{
	pIds = NULL;
	pIgnoreMode = false;
	pRecord = false;

	objectsToRecord(_objectIds);
}


DescriptorCreator::DescriptorCreator( int _id0, int _id1, int _id2, int _id3, int _id4, int _id5, int _id6, int _id7, int _id8, int _id9 )
{
	pIds = NULL;
	pIgnoreMode = false;
	pRecord = false;

    Ptr<vector<unsigned int> > v = new vector<unsigned int>();
	v->push_back(_id0);
	if( _id1!=-1 ) v->push_back(_id1);
	if( _id2!=-1 ) v->push_back(_id2);
	if( _id3!=-1 ) v->push_back(_id3);
	if( _id4!=-1 ) v->push_back(_id4);
	if( _id5!=-1 ) v->push_back(_id5);
	if( _id6!=-1 ) v->push_back(_id6);
	if( _id7!=-1 ) v->push_back(_id7);
	if( _id8!=-1 ) v->push_back(_id8);
	if( _id9!=-1 ) v->push_back(_id9);

	objectsToRecord(v);
}


DescriptorCreator::~DescriptorCreator(void)
{
	unlink();

    for( list<Ptr<ObjectHandler> >::iterator it=pOHLink.begin(); it!=pOHLink.end(); it++ )
	{
		(*(*it).refcount)++; // keep the Ptr<> object from attempting to delete the referenced ObjectHandler
	}
}


void DescriptorCreator::objectsToRecord( Ptr<vector<unsigned int> > _objectIds )
{
	pIgnoreMode = false;
	pIds = _objectIds;
}


void DescriptorCreator::objectsNotToRecord( Ptr<vector<unsigned int> > _objectIds )
{
	pIgnoreMode = true;
	pIds = _objectIds;
}


void DescriptorCreator::recordObject( unsigned int _objId )
{
	if( pIgnoreMode ) removeIdFromList( _objId );
	pIds->push_back( _objId );
	return;
}


void DescriptorCreator::removeObject( unsigned int _objId )
{
	if( pIgnoreMode ) pIds->push_back( _objId );
	removeIdFromList( _objId );
}



bool DescriptorCreator::fill( Mat& _containingImage, Mat& _descriptors, unsigned int _objId )
{
	if( !pRecord )
	{
		return false;
	}

	if( !pIgnoreMode )
	{
		bool inSet=false;
        for( size_t i=0; i<pIds->size(); i++ )
		{
			if( _objId == (*pIds)[i] )
			{
				inSet = true;
				break;
			}
		}
		if(!inSet) return false;
	}
	else
	{
        for( size_t i=0; i<pIds->size(); i++ )
		{
			if( _objId == (*pIds)[i] )
			{
				return false;
			}
		}
	}

	if( pSnapshot.empty() ) _containingImage.copyTo( pSnapshot ); // deep copy since _containingImage is a temporary one
	pDescriptorSet.push_back( _descriptors );
	return true;
}


void DescriptorCreator::record()
{
	pRecord = true;
}


bool DescriptorCreator::isRecording()
{
	return pRecord;
}


void DescriptorCreator::stop()
{
	pRecord = false;
}


void DescriptorCreator::clear()
{
	pIds->clear();
}


bool DescriptorCreator::create()
{
	if( pRecord || pDescriptorSet.empty() ) return false;

	Ptr<GenericObject::FeatureData> newFeatureDescriptor = new GenericObject::FeatureData( name, pDescriptorSet, pSnapshot );
	newFeatureDescriptor->info = info;

	if( newFeatureDescriptor->save() )
	{
		GenericObject::availableFeatureSets.push_back( newFeatureDescriptor );
		return true;
	}
	else return false;
}


bool DescriptorCreator::unlink()
{
	if( pOHLink.empty() ) return false;
	
	bool somethingRemoved=false;
	while( !pOHLink.empty() )
	{
        list<Ptr<ObjectHandler> >::iterator it=pOHLink.begin();
		if( (*(*it).refcount)>1 ) somethingRemoved=(*it)->removeDescriptorCreator( this ); // the refcount counter is initialized and hold at 2, should be 1 if the destructor of the object referenced has been called already // the function removeDescriptorCreator also deletes the actual element (not the fastest method as it is searched again, but that little time offset is accepted for a simpler solution)
		else
		{
			(*it).addref(); // don't call the objecthandler destructor
			pOHLink.erase(it);
		}
	}
	return somethingRemoved;
}


void DescriptorCreator::removeIdFromList( unsigned int _objId )
{
	vector<unsigned int>::iterator it, end;
	end = pIds->end();
	for( it=pIds->begin(); it!=end; it++ )
	{
		if( *it==_objId )
		{
			pIds->erase( it );
			break;
		}
	}
	return;
}


unsigned int DescriptorCreator::dataEntrySize()
{
	return pDescriptorSet.rows;
}
