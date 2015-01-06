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

*/#include "GenericObject.h"

/** class used to record and create new descriptor sets */

class DescriptorCreator
{
	friend class ObjectHandler;
public:
	DescriptorCreator();
	/** constructor with ids of objects for which the descriptors shall be created. */
    DescriptorCreator( Ptr<vector<unsigned int> > _objectIds);
    DescriptorCreator(int _id0, int _id1=-1, int _id2=-1, int _id3=-1, int _id4=-1, int _id5=-1, int _id6=-1, int _id7=-1, int _id8=-1, int _id9=-1 );
	~DescriptorCreator(void);

	/** List with the ids of those objects whose descriptors shall be created. Previous object lists are disregarded. */
    void objectsToRecord( Ptr<vector<unsigned int> > _objectIds );
	/** List with the ids of those objects whose descriptors shall not be created. previous object lists are disregarded. */
    void objectsNotToRecord( Ptr<vector<unsigned int> > _objectIds );

	/** add another object to the list of those objects whose descriptors shall be created */
	void recordObject( unsigned int _objId );
	/** remove an object from the list of those objects whose descriptors shall be created */
	void removeObject( unsigned int _objId );

	/** adds new descriptors to the set if it is currently recording and the object id is a match, returns true if the descriptors were added, false if not */
	bool fill( Mat& _containingImage, Mat& _descriptors, unsigned int _objId );

	/** user given name for the descriptor set */
	string name;
	/** user given information about the descriptor set */
	string info;
	
	/** starts 'recording' of descriptors */
	void record();

	bool isRecording();

	/** stops 'recording' of descriptors */
	void stop();
	/** clears the list of objects whose features are to be recorded */
	void clear();
	/** creates the descriptor set, writes it to file */
	bool create();
	/** unlinks the descriptor creator from all object handlers it has been associated with, returns true if something was done, false if it wasn't linked in the first place.*/
	bool unlink();

	/** returns the number of descriptors the set contains */
    unsigned int dataEntrySize();

private:
	bool pRecord; // indicates if descriptor is recording or not

    Ptr<vector<unsigned int> > pIds; // ids of objects to be recorded or ignored, depending on ignoreMode
	bool pIgnoreMode; // false: pIds are ids of objects to be recorded; true: pIds are ids of objects to be ignored

	Mat pDescriptorSet; // recorded descriptor set
	Mat pSnapshot; // image of first picture from which descriptors were extracted

	/** removes the id from the list */
	void removeIdFromList( unsigned int _objId );

    list<Ptr<ObjectHandler> > pOHLink;

};

