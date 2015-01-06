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
#include <string>
#include "opencv2/core/core.hpp"
#include <map>
#include <iostream>
#include "genericmultilevelmap.hpp"

using namespace std;
using namespace cv;

/** abstract base class for algorithms for "simple" image transformations:
*	one image as input, the edited image as output
*/


class IPAlgorithm
{
public:
	IPAlgorithm(void);
	~IPAlgorithm(void);

	/** sets whether the filter is active or not */
	bool active;

	/** function to register virtual constructor functions
	* of child classes
	*/
	static bool registerAlgorithm( string _name, Ptr<IPAlgorithm>(*_createInstance)() );

	/** returns the number of algorithms that have been registered */
	static int numberOfAlgorithms();

	/** function to create registered algorithm class objects by name
	*
	* @return: NULL pointer if class with given name wasn't found
	*/
	static Ptr<IPAlgorithm> createAlgorithm( string _name );

	/** function to create registered algorithm class objects by id in list
	*
	* @return: NULL pointer if class with given name wasn't found
	*/
	static Ptr<IPAlgorithm> createAlgorithm( int _id );

	/** executes the process function and any additonal "parental class functions" if there are any */
	bool apply( Mat& _image );

	/** abstract function for reading the current settings (including info: info may be added to the map under the reserved "info" keyword) */
	virtual void getOptions( st_is::GenericMultiLevelMap<string>& _options) const =0;

	/** abstract function for setting the options */
	virtual bool setOptions( st_is::GenericMultiLevelMap<string>& _options )=0;

	/** virtual constructor, shall return pointer to
	* new instantiation of class object, must be registered
	* in this base class
	*/
	//virtual static Ptr<IPAlgorithm> createInstance()=0;

	/** returns the name of the algorithm
	*/
	virtual string algorithmName() const =0;

	/** returns a description of the algorithm */
	virtual string description() const =0;

	/** returns a string which can be used to
	*  reinstate the algorithms state
	*/
	virtual string options() const =0;

	/** reinstate the algorithm to a previous state
	*/
	virtual bool setup( string _setting )=0;

	/** load standard settings */
	virtual void resetSetting()=0;

protected:
	/** applies the algorithm to the image
	*/
	virtual bool process( Mat& _image )=0;

private:
	static map<string,Ptr<IPAlgorithm>(*)()>* registeredAlgorithms;

};

