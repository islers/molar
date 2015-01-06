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


/** class to store the program options */
#include "genericmultilevelmap.hpp"
#include <ctime>


using namespace st_is;
using namespace std;

class Options
{
public:
	Options(void);
	~Options(void);

	/** ensures that the class is setup - call if class objects are used in static variable initializations */
	static void load_options();

	/** saves the current options to the option file, overwriting the old */
	static void save_options();

	static const std::string generalSettingsFilename;

	/** resets the general settings to the standard values and (over)writes the xml file
	*/
	static void resetGeneralSettings();

	static void resetUserSettings();

	/** creates the general settings object with standard values
	*/
	static GenericMultiLevelMap<string>* generalSettingsStandard();
	
	/** initializes the general settings object at startup
	*/
	static GenericMultiLevelMap<string>* setupGeneral();


	static GenericMultiLevelMap<string>* General;

};
