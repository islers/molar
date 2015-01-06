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

#include "IPAlgorithm.h"
#include "opencv2/imgproc/imgproc.hpp"

/** simple algorithm that performs contrast and brightness adjustment
*	**********************************************************************
*	sped up by lookuptable, calculated each time brightness offset b or contrast factor c are changed
*	outColor = c * inColor + B
*
*/

#define CONTRASTBRIGHTNESSADJUSTMENT_ALGORITHM_NAME "contrast brightness adjustment"

class ContrastBrightnessAdjustment:public IPAlgorithm
{
public:
	ContrastBrightnessAdjustment(void);
	ContrastBrightnessAdjustment( double _contrastFactor, double _brightnessOffset );
	ContrastBrightnessAdjustment( ContrastBrightnessAdjustment& _cba );

	~ContrastBrightnessAdjustment(void);

	static Ptr<IPAlgorithm> createInstance();

	void init( double _contrastFactor, double _brightnessOffset );
	void setContrast( double _contrastFactor );
	void setBrightness( double _brightnessOffset );

	double contrast() const;
	double brightness() const;

	/** returns the name of the algorithm
	*/
	string algorithmName() const;

	string description() const;


	/** getting the options */
	void getOptions( st_is::GenericMultiLevelMap<string>& _options ) const;

	/** setting the options: contrast factor, brightness offset */
	bool setOptions( st_is::GenericMultiLevelMap<string>& _options );

	/** returns a string which can be used to
	*  reinstate the algorithms state
	*/
	string options() const;

	/** reinstate the algorithm to a previous state
	*/
	bool setup( string _setting );

	/** load standard settings */
	void resetSetting();
protected:

	/** applies the algorithm to the image
	*/
	bool process( Mat& _image );

private:
	double pBrightnessOffset;
	double pContrastFactor;
	Mat pLookUpTable;

	static bool isRegistered;

	/** recalculates the lookuptable, based on pBrightnessOffest and pContrastFactor */
	void recalculateLUT();
};

