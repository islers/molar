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

#include "ContrastBrightnessAdjustment.h"


bool ContrastBrightnessAdjustment::isRegistered = IPAlgorithm::registerAlgorithm ( CONTRASTBRIGHTNESSADJUSTMENT_ALGORITHM_NAME, &ContrastBrightnessAdjustment::createInstance );


ContrastBrightnessAdjustment::ContrastBrightnessAdjustment(void)
{
	resetSetting();
}


ContrastBrightnessAdjustment::ContrastBrightnessAdjustment( double _contrastFactor, double _brightnessOffset )
{
	pBrightnessOffset = _brightnessOffset;
	pContrastFactor = _contrastFactor;

	recalculateLUT();
}


ContrastBrightnessAdjustment::ContrastBrightnessAdjustment( ContrastBrightnessAdjustment& _cba )
{
	pBrightnessOffset = _cba.pBrightnessOffset;
	pContrastFactor = _cba.pContrastFactor;
	
	recalculateLUT();
}


ContrastBrightnessAdjustment::~ContrastBrightnessAdjustment(void)
{

}


Ptr<IPAlgorithm> ContrastBrightnessAdjustment::createInstance()
{
	IPAlgorithm* newCBAInstance = new ContrastBrightnessAdjustment();
	return Ptr<IPAlgorithm>( newCBAInstance );
}


void ContrastBrightnessAdjustment::init( double _contrastFactor, double _brightnessOffset )
{
	pBrightnessOffset = _brightnessOffset;
	pContrastFactor = _contrastFactor;

	recalculateLUT();
	return;
}


void ContrastBrightnessAdjustment::setContrast( double _contrastFactor )
{
	pContrastFactor = _contrastFactor;

	recalculateLUT();
	return;
}


void ContrastBrightnessAdjustment::setBrightness( double _brightnessOffset )
{
	pBrightnessOffset = _brightnessOffset;

	recalculateLUT();
	return;
}


double ContrastBrightnessAdjustment::contrast() const
{
	return pContrastFactor;
}


double ContrastBrightnessAdjustment::brightness() const
{
	return pBrightnessOffset;
}


string ContrastBrightnessAdjustment::algorithmName() const
{
	return CONTRASTBRIGHTNESSADJUSTMENT_ALGORITHM_NAME;
}


string ContrastBrightnessAdjustment::description() const
{
	return "Algorithm for a simple contrast and brightness adjustment. Output colors are determined by the formula outColor = c * inColor + B, where c is the contrast and B is the brightness.";
}


bool ContrastBrightnessAdjustment::process( Mat& _image )
{
	LUT(_image ,pLookUpTable ,_image);

	return true;
}


void ContrastBrightnessAdjustment::getOptions( st_is::GenericMultiLevelMap<string>& _options ) const
{
	_options["contrast_factor"].as<double>() = pContrastFactor;
	_options["contrast_factor"]["info"].as<string>() = "Sets the contrast factor c in 'outColor = c * inColor + B'";
	_options["brightness_offset"].as<double>() = pBrightnessOffset;
	_options["brightness_offset"]["info"].as<string>() = "Sets the brightness offset B in 'outColor = c * inColor + B'";

	return;
}


bool ContrastBrightnessAdjustment::setOptions( st_is::GenericMultiLevelMap<string>& _options )
{
	if( _options.hasKey("contrast_factor") ) pContrastFactor = _options["contrast_factor"].as<double>();
	if( _options.hasKey("brightness_offset") ) pBrightnessOffset = _options["brightness_offset"].as<double>();
	
	recalculateLUT();
	return true;
}


string ContrastBrightnessAdjustment::options() const
{
	stringstream converter;
	converter << pContrastFactor<< " " << pBrightnessOffset ;

	string output1;
	string output2;
	converter >> output1 >> output2;

	return output1 + " " + output2;
}


bool ContrastBrightnessAdjustment::setup( string _setting )
{
	stringstream converter;

	converter << _setting;
	converter >> pContrastFactor;
	converter >> pBrightnessOffset;

	recalculateLUT();

	return true;
}


void ContrastBrightnessAdjustment::resetSetting()
{
	pBrightnessOffset = 0;
	pContrastFactor = 1.0;

	recalculateLUT();

	return;
}


void ContrastBrightnessAdjustment::recalculateLUT()
{
	Mat lookUpTable(1,256,CV_8U);
	uchar* element = lookUpTable.data;
	for( int i=0; i<256; i++ )
	{
		int pixel = pContrastFactor*i + pBrightnessOffset;
		if( pixel < 0 ) pixel = 0;
		if( pixel > 255 ) pixel = 255;
		element[i]=pixel;
	}
	pLookUpTable = lookUpTable;

	return;
}
