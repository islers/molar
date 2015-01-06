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
#include <deque>
#include "Options.h"

using namespace std;

/** linear algorithm that extends the color range of the input RGB image to the full spectrum 
*	Seldomly occuring colors in the very dark and very bright spectrum are deleted
*	and the room gained is used to expand the color range of the rest of the image
*/

#define	COLORRANGEEXPANSION_ALGORITHM_NAME	"color range expansion"

class ColorRangeExpansion:public IPAlgorithm
{
public:
	ColorRangeExpansion(void);
	~ColorRangeExpansion(void);

	static Ptr<IPAlgorithm> createInstance();
	static void loadStandardSettings();

	/** returns the name of the algorithm
	*/
	string algorithmName() const;

	string description() const;

	/** returns a string which can be used to
	*  reinstate the algorithms state
	*/
	string options() const;

	/** reinstate the algorithm to a previous state
	*/
	bool setup( string _setting );


	void getOptions( GenericMultiLevelMap<string>& _options) const;

	/** sets the options:
	*
	* recalculation_interval: (int) indicates how often the histogram is being recalculated:  0 = only the first time, negative = never (using default values for brightest and darkest color), 1 = each time, 2 = every second time, 3 = every third, etc
	* brightest_occuring_color: (unsigned int) upper expansion end of the algorithm, can be calculated automatically (is detected automatically if recalculation_interval isn't different) Is ignored if the recalculation interval isn't <0
	* darkest_occuring_color: (unsigned int) lower expansion end of the algorithm, can be calculated automatically (is detected automatically if recalculation_interval isn't different) Is ignored if the recalculation interval isn't <0
	* small_peak_neglection_bandwidth: (int) Defines a bandwidth over the smallest histogram peak in which all peaks are neglected: when calculating the set of peaks which are used to determine which is the brightest color that still contains actual information. every peak with value<=minPeakValue+pSmallPeakNeglectionBandwidth*minPeakValue is disregarded. Thus, for pSmallPeakNeglectionBandwidth<0 no peaks are disregarded. default: -1
	* peak_exclusiveness_width: (int) specifies the color range over which the calculated peaks need to be a local maxima. default is 20
	* dark_peak_relevance_threshold: (float) defines how often a dark color must occur in order not to be neglected when calculating the darkest occuring color
	*
	*/
	bool setOptions( GenericMultiLevelMap<string>& _options );

	void setBrightestColor( unsigned int _color);
	void setDarkestColor( unsigned int _color);

	//option setup
	void resetSetting();
private:

	/** applies the algorithm to the image
	*/
	bool process( Mat& _image );

	vector<Mat> pHistograms;
	Vector<unsigned int> pBrightestPeak; //empty if no peak was found otherwhise: [color, nr of occurances in image]
	unsigned int pStepsToRecalculation;
	
	/** calculates the brightest occuring color, (which means it basically searches for the upper end of the brightest peak */
	bool calculateBrightestOccuringColor();

	/** searches for the darkest color peak, first color that occurs at least pDarkPeakRelevanceThreshold times*/
	bool calculateDarkestOccuringColor();

	/** calculates the brightest peak over all the histograms and writes it into the class member, bottomSizePercentage<=-1 doesn't neglect any peaks after they have been calculated
	* @return	false	if no peaks have been found
	*/
	bool calculateBrightestPeak( unsigned int peakExclusivenessWidth, double bottomSizePercentage );

	/** calculates the histogram for every channel of the image
	*	@ return: true if new histograms have been calculated
	*/
	bool calculateHistograms( Mat& _image );

	/** returns true if at this step peak and histograms should be recalculated */
	bool recalculateSettings();

	/** temporary storage for expansion factor, gets calculated through pBrightestOccuringColor and pDarkestOccuringColor */
	double tExpansionFactor;
	Mat pLookUpTable;
	void calculateExpansionFactor();

	static bool isRegistered;
	
	/* parameters ****************************************************************************************************/

	/** upper expansion end of the algorithm, can be calculated automatically, access only through setBrightestColor(uint) */
	unsigned int pBrightestOccuringColor;

	/** lower expansion end of the algorithm, can be calculated automatically, acces only through setDarkestColor(uint) */
	unsigned int pDarkestOccuringColor;

	/** defines how often a dark color must occur in order not to be neglected when calculating the darkest occuring color*/
	float pDarkPeakRelevanceThreshold;

	/** indicates how often the histogram is being recalculated
	* 0 = only the first time, negative = never (using default values for brightest and darkest color), 1 = each time, 2 = every second time, 3 = every third, etc */
	int pRecalculationInterval;

	/** Defines a bandwidth over the smallest histogram peak in which all peaks are neglected
	*  when calculating the set of peaks which are used to determine which is the brightest color
	* that still contains actual information. every peak with value<=minPeakValue+pSmallPeakNeglectionBandwidth*minPeakValue
	* is disregarded. Thus, for pSmallPeakNeglectionBandwidth<0 no peaks are disregarded. default: -1
	*/
	int pSmallPeakNeglectionBandwidth;

	/** specifies the color range over which the calculated peaks need to be a local maxima
	* default: 20
	*/
	int pPeakExclusivenessWidth;
};

