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

#include "ColorRangeExpansion.h"



bool ColorRangeExpansion::isRegistered = IPAlgorithm::registerAlgorithm ( COLORRANGEEXPANSION_ALGORITHM_NAME, &ColorRangeExpansion::createInstance );


ColorRangeExpansion::ColorRangeExpansion(void)
{
	resetSetting();
}


ColorRangeExpansion::~ColorRangeExpansion(void)
{
}


Ptr<IPAlgorithm> ColorRangeExpansion::createInstance()
{
	IPAlgorithm* newECRInstance = new ColorRangeExpansion();
	return Ptr<IPAlgorithm>( newECRInstance );
}


void ColorRangeExpansion::resetSetting()
{
	pSmallPeakNeglectionBandwidth = -1;
	pPeakExclusivenessWidth = 40;
	pRecalculationInterval = 0;
	pStepsToRecalculation = 1;
	pBrightestOccuringColor = 255;
	pDarkestOccuringColor = 0;
	pDarkPeakRelevanceThreshold = 0;
	return;
}


string ColorRangeExpansion::algorithmName() const
{
	return COLORRANGEEXPANSION_ALGORITHM_NAME;
}


string ColorRangeExpansion::description() const
{
	return "Linear algorithm that extends the color range of the input RGB image to the full spectrum. Seldomly occuring colors in the very dark and very bright spectrum are ignored and the room gained is used to expand the color range of the rest of the image";
}

bool ColorRangeExpansion::process( Mat& _image )
{
    /*ofstream file;
	file.open("colorrangetime.txt",std::ios_base::app);
    double time = 1000*((double)clock())/CLOCKS_PER_SEC;*/
	// update histograms and brightest peaks if necessary
	if( recalculateSettings() )
	{
		calculateHistograms( _image );
		calculateBrightestPeak( pPeakExclusivenessWidth, pSmallPeakNeglectionBandwidth );
		calculateBrightestOccuringColor();
		calculateDarkestOccuringColor();
	}

	//_image=(_image-pDarkestOccuringColor)*tExpansionFactor; //-> simple version: about two times slower than using look up table
	//convertScaleAbs(_image,_image,tExpansionFactor,-pDarkestOccuringColor*tExpansionFactor); -> about 5 ms slower than simple version
	LUT(_image ,pLookUpTable ,_image);
	
    /*file<<(1000*((double)clock())/CLOCKS_PER_SEC)-time<<endl;
    file.close();*/
	return true;
}


string ColorRangeExpansion::options() const
{
	stringstream converter;
	converter << pRecalculationInterval;

	string output;
	converter >> output;
	return output;
}


bool ColorRangeExpansion::setup( string _setting )
{
	stringstream converter;

	converter << _setting;
	converter >> pRecalculationInterval;

	return true;
}


void ColorRangeExpansion::getOptions( GenericMultiLevelMap<string>& _options ) const
{
	_options["recalculation_interval"].as<int>()=pRecalculationInterval;
	_options["recalculation_interval"]["info"].as<string>()="indicates how often the histogram is being recalculated:  0 = only the first time, negative = never (using default values for brightest and darkest color), 1 = each time, 2 = every second time, 3 = every third, etc";
	_options["brightest_occuring_color"].as<unsigned int>()=pBrightestOccuringColor;
	_options["brightest_occuring_color"]["info"].as<string>()="upper expansion end of the algorithm, can be calculated automatically (is detected automatically if recalculation_interval isn't different) Is ignored if the recalculation interval isn't <0";
	_options["darkest_occuring_color"].as<unsigned int>()=pDarkestOccuringColor;
	_options["darkest_occuring_color"]["info"].as<string>()="lower expansion end of the algorithm, can be calculated automatically (is detected automatically if recalculation_interval isn't different) Is ignored if the recalculation interval isn't <0";
	_options["small_peak_neglection_bandwidth"].as<int>()=pSmallPeakNeglectionBandwidth;
	_options["small_peak_neglection_bandwidth"]["info"].as<string>()="Defines a bandwidth over the smallest histogram peak in which all peaks are neglected: when calculating the set of peaks which are used to determine which is the brightest color that still contains actual information. every peak with value<=minPeakValue+pSmallPeakNeglectionBandwidth*minPeakValue is disregarded. Thus, for pSmallPeakNeglectionBandwidth<0 no peaks are disregarded. default: -1";
	_options["peak_exclusiveness_width"].as<int>()=pPeakExclusivenessWidth;
	_options["peak_exclusiveness_width"]["info"].as<string>()="specifies the color range over which the calculated peaks need to be a local maxima. default is 20";
	_options["dark_peak_relevance_threshold"].as<float>()=pDarkPeakRelevanceThreshold;
	_options["dark_peak_relevance_threshold"]["info"].as<string>()="defines how often a dark color must occur in order not to be neglected when calculating the darkest occuring color";
		
	return;
}


bool ColorRangeExpansion::setOptions( GenericMultiLevelMap<string>& _options )
{
	if( _options.hasKey("recalculation_interval") ) pRecalculationInterval = _options["recalculation_interval"].as<int>();
	if( _options.hasKey("brightest_occuring_color") ) pBrightestOccuringColor = _options["brightest_occuring_color"].as<unsigned int>();
	if( _options.hasKey("darkest_occuring_color") ) pDarkestOccuringColor = _options["darkest_occuring_color"].as<unsigned int>();
	if( _options.hasKey("small_peak_neglection_bandwidth") ) pSmallPeakNeglectionBandwidth = _options["small_peak_neglection_bandwidth"].as<int>();
	if( _options.hasKey("peak_exclusiveness_width") ) pPeakExclusivenessWidth = _options["peak_exclusiveness_width"].as<int>();
	if( _options.hasKey("dark_peak_relevance_threshold") ) pDarkPeakRelevanceThreshold = _options["dark_peak_relevance_threshold"].as<float>();
	
	calculateExpansionFactor();
	return true;
}


void ColorRangeExpansion::setBrightestColor( unsigned int _color)
{
	pBrightestOccuringColor = _color;
	calculateExpansionFactor();
	return;
}


void ColorRangeExpansion::setDarkestColor( unsigned int _color)
{
	pDarkestOccuringColor = _color;
	calculateExpansionFactor();
	return;
}

void ColorRangeExpansion::calculateExpansionFactor()
{
	tExpansionFactor = 255.0/(pBrightestOccuringColor-pDarkestOccuringColor);

	Mat lookUpTable(1,256,CV_8U);
	uchar* element = lookUpTable.data;
	for( int i=0; i<256; i++ )
	{
		int pixel = i-pDarkestOccuringColor;
		pixel = (pixel<0)?0:pixel;
		pixel*=tExpansionFactor;
		pixel = (pixel>255)?255:pixel;
		element[i]=pixel;
	}
	pLookUpTable=lookUpTable;
	
	return;
}


bool ColorRangeExpansion::calculateBrightestOccuringColor()
{
	//calculate upper end of image color range, starting at brightest peak
	int brightPeakEnd = pHistograms[0].total();
	int countLowLevelColors=0;

	Mat Histogram = pHistograms[0];
    for( size_t i=1 ; i<pHistograms.size(); i++ ) Histogram+=pHistograms[i];
	
    for( size_t colorId = pBrightestPeak[0]+1; colorId<Histogram.total(); colorId++ )
	{
		if( Histogram.at<float>(colorId) < 0.01*pBrightestPeak[1] )
		{
			countLowLevelColors++;
			if( countLowLevelColors>=3 )
			{
				brightPeakEnd = colorId;
				break;
			}
		}
		else countLowLevelColors=0;
	}

	if( !pBrightestPeak.empty() ) setBrightestColor(brightPeakEnd);
	else return false;

	return true;
}


bool ColorRangeExpansion::calculateDarkestOccuringColor()
{
	//find lower end of image color range
	int darkestOccuringColor = 0;

	Mat Histogram = pHistograms[0];
    for( size_t i=1 ; i<pHistograms.size(); i++ ) Histogram+=pHistograms[i];

    for( size_t colorId = 1; colorId < Histogram.total(); colorId++ )
	{
		if( Histogram.at<float>(colorId)>pDarkPeakRelevanceThreshold )
		{
			darkestOccuringColor = colorId-1;
			break;
		}
	}

	setDarkestColor(darkestOccuringColor);
	
	return true;
}


bool ColorRangeExpansion::calculateBrightestPeak( unsigned int peakExclusivenessWidth, double bottomSizePercentage )
{
	int peakCandidate;
	double peakValue;
	int minEdgeWidth = std::ceil( (double)peakExclusivenessWidth/2 );

	int edgeWidthCount;
	int lastSameHeightPeak;

    Vector< deque< Vector<unsigned int> > > peakLists;
	peakLists.resize( pHistograms.size() );

	pBrightestPeak = Vector<unsigned int>();

	// creates histograms for all channels
    for( size_t histIndex=0;histIndex<pHistograms.size();histIndex++ )
	{
		peakValue = pHistograms[histIndex].at<float>(0);
		peakCandidate=0;
		edgeWidthCount=0;
		lastSameHeightPeak=-1;

        for( size_t color=1; color<pHistograms[histIndex].total(); color++ )
		{
			if( pHistograms[histIndex].at<float>(color) > peakValue )
			{
				peakCandidate = color;
				peakValue = pHistograms[histIndex].at<float>(color);
				edgeWidthCount = 0;
				lastSameHeightPeak = -1;
			}
			else if( pHistograms[histIndex].at<float>(color) == peakValue )
			{
				lastSameHeightPeak = color; //simple version: for a series of peaks of same height, the position of the peak is calculated as the middle of the first and the final peak...
				edgeWidthCount = 0;
			}
			else // histogram value < peakValue
			{
				edgeWidthCount++;
				if( edgeWidthCount==minEdgeWidth )
				{
					if( lastSameHeightPeak==-1 )
					{
						Vector<unsigned int> infoPair;
						infoPair.push_back(peakCandidate);
						infoPair.push_back(peakValue);
						peakLists[histIndex].push_back(infoPair);
					}
					else
					{
						Vector<unsigned int> infoPair;
						infoPair.push_back( std::ceil( ((double)peakCandidate+lastSameHeightPeak)/2 ) );
						infoPair.push_back(peakValue);
						peakLists[histIndex].push_back(infoPair);
					}
					peakCandidate=-1;
					peakValue=0;
				}
			}
		}

		
		if( peakLists[histIndex].size()==0 ) break;
		
        vector< deque<Vector<unsigned int> >::iterator > invalidPeaks;

		//look if higher peaks exist at darker color inside range and remove peak if so
        for( deque<Vector<unsigned int> >::iterator peak = peakLists[histIndex].begin()+1; peak!=peakLists[histIndex].end() ; peak++ )
		{
            for( unsigned int color=(*peak)[0]-1; color>=((*peak)[0]-minEdgeWidth); color-- )
            {
				if( pHistograms[histIndex].at<float>(color) > (*peak)[1] )
				{
					invalidPeaks.push_back(peak);
                    deque<Vector<unsigned int> >::iterator tempPeak = peak;
					peak=peakLists[histIndex].erase(peak);
					peak--;
					break;
				}
			}
		}

		// find lowest peak and drop all peaks that are only bottomSizePercentage
		// times or less higher than the lowest, including the lowest

		Vector<unsigned int> minPeak=peakLists[histIndex][0];
        for( size_t peak=1; peak<peakLists[histIndex].size();peak++ )
		{
			if( peakLists[histIndex][peak][1]<minPeak[1] ) minPeak=peakLists[histIndex][peak];
		}

		//filter peaks out that are too low
		unsigned int lowerPeakHeightBoundary = minPeak[1]+bottomSizePercentage*minPeak[1];
        for( deque<Vector<unsigned int> >::iterator peak = peakLists[histIndex].begin(); peak!=peakLists[histIndex].end(); peak++ )
		{
			if( (*peak)[1]<=lowerPeakHeightBoundary ) peakLists[histIndex].erase(peak);
		}

		
		
		// find the highest peak with brightest color in the set of found peaks
		if( histIndex==0) pBrightestPeak=peakLists[histIndex].back();
		else if( pBrightestPeak[0]<peakLists[histIndex].back()[0] ) pBrightestPeak=peakLists[histIndex].back();
	}

	if( pBrightestPeak.empty() ) return false;
	
	return true;

}


bool ColorRangeExpansion::calculateHistograms( Mat& _image )
{
	
	int histSize = 256;
	float range[] = {0,256};
	const float* histRange = {range};
	bool uniform=true, accumulate=false;

	vector<Mat> splitChannels;
	split( _image,splitChannels );

	pHistograms.resize( splitChannels.size() );

    for( size_t i=0; i<splitChannels.size(); i++ )
	{
		calcHist( &splitChannels[i], 1, 0, Mat(), pHistograms[i], 1, &histSize, &histRange, uniform, accumulate );
	}

	return true;
}


bool ColorRangeExpansion::recalculateSettings()
{
    if( pRecalculationInterval<0 || (pRecalculationInterval==0 && pHistograms.size() !=0) )
	{
		return false;
	}
	else if( pStepsToRecalculation!=1 )
	{
		pStepsToRecalculation--;
		return false;
	}
	pStepsToRecalculation = pRecalculationInterval;
	return true;
}
