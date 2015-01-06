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


#include "SceneHandler.h"
#include <iostream>

SceneHandler::SceneHandler( bool _initializeAllObjectClasses ):pObjectSet( this,&pVideo ),pObservationArea()
{
	pFrameRate = standardFrameRate;
	setupOptions();
	
	if( _initializeAllObjectClasses )
	{
		vector<string> typesNotInTheScene; // none...
		typesNotInScene( typesNotInTheScene ); // initializes all classes
	}
	// else: not necessary since it is standard that classes are not initialized

}

SceneHandler::SceneHandler( double _frameRate ):pObjectSet( this,&pVideo ),pObservationArea()
{
	setFrameRate(_frameRate);
	setupOptions();
}


SceneHandler::~SceneHandler(void)
{
	/*Mat descriptorSet;
	//descriptorSet.push_back( ThickHelix::descriptors1 );
	for( int i=0;i<17; i++ ) descriptorSet.push_back( ThickHelix::descriptors1 );
	descriptorSet.push_back( ThickHelix::descriptors2 );
	//descriptorSet.push_back( ThickHelix::descriptors3 );
	cout<<endl<<"Number of thick helix samples: "<<ThickHelix::descriptors1.size().height;
	cout<<endl<<"Number of thin helix samples: "<<ThickHelix::descriptors2.size().height;
	//cout<<endl<<"Number of negative samples: "<<ThickHelix::descriptors3.size().height;
	
	Mat posAnswer, negAnswer, dataLabel, first, second, third;
	CvBoost boost;

	// 1 *************************
	posAnswer = Mat::ones( 17*ThickHelix::descriptors1.size().height, 1,CV_32F );
	negAnswer = Mat::zeros( ThickHelix::descriptors2.size().height, 1,CV_32F );

	dataLabel = posAnswer;
	dataLabel.push_back(negAnswer);
	
	boost.clear();
	boost.train( descriptorSet, CV_ROW_SAMPLE, dataLabel );
	boost.save("CrystalBoostClassifier_std.xml.gz","standard_boost_classifier");*/
	
	// 2 *************************
	/*first = Mat::zeros( ThickHelix::descriptors1.size().height, 1,CV_32F );
	second = Mat::ones( ThickHelix::descriptors2.size().height, 1,CV_32F );
	third = Mat::zeros( ThickHelix::descriptors3.size().height, 1,CV_32F );

	dataLabel = first;
	dataLabel.push_back(second);
	dataLabel.push_back(third);
	
	boost.clear();
	boost.train( descriptorSet, CV_ROW_SAMPLE, dataLabel );
	boost.save("ThinHelixBoostClassifier_std.xml.gz","standard_boost_classifier");*/
	saveProject("lastproject.swsc");
}


void SceneHandler::setupOptions()
{
	Options::load_options();
	buffering_activated = (*Options::General)["runtime"]["buffer"]["activated"].as<bool>();
	draw_observation_area = (*Options::General)["display"]["general"]["draw_observation_area"].as<bool>();
	draw_observation_area_color = Scalar( (*Options::General)["display"]["general"]["draw_observation_area"]["B"].as<double>(), (*Options::General)["display"]["general"]["draw_observation_area"]["G"].as<double>(), (*Options::General)["display"]["general"]["draw_observation_area"]["R"].as<double>() );
	create_preprocess_filter_image = (*Options::General)["display"]["objects"]["create_preprocess_filter_image"].as<bool>();
	create_prethreshold_filter_image = (*Options::General)["display"]["objects"]["create_prethreshold_filter_image"].as<bool>();
}
bool SceneHandler::create_preprocess_filter_image;
bool SceneHandler::create_prethreshold_filter_image;
bool SceneHandler::draw_observation_area;
Scalar SceneHandler::draw_observation_area_color;
bool SceneHandler::buffering_activated;



bool SceneHandler::saveProject( string _filePath )
{
	// using GenericMultiLevelMap object
	GenericMultiLevelMap<string> projectSettings;
	projectSettings["general_options"] = (*Options::General);

	//preprocessing
	int i1=0;
    list<Ptr<IPAlgorithm> >::iterator it1=pPreProcessStack.begin();
	for( ; it1!=pPreProcessStack.end(); it1++,i1++ )
	{
		stringstream conv; string iter;
		conv<<i1; conv>>iter;
		projectSettings["preprocessing"]["filter"+iter].as<string>()=(*it1)->algorithmName();
		projectSettings["preprocessing"]["filter"+iter]["activated"].as<bool>() = (*it1)->active;
		(*it1)->getOptions( projectSettings["preprocessing"]["filter"+iter]["options"] );
	}

	//internal preprocessing
	int i2=0;
    list<Ptr<IPAlgorithm> >:: iterator it2=pInternPreProcessStack.begin();
	for( ; it2!=pInternPreProcessStack.end(); it2++,i2++ )
	{
		stringstream conv; string iter;
		conv<<i2; conv>>iter;
		projectSettings["internpreprocessing"]["filter"+iter].as<string>()=(*it2)->algorithmName();
		projectSettings["internpreprocessing"]["filter"+iter]["activated"].as<bool>() = (*it2)->active;
		(*it2)->getOptions( projectSettings["internpreprocessing"]["filter"+iter]["options"] );
	}

	projectSettings["observation_area"]["x"].as<int>() = pObservationArea.x;
	projectSettings["observation_area"]["y"].as<int>() = pObservationArea.y;
	projectSettings["observation_area"]["width"].as<int>() = pObservationArea.width;
	projectSettings["observation_area"]["height"].as<int>() = pObservationArea.height;

	pObjectSet.saveSettings( projectSettings["objects"] );

	try
	{
		projectSettings.saveToXML(_filePath);
		return true;
	}
	catch(...)
	{
		cerr<<endl<<"Could not safe the project.";
		return false;
	}
}



bool SceneHandler::loadFromFile( string _filePath )
{
	GenericMultiLevelMap<string> prjSettings;

	try
	{
		prjSettings.initFromXML(_filePath);

		// options
		if( prjSettings.hasKey("general_options") ) (*Options::General).mirrorTwins( prjSettings["general_options"],GenericMultiLevelMap<string>::IGNOREEMPTYSTRING );
		
		// preprocessing
		if( prjSettings.hasKey("preprocessing") )
		{
			for( GenericMultiLevelMap<string>::iterator it = prjSettings["preprocessing"].begin(); it!=prjSettings["preprocessing"].end(); it++ )
			{
				Ptr<IPAlgorithm> newAlg = addPreProcessingAlgorithm( (*it).second.as<string>(), (*it).second["options"] );
				newAlg->active = (*it).second["activated"].as<bool>();
			}
		}
		
		// internal preprocessing
		if( prjSettings.hasKey("internpreprocessing") )
		{
			for( GenericMultiLevelMap<string>::iterator it = prjSettings["internpreprocessing"].begin(); it!=prjSettings["internpreprocessing"].end(); it++ )
			{
				Ptr<IPAlgorithm> newAlg = addInternPreProcessingAlgorithm( (*it).second.as<string>(), (*it).second["options"] );
				newAlg->active = (*it).second["activated"].as<bool>();
			}
		}
		
		if( prjSettings.hasKey("observation_area") )
		{
			if( prjSettings["observation_area"].hasKey("x") &&
				prjSettings["observation_area"].hasKey("y") &&
				prjSettings["observation_area"].hasKey("width") &&
				prjSettings["observation_area"].hasKey("height") )
			{
				pObservationArea = Rect( prjSettings["observation_area"]["x"].as<int>(), prjSettings["observation_area"]["y"].as<int>(), prjSettings["observation_area"]["width"].as<int>(), prjSettings["observation_area"]["height"].as<int>() );
			}
		}

		if( prjSettings.hasKey("objects") )
		{
			pObjectSet.loadFromMap( prjSettings["objects"] );
		}
		

		return true;
	}
	catch(...)
	{
		return false;
	}
}



void SceneHandler::pushFrame( Mat _frame, double _time )
{
	pTime = msTime(); // safe the actual global time
	//ellipse(_frame, Point(30,30), Size(20,18),40,0,360,Scalar(40,40,40),3);
	/*if( true||count<175 ) // write additional object into video
	{
		stringstream converter;
		string id;
		converter<<count;
		converter>>id;
		converter.clear();

		Mat img = imread( "D:\\Benutzer\\stewess\\Documents\\ETH\\FS 2014\\Bachelorarbeit\\negativ set\\noabf - Kopie (199).png", CV_LOAD_IMAGE_COLOR);
		//Mat img = imread( "D:\\Benutzer\\stewess\\Documents\\ETH\\FS 2014\\Bachelorarbeit\\single robot set\\1\\extract3.png", CV_LOAD_IMAGE_COLOR);
		bitwise_not(img,img);
		
		int longerSide = max(img.rows,img.cols);
		Mat quadImg = Mat::ones(Size(longerSide+1,longerSide+1),CV_8UC3);
		Mat quadPart = quadImg( Range(longerSide/2-img.rows/2,longerSide/2-img.rows/2+img.rows),Range(longerSide/2-img.cols/2,longerSide/2-img.cols/2+img.cols) );
		quadPart += img;
		
		double degree=sin((double)count/1000)*600*sin((double)count/100);
		Mat rot = getRotationMatrix2D( Point(longerSide/2,longerSide/2),degree,1);
		warpAffine( quadImg,quadImg,rot,quadImg.size() );
		
		Point center(quadImg.cols/2,quadImg.rows/2);
		double count = (double)SceneHandler::count/5;
		Point trajection( 10+10*sin(((double)count)/10),20+20*sin(((double)count)/20));
		//Point trajection(count/4+max(count/2*sin(((double)count)/20),0),count+20*sin(((double)count)/20));
		Point pos = center+trajection;
		
		if( pos.y+quadImg.rows/2<=_frame.rows && pos.x+quadImg.cols/2<=_frame.cols )
		{
			bitwise_not(_frame,_frame);
			Mat partImg = _frame( Range(pos.y-quadImg.rows/2,pos.y-quadImg.rows/2+quadImg.rows),Range(pos.x-quadImg.cols/2,pos.x-quadImg.cols/2+quadImg.cols) );
			partImg+=quadImg;
			bitwise_not(_frame,_frame);
		}
	}
	count++;*/
	// initial processing and conversion
	//double time1 = msTime();

	initialProcessing( _frame );
	
	// apply pre processing stage to edit image
    //double t=msTime();
	Mat toEdit=_frame.clone();
	
	
	// preprocessing
	preProcess( toEdit );

	if( create_preprocess_filter_image ) toEdit.copyTo( pPreProcessImage );
	
	//double time2 = msTime();

	// buffering
	Mat forCalculations; //use only grey image for further processing
	if( buffering_activated )
	{
        Frame toAdd(_frame, toEdit, _time );
        pVideo << toAdd;
		forCalculations = pVideo.grey();
	}
	else
	{
		Frame myFrame( _frame, toEdit, _time );
		pWorkingFrame = myFrame.grey();
		forCalculations = pWorkingFrame;
	}
	
	Mat displayOutput = pVideo.loadMat();

	// test if an observation range has been specified
	if( pObservationArea.area()!=0 )
	{
		int xMin = max( pObservationArea.x, 0 );
		int xMax = min( pObservationArea.x+pObservationArea.width, forCalculations.size().width );
		int yMin = max( pObservationArea.y, 0 );
		int yMax = min( pObservationArea.y+pObservationArea.height, forCalculations.size().height );
		Rect draw(xMin,yMin,xMax-xMin,yMax-yMin);
		if( draw_observation_area ) rectangle(displayOutput, draw, draw_observation_area_color );
		Range colRange( xMin, xMax );
		Range rowRange( yMin, yMax );

		forCalculations = forCalculations(rowRange,colRange);
		displayOutput = displayOutput(rowRange,colRange);
	}
	
	//double time3 = msTime();
	// intern preprocessing
	preProcessIntern( forCalculations );


	if( create_prethreshold_filter_image ) forCalculations.copyTo( pPreThresholdImage );

	//double time4 = msTime();

	
	/*ofstream file;
	file.open( "chronometries.txt", ios_base::app );
	file<<time2-time1<<" "<<time3-time2<<" "<<time4-time3<<" "<<time4-time1<<" ";
	file.close();*/


	pObjectSet.pushFrame( forCalculations, displayOutput, _time );
	//double time5 = msTime();
	
	/*file.open( "chronometries.txt", ios_base::app );
	file<<" "<<time5-time1<<endl;
	file.close();*/
	
	return;
}



Mat const& SceneHandler::getOriginalFrame( unsigned int _frameNumber ) const
{
	if( !buffering_activated ) return pWorkingFrame;
	return pVideo.loadOriginal(_frameNumber);
}

Mat SceneHandler::operator[]( unsigned int _frameNumber )
{
	if( !buffering_activated ) return pWorkingFrame;
	return pVideo[_frameNumber].edited();
}

Mat SceneHandler::last()
{
	if( !buffering_activated ) return pWorkingFrame;
	Mat lastEdited;
	pVideo >> lastEdited;
	return lastEdited;
}

Mat SceneHandler::grey()
{
	if( !buffering_activated ) return pWorkingFrame;
	Mat greyImg = pVideo.grey();
	return greyImg;
}

Mat SceneHandler::resized( double _factor )
{
	Mat resizedVersion;
	resize( last(), resizedVersion, Size( last().size().width*_factor, last().size().height*_factor ) );
	return resizedVersion;
}

Mat SceneHandler::operator>>( Mat& _frame )
{
	if( !buffering_activated )
	{
		_frame = pWorkingFrame;
		return pWorkingFrame;
	}
	pVideo >> _frame;
	return _frame;
}

bool SceneHandler::operator<<( VideoCapture& _vc )
{
	Mat newframe;

	double time = _vc.get(CV_CAP_PROP_POS_MSEC);
	_vc >> newframe;

	if( !newframe.data )
	{
		cout<<endl<<"Video ended."<<endl;
		return false;
	}

	pushFrame( newframe, time );
	return true;
}


double SceneHandler::timeLeft()
{
	return (pTime+1000/pFrameRate) - msTime();
}


double SceneHandler::getFrameRate() const
{
	return pFrameRate;
}

void SceneHandler::setFrameRate( double& _newFPS )
{
	pFrameRate = _newFPS;
	return;
}


void SceneHandler::setFrameRate( VideoCapture& _vc )
{
	double newFPS = _vc.get(CV_CAP_PROP_FPS);
    if( isnan(newFPS) ) pFrameRate = (*Options::General)["general_settings"]["user_set_framerate"].as<int>();
    else pFrameRate = newFPS;
	return;
}


// [GENERAL IMAGE CONVERSION]
// ***************************************************************************************************************

bool SceneHandler::initialProcessing( Mat& /*_image*/ )
{
	return true;
}


// [PREPROCESSING] video frame preprocessing functions and variables
// ***************************************************************************************************************
bool SceneHandler::preProcess( Mat& _image )
{
	bool success=true;

	for( list< Ptr<IPAlgorithm> >::iterator it=pPreProcessStack.begin(); it!= pPreProcessStack.end() ; it++ )
	{
		if( (**it).active ) success = success && (**it).apply(_image);
	}
	return success;
}

bool SceneHandler::preProcessIntern( Mat& _image )
{
	bool success=true;

	for( list< Ptr<IPAlgorithm> >::iterator it=pInternPreProcessStack.begin(); it!= pInternPreProcessStack.end() ; it++ )
	{
		if( (**it).active ) success = success && (**it).apply(_image);
	}
	return success;
}

list<string> SceneHandler::preProcessStackInfo()
{
	list<string> info;

	
	for( list< Ptr<IPAlgorithm> >::iterator it=pPreProcessStack.begin(); it!= pPreProcessStack.end() ; it++ )
	{
		info.push_back( (**it).algorithmName() );
	}

	return info;
}

list<string> SceneHandler::internPreProcessStackInfo()
{
	list<string> info;

	
	for( list< Ptr<IPAlgorithm> >::iterator it=pInternPreProcessStack.begin(); it!= pInternPreProcessStack.end() ; it++ )
	{
		info.push_back( (**it).algorithmName() );
	}

	return info;
}

Ptr<IPAlgorithm> SceneHandler::addPreProcessingAlgorithm( string _name, GenericMultiLevelMap<string> _options, int _pos )
{
	Ptr<IPAlgorithm> toAdd = IPAlgorithm::createAlgorithm( _name );
	toAdd->setOptions( _options );

	if( toAdd.empty() ) return NULL;

    if( _pos < 0 || (unsigned int)_pos >= pPreProcessStack.size() )
	{
		pPreProcessStack.push_back( toAdd );
		return toAdd;
	}
	else if( _pos==0 )
	{
		pPreProcessStack.push_front( toAdd );
		return toAdd;
	}

	list< Ptr<IPAlgorithm > >::iterator insertPos;
	int i=_pos;
	for( list< Ptr<IPAlgorithm> >::iterator it=pPreProcessStack.begin(); it!=pPreProcessStack.end() ; it++,i-- )
	{
		if( i==0 )
		{
			insertPos=it;
			break;
		}
	}
	pPreProcessStack.insert( insertPos, toAdd );
	return toAdd;
}

void SceneHandler::removePreProcessingAlgorithm( int _idx )
{
	list< Ptr<IPAlgorithm> >::iterator it,end;
	it = pPreProcessStack.begin();
	end = pPreProcessStack.end();
	int i = 0;

	for( ; it!=end ; it++, i++ )
	{
		if( i==_idx )
		{
			pPreProcessStack.erase( it );
			break;
		}
	}
	return;
}

void SceneHandler::movePreProcessAlgorithmUp( int _idx )
{
	if( _idx == 0 ) return;

	list< Ptr<IPAlgorithm> >::iterator it,end;
	it = pPreProcessStack.begin();
	end = pPreProcessStack.end();
	int i = 0;

	for( ; it!=end ; it++, i++ )
	{
		if( i==_idx )
		{
			Ptr<IPAlgorithm> foundAlgorithm = (*it);
			list< Ptr<IPAlgorithm> >::iterator algorithmPtr = pPreProcessStack.erase( it );
			algorithmPtr--;
			pPreProcessStack.insert( algorithmPtr, foundAlgorithm );
			break;
		}
	}
	return;
}

void SceneHandler::movePreProcessAlgorithmDown( int _idx )
{
    if( (unsigned int)_idx == (pPreProcessStack.size()-1) && _idx>=0 ) return;

	list< Ptr<IPAlgorithm> >::iterator it,end;
	it = pPreProcessStack.begin();
	end = pPreProcessStack.end();
	int i = 0;

	for( ; it!=end ; it++, i++ )
	{
		if( i==_idx )
		{
			Ptr<IPAlgorithm> foundAlgorithm = (*it);
			list< Ptr<IPAlgorithm> >::iterator algorithmPtr = pPreProcessStack.erase( it );
			algorithmPtr++;
			pPreProcessStack.insert( algorithmPtr, foundAlgorithm );
			break;
		}
	}
	return;
}

Ptr<IPAlgorithm> SceneHandler::addInternPreProcessingAlgorithm( string _name, GenericMultiLevelMap<string> _options, int _pos )
{
	Ptr<IPAlgorithm> toAdd = IPAlgorithm::createAlgorithm( _name );
	toAdd->setOptions( _options );

	if( toAdd.empty() ) return NULL;

    if( _pos < 0 || (unsigned int)_pos >= pInternPreProcessStack.size() )
	{
		pInternPreProcessStack.push_back( toAdd );
		return toAdd;
	}
	else if( _pos==0 )
	{
		pInternPreProcessStack.push_front( toAdd );
		return toAdd;
	}

	list< Ptr<IPAlgorithm > >::iterator insertPos;
	int i=_pos;
	for( list< Ptr<IPAlgorithm> >::iterator it=pInternPreProcessStack.begin(); it!=pInternPreProcessStack.end() ; it++,i-- )
	{
		if( i==0 )
		{
			insertPos=it;
			break;
		}
	}
	pInternPreProcessStack.insert( insertPos, toAdd );
	return toAdd;
}

void SceneHandler::removeInternPreProcessingAlgorithm( int _idx )
{
	list< Ptr<IPAlgorithm> >::iterator it,end;
	it = pInternPreProcessStack.begin();
	end = pInternPreProcessStack.end();
	int i = 0;

	for( ; it!=end ; it++, i++ )
	{
		if( i==_idx )
		{
			pInternPreProcessStack.erase( it );
			break;
		}
	}
	return;
}

void SceneHandler::moveInternPreProcessAlgorithmUp( int _idx )
{
	if( _idx == 0 ) return;

	list< Ptr<IPAlgorithm> >::iterator it,end;
	it = pInternPreProcessStack.begin();
	end = pInternPreProcessStack.end();
	int i = 0;

	for( ; it!=end ; it++, i++ )
	{
		if( i==_idx )
		{
			Ptr<IPAlgorithm> foundAlgorithm = (*it);
			list< Ptr<IPAlgorithm> >::iterator algorithmPtr = pInternPreProcessStack.erase( it );
			algorithmPtr--;
			pInternPreProcessStack.insert( algorithmPtr, foundAlgorithm );
			break;
		}
	}
	return;
}

void SceneHandler::moveInternPreProcessAlgorithmDown( int _idx )
{
    if( (unsigned int)_idx == (pInternPreProcessStack.size()-1) && _idx>=0 ) return;

	list< Ptr<IPAlgorithm> >::iterator it,end;
	it = pInternPreProcessStack.begin();
	end = pInternPreProcessStack.end();
	int i = 0;

	for( ; it!=end ; it++, i++ )
	{
		if( i==_idx )
		{
			Ptr<IPAlgorithm> foundAlgorithm = (*it);
			list< Ptr<IPAlgorithm> >::iterator algorithmPtr = pInternPreProcessStack.erase( it );
			algorithmPtr++;
			pInternPreProcessStack.insert( algorithmPtr, foundAlgorithm );
			break;
		}
	}
	return;
}


const Mat& SceneHandler::preProcessImage()
{
	return pPreProcessImage;
}



const Mat& SceneHandler::preThresholdImage()
{
	return pPreThresholdImage;
}



ObjectHandler& SceneHandler::objects()
{
	return pObjectSet;
}


void SceneHandler::typesInScene( vector<string>& _names )
{
	pObjectSet.typesInScene(_names);
	return;
}


void SceneHandler::typesInScene( string _name1, string _name2, string _name3, string _name4, string _name5, string _name6, string _name7, string _name8 )
{
	vector<string> names;

	names.push_back(_name1);
	if(_name2!="") names.push_back(_name2);
	if(_name3!="") names.push_back(_name3);
	if(_name4!="") names.push_back(_name4);
	if(_name5!="") names.push_back(_name5);
	if(_name6!="") names.push_back(_name6);
	if(_name7!="") names.push_back(_name7);
	if(_name8!="") names.push_back(_name8);
	
	pObjectSet.typesInScene(names);
	return;
}


void SceneHandler::typesNotInScene( vector<string>& _names )
{
	pObjectSet.typesNotInScene(_names);
	return;
}


void SceneHandler::typesNotInScene( string _name1, string _name2, string _name3, string _name4, string _name5, string _name6, string _name7, string _name8 )
{
	vector<string> names;

	names.push_back(_name1);
	if(_name2!="") names.push_back(_name2);
	if(_name2!="") names.push_back(_name3);
	if(_name2!="") names.push_back(_name4);
	if(_name2!="") names.push_back(_name5);
	if(_name2!="") names.push_back(_name6);
	if(_name2!="") names.push_back(_name7);
	if(_name2!="") names.push_back(_name8);

	pObjectSet.typesNotInScene(names);
	return;
}


void SceneHandler::setObservationArea( int _upperLeftX, int _upperLeftY, int _width, int _height )
{
	pObservationArea = Rect( _upperLeftX, _upperLeftY, _width, _height );
	return;
}


const Rect& SceneHandler::getObservationArea()
{
	return pObservationArea;
}


void SceneHandler::setThreshold( double _threshold )
{
	pObjectSet.setThreshold( _threshold );
	return;
}


double SceneHandler::getThreshold()
{
	return pObjectSet.pThreshold;
}
