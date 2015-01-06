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

#include "Dynamics.h"


GenericObject::Dynamics::Dynamics( Ptr<GenericObject> _objectPointer )
{
	pObjectPointer = _objectPointer;
}


Ptr<GenericObject::Dynamics> GenericObject::Dynamics::createDynamics( const string& _dynamicsType, Ptr<GenericObject> _objectPointer )
{
	if( dynamicsList->count( _dynamicsType ) == 0 ) return NULL;

	return (*dynamicsList)[_dynamicsType].create(_objectPointer);
}


void GenericObject::Dynamics::setStandardSettings( const string& _dynamicsType, GenericMultiLevelMap<string>& _target )
{
	if( dynamicsList->count( _dynamicsType ) == 0 ) return;

	_target = (*dynamicsList)[_dynamicsType].options();
	return;
}


Ptr<SceneObject::State> GenericObject::Dynamics::newState( Mat& _state, RectangleRegion& /*_region*/, vector<Point>& /*_contour*/, Mat& /*_invImg*/, double _time )
{
	SceneObject::State* newState = new SceneObject::State( _state.at<float>(0), _state.at<float>(1), _time, _state.at<float>(2), 0.0, pObjectPointer->type() );
	return Ptr<SceneObject::State>(newState);
}



bool GenericObject::Dynamics::addState( Ptr<SceneObject::State> _newState, RectangleRegion& _regionOfInterest, vector<Point>& _contour )
{
	pHistory().push_back( _newState );

	if( !trace_states() )
	{
		while( pHistory().size()>3 ) pHistory().pop_front();
	}

	_regionOfInterest.points( pROI() );
	pLastContour() = _contour;

	if( pTimeSincePredictionReset()>=-1 ) pTimeSincePredictionReset()++;
	if( pTimeSincePredictionReset() > 1 ) pTimeSincePredictionReset()=-1; // prediction reset has no effect anymore

	return true;
}


bool GenericObject::Dynamics::exportData( string _filePath, string _dataPointSeparator, string _timeStepSeparator )
{
	try
	{
		std::ofstream file;
        file.open( _filePath.c_str(),std::ios_base::trunc ); // old file is overwritten

		file<<"time"<<_dataPointSeparator<<"x_pos"<<_dataPointSeparator<<"y_pos"<<_dataPointSeparator<<"angle"<<_timeStepSeparator;

        for( size_t i=0;i<pHistory().size(); i++ )
		{
			file<<pHistory()[i]->time<<_dataPointSeparator<<pHistory()[i]->x<<_dataPointSeparator<<pHistory()[i]->y<<_dataPointSeparator<<pHistory()[i]->angle<<_timeStepSeparator;
		}

		file.close();
		return true;
	}
	catch(...)
	{
		cerr<<endl<<"SceneObject::exportData:: An error occured when trying to write to file "<<_filePath<<endl;
		return false;
	}
}



Ptr<SceneObject::State> GenericObject::Dynamics::state()
{
	return pHistory().back();
}


Mat GenericObject::Dynamics::history()
{
	Mat stateHistory;

    for( size_t i=0; i<pHistory().size(); i++ )
	{
		Mat state;
		state.create(1,3,CV_64FC1);
		state.at<double>(0)=pHistory()[i]->x;
		state.at<double>(1)=pHistory()[i]->y;
		state.at<double>(2)=pHistory()[i]->angle;

		stateHistory.push_back(state);
	}
	return stateHistory;
}


Mat GenericObject::Dynamics::currentState()
{
	Mat state;
	state.create(1,3,CV_64FC1);
	state.at<double>(0)=pHistory().back()->x;
	state.at<double>(1)=pHistory().back()->y;
	state.at<double>(2)=pHistory().back()->angle;
	return state;
}


Point GenericObject::Dynamics::pos()
{
	if( pHistory().empty() ) return Point(0,0);
	return Point( (int)pHistory().back()->x, (int)pHistory().back()->y );
}


double GenericObject::Dynamics::angle()
{
	return pHistory().back()->angle;
}


Point2f GenericObject::Dynamics::direction()
{
	double actAngle = angle();
	double x = cos(actAngle);
	double y = sin(actAngle);
	return Point( x, y );
}


double GenericObject::Dynamics::area()
{
	return pHistory().back()->area;
}


double GenericObject::Dynamics::velX()
{
	if( pHistory().size()==0 ) return 0;
	else if( pHistory().size()==1 || pTimeSincePredictionReset()==0 )
	{
		return 0;
	}

	Ptr<State> last_1 = pHistory()[ pHistory().size()-1 ];
	Ptr<State> last_2 = pHistory()[ pHistory().size()-2 ];

	return (*last_1).x-(*last_2).x;
}


double GenericObject::Dynamics::velY()
{
	if( pHistory().size()==0 ) return 0;
	else if( pHistory().size()==1 || pTimeSincePredictionReset()==0 )
	{
		return 0;
	}

	Ptr<State> last_1 = pHistory()[ pHistory().size()-1 ];
	Ptr<State> last_2 = pHistory()[ pHistory().size()-2 ];

	return (*last_1).y-(*last_2).y;
}


double GenericObject::Dynamics::velAng()
{
	if( pHistory().size()==0 ) return 0;
	else if( pHistory().size()==1 || pTimeSincePredictionReset()==0 )
	{
		return 0;
	}

	Ptr<State> last_1 = pHistory()[ pHistory().size()-1 ];
	Ptr<State> last_2 = pHistory()[ pHistory().size()-2 ];

	return calcAngleVelocity( (*last_2).angle, (*last_1).angle );
}


double GenericObject::Dynamics::accX()
{
	if( pHistory().size()==0 ) return 0;
	else if( pHistory().size()==1 || pTimeSincePredictionReset()==0 )
	{
		return 0;
	}
	else if( pHistory().size()==2 || pTimeSincePredictionReset()==1 )
	{
		return 0;
	}

	Ptr<State> last_1 = pHistory()[ pHistory().size()-1 ];
	Ptr<State> last_2 = pHistory()[ pHistory().size()-2 ];
	Ptr<State> last_3 = pHistory()[ pHistory().size()-3 ];

	double velX_1 = (*last_1).x-(*last_2).x;
	double velX_2 = (*last_2).x-(*last_3).x;
	return velX_1-velX_2;
}


double GenericObject::Dynamics::accY()
{
	if( pHistory().size()==0 ) return 0;
	else if( pHistory().size()==1 || pTimeSincePredictionReset()==0 )
	{
		return 0;
	}
	else if( pHistory().size()==2 || pTimeSincePredictionReset()==1 )
	{
		return 0;
	}

	Ptr<State> last_1 = pHistory()[ pHistory().size()-1 ];
	Ptr<State> last_2 = pHistory()[ pHistory().size()-2 ];
	Ptr<State> last_3 = pHistory()[ pHistory().size()-3 ];

	double velY_1 = (*last_1).y-(*last_2).y;
	double velY_2 = (*last_2).y-(*last_3).y;
	return velY_1-velY_2;
}


double GenericObject::Dynamics::accAng()
{
	if( pHistory().size()==0 ) return 0;
	else if( pHistory().size()==1 || pTimeSincePredictionReset()==0 )
	{
		return 0;
	}
	else if( pHistory().size()==2 || pTimeSincePredictionReset()==1 )
	{
		return 0;
	}

	Ptr<State> last_1 = pHistory()[ pHistory().size()-1 ];
	Ptr<State> last_2 = pHistory()[ pHistory().size()-2 ];
	Ptr<State> last_3 = pHistory()[ pHistory().size()-3 ];

	double angSp_1 = calcAngleVelocity( (*last_2).angle, (*last_1).angle );
	double angSp_2 = calcAngleVelocity( (*last_3).angle, (*last_2).angle );
	return angSp_1-angSp_2;
}


double GenericObject::Dynamics::calcAngleVelocity( double _angleBefore, double _angleAfter )
{
	double candidate1 = _angleAfter - _angleBefore;
	double candidate2 = _angleAfter + ObjectHandler::two_pi - _angleBefore;
	double candidate3 = _angleAfter - ObjectHandler::two_pi - _angleBefore;
	
	double ac1 = abs(candidate1);
	double ac2 = abs(candidate2);
	double ac3 = abs(candidate3);

	if( ac1<ac2 && ac1<ac3 ) return candidate1;
	if( ac2<ac3 ) return candidate2;
	return candidate3;
}


RectangleRegion GenericObject::Dynamics::lastROI()
{
	return RectangleRegion( pROI()[0], pROI()[1], pROI()[2], pROI()[3] );
}


void GenericObject::Dynamics::lastContour( vector<Point>& _contour )
{
	_contour=pLastContour();
	return;
}



bool GenericObject::Dynamics::drawLayer( Mat& _img, unsigned int _layerLevel, Scalar _color )
{
	if( pHistory().empty() ) return false;
	switch( _layerLevel ){
		case 0:
			drawPath( _img,_color );
			return false;
		case 1:
			drawDirection( _img, _color );
			return false;
		case 2:
			drawName( _img );
			return true;
		default:
			return true;
	}
}



void GenericObject::Dynamics::drawPath( Mat& _img, Scalar _color )
{
	if( pHistory().size()<2 ) return;

	/*for( int i=1; i<pHistory().size(); i++ )
	{
		line( _img, Point( pHistory()[i-1]->x, pHistory()[i-1]->y ), Point( pHistory()[i]->x, pHistory()[i]->y ), _color, 1 );
	}*/

	if( GenericObject::path_length() == -2 ) // the passed image is the central path image, draw last made step on it
	{
		unsigned int lastElementId = pHistory().size();
		line( _img, Point( pHistory()[lastElementId-2]->x, pHistory()[lastElementId-2]->y ), Point( pHistory()[lastElementId-1]->x, pHistory()[lastElementId-1]->y ), Scalar( 255-_color[0],255-_color[1],255-_color[2] ), 1 );
		return;
	}
	else if( GenericObject::path_length() == -1 ) // draw last made step into the object's own path image
	{
		if( pPathMat() == NULL ) pPathMat() = new Mat( _img.size().height, _img.size().width, CV_8UC3, CV_RGB(0,0,0) );
		else if( pFramesSinceLastFade()++ == GenericObject::path_fadeout_speed() )
		{
			*pPathMat() -= Mat( pPathMat()->size().height, pPathMat()->size().width, CV_8UC3, CV_RGB(1,1,1) );
			pFramesSinceLastFade() = 0;
		}

		unsigned int lastElementId = pHistory().size();
	
		line( *pPathMat(), Point( pHistory()[lastElementId-2]->x, pHistory()[lastElementId-2]->y ), Point( pHistory()[lastElementId-1]->x, pHistory()[lastElementId-1]->y ), Scalar( 255-_color[0],255-_color[1],255-_color[2] ), 1 );

		_img -= *pPathMat();
		return;
	}
	else if( GenericObject::path_length() == 0 || GenericObject::path_length() == 1 || GenericObject::path_length() < -2 ) return;

	// a number of steps that are to be drawn is defined
	
	unsigned int pathStep = path_step_length;
	if( pathStep<1 ) pathStep=1;

	int nrOfSteps = GenericObject::path_length()/pathStep;
	int lastStepOvershoot = pHistory().size() % pathStep;
	
	
	vector<Point> path;

	// startpoint on pathStep scale
	int startPoint = pHistory().size()-nrOfSteps*pathStep; // nrOfSteps*pathStep is not equal the path_length as nrOfSteps as an integer is rounded downwards
	
	if( startPoint<0 ) startPoint = 0;
	else
	{
		path.push_back( Point(pHistory()[startPoint]->x,pHistory()[startPoint]->y) );
		startPoint += pathStep-lastStepOvershoot;
	}


    for( size_t i=startPoint; i<pHistory().size(); i+=pathStep )
	{
		path.push_back( Point(pHistory()[i]->x,pHistory()[i]->y) );
		if( i+pathStep >= pHistory().size() ) path.push_back( Point(pHistory()[pHistory().size()-1]->x,pHistory()[pHistory().size()-1]->y) );
	}
	int size = path.size();
	const cv::Point *pts = (const cv::Point*) Mat(path).data;
	polylines(_img,&pts,&size,1,false,_color,1);
	return;
}


void GenericObject::Dynamics::drawDirection( Mat& _img, Scalar _color )
{
	if( pHistory().size()==0 ) return;

	Ptr<State> current = pHistory().back();
	vector<double> myDirection = direction( current->angle );
	vector<double> myOrthDirection(2);
	myOrthDirection[0]=myDirection[1];
	myOrthDirection[1]=-myDirection[0];

	Point eins( current->x-10*myDirection[0], current->y-10*myDirection[1] );
	Point zwei( current->x+10*myDirection[0], current->y+10*myDirection[1] );

	line( _img, eins, zwei, _color, 2, 8 );
}


void GenericObject::Dynamics::drawName( Mat& _img )
{
	stringstream nameBuilder;

	string className;
	Scalar color;
	
	pObjectPointer->classProperties( className, color );
	
	nameBuilder<<"id#"<<pObjectPointer->pObjectId<<":";
	string name;
	nameBuilder>>name;
	name += className;

	Point actualPosition=pos();
	Point textPosition = actualPosition;
	
	Point offset(10,15);
	int fontFace =  FONT_HERSHEY_PLAIN;
	double fontScale = 0.7;
	int thickness = 1;
	int baseline;
	double rightBorderDistance = 5; // [px]
	double bottomBorderDistance = 5; // [px]
	int rectDistX = 2; // [px] backing rectangle: additional width left and right
	int rectDistY = 3; // [px] backing rectangle: additional height top and bottom
	Size textSize = getTextSize( name, fontFace, fontScale, thickness, &baseline );
	if( textPosition.x+offset.x+textSize.width+rightBorderDistance+rectDistX > _img.cols ) textPosition.x = _img.cols-textSize.width-rightBorderDistance-offset.x-rectDistX;
	if( textPosition.y+offset.y+textSize.height+bottomBorderDistance+rectDistY > _img.rows ) textPosition.y = _img.rows-textSize.height-bottomBorderDistance-offset.y-rectDistY;
	
	Point rectPos( textPosition.x+offset.x-rectDistX, textPosition.y+offset.y-textSize.height-rectDistY );
	Mat opaque( textSize.height+2*rectDistY, textSize.width+2*rectDistX, CV_8UC3, CV_RGB(190,190,190) );
	
	Mat toOverlay = _img( Range( rectPos.y, rectPos.y+opaque.rows ),Range( rectPos.x, rectPos.x+opaque.cols ) );
	
	toOverlay = toOverlay+opaque;
	putText( _img, name, textPosition+offset,  fontFace, fontScale, color, thickness, 8, false );
	circle( _img, actualPosition, 3, color, 2 );
	return;
}


vector<double> GenericObject::Dynamics::direction( double _angle )
{
	vector<double> direction(2);
	direction[0]=cos(_angle); direction[1]=sin(_angle);
	return direction;
}


deque< Ptr<SceneObject::State> >& GenericObject::Dynamics::pHistory()
{
	return pObjectPointer->pHistory;
}


bool& GenericObject::Dynamics::trace_states()
{
	return SceneObject::trace_states;
}


Point* GenericObject::Dynamics::pROI()
{
	return pObjectPointer->pROI;
}


vector<Point>& GenericObject::Dynamics::pLastContour()
{
	return pObjectPointer->pLastContour;
}


int& GenericObject::Dynamics::pTimeSincePredictionReset()
{
	return pObjectPointer->pTimeSincePredictionReset;
}


Ptr<Mat>& GenericObject::Dynamics::pPathMat()
{
	return pObjectPointer->pPathMat;
}


int &GenericObject::Dynamics::pFramesSinceLastFade()
{
	return pObjectPointer->pFramesSinceLastFade;
}


int GenericObject::Dynamics::getUniqueId()
{
	return ++idCounter;
}


int GenericObject::Dynamics::registerDynamics( DynamicsFacEntry _newEntry )
{
	if( dynamicsList == NULL )
	{
		dynamicsList = new map< string , DynamicsFacEntry >();
	}
	// only register new object if none with the same name doesn't exist already
	if( dynamicsList->count( _newEntry.name() )>0 )
	{
		cerr<<endl<<"Failed to register Dynamics type with name "<<_newEntry.name()<<": A Dynamics type with that name already exists.";
		return -1;
	}
	(*dynamicsList)[ _newEntry.name() ] = _newEntry;

	if( dynamicsList->size()-2 > idCounter ) idCounter = dynamicsList->size()-2;
	return ++idCounter;
}


void GenericObject::Dynamics::availableDynamics( vector<string>& _availableDynamics )
{
	if( !_availableDynamics.empty() ) _availableDynamics.clear();

	map<string,DynamicsFacEntry>::iterator it,end;
	end = dynamicsList->end();

	for( it = dynamicsList->begin(); it!=end; it++ )
	{
		_availableDynamics.push_back( it->first );
	}
	return;
}


void GenericObject::Dynamics::dynamicsInfo( vector<string>& _dynamicsInfo )
{
	if( !_dynamicsInfo.empty() ) _dynamicsInfo.clear();

	map<string,DynamicsFacEntry>::iterator it,end;
	end = dynamicsList->end();

	for( it = dynamicsList->begin(); it!=end; it++ )
	{
		_dynamicsInfo.push_back( it->second.info() );
	}
	return;
}

map< string , GenericObject::Dynamics::DynamicsFacEntry >* GenericObject::Dynamics::dynamicsList;
unsigned int GenericObject::Dynamics::idCounter = 0;


GenericObject::Dynamics::DynamicsFacEntry::DynamicsFacEntry()
{
}


GenericObject::Dynamics::DynamicsFacEntry::DynamicsFacEntry( string _name, Ptr<GenericObject::Dynamics>(*_creatorFunc)( Ptr<GenericObject> _objectPointer ), string _info, GenericMultiLevelMap<string> _options )
{
	pName = _name;
	pInfo = _info;
	pOptions = _options;
	pCreatorFunc = _creatorFunc;
}


string GenericObject::Dynamics::DynamicsFacEntry::name()
{
	return pName;
}


string GenericObject::Dynamics::DynamicsFacEntry::info()
{
	return pInfo;
}


Ptr<GenericObject::Dynamics> GenericObject::Dynamics::DynamicsFacEntry::create( Ptr<GenericObject> _objectPointer )
{
	return pCreatorFunc( _objectPointer );
}


GenericMultiLevelMap<string> GenericObject::Dynamics::DynamicsFacEntry::options()
{
	return pOptions;
}
