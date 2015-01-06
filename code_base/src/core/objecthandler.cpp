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
#include "GenericObject.h"

#define SHOWMATCHINGSTEPS 0

ObjectHandler::ObjectHandler(SceneHandler* _sceneLink, VideoBuffer* _videoLink):pScene(_sceneLink),pVideo(_videoLink), pObjectTypesInScene(),pActualTime(0)
{
	setupOptions();
	pTimeToRecalculation = recalculation_interval;
	pPathMat = NULL;
	pFramesSinceLastFade = 0;

	if( !_sceneLink->last().empty() )
	{
		pImageHeight = _sceneLink->last().size().height;
		pImageWidth = _sceneLink->last().size().width;
	}
	else
	{
		pImageHeight = 0;
		pImageWidth = 0;
	}

	pair<Ptr<SceneObject>,int> emptyCounter;
	emptyCounter.first=Ptr<SceneObject>();
	emptyCounter.second=-1;
	pMissingCount.push_back(emptyCounter);
	pThreshold = static_threshold;
	pEstimatedClassificationTime = (*Options::General)["automatically_updated_program_data"]["classification_update_time_estimate"].as<double>();


	setupObjectTypesInfo(false); // standard setting is that no object classes are initialized or set as occuring in scene
}


ObjectHandler::~ObjectHandler(void)
{
	(*Options::General)["automatically_updated_program_data"]["classification_update_time_estimate"].as<double>() = pEstimatedClassificationTime;
	
}


void ObjectHandler::saveSettings( GenericMultiLevelMap<string>& _settings )
{
	

    for( size_t i=0 ; i<pObjectTypesInScene.size(); i++ )
	{
		stringstream converter;
		string count;
		converter<<i;
		converter>>count;

		// object types in scene
		if(pObjectTypesInScene[i])
		{
			_settings["object_types_in_scene"][ "type"+count ].as<string>()=(*SceneObject::objectList)[i];
		}

		// class options (can be saved as part of the scene instead of always altering the classes' standard settings which are saved in their own files
		 _settings["object_class_settings"][ "class"+count ].as<string>() = (*SceneObject::objectList)[i];
		(*SceneObject::getOptionsFunctionList)[i]( _settings["object_class_settings"][ "class"+count ],i );

	}
	_settings["classification_time_estimate"].as<double>()=pEstimatedClassificationTime;
	
	_settings["threshold"].as<double>() = pThreshold;	


	return;
}


void ObjectHandler::loadFromMap( GenericMultiLevelMap<string>& _map )
{
	// object class settings - needs to happen before types in scene because the correct features need to be set as active or/and inactive for generic object classes
	if( _map.hasKey("object_class_settings") )
	{
		for( GenericMultiLevelMap<string>::iterator it = _map["object_class_settings"].begin(); it != _map["object_class_settings"].end(); it++ )
		{
			string className = (*it).second.as<string>();

			int classId = SceneObject::getClassId( className );
			if( classId!=-1 ) (*SceneObject::setOptionsFunctionList)[classId]( (*it).second, classId );
		}
	}

	if( _map.hasKey("object_types_in_scene") )
	{
		vector<string> objectsInScene;
		for( GenericMultiLevelMap<string>::iterator it = _map["object_types_in_scene"].begin(); it!= _map["object_types_in_scene"].end(); it++ )
		{
			objectsInScene.push_back( (*it).second.as<string>() );
		}
		typesInScene( objectsInScene );
	}
	else // initialize empty
	{
		vector<string> empty;
		typesInScene(empty);
	}
	
	if( _map.hasKey("classification_time_estimate") ){ pEstimatedClassificationTime = _map["classification_time_estimate"].as<double>(); }
	
	if( _map.hasKey("threshold") ) pThreshold = _map["threshold"].as<double>();

	return;
}


void ObjectHandler::buildObjList( vector< list<Ptr<SceneObject> >::iterator >& _objList )
{
    for( list<Ptr<SceneObject> >::iterator it=pCategorized.begin(); it!=pCategorized.end(); it++ ) _objList.push_back( it );
    for( list<Ptr<SceneObject> >::iterator it=pUncategorized.begin(); it!=pUncategorized.end(); it++ ) _objList.push_back( it );
    for( list<Ptr<SceneObject> >::iterator it=pMissing.begin(); it!=pMissing.end(); it++ ) _objList.push_back( it );
	return;
}


void ObjectHandler::pushFrame( Mat& _img, Mat& _outputImage, double _time )
{

	//double time1 = SceneHandler::msTime();

	pImageHeight = _img.size().height;
	pImageWidth = _img.size().width;
	pActualTime = _time;
	/*cout<<endl<<"Nr of categorized objects:"<<pCategorized.size();
	cout<<endl<<"Nr of uncategorized objects:"<<pUncategorized.size();
	cout<<endl<<"Nr of missing objects:"<<pMissing.size();
	cout<<endl<<"Nr of lost objects:"<<pLost.size()<<endl;*/
	
	// useful image versions
	Mat greyImg = _img;
	Mat invGreyImg;
	Mat binaryImg;

	bitwise_not( greyImg, invGreyImg ); // calculates image inverse
	threshold( greyImg, binaryImg, pThreshold, 255, THRESH_BINARY_INV ); // calculates inverted binary black white image
	Mat corners;
	//imshow("binary",binaryImg);
	// list with all active objects
    vector< list<Ptr<SceneObject> >::iterator > objList;
	buildObjList( objList );

	/*for( int i=0;i<objList.size();i++ )
	{
		
		vector<Point> predictedROI;
		( **objList[ i ] ).predictROI( predictedROI );
		//cout<<endl<<"Predicted region "<<i<<": "<<Mat(predictedROI);
		RectangleRegion predictedRegion( predictedROI[0], predictedROI[1], predictedROI[2], predictedROI[3] );
		predictedRegion.draw(_outputImage,Scalar(38,38,255) );
	}*/

	// find objects in frame

	//Mat binaryColor; // necessary in a separate if statement because the contour operation alters the binary input image
	if( create_threshold_detection_image )
	{
		binaryImg.copyTo(pThresholdImage);
		cv::cvtColor(pThresholdImage,pThresholdImage, CV_GRAY2BGR );
	}

	vector< vector<Point> > contours;
	vector< RectangleRegion > regions;
	objRegions( binaryImg, contours, regions );

	
	//for(int i=0;i<regions.size();i++) regions[i].draw(colorImg,Scalar(30,20,240),2);

	if( create_threshold_detection_image )
	{
		//pThresholdImage = binaryColor;
		drawContours( pThresholdImage, contours, -1, Scalar(250,180,110),2 );

		if( draw_predicted_regions )
		{
            for( size_t i=0;i<objList.size();i++ )
			{
		
				vector<Point> predictedROI;
				( **objList[ i ] ).predictROI( predictedROI );
				RectangleRegion predictedRegion( predictedROI[0], predictedROI[1], predictedROI[2], predictedROI[3] );
				predictedRegion.draw(pThresholdImage,Scalar(250,230,200) );
			}
		}
	}

	
	// find roi regions
	vector<Rect> roiRegions;
	calculateROIs( regions, roiRegions );
		
	//for(int i=0;i<regions.size();i++) rectangle( colorImg, roiRegions[i],Scalar(30,20,240),2);

	// set RotatedRectangles relative to the ROIs
	vector<RectangleRegion> transformedRegions;
	transformToRelative( roiRegions, regions, transformedRegions );
	
	// calculate sub images that contain the found objects
	vector<Mat> invROIs;
	calculateROIMats( invGreyImg, roiRegions, invROIs );
	

	/*Mat colorImg( binaryImg.size(),CV_8UC3);
	cvtColor(binaryImg,colorImg,CV_GRAY2RGB);
	resize(invROIs[1],colorImg,Size(3*invROIs[1].size().width,3*invROIs[1].size().height) );
	imwrite( "D:\\Benutzer\\stewess\\Documents\\ETH\\FS 2014\\Bachelorarbeit\\Matlab\\subimage.jpg",colorImg ); exit(1);
	*/
	// calculate states of the object regions found
	Mat objectStates;
	roiProperties( invROIs, transformedRegions, roiRegions, objectStates );

	//double time2 = SceneHandler::msTime();

	// calculate predicted states of already found objects
	Mat predictedStates;
	predictProperties( objList, predictedStates );

	// match objects
	vector<int> objectMapping;
	vector<bool> foundObjects;
	matchObjects( objectStates, predictedStates, contours, objectMapping, foundObjects );

	#if SHOWMATCHINGSTEPS==1
	showMatchWindow(objList,predictedStates, objectStates, objectMapping, "after matchObjects()"); 
	#endif

	// look for previously calculated objects that couldn't be matched to actual frame
    vector<vector<int> > objectGroups;
	findPotentialAreaMatchesForMissingObjects( predictedStates, _outputImage, objList, transformedRegions, contours, objectMapping, foundObjects, objectGroups );
	
	#if SHOWMATCHINGSTEPS==1
	showMatchWindow(objList,predictedStates, objectStates, objectMapping, "after findPotentialAreaMatchesForMissingObjects()");
	#endif

	locateMissingObjects( invGreyImg, _outputImage, objectStates, objList, objectGroups, objectMapping, foundObjects, contours, regions, roiRegions, invROIs );
	
	#if SHOWMATCHINGSTEPS==1
	showMatchWindow(objList,predictedStates, objectStates, objectMapping, "after locateMissingObjects()");
	#endif

	// update the object lists
	updateObjects( objList, objectStates, predictedStates, objectMapping, foundObjects, contours, regions, invROIs );

	//double time3 = SceneHandler::msTime();
	
	/*ofstream file;
	file.open( "chronometries.txt", ios_base::app );
	file<<time2-time1<<" "<<time3-time2;
	file.close();*/

	// draw into output frame
	draw( _outputImage );

	// classify objects
    updateClassifications( greyImg );

	/*
	for( int i=0; i<objectStates.size().height;i++ )
	{
		circle( _outputImage,Point( objectStates.at<float>(i,0),objectStates.at<float>(i,1)),10, Scalar(80,232,30) );
		//rectangle( _outputImage, roiRegions[i], Scalar(80,232,30), 1 );
		vector<double> myDirection = SceneObject::direction( objectStates.at<float>(i,2) );
		Point eins( objectStates.at<float>(i,0)-40*myDirection[0], objectStates.at<float>(i,1)-40*myDirection[1] );
		Point zwei( objectStates.at<float>(i,0)+40*myDirection[0], objectStates.at<float>(i,1)+40*myDirection[1] );
		line( _outputImage, eins, zwei, Scalar(80,232,30), 1, 8 );

		
	}*/
	
	//_outputImage=binaryImg;
	return;
}


void ObjectHandler::typesInScene( vector<string>& _names )
{
	setupObjectTypesInfo(false);

	if( _names.size()==0 ) return;

	// first initialize the pObjectTypesInScene array as indicated
    for( size_t i=0;i<SceneObject::objectList->size();i++ )
	{
        for( size_t j=0;j<_names.size();j++ )
		{
			if( (*SceneObject::objectList)[i]==_names[j] )
			{
				pObjectTypesInScene[i] = true;
			}
		}
	}
	
	// initialize all classes that are set as occuring and set the classes as non-occuring if initialization fails
    for( size_t i=0;i<pObjectTypesInScene.size();i++ )
	{
		if( pObjectTypesInScene[i] ) pObjectTypesInScene[i] = (*SceneObject::initializerList)[i](i,this); // initializes the object class - The fact that the pObjectTypesInScene array changes during this iteration but might be accessed in the initializer function shouldn't be a problem since a class for which the initialization fails most likely will fail at providing a negative feature set as well
	}

	return;
}


void ObjectHandler::typesNotInScene( vector<string>& _names )
{
	setupObjectTypesInfo(true);

    for( size_t i=0;i<SceneObject::objectList->size();i++ )
	{
        for( size_t j=0;j<_names.size();j++ )
		{
			if( (*SceneObject::objectList)[i]==_names[j] ) pObjectTypesInScene[i] = false;
		}
	}

    for( size_t i=0;i<pObjectTypesInScene.size(); i++ )
	{
		if( pObjectTypesInScene[i] ) pObjectTypesInScene[i] = (*SceneObject::initializerList)[i](i,this); // initializes the object class - See above explanation why this shouldn't be a problem.
		
	}

	return;
}


void ObjectHandler::setThreshold( double _threshold )
{
	pThreshold = _threshold;
	return;
}


Ptr<DescriptorCreator> ObjectHandler::newDescriptorCreator( Ptr<vector<unsigned int> > _objectIds )
{
	Ptr<DescriptorCreator> newDC( new DescriptorCreator(_objectIds) );
	addDescriptorCreator( newDC );

	return newDC;
}


Ptr<DescriptorCreator> ObjectHandler::newDescriptorCreator( int _id0, int _id1, int _id2, int _id3, int _id4, int _id5, int _id6, int _id7, int _id8, int _id9 )
{
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

	return newDescriptorCreator(v);
}


bool ObjectHandler::addDescriptorCreator( Ptr<DescriptorCreator> _dC )
{
	return addDescriptorCreator(_dC.obj);
}


bool ObjectHandler::addDescriptorCreator( DescriptorCreator* _dC )
{
	pDescriptorCreators.push_back( _dC );

	if( pDescriptorSharePointer==NULL )
	{
		pDescriptorSharePointer = this;
		pDescriptorSharePointer.addref(); // make sure that the destructor of the objecthandler isn't called a second time on destruction by the Ptr<> object
	}
	_dC->pOHLink.push_back( pDescriptorSharePointer );
	(*pDescriptorSharePointer.refcount)--;
	return true;
}


bool ObjectHandler::removeDescriptorCreator( Ptr<DescriptorCreator> _dC )
{
	return removeDescriptorCreator(_dC.obj);
}


bool ObjectHandler::removeDescriptorCreator( DescriptorCreator* _dC )
{
	bool toReturn = false;
	for( list<DescriptorCreator*>::iterator it = pDescriptorCreators.begin(); it != pDescriptorCreators.end(); it++ )
	{
		if( (*it) == _dC )
		{
			pDescriptorCreators.erase( it );
			toReturn = true;
			break;
		}
	}
    for( list<Ptr<ObjectHandler> >::iterator it = _dC->pOHLink.begin(); it != _dC->pOHLink.end(); it++ )
	{
		if( (*it) == pDescriptorSharePointer )
		{
			(*pDescriptorSharePointer.refcount)++;
			_dC->pOHLink.erase( it );
			break;
		}
	}
	return toReturn;
}


void ObjectHandler::activeIdList( vector<unsigned int>& _idList )
{
	if( !_idList.empty() ) _idList.clear();

	list< Ptr<SceneObject> >::iterator it,end;

	end = pCategorized.end();
	for( it = pCategorized.begin(); it!=end ; it++ ) _idList.push_back( (*it)->id() );

	end = pUncategorized.end();
	for( it = pUncategorized.begin(); it!=end ; it++ ) _idList.push_back( (*it)->id() );

	end = pMissing.end();
	for( it = pMissing.begin(); it!=end ; it++ ) _idList.push_back( (*it)->id() );

	return;
}


unsigned int ObjectHandler::lowestActiveId()
{
	vector<unsigned int> idList;
	activeIdList(idList);

	if( idList.empty() ) return 0;

	unsigned int lowestId = idList[0];
	
    for( size_t i=1; i<idList.size(); i++ )
	{
		if( idList[i]<lowestId ) lowestId = idList[i];
	}

	return lowestId;
}


unsigned int ObjectHandler::highestActiveId()
{
	vector<unsigned int> idList;
	activeIdList(idList);

	if( idList.empty() ) return 0;

	unsigned int highestId = idList[0];

    for( size_t i=1; i<idList.size(); i++ )
	{
		if( idList[i]>highestId ) highestId = idList[i];
	}

	return highestId;
}


Ptr<SceneObject> ObjectHandler::getObj( unsigned int _objId )
{
	Ptr<SceneObject> obj;

	list< Ptr<SceneObject> >::iterator it, end;

	if( pUncategorized.size()!=0 )
	{
		end = pUncategorized.end();
		for( it=pUncategorized.begin(); it!=end; it++ )
		{
			if( (*it)->id() == _objId ) return (*it);
		}
	}
	else if( pCategorized.size()!=0 )
	{
		end = pCategorized.end();
		for( it=pCategorized.begin(); it!=end; it++ )
		{
			if( (*it)->id() == _objId ) return (*it);
		}
	}
	else if( pMissing.size()!=0 )
	{
		end = pMissing.end();
		for( it=pMissing.begin(); it!=end; it++ )
		{
			if( (*it)->id() == _objId ) return (*it);
		}
	}
	else if( pLost.size()!=0 )
	{
		end = pLost.end();
		for( it=pLost.begin(); it!=end; it++ )
		{
			if( (*it)->id() == _objId ) return (*it);
		}
	}
	else return NULL;

    return NULL;
}


void ObjectHandler::objRegions( Mat& _binaryImg, vector< vector<Point> >& _contours, vector<RectangleRegion>& _regions )
{
	vector< vector<Point> > tempContours;
	findContours( _binaryImg, tempContours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE );

	// filter out regions that are too small
    for( uint i=0;i<tempContours.size();i++ )
	{
		double tempArea = contourArea( tempContours[i] );
		double tempContourLength = arcLength( tempContours[i], true );
		if( ( tempArea>min_area || tempContourLength > contour_length_switch ) && tempArea<max_area )
		{
			_contours.push_back(tempContours[i]);
            RotatedRect temp = minAreaRect(tempContours[i]);
            _regions.push_back( RectangleRegion( temp ) );
		}

	}
	return;
}


void ObjectHandler::calculateROIs( vector<RectangleRegion>& _regions, vector<Rect>& _uprightRegions )
{
    for( size_t i=0; i<_regions.size(); i++ )
	{
		Rect upright;
		calculateROI( _regions[i], upright );
		_uprightRegions.push_back( upright );
	}
	return;
}


void ObjectHandler::calculateROI( RectangleRegion& _region, Rect& _uprightRegion )
{
	vector<Point> vertices;
	_region.points( vertices );
	_uprightRegion = boundingRect(vertices);
	return;
}


void ObjectHandler::calculateROIMats( Mat& _img, vector<Rect>& _roiRegions, vector<Mat>& _roiMats )
{
    for( size_t i=0; i<_roiRegions.size(); i++ )
	{
		Mat roiMat;
		calculateROIMat( _img, _roiRegions[i], roiMat );
		_roiMats.push_back(roiMat);
	}
	return;
}

void ObjectHandler::calculateROIMat( Mat& _img, Rect& _roiRegion, Mat& _roiMat )
{
	unsigned int leftBorder = (_roiRegion.x>=0)?_roiRegion.x:0;
	unsigned int upperBorder = (_roiRegion.y>=0)?_roiRegion.y:0;
		
	unsigned int ri_b = leftBorder+_roiRegion.width;
	unsigned int lo_b = upperBorder+_roiRegion.height;
    unsigned int rightBorder = ri_b<=(unsigned int)_img.size().width?ri_b:_img.size().width;
    unsigned int lowerBorder = lo_b<=(unsigned int)_img.size().height?lo_b:_img.size().height;
		
	Range rowRange( upperBorder, lowerBorder );
	Range colRange( leftBorder, rightBorder );
	_roiMat = _img( rowRange, colRange );
	return;
}

void ObjectHandler::transformToRelative( vector<Rect>& _roiRegions, vector<RectangleRegion>& _inputRegions, vector<RectangleRegion>& _transformedRegions )
{
	
	if( _roiRegions.size()!=_inputRegions.size() )
	{
		cerr<<endl<<"ObjectHandler::transformToRelative: Attempting to apply function when input vectors do not have the same size. Function is not executed."<<endl;
		return;
	}
	_transformedRegions = _inputRegions;
    for( size_t i=0;i<_roiRegions.size();i++ )
	{
		transformToRelative( _roiRegions[i], _transformedRegions[i] );
	}
	return;
}

void ObjectHandler::transformToRelative( Rect& _roiRegion, RectangleRegion& _region )
{
	_region.setNewOrigin( Point( _roiRegion.x,_roiRegion.y ) );
	return;
}


void ObjectHandler::roiProperties( vector<Mat>& _roiImages, vector<RectangleRegion>& _objectRegions, vector<Rect>& _imgPositions, Mat& _states )
{
    for( uint i=0;i<_roiImages.size();i++ )
	{
		double area;
        Mat temp = calcROIstate( _roiImages[i], _objectRegions[i], _imgPositions[i], area );
        _states.push_back( weightDimensions( temp ) );
	}
	return;
}


Mat ObjectHandler::calcROIstate( Mat& _roiImg, RectangleRegion& _region, Rect& _imgPosition, double& _area )
{
	Mat mask = imageMask( _roiImg, _region );
	
	Mat maskedImg = _roiImg.mul(mask); //create the masked version of the image
	
	Moments imgMoments = moments( maskedImg );

	_area = imgMoments.m00;
	
	Mat state;
	state.create( 1,3,CV_32FC1 );
	state.at<float>(0)=imgMoments.m10/imgMoments.m00+_imgPosition.x; // centroid x position [px]
	state.at<float>(1)=imgMoments.m01/imgMoments.m00+_imgPosition.y; // centroid y position [px]
	double a = imgMoments.mu20;
	double b = 2*imgMoments.mu11;
	double c = imgMoments.mu02;
	double angle = 0.5*atan( b/(a-c) );

	// use fitline through bounding rectangle to decide about moment direction ambiguity
	vector<Point> boundingPoints;
	_region.points( boundingPoints );
	Vec4f line;
	fitLine( boundingPoints, line, CV_DIST_L2, 0, 0.01, 0.01 );

	double approximateAngle=asin(line[1]);
	if( abs(approximateAngle-angle)>pi_quarter ) angle+=pi_half;

	state.at<float>(2)=(float)angle; // direction angle [rad]

	return state;
}


Mat ObjectHandler::imageMask( Mat& _targetImage, RectangleRegion& _unmaskedRegion )
{
	Mat mask = Mat::zeros( _targetImage.size().height, _targetImage.size().width, CV_8UC1 );
	Point unmaskRegion[4];
	_unmaskedRegion.pointArray( unmaskRegion );
	fillConvexPoly( mask, unmaskRegion, 4, Scalar(1) );
	return mask;
}


Mat ObjectHandler::weightDimensions( Mat& _state )
{
	return _state;
}


Mat ObjectHandler::unweightDimensions( Mat& _state )
{
	return _state;
}


void ObjectHandler::predictProperties( vector< list<Ptr<SceneObject> >::iterator >& _objList, Mat& _predictedStates )
{

    for( uint i=0;i<_objList.size(); i++ )
	{
        Mat temp = (**_objList[i]).predictState();
        _predictedStates.push_back( weightDimensions( temp ) );
		/*Mat prediction = (**_objList[i]).predictState();
		
		stringstream conv; string nr;
		conv<<(**_objList[i]).id();conv>>nr;

		ofstream file;
		file.open( "prediction"+nr+".txt", ios_base::app );
		file<<prediction.at<float>(0)<<" "<<prediction.at<float>(1)<<" "<<prediction.at<float>(2)<<endl;
		file.close();

		_predictedStates.push_back( weightDimensions(prediction) );*/
	}

	/*if(_objList.size()>0) // average prediction time calculation
	{
		Scalar blu; string name;
		(**_objList[0]).classProperties(name,blu);
		cout<<"Starting prediction time measurement for type: "<<name<<endl;
		double startTime = SceneHandler::msTime();
		for( int i=0; i<100000; i++ )(**_objList[0]).predictState();
		cout<<"Average calculation time was "<<(SceneHandler::msTime()-startTime)/100000<<" ms."<<endl;
		waitKey(0);
	}*/

	return;
}


void ObjectHandler::matchObjects( Mat& _states, Mat& _predictedStates, vector< vector<Point> >& _contours, vector<int>& _objectMapping, vector<bool>& _foundObjects )
{
	_objectMapping.resize( _states.size().height );
	_foundObjects.resize( _predictedStates.size().height );

    for( size_t i=0; i<_objectMapping.size(); i++ ) _objectMapping[i]=-1;
    for( size_t i=0; i<_foundObjects.size(); i++ ) _foundObjects[i]=false;

	BFMatcher matcher( NORM_L2, false ); // non-directional pairs in order to be able to deal with non-convex structures
	vector<DMatch> matches;
	
	if( _predictedStates.size().height!=0 )
	{
		matcher.match( _states, _predictedStates, matches );

		vector< vector<int> > temporaryObjectMatchings; //allows at this time for multiple predicted states to be matched to one contour ("new state")
		
		// build inversed matching array to test if a predicted state was matched to more than one contour
		temporaryObjectMatchings.resize( _predictedStates.size().height );
        for( size_t i=0; i<matches.size();i++ )
		{
			if( matches[i].distance>max_matching_distance_mismatch )
			{
				continue; // reject match if points are too far away
			}
			temporaryObjectMatchings[ matches[i].trainIdx ].push_back( matches[i].queryIdx );
			//_objectMapping[ matches[i].queryIdx ] = matches[i].trainIdx; // used here for case with bi-directional matching when no multiple matches were possible
			_foundObjects[ matches[i].trainIdx ] = true;
		}
		
		// check for multiple matchings and choose closer solutions if such occur (using contour distance)
        for( size_t i=0; i<temporaryObjectMatchings.size(); i++ )
		{
			if( temporaryObjectMatchings[i].size() == 1 ) _objectMapping[ temporaryObjectMatchings[i][0] ] = i;
			else if( temporaryObjectMatchings[i].size() > 1 ) // multiple matches
			{

				int chosenContour = temporaryObjectMatchings[i][0];
				Point2f predictedCenterPosition( _predictedStates.at<float>(i,0), _predictedStates.at<float>(i,1) );
				double closestDistance = pointPolygonTest( _contours[ temporaryObjectMatchings[i][0] ], predictedCenterPosition, true );
				double testDistance;
                for( size_t t=1;t<temporaryObjectMatchings[i].size();t++ )
				{
					testDistance = pointPolygonTest( _contours[ temporaryObjectMatchings[i][t] ], predictedCenterPosition, true );
					if( testDistance<closestDistance )
					{
						closestDistance = testDistance;
						chosenContour = temporaryObjectMatchings[i][t];
					}
				}
				_objectMapping[ chosenContour ] = i;
			}
			// else: then no predicted state could be matched to the object -> thus considered "new" at this stage
		}
		
		
		
	}
	return;
}


void ObjectHandler::findPotentialAreaMatchesForMissingObjects( Mat& _predictedStates, Mat& /*_outputImage*/, vector< list<Ptr<SceneObject> >::iterator >& _objList, vector<RectangleRegion>& _regions, vector< vector<Point> >& _contours, vector<int>& _objectMapping, vector<bool>& _foundObjects, vector<vector<int> >& _objectGroups )
{
	_objectGroups.resize( _objectMapping.size() );
	vector<int> objectsToIterate; // holds the indexes of all objects that are to be iterated

    for( size_t i=0; i<_foundObjects.size(); i++ ) // go through all objects of previous calculation
	{
		if( !_foundObjects[i] ) // if the object is missing
		{
			objectsToIterate.push_back(i);
		}
	}
	while( objectsToIterate.size()>0 ) // the while loop is chosen instead of recursive function calls from inside the objects' function to functions of other objects
	{
		// needed variables: _predictedStates, _regions, _contours, objectsToSetMissing
		int matchingAreaIdx;
		matchingAreaIdx = (**_objList[objectsToIterate.back()]).findMatchingArea(_regions,_contours);
		
		if( matchingAreaIdx<0 ) // object couldn't find a suitable match
		{
			int i = objectsToIterate.back();

			Point predictedCenterPosition( _predictedStates.at<float>(i,0), _predictedStates.at<float>(i,1) );
			Point lastCenterPosition = (**_objList[i]).pos();

			if( closeToWindowBorder( predictedCenterPosition ) ) // then it must be assumed that the object left the visible area
			{
				// output message that object left visible area?
			}
			else if( closeToWindowBorder( lastCenterPosition ) )
			{
				// output message that object left visible area?
			}
			else // fallback case if all previous methods failed but the object is actually expected to be somewhere inside the visible area. The object is lost most likely because it suddenly disappeared out of the image. This can for instance happen if the object was too bright and discarded in the thresholded image. Often such an extremely bright "state" doesn't hold up more than a few frames, which means the algorithm might still be able to recover the object from the missing list. If this was not the cause then there was a major problem with the prediction and the object additionally moved away from the old position.
			{ 
				cerr<<endl<<"ObjectHandler::findMissingObjects::Lost (at least temporarily) track of the Object with id "<<(**_objList[i]).id()<<". Most likely it became too bright for a moment.";
					
			}
			objectsToIterate.pop_back();
			continue;
		}

		_objectGroups[ matchingAreaIdx ].push_back( objectsToIterate.back() ); // build groups for matching of groups of missing objects to a found region
		objectsToIterate.pop_back();
				
		if( _objectMapping[ matchingAreaIdx ]!=-2 ) // if the matchObjects(...) match hasn't been invalidated yet
		{
			if( _objectMapping[ matchingAreaIdx ]!=-1 )
			{ 
				objectsToIterate.push_back( _objectMapping[ matchingAreaIdx ] );
				_foundObjects[ _objectMapping[ matchingAreaIdx ] ] = false;
				//cout<<endl<<"Object with id "<< (**_objList[_objectMapping[ matchCandidates[0] ]]).id() <<" was already matched to region.";
			}
			_objectMapping[ matchingAreaIdx ]=-2; //invalidate previously found match from matchObjects(...)
		}
	}

	// if only one missing object was matched to an area, accept the match and register it in objectMapping, delete the entry in objectGroups
    for( size_t area=0; area<_objectGroups.size(); area++ )
	{
		if( _objectGroups[area].size()==1 )
		{
			_objectMapping[area] = _objectGroups[area][0];
			_foundObjects[_objectGroups[area][0]] = true;
			_objectGroups[area].clear();
		}
	}

}


void ObjectHandler::findMissingObjects( Mat& _predictedStates, Mat& /*_outputImage*/, vector< list<Ptr<SceneObject> >::iterator >& _objList, vector<RectangleRegion>& _regions, vector< vector<Point> >& _contours, vector<int>& _objectMapping, vector<bool>& _foundObjects, vector<vector<int> >& _objectGroups )
{
	_objectGroups.resize( _objectMapping.size() );

	vector<int> objectsToSetMissing; // the idx of any object that was marked as found and matched to an area that is in the following being matched to more than one object is set to missing again and only set back to found later on if the algorithms were able to locate it inside the area: the array will contain the indexes of all objects concerned (->can't be done directly since that would mess up the iteration with the !foundObject[i] condition)
	
    for( size_t i=0; i<_foundObjects.size(); i++ ) // go through all objects of previous calculation
	{
		if( !_foundObjects[i] ) // if the object is missing
		{
			vector<int> matchCandidates; // array to hold the index of all contours of which the object possibly could be a part of

			Point predictedCenterPosition( _predictedStates.at<float>(i,0), _predictedStates.at<float>(i,1) );
			Point usedCenterPosition = predictedCenterPosition; // stores the center position for which the object finally has been matched (either the predicted or the old position)

            for( size_t r=0; r<_objectMapping.size(); r++ ) // check for each new state if it is a candidate for a match
			{
				Point relativeCenterPosition = predictedCenterPosition - _regions[r].getOrigin();
				if( _regions[r].contains( relativeCenterPosition ) ) // if predicted center lies inside the rectangle boundary of the region
				{
					matchCandidates.push_back(r);
				}
			}
			if( matchCandidates.size()==0 && use_old_state_fallback ) // check the old state instead of the predicted if no match was found for the latter
			{
				Point lastCenterPosition = (**_objList[i]).pos();
				usedCenterPosition = lastCenterPosition; // outside of check whether it is actually used in order not to lie in for loop - and its correct since the prediction position failed and either the old will succeed or none will in which case the variable isn't used

                for( size_t r=0; r<_objectMapping.size(); r++ )
				{
					Point relativeCenterPosition = lastCenterPosition - _regions[r].getOrigin();
					if( _regions[r].contains( relativeCenterPosition ) ) // if predicted center lies inside the rectangle boundary of the region
					{
						//cout<<endl<<"Found match for object "<<(**_objList[i]).id()<<" using old position."<<endl;
						matchCandidates.push_back(r);
						(**_objList[i]).resetPrediction(); // since the match is found at the old position but the predicted position didn't lie in any contour, it can be safely assumed that there was a situation where the used prediction method failed. This call tells so to the object, which than can react accordingly (or not)
					}
				}
			}
			else if( matchCandidates.size()==0 )
			{
				vector<Point> roi;
				(**_objList[i]).predictROI(roi);
				RectangleRegion predictedRegion( roi[0],roi[1],roi[2],roi[3] );
				vector<Point2f> gridPoints;
				predictedRegion.gridPoints( gridPoints, max_object_gridpoint_distance );
				
				int areaIdxWithHighestOverlap=-1, highestOverlap=0;

                for( size_t r=0; r<_objectMapping.size(); r++ )
				{
					int overlap=0;
                    for( size_t pt=0; pt<gridPoints.size(); pt++ ) // current matching criteria is the contour which lies the closest to the predicted center
					{
						
						if( pointPolygonTest( _contours[ r ], gridPoints[pt], false ) ) // tested points lies inside contour
						{
							overlap++;
						}
					}
					if( overlap>highestOverlap )
					{
						areaIdxWithHighestOverlap = r;
						highestOverlap=overlap;
					}
				}
				if( areaIdxWithHighestOverlap!=-1 ) matchCandidates.push_back(areaIdxWithHighestOverlap);
				
			}


			if( matchCandidates.size()==1 ) // if only one candidate was found, directly accept it without additional tests
			{
				_objectGroups[ matchCandidates[0] ].push_back( i ); // build groups for matching of groups of missing objects to a found region
				
				if( _objectMapping[ matchCandidates[0] ]!=-2 ) // if the matchObjects(...) match hasn't been invalidated yet
				{
					if( _objectMapping[ matchCandidates[0] ]!=-1 )
					{ 
						_objectGroups[ matchCandidates[0] ].push_back( _objectMapping[ matchCandidates[0] ] );
						objectsToSetMissing.push_back( _objectMapping[ matchCandidates[0] ]);
						//cout<<endl<<"Object with id "<< (**_objList[_objectMapping[ matchCandidates[0] ]]).id() <<" was already matched to region.";
					}
					_objectMapping[ matchCandidates[0] ]=-2; //invalidate previously found match from matchObjects(...)
				}
			}
			else if( matchCandidates.size()>=1 ) // more than one candidate match ->additional calculations necessary
			{
				int match = matchCandidates[0];
				double closestDistance = pointPolygonTest( _contours[ match ], usedCenterPosition, true );
				double testDistance;

                for( size_t r=0; r<matchCandidates.size(); r++ ) // current matching criteria is the contour which lies the closest to the predicted center
				{
					testDistance = pointPolygonTest( _contours[ matchCandidates[0] ], usedCenterPosition, true );
					if( testDistance < closestDistance )
					{
						testDistance = closestDistance;
						match=matchCandidates[r];
					}
				}
				// now the index of the matching contour is stored in "match"
				_objectGroups[ match ].push_back( i ); // build groups for matching of groups of missing objects to a found region
				if( _objectMapping[ match ]!=-2 ) // if the matchObjects(...) match hasn't been invalidated yet
				{
					if( _objectMapping[ match ]!=-1 )
					{ 
						_objectGroups[ match ].push_back( _objectMapping[ match ] );
						objectsToSetMissing.push_back( _objectMapping[ matchCandidates[0] ]);
						/*cout<<endl<<"Object was already matched to region.";*/}
					_objectMapping[ match ]=-2; //invalidate previously found match from matchObjects(...)
				}
			}
			else // object could not be matched so far
			{
				if( closeToWindowBorder( predictedCenterPosition ) ) // then it must be assumed that the object left the visible area
				{
					// output message that object left visible area?
				}
				else if( closeToWindowBorder( usedCenterPosition ) )
				{
					// output message that object left visible area?
				}
				else // fallback case if all previous methods failed but the object is actually expected to be somewhere inside the visible area. The object is lost most likely because it suddenly disappeared out of the image. This can for instance happen if the object was too bright and discarded in the thresholded image. Often such an extremely bright "state" doesn't hold up more than a few frames, which means the algorithm might still be able to recover the object from the missing list. If this was not the cause then there was a major problem with the prediction and the object additionally moved away from the old position.
				{ 
					cerr<<endl<<"ObjectHandler::findMissingObjects::Lost (at least temporarily) track of the Object with id "<<(**_objList[i]).id()<<". Most likely it became too bright for a moment.";
					/*int match = 0;
					double closestDistance = pointPolygonTest( _contours[ match ], predictedCenterPosition, true );
					double testDistance;

					for( int r=1; r<_contours.size(); r++ ) // current matching criteria is the contour which lies the closest to the predicted center
					{
						testDistance = pointPolygonTest( _contours[ r ], predictedCenterPosition, true );
						if( testDistance < closestDistance )
						{
							testDistance = closestDistance;
							match=r;
						}
					}
					
					// now the index of the matching contour is stored in "match"
					_objectGroups[ match ].push_back( i ); // build groups for matching of groups of missing objects to a found region
					if( _objectMapping[ match ]!=-2 ) // if the matchObjects(...) match hasn't been invalidated yet
					{ cout<<endl<<"Match was not invalidated yet.";
						if( _objectMapping[ match ]!=-1 ){ _objectGroups[ match ].push_back( _objectMapping[ match ] ); cout<<endl<<"Object was already matched to region.";}
						_objectMapping[ match ]=-2; //invalidate previously found match from matchObjects(...)
					}*/
				}
			}

		}
	}

	// set objects that are no longer single matched to areas as being missing again
    for( size_t i=0 ; i<objectsToSetMissing.size(); i++ )
	{
		_foundObjects[ objectsToSetMissing[i] ] = false;
	}
	
	// if only one missing object was matched to an area, accept the match and register it in objectMapping, delete the entry in objectGroups
    for( size_t area=0; area<_objectGroups.size(); area++ )
	{
		if( _objectGroups[area].size()==1 )
		{
			_objectMapping[area] = _objectGroups[area][0];
			_foundObjects[_objectGroups[area][0]] = true;
			_objectGroups[area].clear();
		}
	}
}


void ObjectHandler::locateMissingObjects( Mat& /*_invGreyImg*/, Mat& /*_outputImage*/, Mat& _objectStates, vector< list<Ptr<SceneObject> >::iterator >& _objList, vector<vector<int> >& _objectGroups, vector<int>& _objectMapping, vector<bool>& _foundObjects, vector<vector<Point> >& _contours, vector<RectangleRegion>& _regions, vector<Rect>& _roiRegions, vector<Mat>& _invROIs )
{
	

    for( size_t grpId = 0; grpId<_objectGroups.size() ; grpId++ ) // for each group: grp id matches id of associated "found object" (structure) in actual frame
	{
		if( _objectGroups[grpId].empty() ) continue;

		// find minimal upright rectangle that contains both the whole structure and all previous ROI positions
				
		Rect roi = _roiRegions[grpId]; // upright rectangle of combined structure
		
		
        for( size_t objId=0; objId<_objectGroups[grpId].size(); objId++ ) // iterate through all objects in group to adjust the region borders if necessary
		{
			RectangleRegion lastRROI = (**_objList[_objectGroups[grpId][objId]]).lastROI(); // last roi for each object of group
			vector<double> boundaries;
			lastRROI.getBoundaries( boundaries ); // boundaries [left|top|right|low] - calculate boundary values for each last roi
			
			double widthTemp=boundaries[2]-roi.x;
			double heightTemp=boundaries[1]-roi.y;
			if( boundaries[0] < roi.x && boundaries[0]>=0 ) roi.x=boundaries[0];
			if( boundaries[3] < roi.y && boundaries[3]>=0 ) roi.y=boundaries[3];
			if( widthTemp > roi.width ) roi.width=widthTemp;
			if( heightTemp > roi.height ) roi.height=heightTemp;
		}

		// region of interest containing the whole structure:
		Mat groupROI = pVideo->grey();
		int lowerYBoundary = (roi.y>=0)?roi.y:0;
		int lowerXBoundary = (roi.x>=0)?roi.x:0;
		int upperYBoundary = roi.y+roi.height;
		int upperXBoundary = roi.x+roi.width;
		upperYBoundary = (upperYBoundary<=groupROI.rows)?upperYBoundary:groupROI.rows;
		upperXBoundary = (upperXBoundary<=groupROI.cols)?upperXBoundary:groupROI.cols;

		groupROI = groupROI( Range(lowerYBoundary,upperYBoundary),Range(lowerXBoundary,upperXBoundary) );
		
		
		// corner harris precalculations: calculate relevant corner points
		Mat cH;

		cornerHarris(groupROI,cH,6,5,0.04); //3-11-0.07 //src,dst,blocksize,ksize(1,3,5or7),k //6-5-0.04
		
		Mat localMaxima;

		calcLocalMaxima( cH,localMaxima,3,0.1 );


		if( localMaxima.empty() ) continue;
		if( localMaxima.size().height<2 ) continue; // not enough points in set
		/*
		for( int i=0; i<localMaxima.rows; i++ )
		{
			Point maxima(localMaxima.at<float>(i,0),localMaxima.at<float>(i,1));
			circle( _outputImage,maxima+Point(roi.x,roi.y),5,Scalar(250,90,180) );
		}*/
		//imshow("corner harris group",cH);


		// EXTENDED (HOPEFULLY IMPROVED) CORNER HARRIS METHOD ******************************************************************************************

		double intensityWeight = 10; // (5 worked for old method) new: 10
		double queryIntensity = 1;
		double lengthWeight = 0.4; // (1.3 worked for old method) new: 0.4
		double angleWeight = 40; // angle in rad (40 works)
		double maxCandidateDistance = 20; // new:20, since NORM_L2 is set as match criteria, this indicates the maximal distance corner harris points may have from the predicted edge points in order to remain candidates [in px], set a little higher since the number of points found is also considered an indicator for the exclusiveness of the edge point position
		
		double insideOffsetPercentage = 0.40; // percentage of height the outer search points are moved inwards

		Mat predictedFeatures; // four point pairs per predicted object position // ( anchor x, anchor y, second x, second y, anchor intensity (1*intensityWeight), second intensity (1*intensityWeight), edge length, angle )
		
        for( size_t objId=0; objId<_objectGroups[grpId].size(); objId++ ) // for every object in the group that is in the iteration at the moment
		{
			vector<Point> objCorners; // corners of predicted ROI of the object
			(**_objList[_objectGroups[grpId][objId]]).predictROI(objCorners);
			RectangleRegion predictedROI( objCorners[0],objCorners[1],objCorners[2],objCorners[3] );
			predictedROI.setNewOrigin( Point2f(lowerXBoundary,lowerYBoundary) );

			vector<Point2f> shorterEdge_1, shorterEdge_2;
			predictedROI.shorterEdgePoints( shorterEdge_1 ); //load first shorter edge
			predictedROI.shorterEdgePoints( shorterEdge_2, 1 ); //load second shorter edge


			shorterEdge_1.push_back( Point2f(0.5*(shorterEdge_1[0].x+shorterEdge_1[1].x), 0.5*(shorterEdge_1[0].y+shorterEdge_1[1].y)) );
			shorterEdge_2.push_back( Point2f(0.5*(shorterEdge_2[0].x+shorterEdge_2[1].x), 0.5*(shorterEdge_2[0].y+shorterEdge_2[1].y)) );
			
			
			Point2f edge1OffsetVector = insideOffsetPercentage*( shorterEdge_1[1]-shorterEdge_1[0] );
			shorterEdge_1[0] = shorterEdge_1[0] + edge1OffsetVector;
			shorterEdge_1[1] = shorterEdge_1[1] - edge1OffsetVector;

			Point2f edge2OffsetVector = insideOffsetPercentage*( shorterEdge_2[1]-shorterEdge_2[0] );
			shorterEdge_2[0] = shorterEdge_2[0] + edge2OffsetVector;
			shorterEdge_2[1] = shorterEdge_2[1] - edge2OffsetVector;

			
			Mat shorterEdge1(3,2,CV_32FC1);
			shorterEdge1.at<float>(0,0) = shorterEdge_1[0].x;
			shorterEdge1.at<float>(0,1) = shorterEdge_1[0].y;
			shorterEdge1.at<float>(1,0) = shorterEdge_1[1].x;
			shorterEdge1.at<float>(1,1) = shorterEdge_1[1].y;
			shorterEdge1.at<float>(2,0) = shorterEdge_1[2].x; // middle point of first edge
			shorterEdge1.at<float>(2,1) = shorterEdge_1[2].y;

			Mat shorterEdge2(3,2,CV_32FC1);
			shorterEdge2.at<float>(0,0) = shorterEdge_2[0].x;
			shorterEdge2.at<float>(0,1) = shorterEdge_2[0].y;
			shorterEdge2.at<float>(1,0) = shorterEdge_2[1].x;
			shorterEdge2.at<float>(1,1) = shorterEdge_2[1].y;
			shorterEdge2.at<float>(2,0) = shorterEdge_2[2].x; // middle point of second edge
			shorterEdge2.at<float>(2,1) = shorterEdge_2[2].y;

			Mat midpoint1(1,2,CV_32FC1);
			Mat midpoint2(1,2,CV_32FC1);
			midpoint1.at<float>(0) = shorterEdge_1[2].x;
			midpoint1.at<float>(1) = shorterEdge_1[2].y;
			midpoint2.at<float>(0) = shorterEdge_2[2].x;
			midpoint2.at<float>(1) = shorterEdge_2[2].y;

			// build 9 feature pairs
			Mat edgeFeatures(9,8,CV_32FC1);

			// write remaining features
			for( int i=0; i<9; i++ )
			{
				edgeFeatures.at<float>(i,0) = shorterEdge_1[ (int)(i/3) ].x;
				edgeFeatures.at<float>(i,1) = shorterEdge_1[ (int)(i/3) ].y;
				edgeFeatures.at<float>(i,2) = shorterEdge_2[ i%3 ].x;
				edgeFeatures.at<float>(i,3) = shorterEdge_2[ i%3 ].y;

				Point2f currentEdge(edgeFeatures.at<float>(i,0)-edgeFeatures.at<float>(i,2),edgeFeatures.at<float>(i,1)-edgeFeatures.at<float>(i,3));
				edgeFeatures.at<float>(i,4) = queryIntensity*intensityWeight;
				edgeFeatures.at<float>(i,5) = queryIntensity*intensityWeight;
				edgeFeatures.at<float>(i,6) = norm( Mat( currentEdge ), NORM_L2 )*lengthWeight;
				edgeFeatures.at<float>(i,7) = (double) fastAtan2( currentEdge.y,currentEdge.x )*acos(-1.0)/180*angleWeight;
			}
			/*
			edgeFeatures.at<float>(0,0) = shorterEdge_1[ 2 ].x;
			edgeFeatures.at<float>(0,1) = shorterEdge_1[ 2 ].y;
			edgeFeatures.at<float>(0,2) = shorterEdge_2[ 2 ].x;
			edgeFeatures.at<float>(0,3) = shorterEdge_2[ 2 ].y;

			Point2f currentEdge(edgeFeatures.at<float>(0,0)-edgeFeatures.at<float>(0,2),edgeFeatures.at<float>(0,1)-edgeFeatures.at<float>(0,3));
			edgeFeatures.at<float>(0,4) = queryIntensity*intensityWeight;
			edgeFeatures.at<float>(0,5) = queryIntensity*intensityWeight;
			edgeFeatures.at<float>(0,6) = norm( Mat( currentEdge ), NORM_L2 )*lengthWeight;
			edgeFeatures.at<float>(0,7) = (double) fastAtan2( currentEdge.y,currentEdge.x )*acos(-1.0)/180*angleWeight;
			*/
			// find object edge candidates in corner harris maxima set
            vector<vector<DMatch> > cloud_1, cloud_2;
			Mat cornerHarrisCandidates = localMaxima( Range(0,localMaxima.rows), Range(0,2) );

			BFMatcher matcher( NORM_L2, false );
			matcher.radiusMatch( shorterEdge1, cornerHarrisCandidates, cloud_1, maxCandidateDistance );
			matcher.radiusMatch( shorterEdge2, cornerHarrisCandidates, cloud_2, maxCandidateDistance );
			//matcher.radiusMatch( midpoint1, cornerHarrisCandidates, cloud_1, maxCandidateDistance );
			//matcher.radiusMatch( midpoint2, cornerHarrisCandidates, cloud_2, maxCandidateDistance );

			if( cloud_1[0].size()==0 && cloud_1[1].size()==0 && cloud_2[0].size()==0 && cloud_2[1].size() ) continue; // no matches found for objects
			//if( cloud_1[0].size()==0 && cloud_2[0].size()==0 ) continue; // no matches found for objects

			Mat cloud1, cloud2; // merge clouds for the two points on each short edge, check for double point occurences and remove them

            for( size_t i=0; i<cloud_1[0].size(); i++ )
			{
				if( predictedROI.contains( Point(localMaxima.row(cloud_1[0][i].trainIdx).at<float>(0),localMaxima.row(cloud_1[0][i].trainIdx).at<float>(1)) ) ) //only add the point as candidate if it lies inside the predicted region of interest
				{
					cloud1.push_back( localMaxima.row(cloud_1[0][i].trainIdx) );
				}
			}
            for( size_t pt=1;pt<cloud_1.size();pt++ )
			{
                for( size_t i=0; i<cloud_1[pt].size(); i++ )
				{
					bool pointAlreadyAdded = false;
					if( predictedROI.contains( Point(cornerHarrisCandidates.row(cloud_1[pt][i].trainIdx).at<float>(0),cornerHarrisCandidates.row(cloud_1[pt][i].trainIdx).at<float>(1)) ) ) //only add the point as candidate if it lies inside the predicted region of interest
				
                    for( int j=0; j<cloud1.rows; j++ ) // check if point is already added
					{
						if( cloud1.at<float>(j,0)==cornerHarrisCandidates.row( cloud_1[pt][i].trainIdx ).at<float>(0) &&
							cloud1.at<float>(j,1)==cornerHarrisCandidates.row( cloud_1[pt][i].trainIdx ).at<float>(1)
							) pointAlreadyAdded = true;
					}
					if( !pointAlreadyAdded ) cloud1.push_back( localMaxima.row( cloud_1[pt][i].trainIdx ) );
				}
			}

            for( size_t i=0; i<cloud_2[0].size(); i++ )
			{
				if( predictedROI.contains( Point(localMaxima.row(cloud_2[0][i].trainIdx).at<float>(0),localMaxima.row(cloud_2[0][i].trainIdx).at<float>(1)) ) ) //only add the point as candidate if it lies inside the predicted region of interest
				{
					cloud2.push_back( localMaxima.row(cloud_2[0][i].trainIdx) );
				}
			}


            for( size_t pt=1;pt<cloud_2.size();pt++ )
			{
                for( size_t i=0; i<cloud_2[pt].size(); i++ )
				{
					bool pointAlreadyAdded = false;
					if( predictedROI.contains( Point(cornerHarrisCandidates.row(cloud_2[pt][i].trainIdx).at<float>(0),cornerHarrisCandidates.row(cloud_2[pt][i].trainIdx).at<float>(1)) ) ) //only add the point as candidate if it lies inside the predicted region of interest
				
                    for( int j=0; j<cloud2.rows; j++ ) // check if point is already added
					{
						if( cloud2.at<float>(j,0)==cornerHarrisCandidates.row( cloud_2[pt][i].trainIdx ).at<float>(0) &&
							cloud2.at<float>(j,1)==cornerHarrisCandidates.row( cloud_2[pt][i].trainIdx ).at<float>(1)
							) pointAlreadyAdded = true;
					}
					if( !pointAlreadyAdded ) cloud2.push_back( localMaxima.row( cloud_2[pt][i].trainIdx ) );
				}
			}

			Point2f zero, one, two, three, center;
			double angle;

			if( cloud1.rows!=0 && cloud2.rows!=0 ) // point clouds found on both sides of expected object position
			{
				// build all possible pairs and the respective features based on point clouds
				Mat candidateFeatures;
				for( int i=0; i<cloud1.rows; i++ )
				{
					for( int j=0; j<cloud2.rows; j++ )
					{
						Mat candidateFeature(1,8,CV_32FC1);
						candidateFeature.at<float>(0) = cloud1.at<float>(i,0);
						candidateFeature.at<float>(1) = cloud1.at<float>(i,1);
						candidateFeature.at<float>(2) = cloud2.at<float>(j,0);
						candidateFeature.at<float>(3) = cloud2.at<float>(j,1);

						Point2f currentEdge( candidateFeature.at<float>(0)-candidateFeature.at<float>(2),candidateFeature.at<float>(1)-candidateFeature.at<float>(3) );

						candidateFeature.at<float>(4) = cloud1.at<float>(i,2)*intensityWeight;
						candidateFeature.at<float>(5) = cloud2.at<float>(j,2)*intensityWeight;
						candidateFeature.at<float>(6) = norm( Mat( currentEdge ), NORM_L2 )*lengthWeight;
						candidateFeature.at<float>(7) = (double) fastAtan2( currentEdge.y,currentEdge.x )*acos(-1.0)/180*angleWeight;

						candidateFeatures.push_back(candidateFeature);
					}
				}

				// now match the features!
				vector<DMatch> objectCandidates;
				matcher.match(edgeFeatures,candidateFeatures,objectCandidates);

				// 9 candidates per object are found - take the closest candidate
				// find closest candidate
				DMatch bestMatch = objectCandidates[0];
                for(unsigned int i=1;i<objectCandidates.size();i++)
				{
					if( objectCandidates[i].distance<bestMatch.distance )
					{
						bestMatch = objectCandidates[i];
					}
				}

				// get old state information
				RectangleRegion lastRROI = (**_objList[_objectGroups[grpId][objId]]).lastROI();
				double oldHeight = lastRROI.height();
				double oldWidth = lastRROI.width();


				// calculate new state
				/*      1------------------2
				*		|				   |
				*		|				   |
				*		0------------------3
				*/
				Mat foundMatch = candidateFeatures.row(bestMatch.trainIdx);

				/*cout<<endl<<"found match: "<<foundMatch;
				cout<<endl<<"query: "<<edgeFeatures.row(bestMatch.queryIdx);
				Mat distanceMetrics = candidateFeatures.row(bestMatch.trainIdx) - edgeFeatures.row(bestMatch.queryIdx);
				cout<<endl<<"Distance Metrics: "<<distanceMetrics<<endl;*/

				// unweight elements
				foundMatch.at<float>(6) /= lengthWeight;
				foundMatch.at<float>(7) /= angleWeight;

                //int symmetrySwitch = 1; // used to switch between the symmetric cross point edge and straight point edge versions


				Mat axisVector = foundMatch( Range(0,1), Range(2,4) ) - foundMatch( Range(0,1), Range(0,2) );
				normalize(axisVector,axisVector);
				Mat halfLengthVector = 0.5*oldWidth*axisVector;
				Mat axisNormalVector(1,2,CV_32FC1);
				axisNormalVector.at<float>(0) = -axisVector.at<float>(1);
				axisNormalVector.at<float>(1) = axisVector.at<float>(0);
				Mat halfHeightVector = 0.5*oldHeight*axisNormalVector;

				Mat centerFromFirstEdge = foundMatch( Range(0,1), Range(0,2) ) + halfLengthVector;
				Mat centerFromSecondEdge = foundMatch( Range(0,1), Range(2,4) ) - halfLengthVector;

				// weights the center position according to the number of matches in cloud1 and cloud2: matches from clouds with fewer elements are considered better matches and more likely more accurate... (square is taken in order to augment the effect)
				double cloud1RowsSquare = cloud1.rows*cloud1.rows;
				double cloud2RowsSquare = cloud2.rows*cloud2.rows;

				Mat center_t = (1/(cloud1RowsSquare+cloud2RowsSquare))*(cloud2RowsSquare*centerFromFirstEdge+cloud1RowsSquare*centerFromSecondEdge);

				Mat middleSide1 = center_t - halfLengthVector;
				Mat middleSide2 = center_t + halfLengthVector;

				zero = Point2f( middleSide1 ) - Point2f( halfHeightVector );
				one = Point2f( middleSide1 ) + Point2f( halfHeightVector );
				two = Point2f( middleSide2 ) + Point2f( halfHeightVector );
				three = Point2f( middleSide2 ) - Point2f( halfHeightVector );
				center = Point2f( center_t );
				angle = (double) fastAtan2( three.y-zero.y, three.x-zero.x )*acos(-1.0)/180;

				//cout<<endl<<"Row information-- Cloud 1: "<<cloud1.rows<<" elements, Cloud 2: "<<cloud2.rows<<" elements"<<endl;
				// move the center relative to the number of elements found for both sides

			}
			else if( cloud1.rows!=0 ) // point cloud found only on first side
			{
				//cout<<endl<<"Previous object match was dropped because only on one side corner harris points were found."<<endl;
				continue; // at the moment the object is considered missing in this case
			}
			else // point cloud found only on second side
			{
				//cout<<endl<<"Previous object match was dropped because on neither side corner harris points were found for object "<<(**_objList[_objectGroups[grpId][objId]]).id()<<endl;
				continue; // at the moment the object is considered missing in this case
			}

			// transform coordinates back to image coordinates
			Point2f transform(roi.x,roi.y);
			center+=transform;
			zero+=transform;
			one+=transform;
			two+=transform;
			three+=transform;


			// add the features that have been found in the algorithm

			//circle( _outputImage,0.5*(zero+one),5,Scalar(70,210,30) );
			//circle( _outputImage,0.5*(two+three),5,Scalar(70,210,30) );

			Mat state;
			state.create( 1,3,CV_32FC1 );
			state.at<float>(0) = center.x;
			state.at<float>(1) = center.y;
			state.at<float>(2) = angle;
			_objectStates.push_back( weightDimensions(state) ); // add new state

			_objectMapping.push_back( _objectGroups[grpId][objId] ); // add matching for the new stage
			_foundObjects[ _objectGroups[grpId][objId] ] = true; // proclaim the object as being found

			_contours.push_back( vector<Point>() ); // add empty contour since no contour was calculated
			RectangleRegion newROI( zero, one, two, three );

			_regions.push_back( newROI );

			_invROIs.push_back( Mat() );
			predictedFeatures.push_back(edgeFeatures);
		}

		continue;
		// END OF EXTENDED METHOD *************************************************************************************************

		/*
		// build Mat with corners of predicted positions
		double intensityWeight = 5; // (5 works)
		double lengthWeight = 1.3; // (1.3 works)
		double angleWeight = 42.0; // angle in rad (40 works)
		Mat oldEdgeFeatures; // ( anchor x, anchor y, anchor intensity (1*intensityWeight), second x, second y, edge length, second intensity (1*intensityWeight), angle )
		for( int objId=0; objId<_objectGroups[grpId].size(); objId++ )
		{
			vector<Point> objCorners;
			(**_objList[_objectGroups[grpId][objId]]).predictROI(objCorners);
			RectangleRegion predictedROI( objCorners[0],objCorners[1],objCorners[2],objCorners[3] );
			vector<Point> shorterEdge_1;
			predictedROI.shorterEdgePoints( shorterEdge_1 ); //load first shorter edge
			vector<Point> shorterEdge_2;
			predictedROI.shorterEdgePoints( shorterEdge_2, 1 ); //load second shorter edge

			Mat edgeFeature(1,8,CV_32FC1);

			double anchorX = 0.5*(shorterEdge_1[0].x+shorterEdge_1[1].x)-roi.x;
			double anchorY = 0.5*(shorterEdge_1[0].y+shorterEdge_1[1].y)-roi.y;
			double secondX = 0.5*(shorterEdge_2[0].x+shorterEdge_2[1].x)-roi.x;
			double secondY = 0.5*(shorterEdge_2[0].y+shorterEdge_2[1].y)-roi.y;
			Point2f edge(anchorX-secondX,anchorY-secondY);

			edgeFeature.at<float>(0) = anchorX;
			edgeFeature.at<float>(1) = anchorY;
			edgeFeature.at<float>(2) = intensityWeight;
			edgeFeature.at<float>(3) = secondX;
			edgeFeature.at<float>(4) = secondY;
			edgeFeature.at<float>(5) = norm( Mat(edge), NORM_L2 )*lengthWeight;
			edgeFeature.at<float>(6) = intensityWeight;
			edgeFeature.at<float>(7) = (double) fastAtan2( edge.y,edge.x )*acos(-1.0)/180*angleWeight;
			
			oldEdgeFeatures.push_back(edgeFeature);
		}
		// find best matches in relevant corner point set to predicted old anchors
		BFMatcher matcher( NORM_L2, false ); // not only looking for bidirectional matches
		vector<DMatch> anchorMatches;
		Mat cornerPoints = localMaxima( Range(0,localMaxima.rows), Range(0,3) );
		Mat oldAnchors = oldEdgeFeatures( Range(0,oldEdgeFeatures.rows), Range(0,3) );
		matcher.match( oldAnchors, cornerPoints, anchorMatches );
		
		// build edge features based on found anchors and match for each
		Mat oldFeatures = oldEdgeFeatures( Range(0,oldEdgeFeatures.rows), Range(3,oldEdgeFeatures.cols) );
		
		Mat secondMatches;
		for( int matchId=0; matchId<anchorMatches.size(); matchId++ )
		{
			// build mat with new feature candidates
			Mat edgeCandidates;
			for( int i=0; i<localMaxima.rows; i++ )
			{
				Mat feature(1,5,CV_32FC1);

				double anchorX = cornerPoints.at<float>(anchorMatches[matchId].trainIdx,0);
				double anchorY = cornerPoints.at<float>(anchorMatches[matchId].trainIdx,1);
				double secondX = localMaxima.at<float>(i,0);
				double secondY = localMaxima.at<float>(i,1);
				Point2f edge( anchorX-secondX, anchorY-secondY );


				feature.at<float>(0) = secondX; // second x
				feature.at<float>(1) = secondY; // second y
				feature.at<float>(2) = norm( Mat(edge), NORM_L2 )*lengthWeight; // edge length
				feature.at<float>(3) = localMaxima.at<float>(i,2)*intensityWeight; // second intensity
				feature.at<float>(4) = (double) fastAtan2( edge.y,edge.x )*acos(-1.0)/180*angleWeight; //edge angle

				edgeCandidates.push_back( feature );
			}
			vector<DMatch> secondMatch;

			Mat queryEdge = oldFeatures.row(matchId);
			matcher.match( queryEdge,edgeCandidates,secondMatch );
			secondMatches.push_back( edgeCandidates.row( secondMatch[0].trainIdx ) );
		}
		
		for( int objId=0; objId<_objectGroups[grpId].size(); objId++ )
		{
			double anchorX = cornerPoints.at<float>(anchorMatches[objId].trainIdx,0);
			double anchorY = cornerPoints.at<float>(anchorMatches[objId].trainIdx,1);
			double secondX = secondMatches.at<float>(objId,0);
			double secondY = secondMatches.at<float>(objId,1);
			
			circle( _outputImage,Point(anchorX,anchorY)+Point(roi.x,roi.y),5,Scalar(70,210,30) );
			circle( _outputImage,Point(secondX,secondY)+Point(roi.x,roi.y),5,Scalar(70,210,30) );

			Point newPosition( 0.5*(anchorX+secondX)+roi.x,0.5*(anchorY+secondY)+roi.y );
			circle( _outputImage,newPosition,5,Scalar(250,90,180) );
			double newAngle = secondMatches.at<float>(objId,4)/angleWeight;

			Mat state;
			state.create( 1,3,CV_32FC1 );
			state.at<float>(0) = newPosition.x;
			state.at<float>(1) = newPosition.y;
			state.at<float>(2) = newAngle;
			_objectStates.push_back( weightDimensions(state) ); // add new state

			_objectMapping.push_back( _objectGroups[grpId][objId] ); // add matching for the new stage
			_foundObjects[ _objectGroups[grpId][objId] ] = true; // proclaim the object as being found

			_contours.push_back( vector<Point>() );

			RectangleRegion lastRROI = (**_objList[_objectGroups[grpId][objId]]).lastROI();
			double prevRegionHeight = lastRROI.height();
			double prevRegionWidth = secondMatches.at<float>(objId,2);

			double height = prevRegionHeight;
			double width = prevRegionWidth;

			vector<double> newDirection = SceneObject::direction(newAngle);
			vector<double> orthDirection; orthDirection.resize(2);
			orthDirection[0] = newDirection[1];
			orthDirection[1] = -newDirection[0];
			double heightOffset = height/2;
			double widthOffset = width/2;
			
			Point eins( newPosition.x-widthOffset*newDirection[0]-heightOffset*orthDirection[0], newPosition.y-widthOffset*newDirection[1]-heightOffset*orthDirection[1] );
			Point zwei( newPosition.x+widthOffset*newDirection[0]-heightOffset*orthDirection[0], newPosition.y+widthOffset*newDirection[1]-heightOffset*orthDirection[1] );
			Point drei( newPosition.x-widthOffset*newDirection[0]+heightOffset*orthDirection[0], newPosition.y-widthOffset*newDirection[1]+heightOffset*orthDirection[1] );
			Point vier( newPosition.x+widthOffset*newDirection[0]+heightOffset*orthDirection[0], newPosition.y+widthOffset*newDirection[1]+heightOffset*orthDirection[1] );
			RectangleRegion newROI(eins,zwei,vier,drei);

			_regions.push_back( newROI );

			_invROIs.push_back( Mat() );
		}
		*/
		/*
		corners*=1;
		cout<<endl<<"Number of found localMaxima: "<<localMaxima.rows<<endl;
		

		//threshold( corners, corners, 0.2, 1, THRESH_BINARY );
		imshow("binary",binaryImg);
		//imwrite("binaryImage.png",binaryImg);
		imshow("corner harris",corners);
		//imwrite("localMaximaAlgorithmOutput.png",corners);
		//imwrite("greyImage.png",greyImg);*/
		//waitKey(0);

		/*for ( int objId=0; objId<_objectGroups[grpId].size(); objId++ ) // for each missed object that was matched to group
		{*/
			// calculation using corner harris /////////////////////////////////////////////////////////





			// calculation using optical flow //////////////////////////////////////////////////////////////////////////////

			/*// find upright rectangle (region) that contains both the whole last object and the whole new structure
			RectangleRegion lastRROI = (**_objList[_objectGroups[grpId][objId]]).lastROI();
			vector<double> boundaries;
			lastRROI.getBoundaries( boundaries ); // boundaries [left|top|right|low]

			Rect roi = _roiRegions[grpId]; // upright rectangle of combined structure
			double widthTemp=boundaries[2]-roi.x;
			double heightTemp=boundaries[1]-roi.y;
			if( boundaries[0] < roi.x ) roi.x=boundaries[0];
			if( boundaries[3] < roi.y ) roi.y=boundaries[3];
			if( widthTemp > roi.width ) roi.width=widthTemp;
			if( heightTemp > roi.height ) roi.height=heightTemp;

			Mat preRegion;
			calculateROIMat( pVideo->load(1).grey(), roi, preRegion );
			Mat actRegion;
			calculateROIMat( pVideo->grey(), roi, actRegion );

			vector<Point> featurePoints;
			RectangleRegion relativeROI=lastRROI;
			relativeROI.setNewOrigin(Point(roi.x,roi.y));
			Mat roiMask = imageMask(preRegion,relativeROI);
			goodFeaturesToTrack(preRegion,featurePoints,100,0.1,1.0,roiMask);
			//cout<<endl<<"nr of feature points: "<<featurePoints.size();

			// split last rectangle region of interest and order points accordingly
			vector<RectangleRegion> rrois;
			relativeROI.splitAlongLength(3,rrois);
			vector<vector<Point>> orderedFP;
			orderedFP.resize(3);
			Mat toDisplay, tD;
			calculateROIMat( pVideo->loadMat(),roi,toDisplay );
			toDisplay.copyTo(tD);

			for(int i=0;i<featurePoints.size();i++)
			{
				if( rrois[0].contains(featurePoints[i]) ) orderedFP[0].push_back(featurePoints[i]);
				else if( rrois[2].contains(featurePoints[i]) ) orderedFP[2].push_back(featurePoints[i]);
			}
			//rrois[0].draw( tD,Scalar(250,180,40) );
			rrois[2].draw( tD,Scalar(250,180,40) );
			cout<<endl<<"Size of orderedFP[0]: "<<orderedFP[0].size();
			cout<<endl<<"Size of orderedFP[2]: "<<orderedFP[2].size();
			


			// find ordered points in new frame with optical flow
			vector<vector<Point>> optFlowPoints; optFlowPoints.resize(3);
			vector<vector<uchar>> status; status.resize(3); 
			vector<Mat> error; error.resize(3);
			for(int i=0;i<3;i+=2)
			{
				Mat optFlowPoints_t;
				Mat interestPoints(orderedFP[i]);
				interestPoints=interestPoints.reshape(1);
				interestPoints.convertTo(interestPoints,CV_32FC1);
				calcOpticalFlowPyrLK( preRegion,actRegion,interestPoints,optFlowPoints_t,status[i],error[i]);
				Mat tempPointSet = optFlowPoints_t.reshape(2,1);
				tempPointSet.copyTo(optFlowPoints[i]);
			}

			// filter out points for which no prediction could be found
			vector<vector<Point>> filteredOrderedFP; filteredOrderedFP.resize(3);
			vector<vector<Point>> filteredOptFloP; filteredOptFloP.resize(3);
			for(int i=0;i<3;i+=2)
			{
				for(int p=0;p<orderedFP[i].size();p++)
				{
					if( status[i][p]==1 )
					{
						filteredOrderedFP[i].push_back(orderedFP[i][p]);
						filteredOptFloP[i].push_back(optFlowPoints[i][p]);
						
						//circle( tD,filteredOptFloP[i][p],1,Scalar(100,200,30) );
						//line( tD,filteredOptFloP[i][p],filteredOrderedFP[i][p],Scalar(100,200,30),1 );
					}
				}
			}

			// calculate average features
			Point offsetToOriginalOrigin(roi.x,roi.y);
			Point prePosition = (**_objList[_objectGroups[grpId][objId]]).pos();
			Point fRC = average( filteredOrderedFP[0] )+offsetToOriginalOrigin;
			Point sRC = average( filteredOrderedFP[2] )+offsetToOriginalOrigin;
			Point distVec = fRC-sRC;
			double preLength = sqrt( (double)distVec.x*distVec.x+distVec.y*distVec.y );
			double preAngle = (**_objList[_objectGroups[grpId][objId]]).angle();
			double pi_half = acos(-1.0)/2;
			double pointAngleEstimate = pi_half/90*(double)fastAtan2( distVec.y, distVec.x );
			if( preAngle<-pi_half || preAngle>pi_half ) preAngle+=2*pi_half;
			double relAngle = preAngle-pointAngleEstimate;

			Point firstCtToMid = prePosition-fRC;
			Point secondCtToMid = prePosition-sRC;

			Point firstRegionFlowCenter = average( filteredOptFloP[0] )+offsetToOriginalOrigin;
			Point secondRegionFlowCenter = average( filteredOrderedFP[2] )+offsetToOriginalOrigin;
			Point flDistV = firstRegionFlowCenter-secondRegionFlowCenter;
			double postLength = sqrt( (double)flDistV.x*flDistV.x+flDistV.y*flDistV.y );
			double postAngleEstimate = pi_half/90*(double)fastAtan2( flDistV.y, flDistV.x );

			double stretch = postLength/preLength;
			
			// optical flow based "estimation" of new state
			double newAngleEstimate = postAngleEstimate+relAngle;
			double sinAngle = sin(newAngleEstimate-preAngle);
			double cosAngle = cos(newAngleEstimate-preAngle);
			firstCtToMid=RectangleRegion::rotate( firstCtToMid,sinAngle,cosAngle);
			secondCtToMid=RectangleRegion::rotate( secondCtToMid,sinAngle,cosAngle);

			Point newPosition_e1 = firstRegionFlowCenter+firstCtToMid;
			Point newPosition_e2 = secondRegionFlowCenter+secondCtToMid;
			Point newPosition = newPosition_e1+newPosition_e2;
			newPosition.x/=2;
			newPosition.y/=2;
			
			//Point speedEstimate = newPosition-prePosition;
			
			//cout<<endl<<"speed: "<<speedEstimate;
            cout<<endl<<"pre Angle: "<<preAngle/pi_half*90;
			cout<<endl<<"new Angle: "<<newAngleEstimate/pi_half*90;
			
			Mat state;
			state.create( 1,3,CV_32FC1 );
			state.at<float>(0) = newPosition.x;
			state.at<float>(1) = newPosition.y;
			state.at<float>(2) = newAngleEstimate;
			_objectStates.push_back( weightDimensions(state) ); // add new state

			_objectMapping.push_back( _objectGroups[grpId][objId] ); // add matching for the new stage
			_foundObjects[ _objectGroups[grpId][objId] ] = true; // proclaim the object as being found

			_contours.push_back( vector<Point>() );

			// calculate rectangle region based on state estimation
			
			double prevRegionHeight = lastRROI.height();
			double prevRegionWidth = lastRROI.width();

			double height = prevRegionHeight;
			double width = prevRegionWidth;

			vector<double> newDirection = SceneObject::direction(newAngleEstimate);
			vector<double> orthDirection; orthDirection.resize(2);
			orthDirection[0] = newDirection[1];
			orthDirection[1] = -newDirection[0];
			double heightOffset = height/2;
			double widthOffset = width/2;
			
			Point eins( newPosition.x-widthOffset*newDirection[0]-heightOffset*orthDirection[0], newPosition.y-widthOffset*newDirection[1]-heightOffset*orthDirection[1] );
			Point zwei( newPosition.x+widthOffset*newDirection[0]-heightOffset*orthDirection[0], newPosition.y+widthOffset*newDirection[1]-heightOffset*orthDirection[1] );
			Point drei( newPosition.x-widthOffset*newDirection[0]+heightOffset*orthDirection[0], newPosition.y-widthOffset*newDirection[1]+heightOffset*orthDirection[1] );
			Point vier( newPosition.x+widthOffset*newDirection[0]+heightOffset*orthDirection[0], newPosition.y+widthOffset*newDirection[1]+heightOffset*orthDirection[1] );
			RectangleRegion newROI(eins,zwei,vier,drei);

            cout<<endl<<"x: "<<newDirection[0]<<"y: "<<newDirection[1];
			line( _outputImage, eins, zwei, Scalar(40,220,250), 2, 8 );
			line( _outputImage, drei, vier, Scalar(40,220,250), 2, 8 );
			
			//newROI.draw(_outputImage,Scalar(100,250,100) );

			//imshow("output image",tD);waitKey(0);destroyWindow("output image");

			_regions.push_back( newROI );
			//cout<<endl<<"reached calculation end";
			_invROIs.push_back( Mat() );

			continue;*/
			///////////////////////// end of previous method //////////////////////////////////////////////////////////
			/* some tryout stuff
			//cout<<endl<<blub[0]<<" | "<<blub[1]<<" | "<<blub[2]<<" | "<<blub[3]<<endl;

			//find points for which the optical flow offset shall be calculated
			//use whole contour to test
			Mat preImage = pVideo->load(1).grey();

			vector<Point> interestPoints;
			//lastRROI.draw(preImage,Scalar(50,210,50));
			Mat roiMASK = imageMask(preImage,lastRROI);
			//imshow("preImage",preImage);waitKey(0);destroyWindow("preImage");
			Mat maskedImg = preImage;//.mul(roiMASK); //create the masked version of the image
			//imshow("mask",maskedImg);waitKey(0);
			
			
			goodFeaturesToTrack(maskedImg,interestPoints,100,0.01,3.0,roiMASK);
			
			//(**_objList[ _objectGroups[grpId][objId] ]).lastContour(interestPoints);
			Mat intPoint(interestPoints);


			intPoint=intPoint.reshape(1);
			intPoint.convertTo(intPoint,CV_32FC1);



			Mat newPoints;
			vector<uchar> status; Mat error;
			calcOpticalFlowPyrLK( pVideo->load(1).grey(), pVideo->grey(), intPoint, newPoints, status, error);
			
			Mat optflowContourPrediction = newPoints.reshape(2,1);
			vector<Point> predictedContour;
			optflowContourPrediction.copyTo(predictedContour);
			
			const Point* pts = (const Point*) Mat(predictedContour).data;
			int nrOfPoints = newPoints.rows;
			polylines(_outputImage,&pts, &nrOfPoints,1,true,Scalar(253,103,230),2 );
			


			Mat regionImg; // calculates the image header where the object is predicted to be found
			RotatedRect roi_1 = minAreaRect( predictedContour );
			RectangleRegion roi_2( roi_1 );
			Rect imageRegion=roi_1.boundingRect();
			calculateROIMat( _invGreyImg, imageRegion, regionImg );
			//transformToRelative(imageRegion,roi_2);

			double objArea;
			Mat state = weightDimensions( calcROIstate( regionImg, roi_2, imageRegion, objArea ) );


			// set object as being found
			_objectStates.push_back( state ); // add new state
			_objectMapping.push_back( _objectGroups[grpId][objId] ); // add matching for the new stage
			_foundObjects[ _objectGroups[grpId][objId] ] = true; // proclaim the object as being found

			// update vector containers (for the sake of completeness roiRegions would have to be updated as well but this is omitted for now since it isn't used in any later stage)
			_contours.push_back( predictedContour );
			//region.setNewOrigin(Point(-imageRegion.x,-imageRegion.y)); // transform region back to original frame
			roi_2.draw(_outputImage, Scalar(104,202,247) );
			_regions.push_back( roi_2 );
			_invROIs.push_back( regionImg );


			*/
			//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			/* // calculation using predicted rotated region of interests around the objects
			vector<Point> predictedROI;
			( **_objList[ _objectGroups[grpId][objId] ] ).predictROI( predictedROI );
			RectangleRegion predictedRegion = (**_objList[_objectGroups[grpId][objId]]).lastROI();//( predictedROI[0], predictedROI[1], predictedROI[2], predictedROI[3] );
			
			Rect imageRegion; // calculate upright rectangle around predicted rotated rectangle region
			calculateROI( predictedRegion, imageRegion );

			double prevRegionHeight = predictedRegion.height();
			double prevRegionWidth = predictedRegion.width();

			cout<<endl<<"Previous region area: "<<predictedRegion.area();
			Mat regionImg; // calculates the image header where the object is predicted to be found
			calculateROIMat( _invGreyImg, imageRegion, regionImg );
			RectangleRegion originalRegion = predictedRegion;
			transformToRelative( imageRegion, predictedRegion );

			double objArea;
			Mat state = weightDimensions( calcROIstate( regionImg, predictedRegion, imageRegion, objArea ) );

			if( objArea>=min_area ) // missed object considered found
			{			
				// calculate values used for container updates
				Mat bwImg;
				threshold( regionImg, bwImg, 180, 255, THRESH_BINARY );

				bwImg = bwImg.mul( imageMask( bwImg, predictedRegion ) );
				//imshow("test",bwImg); waitKey(0); destroyWindow("test");
				//find contour
				vector< vector<Point> > tempContours;
				findContours( bwImg, tempContours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE );

				// filter out regions that are too small -> find the actual region
				vector<Point> contour;
				RectangleRegion region;

				int nrOfContourPoints=0;
				for( int i=0;i<tempContours.size();i++ ) nrOfContourPoints+=tempContours[i].size();

				contour.reserve(nrOfContourPoints);
				for( int i=0;i<tempContours.size();i++ ) contour.insert( contour.end(), tempContours[i].begin(), tempContours[i].end() );
				
				polylines(_outputImage,contour,true, Scalar(100,200,250));
				_contours.push_back(contour);
				region =  minAreaRect(contour); 
				region.setLengths( prevRegionHeight,prevRegionWidth );
				cout<<endl<<"New region area:"<<region.area();
				
				if( contour.empty() ) continue;

				
				

			}
			*/
		/*}*/
	}
	return;
}


void ObjectHandler::updateObjects( vector< list<Ptr<SceneObject> >::iterator >& _objList, Mat& _objectStates, Mat& _predictedStates, vector<int>& _objectMapping, vector<bool>& _foundObjects, vector<vector<Point> >& _contours, vector<RectangleRegion>& _regions, vector<Mat>& _invROIs )
{
	unsigned int nrOfCategorized = pCategorized.size();
	unsigned int nrOfUncategorized = pUncategorized.size();
	

	
    for( uint foundObj=0; foundObj<_objectMapping.size(); foundObj++ ) // for all found and new objects
	{
		if( _objectMapping[foundObj]==-2 ) continue; // invalid state (was composed of several distinctive objects)
		if( _objectMapping[foundObj]==-1 ) // new object
		{
			Ptr<SceneObject> newObject = new SceneObject( this );
			Mat state = _objectStates.row(foundObj);
			state = unweightDimensions(state);

			Ptr<SceneObject::State> newState = new SceneObject::State( state.at<float>(0), state.at<float>(1), pActualTime,state.at<float>(2) );
            vector<Point> empty;
            newObject->addState( newState, _regions[foundObj], empty/*_contours[foundObj]*/ );
			pUncategorized.push_front( newObject );
		}
		else
		{
            Mat temp = _objectStates.row(foundObj);
            Mat state = unweightDimensions( temp );


			/*if(_objList.size()>0) // update time measurements
			{
				if( (*_objList[0])->pHistory.size()>1200 )
				{
					string name; cv::Scalar scala;
					(*_objList[0])->classProperties(name,scala);
					// now measure update time
					int count2 = 0;
					double time2 = SceneHandler::msTime();
					for( int i=0; i<100000; i++ )
					{
						Ptr<SceneObject::State> newState = (*_objList[ _objectMapping[foundObj] ])->newState( state, _regions[foundObj], _contours[foundObj], _invROIs[foundObj], pActualTime );
						(*_objList[0])->addState( newState, _regions[foundObj], vector<Point>() );
					}
					cout<<endl<<"Average update time for type "<<name<<": "<<( SceneHandler::msTime()-time2 )/100000<<" ms"<<endl;
				}
			}*/

			
			
			
			Ptr<SceneObject::State> newState = (*_objList[ _objectMapping[foundObj] ])->newState( state, _regions[foundObj], _contours[foundObj], _invROIs[foundObj], pActualTime );
			
			/*stringstream conv; string nr;
			conv<<(**_objList[_objectMapping[foundObj]]).id();conv>>nr;

			ofstream file;
			file.open( "measurement"+nr+".txt", ios_base::app );
			file<<newState->x<<" "<<newState->y<<" "<<newState->angle<<endl;
			file.close();*/
			
            vector<Point> empty;
            (*_objList[_objectMapping[foundObj]])->addState( newState, _regions[foundObj], empty/*_contours[foundObj]*/ );
			
			// if it was previously classified missing
            if( _objectMapping[foundObj]>=0 && (unsigned int)_objectMapping[foundObj]>=nrOfCategorized+nrOfUncategorized ) // id points to an object in the missing object list
			{
				Ptr<SceneObject> refoundObject = (*_objList[ _objectMapping[foundObj] ]);
				removeFromMissing( _objList[_objectMapping[foundObj]] ); // remove the element from the missing list - for lists this doesn't invalidate the other iterators
				
				if( refoundObject->type()==-2 ) pUncategorized.push_front( refoundObject ); // -2 means that the object hasn't been classified yet
				else pCategorized.push_front( refoundObject );
			}
		}
	}
    for( size_t missObj=0; missObj<_foundObjects.size(); missObj++ )
	{
		if( !_foundObjects[missObj] ) // object hasn't been found
		{
			Ptr<SceneObject> missingObject = ( *_objList[missObj] );

			if( missing_state_bridging==1 ) // successively keep predicting the next state and use it as actual state of the object while it is missing
			{
				Mat state = _predictedStates.row(missObj); // assume the position is somewhere where it was predicted
							vector<Point> predictedROI;
				( *missingObject ).predictROI( predictedROI );
				RectangleRegion predictedRegion( predictedROI[0],predictedROI[1],predictedROI[2],predictedROI[3] );

				vector<Point> lastFoundContour;
				(*missingObject).lastContour(lastFoundContour);
				Mat emptyImage;
				Ptr<SceneObject::State> newState = missingObject->newState( state, predictedRegion, lastFoundContour, emptyImage, pActualTime );
				missingObject->addState( newState, predictedRegion, lastFoundContour );
			}
			else if( missing_state_bridging==2 ) // use the previous state of the object as a steady state for the whole time it is missing
			{
				vector<Point> lastFoundContour;
				(*missingObject).lastContour(lastFoundContour);
                RectangleRegion temp = missingObject->lastROI();
                missingObject->addState( missingObject->state(), temp, lastFoundContour );
			}
			// else if: missing_state_bridging==0 (or any other value): create holes in state history

			if( missObj<nrOfCategorized ) // object was previously not missing and already classified
			{
				pCategorized.erase( _objList[missObj] );
				addToMissing( missingObject );
			}
			else if( missObj<nrOfCategorized+nrOfUncategorized ) // object was previously not missing and unclassified
			{
				pUncategorized.erase( _objList[missObj] );
				addToMissing( missingObject );
			}
			// if object was already missing before, do nothing -> is handled with updateMissing() function for all objects
		}
	}
	updateMissing();
}


void ObjectHandler::addToMissing( Ptr<SceneObject> _objectToAdd )
{
	pMissing.push_back( _objectToAdd );
	pair< Ptr<SceneObject>,int> counter;
	counter.first = _objectToAdd;
	counter.second = 0;
	pMissingCount.push_back(counter);
	return;
}


bool ObjectHandler::removeFromMissing( list<Ptr<SceneObject> >::iterator& _object )
{
	for( list< pair< Ptr<SceneObject>,int > >::iterator it = ++pMissingCount.begin(); it!=pMissingCount.end(); it++ )
	{
		if( (*it).first==(*_object) )
		{
			pMissingCount.erase(it);
			break;
		}
	}
	pMissing.erase( _object );
	return false;
}


int& ObjectHandler::getMissingTime( Ptr<SceneObject> _object )
{
    for( list<pair<Ptr<SceneObject>,int> >::iterator it=++pMissingCount.begin();it!=pMissingCount.end();it++ )
	{
		if( (*it).first==_object ) return (*it).second;
	}
	//empty return
	return pMissingCount.front().second;
}


void ObjectHandler::updateMissing()
{

    list<pair<Ptr<SceneObject>,int> >::iterator end = pMissingCount.end();

    for( list<pair<Ptr<SceneObject>,int> >::iterator it=++pMissingCount.begin();it!=end;) // starts at second entry because there is a dummy entry in first place
	{
		(*it).second = (*it).second+1; // increment counter
        if( (unsigned int)((*it).second) > max_object_missing_time && (*it).second>=0 ) // remove objects from list if they have been there too long
		{
			Ptr<SceneObject> lostObject = (*it).first;
			it = pMissingCount.erase(it);
            for( list<Ptr<SceneObject> >::iterator obj=pMissing.begin();obj!=pMissing.end();obj++ )
			{
				if( (*obj)==lostObject )
				{
					pMissing.erase( obj );
					break;
				}
			}			
			pLost.push_back( lostObject );
		}
		else ++it;
	}
	return;
}


void ObjectHandler::updateClassifications( Mat _image )
{
	if( pObjectTypesInScene.size()==0 ) setupObjectTypesInfo(true);

	double timeLeft = pScene->timeLeft()-time_overhead;

    int nrOfClassifiableObjects = pUncategorized.size()+pCategorized.size();

	for( int i=0; i < nrOfClassifiableObjects && (timeLeft > pEstimatedClassificationTime || !time_awareness); i++ )
    {
		double time_1 = SceneHandler::msTime();
		Ptr<SceneObject> objForClassification = getObjForClassification();
		
		if( objForClassification == NULL ) break;

		RectangleRegion roi = objForClassification->lastROI();
		
		double widthExtension = 20;
		double heightExtension = 20;
		vector<double> boundaries;
		roi.getBoundaries( boundaries );

		boundaries[0]-=widthExtension;
		boundaries[1]+=heightExtension;
		boundaries[2]+=widthExtension;
		boundaries[3]-=heightExtension;

		if( boundaries[0]<0 ) boundaries[0]=0;
		if( boundaries[1]>_image.size().height ) boundaries[1]=_image.size().height;
		if( boundaries[2]>_image.size().width ) boundaries[2]=_image.size().width;
		if( boundaries[3]<0 ) boundaries[3]=0;
        if( boundaries[0]>boundaries[2] || boundaries[3]>boundaries[1] ) continue; // out of range

		Mat containingImg = _image( Range( boundaries[3], boundaries[1] ), Range( boundaries[0], boundaries[2] ) );

		roi.setNewOrigin( Point( boundaries[0], boundaries[3] ) );

		// create descriptors;
		Mat descriptors;
        GenericObject::createDescriptors( containingImg, roi, descriptors );

		// create descriptor sets
		fillDescriptorCreators( containingImg, descriptors, objForClassification->id() );

        assureOTISSize();

        for( size_t classId = 0; classId < SceneObject::classifierList->size(); classId++ ) // iterate through all registered classes, calculating their likelihood
		{
			if( !pObjectTypesInScene[classId] ){ continue;} // then the user set a command that this type does not occur in the picture

			double classLikelihood = (*SceneObject::classifierList)[classId]( containingImg, roi, descriptors, objForClassification->id(), classId );

			if( classLikelihood>=0 ) objForClassification->addNewLikelihood( classId, classLikelihood ); // add new likelihood for class to object
        }

		Ptr<SceneObject> objReincarnation = objForClassification->recalculateClass( objForClassification ); // recalculate the objects class - the returned pointer points to the object itself and is already of the new child type if the object type has changed


		if( objReincarnation!=objForClassification ) objForClassification.addref(); // necessary because apparently the copy constructer of the pointer object doesn't increment the reference count - which means it is deleted twice if this isn't added
		
		pCategorized.push_front( objReincarnation );
		timeLeft = pScene->timeLeft()-time_overhead; 
        pEstimatedClassificationTime = SceneHandler::msTime()-time_1;

	}

	return;
}


Ptr<SceneObject> ObjectHandler::getObjForClassification()
{
	Ptr<SceneObject> objForClassification;

	if( pUncategorized.size()!=0 )
	{
		objForClassification = pUncategorized.back();
		pUncategorized.pop_back();
		return objForClassification;
	}
	else if( pCategorized.size()!=0 )
	{
		objForClassification = pCategorized.back();
		pCategorized.pop_back();
		return objForClassification;
	}
	else return NULL;
}


void ObjectHandler::fillDescriptorCreators( Mat& _containingImage, Mat& _descriptors, int _objId )
{
	if( _descriptors.empty() ) return;

	for( list<DescriptorCreator*>::iterator it = pDescriptorCreators.begin(); it != pDescriptorCreators.end(); it++ )
	{
		(*it)->fill( _containingImage, _descriptors, _objId );
	}
	return;
}


Point ObjectHandler::average( vector<Point>& _pointSet )
{
	Point myAvg(0,0);
    for(unsigned int i=0;i<_pointSet.size();i++ ) myAvg+=_pointSet[i];

	myAvg.x/=_pointSet.size();
	myAvg.y/=_pointSet.size();

	return myAvg;
}


void ObjectHandler::extendedLocalMaxima( Mat& _inputImage, Mat& _pointSet, unsigned int _range, double _minValue )
{
	unsigned int doubleRange=2*_range;
	for( int y=_range;y<=_inputImage.rows;y+=doubleRange ) // row iteration
	{
		unsigned int rowEnd = y+_range-1;
		unsigned int rowStart = y-_range;
        rowEnd = (rowEnd>(unsigned int)_inputImage.rows)?_inputImage.rows:rowEnd;

		for( int x=_range;x<=_inputImage.cols;x+=doubleRange ) // col iteration
		{
			unsigned int colEnd = x+_range-1;
            colEnd = (colEnd>(unsigned int)_inputImage.cols)?_inputImage.cols:colEnd;
			unsigned int colStart = x-_range;
			Mat localImage = _inputImage( Range(rowStart,rowEnd), Range(colStart,colEnd) );
			Point localMaxima;
			double maxValue;
			minMaxLoc( localImage, NULL, &maxValue, NULL, &localMaxima );
			if( maxValue>=_minValue )
			{
				Mat localMaxRow(1,3,CV_32FC1);
				localMaxRow.at<float>(0)=localMaxima.x+colStart;
				localMaxRow.at<float>(1)=localMaxima.y+rowStart;
				localMaxRow.at<float>(2)=maxValue;
				_pointSet.push_back( localMaxRow );
			}
		}
	}
	return;
}


void ObjectHandler::calcLocalMaxima( Mat& _inputImage, Mat& _pointSet, unsigned int _range, double _minValue )
{
	unsigned int doubleRange=2*_range;

	
	int yEnd = (int)ceil((double)_inputImage.rows/doubleRange);
	int xEnd = (int)ceil((double)_inputImage.cols/doubleRange);

    Ptr<Ptr<Mat> > tempMaxima = new Ptr<Mat>[yEnd];
	for( int i=0; i<yEnd ;i++ )
	{
		tempMaxima[i] = new Mat[xEnd];
	}
	tempMaxima.addref();

	// first iteration calculates maxima inside square grid
	for( int y=0;y<_inputImage.rows;y+=doubleRange ) // row iteration
	{
		int yIndex = y/doubleRange;

		unsigned int rowEnd = y+doubleRange-1;
		unsigned int rowStart = y;
        rowEnd = (rowEnd>(unsigned int)_inputImage.rows)?_inputImage.rows:rowEnd;
		
		for( int x=0;x<_inputImage.cols;x+=doubleRange ) // col iteration
		{
			int xIndex = x/doubleRange;
			unsigned int colEnd = x+doubleRange-1;
            colEnd = (colEnd>(unsigned int)_inputImage.cols)?_inputImage.cols:colEnd;
			unsigned int colStart = x;
			Mat localImage = _inputImage( Range(rowStart,rowEnd), Range(colStart,colEnd) );
			Point localMaxima;
			double maxValue;
			minMaxLoc( localImage, NULL, &maxValue, NULL, &localMaxima );
			
			Mat localMaxRow(1,4,CV_32FC1);
			localMaxRow.at<float>(0)=localMaxima.x+colStart;
			localMaxRow.at<float>(1)=localMaxima.y+rowStart;
			localMaxRow.at<float>(2)=maxValue;
			localMaxRow.at<float>(3)=1; // indicates that value is a minima
			
			tempMaxima[yIndex][xIndex]=localMaxRow;
			
		}
	}

	// second iteration filters out maxima in square grid that lie too close
	for( int y=0; y<yEnd ;y++ ) // row iteration
	{
		for( int x=0; x<xEnd ;x++ ) // col iteration
		{

			// compare with maxima in grid cell to the right
			if( x<(xEnd-1) )
			{
				if( tempMaxima[y][x].at<float>(0)+_range >= tempMaxima[y][x+1].at<float>(0) ) // maxima lie too close
				{
					if( tempMaxima[y][x].at<float>(2) > tempMaxima[y][x+1].at<float>(2) )
					{
						tempMaxima[y][x+1].at<float>(3)=0; // next is no local maxima
					}
					else if( tempMaxima[y][x].at<float>(2) < tempMaxima[y][x+1].at<float>(2) )
					{
						tempMaxima[y][x].at<float>(3)=0; // no local maxima
					}
					// keep both if they have same value
				}
			}
			// compare with maxima in grid cell below
			if( y<(yEnd-1) )
			{
				if( tempMaxima[y][x].at<float>(1)+_range >= tempMaxima[y+1][x].at<float>(1) ) // maxima lie too close
				{
					if( tempMaxima[y][x].at<float>(2) > tempMaxima[y+1][x].at<float>(2) )
					{
						tempMaxima[y+1][x].at<float>(3)=0; // next is no local maxima
					}
					else if( tempMaxima[y][x].at<float>(2) < tempMaxima[y+1][x].at<float>(2) )
					{
						tempMaxima[y][x].at<float>(3)=0; // no local maxima
					}
					// keep both if they have same value
				}
			}
			// compare with maxima in grid cell below and on the right
			if( y<(yEnd-1)&&x<(xEnd-1) )
			{
				if( tempMaxima[y][x].at<float>(0)+_range >= tempMaxima[y+1][x+1].at<float>(0) && tempMaxima[y][x].at<float>(1)+_range >= tempMaxima[y+1][x+1].at<float>(1) ) // maxima lie too close
				{
					if( tempMaxima[y][x].at<float>(2) > tempMaxima[y+1][x+1].at<float>(2) )
					{
						tempMaxima[y+1][x+1].at<float>(3)=0; // next is no local maxima
					}
					else if( tempMaxima[y][x].at<float>(2) < tempMaxima[y+1][x+1].at<float>(2) )
					{
						tempMaxima[y][x].at<float>(3)=0; // no local maxima
					}
					// keep both if they have same value
				}
			}

			if( tempMaxima[y][x].at<float>(2)>=_minValue && tempMaxima[y][x].at<float>(3)==1 )
			{
				_pointSet.push_back( tempMaxima[y][x](Range(0,1),Range(0,3)) );
			}
		}
	}
	return;
}


bool ObjectHandler::closeToWindowBorder( Point _point )
{
	Mat exampleWindow = pVideo->grey();
	if( exampleWindow.empty() ) return false;

	int lowerXBorder = window_border_range;
	int upperXBorder = exampleWindow.cols-window_border_range;
	int lowerYBorder = window_border_range;
	int upperYBorder = exampleWindow.rows-window_border_range;

	return (_point.x<lowerXBorder) || (_point.x>upperXBorder) || (_point.y<lowerYBorder) || (_point.y>upperYBorder);
}

// note: if there is a problem with the drawing, check if simplification of the boolean calculation x=p()||y leads to non-call of p() if y is true...
void ObjectHandler::draw( Mat& _image )
{
	bool keepDrawing = true;
	unsigned int drawLevel = 0;

	if( pPathMat==NULL ) pPathMat = new Mat( _image.size().height, _image.size().width, CV_8UC3, CV_RGB(0,0,0) );
    else if( pFramesSinceLastFade++ >= (unsigned int)path_fadeout_speed || path_fadeout_speed<0 )
	{
		*pPathMat -= Mat( pPathMat->size().height, pPathMat->size().width, CV_8UC3, CV_RGB(1,1,1) );
		pFramesSinceLastFade = 0;
	}

	while( keepDrawing )
	{
		keepDrawing = false;

        for( list<Ptr<SceneObject> >::iterator it = pCategorized.begin(); it!=pCategorized.end(); it++ )
		{
			if( path_length == -2 && drawLevel == 0 ) keepDrawing = !(*it)->drawLayer( *pPathMat, drawLevel, (*it)->color() ) || keepDrawing;
			else keepDrawing = !(*it)->drawLayer( _image, drawLevel, (*it)->color() ) || keepDrawing;
		}
        for( list<Ptr<SceneObject> >::iterator it = pUncategorized.begin(); it!=pUncategorized.end(); it++ )
		{
			if( path_length == -2 && drawLevel == 0 ) keepDrawing = !(*it)->drawLayer( *pPathMat, drawLevel, (*it)->color() ) || keepDrawing;
			else keepDrawing = !(*it)->drawLayer( _image, drawLevel, (*it)->color() ) || keepDrawing;
		}
		// write central path image into output image
		if( path_length == -2 && drawLevel == 0 ) _image -= *pPathMat;

		drawLevel++;
	}

	return;
}


void ObjectHandler::setupObjectTypesInfo( bool _initValue )
{
    size_t numberOfTypes = SceneObject::objectList->size();

	if( pObjectTypesInScene.size()!=numberOfTypes ) pObjectTypesInScene.resize( numberOfTypes );

    for( size_t i=0;i<numberOfTypes;i++ ) pObjectTypesInScene[i] = _initValue;
	
	return;
}


void ObjectHandler::assureOTISSize()
{
    size_t numberOfTypes = SceneObject::objectList->size();

	if( pObjectTypesInScene.size()!=numberOfTypes ) pObjectTypesInScene.resize( numberOfTypes, false );

	return;
}


void ObjectHandler::showMatchWindow( vector< list<Ptr<SceneObject> >::iterator >& _objList, Mat _predictedStates, Mat _contourPositions, vector<int>& _objectMapping, string _windowName )
{
	Mat test;
	test.create( 480,640,CV_32FC3 );

    for(unsigned int m=0;m<_objectMapping.size();m++)
	{
		if( _objectMapping[m]<0 ) continue;
		Point matchedContour(_contourPositions.at<float>(m,0),_contourPositions.at<float>(m,1));
		Point predictionMatched(_predictedStates.at<float>(_objectMapping[m],0),_predictedStates.at<float>(_objectMapping[m],1));
		line(test,predictionMatched,matchedContour,Scalar(255,255,255),2);
		stringstream conv; string out; conv<<(*_objList[_objectMapping[m]])->id(); conv>>out;
		putText(test,out,Point(matchedContour.x,matchedContour.y-20),0,0.5,Scalar(255,255,255));

	}
	for( int p=0;p<_predictedStates.rows;p++ )
	{
		Point predPos(_predictedStates.at<float>(p,0),_predictedStates.at<float>(p,1));
		stringstream conv; string out; conv<<p; conv>>out;
		circle(test,predPos,5,Scalar(255,0,0),2);
		putText(test,out,Point(predPos.x+7,predPos.y+7),0,1,Scalar(255,0,0));
	}
	for( int p=0;p<_contourPositions.rows;p++ )
	{
		Point predPos(_contourPositions.at<float>(p,0),_contourPositions.at<float>(p,1));
		stringstream conv; string out; conv<<p; conv>>out;
		circle(test,predPos,4,Scalar(0,255,0),1);
		putText(test,out,Point(predPos.x-20,predPos.y+7),0,0.5,Scalar(0,255,0));
    }

    imshow(_windowName,test);

	return;
}


unsigned int ObjectHandler::imageHeight()
{
	return pImageHeight;
}


unsigned int ObjectHandler::imageWidth()
{
	return pImageWidth;
}


const Mat& ObjectHandler::thresholdImage() const
{
	return pThresholdImage;
}



void ObjectHandler::setupOptions()
{
	Options::load_options();
	min_area = (*Options::General)["video_content_descriptions"]["min_area"].as<double>();
	max_area = (*Options::General)["video_content_descriptions"]["max_area"].as<double>();
	contour_length_switch = (*Options::General)["video_content_descriptions"]["contour_length_switch"].as<double>();
	recalculation_interval = (*Options::General)["object_detection"]["recalculation_interval"].as<unsigned int>();
	max_object_missing_time = (*Options::General)["object_detection"]["max_object_missing_time"].as<unsigned int>();
	missing_state_bridging = (*Options::General)["object_detection"]["missing_state_bridging"].as<int>();
	max_matching_distance_mismatch = (*Options::General)["object_detection"]["max_matching_distance_mismatch"].as<double>();
	use_old_state_fallback = (*Options::General)["object_detection"]["use_old_state_fallback"].as<bool>();
	max_object_gridpoint_distance = (*Options::General)["object_detection"]["max_object_gridpoint_distance"].as<double>();
	static_threshold = (*Options::General)["object_detection"]["static_threshold"].as<unsigned int>();
	window_border_range = (*Options::General)["object_detection"]["window_border_range"].as<unsigned int>();
	path_length = (*Options::General)["display"]["objects"]["path_length"].as<int>();
	path_fadeout_speed = (*Options::General)["display"]["objects"]["path_fadeout_speed"].as<int>();
	time_awareness = (*Options::General)["classification"]["time_awareness"].as<bool>();
	time_overhead = (*Options::General)["classification"]["time_overhead"].as<double>();
	create_threshold_detection_image = (*Options::General)["display"]["objects"]["create_threshold_detection_image"].as<bool>();
	draw_predicted_regions = (*Options::General)["display"]["objects"]["draw_predicted_regions"].as<bool>();

	
	return;
}


bool ObjectHandler::typeIsInScene( int _classId )
{
    if( _classId<0 || (unsigned int)_classId>=pObjectTypesInScene.size() ) return false;

	return pObjectTypesInScene[_classId];
}


unsigned int ObjectHandler::recalculation_interval;
double ObjectHandler::min_area;
double ObjectHandler::max_area;
double ObjectHandler::contour_length_switch;
unsigned int ObjectHandler::max_object_missing_time; // [nr of frames]
int ObjectHandler::missing_state_bridging;
double ObjectHandler::max_matching_distance_mismatch;
bool ObjectHandler::use_old_state_fallback;
double ObjectHandler::max_object_gridpoint_distance;
unsigned int ObjectHandler::static_threshold;
unsigned int ObjectHandler::window_border_range;
int ObjectHandler::path_length;
int ObjectHandler::path_fadeout_speed;
bool ObjectHandler::time_awareness;
double ObjectHandler::time_overhead;
bool ObjectHandler::create_threshold_detection_image;
bool ObjectHandler::draw_predicted_regions;

double ObjectHandler::two_pi=2*acos(-1.0);
double ObjectHandler::pi=acos(-1.0);
double ObjectHandler::pi_half=acos(-1.0)/2;
double ObjectHandler::pi_quarter=acos(-1.0)/4;
