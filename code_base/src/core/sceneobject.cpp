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

#include "objecthandler.h"


SceneObject::SceneObject( ObjectHandler* _environmentControl ):pType(-2),pTimeSincePredictionReset(-2)
{
	pEnvironmentControl = _environmentControl;
	pObjectId=objectCount++;
	setupOptions();
	pPathMat = NULL;
	pFramesSinceLastFade = 0;
	
	pAccEMA_S_old.push_back(0);
	pAccEMA_S_old.push_back(0);
	pAccEMA_S_old.push_back(0);
	pVelEMA_S_old.push_back(0);
	pVelEMA_S_old.push_back(0);
	pVelEMA_S_old.push_back(0);
}


SceneObject::SceneObject( Ptr<SceneObject> _toCopy )
{
	pEnvironmentControl = _toCopy->pEnvironmentControl;
	pObjectId = _toCopy->pObjectId;
	pHistory = _toCopy->pHistory;
	pROI[0] = _toCopy->pROI[0];
	pROI[1] = _toCopy->pROI[1];
	pROI[2] = _toCopy->pROI[2];
	pROI[3] = _toCopy->pROI[3];
	pType = _toCopy->pType;
	pTimeSincePredictionReset = _toCopy->pTimeSincePredictionReset;
	pClassLikelihood = _toCopy->pClassLikelihood;
	pPathMat = _toCopy->pPathMat;
	pFramesSinceLastFade = _toCopy->pFramesSinceLastFade;
	
	if( pAccEMA_S_old.size()==0 )
	{
		pAccEMA_S_old.push_back( _toCopy->pAccEMA_S_old[0] );
		pAccEMA_S_old.push_back( _toCopy->pAccEMA_S_old[1] );
		pAccEMA_S_old.push_back( _toCopy->pAccEMA_S_old[2] );
		pVelEMA_S_old.push_back( _toCopy->pVelEMA_S_old[0] );
		pVelEMA_S_old.push_back( _toCopy->pVelEMA_S_old[1] );
		pVelEMA_S_old.push_back( _toCopy->pVelEMA_S_old[2] );
	}
	else
	{
		pAccEMA_S_old[0] = _toCopy->pAccEMA_S_old[0];
		pAccEMA_S_old[1] = _toCopy->pAccEMA_S_old[1];
		pAccEMA_S_old[2] = _toCopy->pAccEMA_S_old[2];
		pVelEMA_S_old[0] = _toCopy->pVelEMA_S_old[0];
		pVelEMA_S_old[1] = _toCopy->pVelEMA_S_old[1];
		pVelEMA_S_old[2] = _toCopy->pVelEMA_S_old[2];
	}

	setupOptions();
}


SceneObject::~SceneObject(void)
{
}


Ptr<SceneObject> SceneObject::newObject()
{
	return Ptr<SceneObject>( new SceneObject(NULL) );
}


unsigned int SceneObject::id()
{
	return pObjectId;
}


void SceneObject::classProperties( string& _className, Scalar& _classColor )
{
	if( pType==-2 )
	{
		_className="unclassified";
		_classColor = Scalar(0,0,0);
	}
	else
	{
		_className="unmatched";
		_classColor = Scalar(20,35,230);
	}
	return;
}


Scalar SceneObject::color()
{
	return Scalar(232,30,37); 

	if( pType==-2 )
	{
		return Scalar(0,0,0);
	}
	else
	{
		return Scalar(35,170,250);
	}
}


bool SceneObject::touchesImageBorder()
{
	int minX = min( min(pROI[0].x,pROI[1].x), min(pROI[2].x,pROI[3].x) );
	int maxX = max( max(pROI[0].x,pROI[1].x), max(pROI[2].x,pROI[3].x) );

	int minY = min( min(pROI[0].y,pROI[1].y), min(pROI[2].y,pROI[3].y) );
	int maxY = max( max(pROI[0].y,pROI[1].y), max(pROI[2].y,pROI[3].y) );

	int imgHeight = pEnvironmentControl->imageHeight();
	int imgWidth = pEnvironmentControl->imageWidth();
	int borderRange = ObjectHandler::window_border_range;

	return minX<=borderRange || minY<=borderRange || maxX>=imgWidth-borderRange || maxY>=imgHeight-borderRange;
}


Ptr<SceneObject::State> SceneObject::newState( Mat& _state, RectangleRegion& /*_region*/, vector<Point>& /*_contour*/, Mat& /*_invImg*/, double _time )
{
	State* newState = new State( _state.at<float>(0), _state.at<float>(1), _time, _state.at<float>(2), type() );
	return Ptr<State>(newState);
}


bool SceneObject::addState( Ptr<SceneObject::State> _newState, RectangleRegion& _regionOfInterest, vector<Point>& _contour )
{
	bool returnvalue = this->addStateProcessing( _newState, _regionOfInterest, _contour );

	// update exponential moving average of acceleration and velocity (these are used for predictions)
	double currAccX = accX();
	double currAccY = accY();
	double currAccAng = accAng();
	pAccEMA_S_old[0] = pAccEMA_alpha*currAccX + (1-pAccEMA_alpha)*pAccEMA_S_old[0];
	pAccEMA_S_old[1] = pAccEMA_alpha*currAccY + (1-pAccEMA_alpha)*pAccEMA_S_old[1];
	pAccEMA_S_old[2] = pAccEMA_alpha*currAccAng + (1-pAccEMA_alpha)*pAccEMA_S_old[2];

	double currVelX = velX();
	double currVelY = velY();
	double currVelAng = velAng();
	pVelEMA_S_old[0] = pVelEMA_alpha*currVelX + (1-pVelEMA_alpha)*pVelEMA_S_old[0];
	pVelEMA_S_old[1] = pVelEMA_alpha*currVelY + (1-pVelEMA_alpha)*pVelEMA_S_old[1];
	pVelEMA_S_old[2] = pVelEMA_alpha*currVelAng + (1-pVelEMA_alpha)*pVelEMA_S_old[2];

	return returnvalue;
}



bool SceneObject::addStateProcessing( Ptr<SceneObject::State> _newState, RectangleRegion& _regionOfInterest, vector<Point>& _contour )
{
	pHistory.push_back( _newState );

	if( !trace_states )
	{
		while( pHistory.size()>3 ) pHistory.pop_front();
	}

	_regionOfInterest.points( pROI );
	pLastContour = _contour;

	if( pTimeSincePredictionReset>=-1 ) pTimeSincePredictionReset++;
	if( pTimeSincePredictionReset > 1 ) pTimeSincePredictionReset=-1; // prediction reset has no effect anymore

	return true;
}


int SceneObject::findMatchingArea( vector<RectangleRegion>& _regions, vector< vector<Point> > _contours)
{
	vector<int> matchCandidates; // array to hold the index of all contours of which the object possibly could be a part of

	Point predictedCenterPosition = predictCtrPosition();
	Point usedCenterPosition = predictedCenterPosition; // stores the center position for which the object finally has been matched (either the predicted or the old position)

    for( size_t r=0; r<_regions.size(); r++ ) // check for each new state if it is a candidate for a match
	{
		Point relativeCenterPosition = predictedCenterPosition - _regions[r].getOrigin();
		if( _regions[r].contains( relativeCenterPosition ) ) // if predicted center lies inside the rectangle boundary of the region
		{
			//cout<<endl<<"obj "<<id()<<" found match through predicted position lying in _region"<<endl;
			matchCandidates.push_back(r);
		}
	}
	if( matchCandidates.size()==0 && use_old_state_fallback ) // check the old state instead of the predicted if no match was found for the latter
	{
		Point lastCenterPosition = pos();
		usedCenterPosition = lastCenterPosition; // outside of check whether it is actually used in order not to lie in for loop - and its correct since the prediction position failed and either the old will succeed or none will in which case the variable isn't used

        for( size_t r=0; r<_regions.size(); r++ )
		{
			Point relativeCenterPosition = lastCenterPosition - _regions[r].getOrigin();
			if( _regions[r].contains( relativeCenterPosition ) ) // if predicted center lies inside the rectangle boundary of the region
			{
				//cout<<endl<<"obj "<<id()<<" found match through old position lying in _region"<<endl;
				matchCandidates.push_back(r);
				resetPrediction(); // since the match is found at the old position but the predicted position didn't lie in any contour, it can be safely assumed that there was a situation where the used prediction method failed. This call tells so to the object, which than can react accordingly (or not)
			}
		}
	}
	if( matchCandidates.size()==0 )
	{
		vector<Point> roi;
		predictROI(roi);
		RectangleRegion predictedRegion( roi[0],roi[1],roi[2],roi[3] );
		vector<Point2f> gridPoints;
		predictedRegion.gridPoints( gridPoints, max_object_gridpoint_distance );
				
		int areaIdxWithHighestOverlap=-1, highestOverlap=0;

        for( size_t r=0; r<_contours.size(); r++ )
		{
			int overlap=0;
            for( size_t pt=0; pt<gridPoints.size(); pt++ ) // calculating the region overlap
			{
						
				if( pointPolygonTest( _contours[ r ], gridPoints[pt], false )==1 ) // tested points lies inside contour
				{
					overlap++;
				}
			}
			if( overlap>highestOverlap )
			{
				//cout<<endl<<"the overlap was "<<overlap<<" given a number of "<<gridPoints.size()<<" points."<<endl;
				areaIdxWithHighestOverlap = r;
				highestOverlap=overlap;
			}
		}
		if( areaIdxWithHighestOverlap!=-1 )
		{
			/*cout<<endl<<"obj "<<id()<<" found match through region overlap"<<endl;

			Mat test;
			test.create( 480,640,CV_32FC3 );
			for( int pt=0; pt<gridPoints.size(); pt++ )
			{
				circle(test,gridPoints[pt],1,Scalar(255,255,255),2);
			}
			drawContours(test,_contours,areaIdxWithHighestOverlap,Scalar(0,255,0));
			imshow("gridpoints",test);
			waitKey(0);*/

			matchCandidates.push_back(areaIdxWithHighestOverlap);
		}
				
	}


	if( matchCandidates.size()==1 ) // if only one candidate was found, directly accept it without additional tests
	{
		return matchCandidates[0]; // build groups for matching of groups of missing objects to a found region		
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
		return match; // build groups for matching of groups of missing objects to a found region
		
	}
	// no matching area in set
	return -1;
}


bool SceneObject::exportData( string _filePath, string _dataPointSeparator, string _timeStepSeparator )
{
	try
	{
		std::ofstream file;
        file.open( _filePath.c_str(),std::ios_base::trunc ); // old file is overwritten

		file<<"time"<<_dataPointSeparator<<"x_pos"<<_dataPointSeparator<<"y_pos"<<_dataPointSeparator<<"angle"<<_timeStepSeparator;

        for( size_t i=0;i<pHistory.size(); i++ )
		{
			file<<pHistory[i]->time<<_dataPointSeparator<<pHistory[i]->x<<_dataPointSeparator<<pHistory[i]->y<<_dataPointSeparator<<pHistory[i]->angle<<_timeStepSeparator;
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


Ptr<SceneObject::State> SceneObject::state()
{
	return pHistory.back();
}


Mat SceneObject::history()
{
	Mat stateHistory;

    for( size_t i=0; i<pHistory.size(); i++ )
	{
		Mat state;
		state.create(1,3,CV_64FC1);
		state.at<double>(0)=pHistory[i]->x;
		state.at<double>(1)=pHistory[i]->y;
		state.at<double>(2)=pHistory[i]->angle;

		stateHistory.push_back(state);
	}
	return stateHistory;
}


Mat SceneObject::currentState()
{
	Mat state;
	state.create(1,3,CV_64FC1);
	state.at<double>(0)=pHistory.back()->x;
	state.at<double>(1)=pHistory.back()->y;
	state.at<double>(2)=pHistory.back()->angle;
	return state;
}


Point SceneObject::pos()
{
	return Point( (int)pHistory.back()->x, (int)pHistory.back()->y );
}


double SceneObject::angle()
{
	return pHistory.back()->angle;
}


Point SceneObject::direction()
{
	double actAngle = angle();
	double x = cos(actAngle);
	double y = sin(actAngle);
	return Point( (int)x, (int)y );
}


double SceneObject::area()
{
	return pHistory.back()->area;
}


double SceneObject::velX()
{
	if( pHistory.size()==0 ) return 0;
	else if( pHistory.size()==1 || pTimeSincePredictionReset==0 )
	{
		return 0;
	}

	Ptr<State> last_1 = pHistory[ pHistory.size()-1 ];
	Ptr<State> last_2 = pHistory[ pHistory.size()-2 ];

	return (*last_1).x-(*last_2).x;
}


double SceneObject::velY()
{
	if( pHistory.size()==0 ) return 0;
	else if( pHistory.size()==1 || pTimeSincePredictionReset==0 )
	{
		return 0;
	}

	Ptr<State> last_1 = pHistory[ pHistory.size()-1 ];
	Ptr<State> last_2 = pHistory[ pHistory.size()-2 ];

	return (*last_1).y-(*last_2).y;
}


double SceneObject::velAng()
{
	if( pHistory.size()==0 ) return 0;
	else if( pHistory.size()==1 || pTimeSincePredictionReset==0 )
	{
		return 0;
	}

	Ptr<State> last_1 = pHistory[ pHistory.size()-1 ];
	Ptr<State> last_2 = pHistory[ pHistory.size()-2 ];

	return calcAngleVelocity( (*last_2).angle, (*last_1).angle );
}


double SceneObject::accX()
{
	if( pHistory.size()==0 ) return 0;
	else if( pHistory.size()==1 || pTimeSincePredictionReset==0 )
	{
		return 0;
	}
	else if( pHistory.size()==2 || pTimeSincePredictionReset==1 )
	{
		return 0;
	}

	Ptr<State> last_1 = pHistory[ pHistory.size()-1 ];
	Ptr<State> last_2 = pHistory[ pHistory.size()-2 ];
	Ptr<State> last_3 = pHistory[ pHistory.size()-3 ];

	double velX_1 = (*last_1).x-(*last_2).x;
	double velX_2 = (*last_2).x-(*last_3).x;
	return velX_1-velX_2;
}


double SceneObject::accY()
{
	if( pHistory.size()==0 ) return 0;
	else if( pHistory.size()==1 || pTimeSincePredictionReset==0 )
	{
		return 0;
	}
	else if( pHistory.size()==2 || pTimeSincePredictionReset==1 )
	{
		return 0;
	}

	Ptr<State> last_1 = pHistory[ pHistory.size()-1 ];
	Ptr<State> last_2 = pHistory[ pHistory.size()-2 ];
	Ptr<State> last_3 = pHistory[ pHistory.size()-3 ];

	double velY_1 = (*last_1).y-(*last_2).y;
	double velY_2 = (*last_2).y-(*last_3).y;
	return velY_1-velY_2;
}


double SceneObject::accAng()
{
	if( pHistory.size()==0 ) return 0;
	else if( pHistory.size()==1 || pTimeSincePredictionReset==0 )
	{
		return 0;
	}
	else if( pHistory.size()==2 || pTimeSincePredictionReset==1 )
	{
		return 0;
	}

	Ptr<State> last_1 = pHistory[ pHistory.size()-1 ];
	Ptr<State> last_2 = pHistory[ pHistory.size()-2 ];
	Ptr<State> last_3 = pHistory[ pHistory.size()-3 ];

	double angSp_1 = calcAngleVelocity( (*last_2).angle, (*last_1).angle );
	double angSp_2 = calcAngleVelocity( (*last_3).angle, (*last_2).angle );
	return angSp_1-angSp_2;
}


double SceneObject::calcAngleVelocity( double _angleBefore, double _angleAfter )
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


double SceneObject::moveAngleIntoRange( double _angle )
{
	double angle = _angle;
	while( angle<0 ) angle += ObjectHandler::two_pi;
	while( angle>ObjectHandler::two_pi ) angle -= ObjectHandler::two_pi;

	return angle;
}


double SceneObject::findClosestAngleEquivalent( double _angleToAdjust, double _angleToApproach )
{
	if( _angleToAdjust<_angleToApproach )
	{
		double temp1 = _angleToAdjust;
		double temp2 = temp1+ObjectHandler::two_pi;
		
		double dist1 = abs(_angleToApproach-temp1);
		double dist2 = abs(_angleToApproach-temp2);

		while( dist2<dist1 )
		{
			temp2+=ObjectHandler::two_pi;
			dist1 = dist2;
			dist2 = abs(_angleToApproach-temp2);
		}
		return temp2-ObjectHandler::two_pi;
	}

	double temp1 = _angleToAdjust;
	double temp2 = temp1-ObjectHandler::two_pi;
		
	double dist1 = abs(_angleToApproach-temp1);
	double dist2 = abs(_angleToApproach-temp2);

	while( dist2<dist1 )
	{
		temp2-=ObjectHandler::two_pi;
		dist1 = dist2;
		dist2 = abs(_angleToApproach-temp2);
	}
	return temp2+ObjectHandler::two_pi;
}


RectangleRegion SceneObject::lastROI()
{
	return RectangleRegion( pROI[0], pROI[1], pROI[2], pROI[3] );
}


void SceneObject::lastContour( vector<Point>& _contour )
{
	_contour=pLastContour;
	return;
}


void SceneObject::predictROI( vector<Point>& _roi )
{
	_roi.push_back( predictPosition( pROI[0] ) );
	_roi.push_back( predictPosition( pROI[1] ) );
	_roi.push_back( predictPosition( pROI[2] ) );
	_roi.push_back( predictPosition( pROI[3] ) );
	return;
}

Mat SceneObject::predictState()
{
	if( pHistory.size()==0 ) return Mat();
	else if( pHistory.size()==1 || pTimeSincePredictionReset==0 )
	{
		Mat prediction;
		prediction.create(1,3,CV_32FC1);
		prediction.at<float>(0)=(float)pHistory.back()->x;
		prediction.at<float>(1)=(float)pHistory.back()->y;
		prediction.at<float>(2)=(float)pHistory.back()->angle;
		return prediction;
	}

	// velocity based prediction
	else if( pHistory.size()==2 || pTimeSincePredictionReset==1 )
	{
		Ptr<State> last = pHistory[ pHistory.size()-1 ];
		Ptr<State> secondlast = pHistory[ pHistory.size()-2 ];

		double velX = (*last).x-(*secondlast).x;
		velX = pVelEMA_alpha*velX + (1-pVelEMA_alpha)*pVelEMA_S_old[0];
		double velY = (*last).y-(*secondlast).y;
		velY = pVelEMA_alpha*velY + (1-pVelEMA_alpha)*pVelEMA_S_old[1];
		double angSpeed = calcAngleVelocity( (*secondlast).angle , (*last).angle );
		angSpeed = pVelEMA_alpha*angSpeed + (1-pVelEMA_alpha)*pVelEMA_S_old[2];

		Mat prediction;
		prediction.create(1,3,CV_32FC1);
		prediction.at<float>(0) = (float)((*last).x+velX);
		prediction.at<float>(1) = (float)((*last).y+velY);
		prediction.at<float>(2) = (float)((*last).angle+angSpeed);
		return prediction;
	}

	// acceleration based prediction
	Ptr<State> last_1 = pHistory[ pHistory.size()-1 ];
	Ptr<State> last_2 = pHistory[ pHistory.size()-2 ];
	Ptr<State> last_3 = pHistory[ pHistory.size()-3 ];

	double velX_1 = (*last_1).x-(*last_2).x;
	double velX_2 = (*last_2).x-(*last_3).x;
	double accX = velX_1-velX_2;
	accX = pAccEMA_alpha*accX + (1-pAccEMA_alpha)*pAccEMA_S_old[0];

	double velY_1 = (*last_1).y-(*last_2).y;
	double velY_2 = (*last_2).y-(*last_3).y;
	double accY = velY_1-velY_2;
	accY = pAccEMA_alpha*accY + (1-pAccEMA_alpha)*pAccEMA_S_old[1];


	double angSp_1 = calcAngleVelocity( (*last_2).angle , (*last_1).angle );
	double angSp_2 = calcAngleVelocity( (*last_3).angle , (*last_2).angle );
	double angAcc = angSp_1-angSp_2;
	angAcc = pAccEMA_alpha*angAcc + (1-pAccEMA_alpha)*pAccEMA_S_old[2];
	
	velX_1 = pVelEMA_alpha*velX_1 + (1-pVelEMA_alpha)*pVelEMA_S_old[0];
	velY_1 = pVelEMA_alpha*velY_1 + (1-pVelEMA_alpha)*pVelEMA_S_old[1];
	angSp_1 = pVelEMA_alpha*angSp_1 + (1-pVelEMA_alpha)*pVelEMA_S_old[2];

	Mat prediction;
	prediction.create(1,3,CV_32FC1);
	prediction.at<float>(0) = (float)(0.5*accX+velX_1+(*last_1).x);
	prediction.at<float>(1) = (float)(0.5*accY+velY_1+(*last_1).y);
	prediction.at<float>(2) = (float)(0.5*angAcc+angSp_1+(*last_1).angle);
	return prediction;
}


void SceneObject::resetPrediction()
{
	pTimeSincePredictionReset=0; // if 0: resetPrediction immediately affects all predictions, if -1: resetPrediction affects all predictions after the next state was added
	return;
}


Point SceneObject::predictPosition( Point _pt )
{
	double angleVel = predictAglVelocity();

	Point predictedCenter = predictCtrPosition();
	Point center( (int)pHistory.back()->x, (int)pHistory.back()->y );

	Point relPos = _pt-center;

	double cosEl = cos(angleVel);
	double sinEl = sin(angleVel);
	double newRelX = cosEl*relPos.x - sinEl*relPos.y;
	double newRelY = sinEl*relPos.x + cosEl*relPos.y;

	Point newRelPos( (int)newRelX, (int)newRelY );

	return predictedCenter+newRelPos;
}

Point SceneObject::predictCtrPosition()
{
	if( pHistory.size()==0 ) return Point(0,0);
	else if( pHistory.size()==1 || pTimeSincePredictionReset==0 ) return Point( (int)pHistory.back()->x, (int)pHistory.back()->y );

	// velocity based prediction
	else if( pHistory.size()==2 || pTimeSincePredictionReset==1 )
	{
		Ptr<State> last = pHistory[ pHistory.size()-1 ];
		Ptr<State> secondlast = pHistory[ pHistory.size()-2 ];

		double velX = (*last).x-(*secondlast).x;
		double velY = (*last).y-(*secondlast).y;
		velX = pVelEMA_alpha*velX + (1-pVelEMA_alpha)*pVelEMA_S_old[0];
		velY = pVelEMA_alpha*velY + (1-pVelEMA_alpha)*pVelEMA_S_old[1];

		return Point( (int)((*last).x+velX), (int)((*last).y+velY) );
	}

	// acceleration based prediction
	Ptr<State> last_1 = pHistory[ pHistory.size()-1 ];
	Ptr<State> last_2 = pHistory[ pHistory.size()-2 ];
	Ptr<State> last_3 = pHistory[ pHistory.size()-3 ];

	double velX_1 = (*last_1).x-(*last_2).x;
	double velX_2 = (*last_2).x-(*last_3).x;
	double accX = velX_1-velX_2;
	accX = pAccEMA_alpha*accX + (1-pAccEMA_alpha)*pAccEMA_S_old[0];

	double velY_1 = (*last_1).y-(*last_2).y;
	double velY_2 = (*last_2).y-(*last_3).y;
	double accY = velY_1-velY_2;
	accY = pAccEMA_alpha*accY + (1-pAccEMA_alpha)*pAccEMA_S_old[1];
	
	velX_1 = pVelEMA_alpha*velX_1 + (1-pVelEMA_alpha)*pVelEMA_S_old[0];
	velY_1 = pVelEMA_alpha*velY_1 + (1-pVelEMA_alpha)*pVelEMA_S_old[1];

	return Point( (int)(0.5*accX+velX_1+(*last_1).x), (int)(0.5*accY+velY_1+(*last_1).y) );
}

Point2f SceneObject::predictCtrFPosition()
{
	if( pHistory.size()==0 ) return Point(0,0);
	else if( pHistory.size()==1 || pTimeSincePredictionReset==0 ) return Point( (int)pHistory.back()->x, (int)pHistory.back()->y );

	// velocity based prediction
	else if( pHistory.size()==2 || pTimeSincePredictionReset==1 )
	{
		Ptr<State> last = pHistory[ pHistory.size()-1 ];
		Ptr<State> secondlast = pHistory[ pHistory.size()-2 ];

		double velX = (*last).x-(*secondlast).x;
		double velY = (*last).y-(*secondlast).y;
		velX = pVelEMA_alpha*velX + (1-pVelEMA_alpha)*pVelEMA_S_old[0];
		velY = pVelEMA_alpha*velY + (1-pVelEMA_alpha)*pVelEMA_S_old[1];

		return Point( (int)((*last).x+velX), (int)((*last).y+velY) );
	}

	// acceleration based prediction
	Ptr<State> last_1 = pHistory[ pHistory.size()-1 ];
	Ptr<State> last_2 = pHistory[ pHistory.size()-2 ];
	Ptr<State> last_3 = pHistory[ pHistory.size()-3 ];

	double velX_1 = (*last_1).x-(*last_2).x;
	double velX_2 = (*last_2).x-(*last_3).x;
	double accX = velX_1-velX_2;
	accX = pAccEMA_alpha*accX + (1-pAccEMA_alpha)*pAccEMA_S_old[0];

	double velY_1 = (*last_1).y-(*last_2).y;
	double velY_2 = (*last_2).y-(*last_3).y;
	double accY = velY_1-velY_2;
	accY = pAccEMA_alpha*accY + (1-pAccEMA_alpha)*pAccEMA_S_old[1];
	
	velX_1 = pVelEMA_alpha*velX_1 + (1-pVelEMA_alpha)*pVelEMA_S_old[0];
	velY_1 = pVelEMA_alpha*velY_1 + (1-pVelEMA_alpha)*pVelEMA_S_old[1];

	return Point2f( (0.5*accX+velX_1+(*last_1).x), (0.5*accY+velY_1+(*last_1).y) );
}

double SceneObject::predictXCtrVelocity()
{
	if( pHistory.size()==0 ) return 0;
	else if( pHistory.size()==1 || pTimeSincePredictionReset==0 ) return 0;

	// velocity based prediction
	else if( pHistory.size()==2 || pTimeSincePredictionReset==1 )
	{
		Ptr<State> last = pHistory[ pHistory.size()-1 ];
		Ptr<State> secondlast = pHistory[ pHistory.size()-2 ];
		
		double velX = pVelEMA_alpha*((*last).x-(*secondlast).x) + (1-pVelEMA_alpha)*pVelEMA_S_old[0];

		return velX;
	}

	// acceleration based prediction
	Ptr<State> last_1 = pHistory[ pHistory.size()-1 ];
	Ptr<State> last_2 = pHistory[ pHistory.size()-2 ];
	Ptr<State> last_3 = pHistory[ pHistory.size()-3 ];

	double velX_1 = (*last_1).x-(*last_2).x;
	double velX_2 = (*last_2).x-(*last_3).x;
	double accX = velX_1-velX_2;
	accX = pAccEMA_alpha*accX + (1-pAccEMA_alpha)*pAccEMA_S_old[0];
	velX_1 = pVelEMA_alpha*velX_1 + (1-pVelEMA_alpha)*pVelEMA_S_old[0];

	return accX+velX_1;
}

double SceneObject::predictXCtrAcceleration()
{
	if( pHistory.size()==0 ) return 0;
	else if( pHistory.size()==1 || pTimeSincePredictionReset==0 ) return 0;
	else if( pHistory.size()==2 || pTimeSincePredictionReset==1 ) return 0;

	// acceleration based prediction
	Ptr<State> last_1 = pHistory[ pHistory.size()-1 ];
	Ptr<State> last_2 = pHistory[ pHistory.size()-2 ];
	Ptr<State> last_3 = pHistory[ pHistory.size()-3 ];

	double velX_1 = (*last_1).x-(*last_2).x;
	double velX_2 = (*last_2).x-(*last_3).x;
	double accX = velX_1-velX_2;
	accX = pAccEMA_alpha*accX + (1-pAccEMA_alpha)*pAccEMA_S_old[0];

	return accX;
}

double SceneObject::predictYCtrVelocity()
{
	if( pHistory.size()==0 ) return 0;
	else if( pHistory.size()==1 || pTimeSincePredictionReset==0 ) return 0;

	// velocity based prediction
	else if( pHistory.size()==2 || pTimeSincePredictionReset==1 )
	{
		Ptr<State> last = pHistory[ pHistory.size()-1 ];
		Ptr<State> secondlast = pHistory[ pHistory.size()-2 ];

		double velY_1 = pVelEMA_alpha*((*last).y-(*secondlast).y) + (1-pVelEMA_alpha)*pVelEMA_S_old[1];

		return velY_1;
	}

	// acceleration based prediction
	Ptr<State> last_1 = pHistory[ pHistory.size()-1 ];
	Ptr<State> last_2 = pHistory[ pHistory.size()-2 ];
	Ptr<State> last_3 = pHistory[ pHistory.size()-3 ];

	double velY_1 = (*last_1).y-(*last_2).y;
	double velY_2 = (*last_2).y-(*last_3).y;
	double accY = velY_1-velY_2;
	accY = pAccEMA_alpha*accY + (1-pAccEMA_alpha)*pAccEMA_S_old[1];
	velY_1 = pVelEMA_alpha*velY_1 + (1-pVelEMA_alpha)*pVelEMA_S_old[1];

	return accY+velY_1;
}

double SceneObject::predictYCtrAcceleration()
{
	if( pHistory.size()==0 ) return 0;
	else if( pHistory.size()==1 || pTimeSincePredictionReset==0 ) return 0;
	else if( pHistory.size()==2 || pTimeSincePredictionReset==1 ) return 0;

	// acceleration based prediction
	Ptr<State> last_1 = pHistory[ pHistory.size()-1 ];
	Ptr<State> last_2 = pHistory[ pHistory.size()-2 ];
	Ptr<State> last_3 = pHistory[ pHistory.size()-3 ];

	double velY_1 = (*last_1).y-(*last_2).y;
	double velY_2 = (*last_2).y-(*last_3).y;
	double accY = velY_1-velY_2;
	accY = pAccEMA_alpha*accY + (1-pAccEMA_alpha)*pAccEMA_S_old[1];

	return accY;
}

double SceneObject::predictAgl()
{
	if( pHistory.size()==0 ) return 0;
	else if( pHistory.size()==1 || pTimeSincePredictionReset==0 ) return pHistory.back()->angle;

	// velocity based prediction
	else if( pHistory.size()==2 || pTimeSincePredictionReset==1 )
	{
		Ptr<State> last = pHistory[ pHistory.size()-1 ];
		Ptr<State> secondlast = pHistory[ pHistory.size()-2 ];

		double velA = calcAngleVelocity( (*secondlast).angle , (*last).angle );
		velA = pAccEMA_alpha*velA + (1-pAccEMA_alpha)*pAccEMA_S_old[2];

		return (*last).angle + velA;
	}

	// acceleration based prediction
	Ptr<State> last_1 = pHistory[ pHistory.size()-1 ];
	Ptr<State> last_2 = pHistory[ pHistory.size()-2 ];
	Ptr<State> last_3 = pHistory[ pHistory.size()-3 ];

	double velA_1 = calcAngleVelocity( (*last_2).angle , (*last_1).angle );
	double velA_2 = calcAngleVelocity( (*last_3).angle , (*last_2).angle );
	double accA = velA_1-velA_2;
	accA = pAccEMA_alpha*accA + (1-pAccEMA_alpha)*pAccEMA_S_old[2];
	
	velA_1 = pAccEMA_alpha*velA_1 + (1-pAccEMA_alpha)*pAccEMA_S_old[2];

	return (*last_1).angle+velA_1+0.5*accA;
}

double SceneObject::predictAglVelocity()
{
	if( pHistory.size()==0 ) return 0;
	else if( pHistory.size()==1 || pTimeSincePredictionReset==0 ) return 0;

	// velocity based prediction
	else if( pHistory.size()==2 || pTimeSincePredictionReset==1 )
	{
		Ptr<State> last = pHistory[ pHistory.size()-1 ];
		Ptr<State> secondlast = pHistory[ pHistory.size()-2 ];
		double angSp_1 = pAccEMA_alpha*calcAngleVelocity( (*secondlast).angle , (*last).angle ) + (1-pAccEMA_alpha)*pAccEMA_S_old[2];

		return angSp_1;
	}

	// acceleration based prediction
	Ptr<State> last_1 = pHistory[ pHistory.size()-1 ];
	Ptr<State> last_2 = pHistory[ pHistory.size()-2 ];
	Ptr<State> last_3 = pHistory[ pHistory.size()-3 ];
	
	double angSp_1 = calcAngleVelocity( (*last_2).angle , (*last_1).angle );
	double angSp_2 = calcAngleVelocity( (*last_3).angle , (*last_2).angle );
	double angAcc = angSp_1-angSp_2;
	angAcc = pAccEMA_alpha*angAcc + (1-pAccEMA_alpha)*pAccEMA_S_old[2];
	
	angSp_1 = pAccEMA_alpha*angSp_1 + (1-pAccEMA_alpha)*pAccEMA_S_old[2];

	return angAcc+angSp_1;
}

double SceneObject::predictAglAcceleration()
{
	if( pHistory.size()==0 ) return 0;
	else if( pHistory.size()==1 || pTimeSincePredictionReset==0 ) return 0;
	else if( pHistory.size()==2 || pTimeSincePredictionReset==1 ) return 0;

	// acceleration based prediction
	Ptr<State> last_1 = pHistory[ pHistory.size()-1 ];
	Ptr<State> last_2 = pHistory[ pHistory.size()-2 ];
	Ptr<State> last_3 = pHistory[ pHistory.size()-3 ];

	double angleVel_1 = calcAngleVelocity( (*last_2).angle , (*last_1).angle );
	double angleVel_2 = calcAngleVelocity( (*last_3).angle , (*last_2).angle );
	double angAcc = angleVel_1-angleVel_2;
	angAcc = pAccEMA_alpha*angAcc + (1-pAccEMA_alpha)*pAccEMA_S_old[2];

	return angAcc;
}

double SceneObject::predictArea()
{
	if( pHistory.size()==0 ) return 0;
	else if( pHistory.size()==1 || pTimeSincePredictionReset==0 ) return pHistory.back()->area;

	// velocity based prediction
	else if( pHistory.size()==2 || pTimeSincePredictionReset==1 )
	{
		Ptr<State> last = pHistory[ pHistory.size()-1 ];
		Ptr<State> secondlast = pHistory[ pHistory.size()-2 ];

		double areaChange = (*last).area-(*secondlast).area;

		return (*last).area+areaChange;
	}

	// acceleration based prediction
	Ptr<State> last_1 = pHistory[ pHistory.size()-1 ];
	Ptr<State> last_2 = pHistory[ pHistory.size()-2 ];
	Ptr<State> last_3 = pHistory[ pHistory.size()-3 ];

	double areaCh_1 = (*last_1).area-(*last_2).area;
	double areaCh_2 = (*last_2).area-(*last_3).area;
	double areaAcc = areaCh_1-areaCh_2;

	return 0.5*areaAcc+areaCh_1+(*last_1).area;
}


void SceneObject::draw( Mat& _img, Scalar _color )
{
	drawPath(_img,_color);
	drawDirection( _img,Scalar(38,38,255) );
	drawName( _img );
	return;
}



bool SceneObject::drawLayer( Mat& _img, unsigned int _layerLevel, Scalar _color )
{
	switch( _layerLevel ){
		case 0:
			drawPath( _img,_color );
			return false;
		case 1:
			drawDirection( _img,Scalar(38,38,255) );
			return false;
		case 2:
			drawName( _img );
			return true;
		default:
			return true;
	}
}



void SceneObject::drawPath( Mat& _img, Scalar _color )
{
	if( pHistory.size()<2 ) return;

	/*for( int i=1; i<pHistory.size(); i++ )
	{
		line( _img, Point( pHistory[i-1]->x, pHistory[i-1]->y ), Point( pHistory[i]->x, pHistory[i]->y ), _color, 1 );
	}*/

	if( ObjectHandler::path_length == -2 ) // the passed image is the central path image, draw last made step on it
	{
		unsigned int lastElementId = pHistory.size();
		line( _img, Point( pHistory[lastElementId-2]->x, pHistory[lastElementId-2]->y ), Point( pHistory[lastElementId-1]->x, pHistory[lastElementId-1]->y ), Scalar( 255-_color[0],255-_color[1],255-_color[2] ), 1 );
		return;
	}
	else if( ObjectHandler::path_length == -1 ) // draw last made step into the object's own path image
	{
		if( pPathMat == NULL ) pPathMat = new Mat( _img.size().height, _img.size().width, CV_8UC3, CV_RGB(0,0,0) );
		else if( pFramesSinceLastFade++ == ObjectHandler::path_fadeout_speed )
		{
			*pPathMat -= Mat( pPathMat->size().height, pPathMat->size().width, CV_8UC3, CV_RGB(1,1,1) );
			pFramesSinceLastFade = 0;
		}

		unsigned int lastElementId = pHistory.size();
	
		line( *pPathMat, Point( pHistory[lastElementId-2]->x, pHistory[lastElementId-2]->y ), Point( pHistory[lastElementId-1]->x, pHistory[lastElementId-1]->y ), Scalar( 255-_color[0],255-_color[1],255-_color[2] ), 1 );

		_img -= *pPathMat;
		return;
	}
	else if( ObjectHandler::path_length == 0 || ObjectHandler::path_length == 1 || ObjectHandler::path_length < -2 ) return;

	// a number of steps that are to be drawn is defined
	
	unsigned int pathStep = path_step_length;
	if( pathStep<1 ) pathStep=1;

	int nrOfSteps = ObjectHandler::path_length/pathStep;
	int lastStepOvershoot = pHistory.size() % pathStep;
	
	
	vector<Point> path;

	// startpoint on pathStep scale
	int startPoint = pHistory.size()-nrOfSteps*pathStep; // nrOfSteps*pathStep is not equal the path_length as nrOfSteps as an integer is rounded downwards
	
	if( startPoint<0 ) startPoint = 0;
	else
	{
		path.push_back( Point(pHistory[startPoint]->x,pHistory[startPoint]->y) );
		startPoint += pathStep-lastStepOvershoot;
	}


    for( size_t i=startPoint; i<pHistory.size(); i+=pathStep )
	{
		path.push_back( Point(pHistory[i]->x,pHistory[i]->y) );
		if( i+pathStep >= pHistory.size() ) path.push_back( Point(pHistory[pHistory.size()-1]->x,pHistory[pHistory.size()-1]->y) );
	}
	int size = path.size();
	const cv::Point *pts = (const cv::Point*) Mat(path).data;
	polylines(_img,&pts,&size,1,false,_color,1);
	return;
}


void SceneObject::drawDirection( Mat& _img, Scalar _color )
{
	if( pHistory.size()==0 ) return;

	Ptr<State> current = pHistory.back();
	vector<double> myDirection = direction( current->angle );
	vector<double> myOrthDirection(2);
	myOrthDirection[0]=myDirection[1];
	myOrthDirection[1]=-myDirection[0];

	Point eins( current->x-10*myDirection[0], current->y-10*myDirection[1] );
	Point zwei( current->x+10*myDirection[0], current->y+10*myDirection[1] );

	line( _img, eins, zwei, _color, 2, 8 );
}


void SceneObject::drawName( Mat& _img )
{
	stringstream nameBuilder;

	string className;
	Scalar color;
	
	classProperties( className, color );

	nameBuilder<<"id#"<<pObjectId<<":"<<className;
	string name;
	nameBuilder>>name;


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


double SceneObject::isType( Mat& /*_img*/, Ptr<Point> /*_boundingRect*/, int /*_objectId*/, int /*_classId*/)
{
	return 0;
}


vector< Ptr<SceneObject> >  SceneObject::findObjects( Mat& /*_image*/, Ptr<Point> /*_boundingRectangle*/ )
{
	vector< Ptr<SceneObject> > newVector;
	return newVector;
}


void SceneObject::addNewLikelihood( unsigned int _classId, double _likelihood )
{
	ensureClassLikelihoodSize();

	
	/*std::ofstream file;
	stringstream converter; string id_str; converter<<id(); converter>>id_str;
	string filename = id_str + "_step.txt";
	file.open( filename,std::ios_base::app );

	file<<_likelihood<<" ";

	file.close();*/

	pClassLikelihood[_classId]=_likelihood;

	return;
}


Ptr<SceneObject> SceneObject::recalculateClass( Ptr<SceneObject> _oldPointer )
{
	ensureClassLikelihoodSize();

	double maxLikelihood=0;
	int maxLikelihoodId;
	/*
	std::ofstream file3;
	file3.open( "0classorder.txt",std::ios_base::app );
	for( int i=0;i<(*SceneObject::objectList).size();i++)
	{
		file3<<(*SceneObject::objectList)[i]<<" ";
	}
	file3.close();


	std::ofstream file2;
	stringstream converter; string id_str; converter<<id(); converter>>id_str;
	string filename2 = id_str + "_step.txt";
	file2.open( filename2,std::ios_base::app );
	file2<<" "<<endl;
	file2.close();
	
	std::ofstream file;
	string filename = id_str + ".txt";

	file.open( filename,std::ios_base::app );
	*/
    for( size_t i=0;i<pClassLikelihood.size();i++ )
	{
		//file << pClassLikelihood[i] << " ";
		if( pClassLikelihood[i]>maxLikelihood )
		{
			maxLikelihoodId=i;
			maxLikelihood=pClassLikelihood[i];
		}
	}
	/*file << endl;
	
	file.close();*/

	Ptr<SceneObject> objectPointer;
	
	if( maxLikelihood<0.5 ) // unknown object
	{
		pType = -1;
		objectPointer = new SceneObject(_oldPointer);
	}
	else if( pType==maxLikelihoodId ) // object stays the same
	{
		return _oldPointer;
	}
	else // new object is built
	{
		pType = maxLikelihoodId; // "must" be set before factory call, since for GenericObject class object otherwhise the maxLikelihoodId would have to be calculated again
		SceneObject* actualObj = dynamic_cast<SceneObject*>(this);
		objectPointer = (*factoryList)[pType]( actualObj );
	}
	return objectPointer;
}


int SceneObject::type()
{
	return pType;
}


int SceneObject::getClassId( string _className )
{
    for( size_t i=0; i<(*objectList).size(); i++ )
	{
		if( (*objectList)[i] == _className ) return i;
	}
	return -1;
}


void SceneObject::setupObjectList()
{
    if( objectList==NULL )
    {
        objectList = new vector< std::string >;
        classifierList = new vector< double (*)( Mat&, RectangleRegion&, Mat&, int, int ) >;
        factoryList = new vector< Ptr<SceneObject> (*)( SceneObject* ) >;
        initializerList = new vector< bool (*)(int, ObjectHandler* )>();
        setOptionsFunctionList = new vector< void (*)( GenericMultiLevelMap<string>& _options, int _classId )>();
        getOptionsFunctionList = new vector< void (*)( GenericMultiLevelMap<string>& _options, int _classId )>();
    }
    return;
}


int SceneObject::registerObject( std::string _name, Ptr<SceneObject> (* _factoryFunction)( SceneObject* _proto ), double (*_classTest)( Mat& _img, RectangleRegion& _boundingRect, Mat&, int _id, int _classId ), bool (*_initializerFunction)( int _classId, ObjectHandler* _environment ), void (*_setOptionsFunction)( GenericMultiLevelMap<string>& _options, int _classId ), void (*_getOptionsFunction)( GenericMultiLevelMap<string>& _options, int _classId )  )
{
    setupObjectList();

	(*objectList).push_back( _name );
	(*classifierList).push_back( _classTest );
	(*factoryList).push_back( _factoryFunction );
	(*initializerList).push_back( _initializerFunction );
	(*setOptionsFunctionList).push_back( _setOptionsFunction );
    (*getOptionsFunctionList).push_back( _getOptionsFunction );

	return ( objectList->size()-1 );
}


bool SceneObject::createClassVectors()
{
	if( objectList==NULL )
	{
		objectList = new vector< std::string >();
		classifierList = new vector< double (*)( Mat&, RectangleRegion&, Mat&, int, int ) >();
		factoryList = new vector< Ptr<SceneObject> (*)( SceneObject* ) >();
		initializerList = new vector< bool (*)(int _classId, ObjectHandler* _environment)>();
		setOptionsFunctionList = new vector< void (*)( GenericMultiLevelMap<string>& _options, int _classId )>();
		getOptionsFunctionList = new vector< void (*)( GenericMultiLevelMap<string>& _options, int _classId )>();
	}
	return true;
}


bool SceneObject::setupOptions()
{
	Options::load_options();
	trace_states = (*Options::General)["object_detection"]["trace_states"].as<bool>();
	path_step_length = (*Options::General)["display"]["objects"]["path_step_length"].as<unsigned int>();
	use_old_state_fallback = (*Options::General)["object_detection"]["use_old_state_fallback"].as<bool>();
	max_object_gridpoint_distance = (*Options::General)["object_detection"]["max_object_gridpoint_distance"].as<double>();
	return true;
}




vector<double> SceneObject::direction( double _angle )
{
	vector<double> direction(2);
	direction[0]=cos(_angle); direction[1]=sin(_angle);
	return direction;
}


void SceneObject::resetObjectCount()
{
	objectCount = 0;
}


void SceneObject::ensureClassLikelihoodSize()
{
	while( pClassLikelihood.size()<objectList->size() ) pClassLikelihood.push_back( Average<double>() );
	return;
}

bool SceneObject::trace_states;
unsigned int SceneObject::path_step_length;
unsigned int SceneObject::objectCount = 0;
bool SceneObject::use_old_state_fallback;
double SceneObject::max_object_gridpoint_distance;
double SceneObject::pAccEMA_alpha = 0.08; // most successful: around 0.08
double SceneObject::pVelEMA_alpha = 0.08; // most successful: around 0.08


vector< std::string >* SceneObject::objectList;
vector< double (*)( Mat& _img, RectangleRegion& _boundingRect, Mat& _descriptors, int _id, int _classId ) >* SceneObject::classifierList;
vector< Ptr<SceneObject> (*)(SceneObject* _proto) >* SceneObject::factoryList;
vector< bool (*)(int _classId, ObjectHandler* _environment)>* SceneObject::initializerList;
vector< void (*)( GenericMultiLevelMap<string>& _options, int _classId )>* SceneObject::setOptionsFunctionList; // list with functions to set class properties
vector< void (*)( GenericMultiLevelMap<string>& _options, int _classId )>* SceneObject::getOptionsFunctionList; // list with functions to get the current class properties
bool SceneObject::pClassVectorsAreSetup = createClassVectors();



// SceneObject::State
SceneObject::State::State(){}

SceneObject::State::~State(){}

SceneObject::State::State( double _x, double _y, double /*time*/, double _angle, double _area, int _type ):x(_x),y(_y),angle(_angle),area(_area),type(_type){}

SceneObject::State::State( Point _pos, double /*time*/, double _angle, double _area, int _type ): x(_pos.x),y(_pos.y),angle(_angle),area(_area),type(_type){}


Point2f SceneObject::State::pos()
{
	return Point2f( x,y );
}
