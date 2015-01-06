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

#include "SimpleFreeMovement.h"


SimpleFreeMovement::SimpleFreeMovement( Ptr<GenericObject> _objectPointer ):Dynamics(_objectPointer)
{
}


SimpleFreeMovement::~SimpleFreeMovement(void)
{
}


Ptr<GenericObject::Dynamics> SimpleFreeMovement::sfmCreator( Ptr<GenericObject> _objectPointer )
{
	return new SimpleFreeMovement(_objectPointer);
}


void SimpleFreeMovement::setOptions( GenericMultiLevelMap<string>& /*_dynamicsOptions*/ )
{
	// the class has currently no options that could be set
	return;
}


void SimpleFreeMovement::getOptions( GenericMultiLevelMap<string>& /*_optionLink*/ )
{
	// empty since the class has no options
	return;
}


string SimpleFreeMovement::info()
{
	return "Unfiltered, free movement dynamics, 3 independent degrees of freedom (position x and y, angle). The prediction is based on a constant acceleration assumption. Tends to overshoot.";
}


void SimpleFreeMovement::predictROI( vector<Point>& _roi )
{
	_roi.push_back( predictPosition( pROI()[0] ) );
	_roi.push_back( predictPosition( pROI()[1] ) );
	_roi.push_back( predictPosition( pROI()[2] ) );
	_roi.push_back( predictPosition( pROI()[3] ) );
	return;
}

Mat SimpleFreeMovement::predictState()
{
	if( pHistory().size()==0 ) return Mat();
	else if( pHistory().size()==1 || pTimeSincePredictionReset()==0 )
	{
		Mat prediction;
		prediction.create(1,3,CV_32FC1);
		prediction.at<float>(0)=(float)pHistory().back()->x;
		prediction.at<float>(1)=(float)pHistory().back()->y;
		prediction.at<float>(2)=(float)pHistory().back()->angle;
		return prediction;
	}

	// velocity based prediction
	else if( pHistory().size()==2 || pTimeSincePredictionReset()==1 )
	{
		Ptr<SceneObject::State> last = pHistory()[ pHistory().size()-1 ];
		Ptr<SceneObject::State> secondlast = pHistory()[ pHistory().size()-2 ];

		double velX = (*last).x-(*secondlast).x;
		double velY = (*last).y-(*secondlast).y;
		double angSpeed = calcAngleVelocity( (*secondlast).angle , (*last).angle );

		Mat prediction;
		prediction.create(1,3,CV_32FC1);
		prediction.at<float>(0) = (float)((*last).x+velX);
		prediction.at<float>(1) = (float)((*last).y+velY);
		prediction.at<float>(2) = (float)((*last).angle+angSpeed);
		return prediction;
	}

	// acceleration based prediction
	Ptr<SceneObject::State> last_1 = pHistory()[ pHistory().size()-1 ];
	Ptr<SceneObject::State> last_2 = pHistory()[ pHistory().size()-2 ];
	Ptr<SceneObject::State> last_3 = pHistory()[ pHistory().size()-3 ];

	double velX_1 = (*last_1).x-(*last_2).x;
	double velX_2 = (*last_2).x-(*last_3).x;
	double accX = velX_1-velX_2;

	double velY_1 = (*last_1).y-(*last_2).y;
	double velY_2 = (*last_2).y-(*last_3).y;
	double accY = velY_1-velY_2;


	double angSp_1 = calcAngleVelocity( (*last_2).angle , (*last_1).angle );
	double angSp_2 = calcAngleVelocity( (*last_3).angle , (*last_2).angle );
	double angAcc = angSp_1-angSp_2;

	Mat prediction;
	prediction.create(1,3,CV_32FC1);
	prediction.at<float>(0) = (float)(0.5*accX+velX_1+(*last_1).x);
	prediction.at<float>(1) = (float)(0.5*accY+velY_1+(*last_1).y);
	prediction.at<float>(2) = (float)(0.5*angAcc+angSp_1+(*last_1).angle);
	return prediction;
}


void SimpleFreeMovement::resetPrediction()
{
	pTimeSincePredictionReset()=0; // if 0: resetPrediction immediately affects all predictions, if -1: resetPrediction affects all predictions after the next SceneObject::State was added
	return;
}


Point SimpleFreeMovement::predictPosition( Point _pt )
{
	double angleVel = predictAglVelocity();

	Point predictedCenter = predictCtrPosition();
	Point center( (int)pHistory().back()->x, (int)pHistory().back()->y );

	Point relPos = _pt-center;

	double cosEl = cos(angleVel);
	double sinEl = sin(angleVel);
	double newRelX = cosEl*relPos.x - sinEl*relPos.y;
	double newRelY = sinEl*relPos.x + cosEl*relPos.y;

	Point newRelPos( (int)newRelX, (int)newRelY );

	return predictedCenter+newRelPos;
}

Point SimpleFreeMovement::predictCtrPosition()
{
	if( pHistory().size()==0 ) return Point(0,0);
	else if( pHistory().size()==1 || pTimeSincePredictionReset()==0 ) return Point( (int)pHistory().back()->x, (int)pHistory().back()->y );

	// velocity based prediction
	else if( pHistory().size()==2 || pTimeSincePredictionReset()==1 )
	{
		Ptr<SceneObject::State> last = pHistory()[ pHistory().size()-1 ];
		Ptr<SceneObject::State> secondlast = pHistory()[ pHistory().size()-2 ];

		double velX = (*last).x-(*secondlast).x;
		double velY = (*last).y-(*secondlast).y;

		return Point( (int)((*last).x+velX), (int)((*last).y+velY) );
	}

	// acceleration based prediction
	Ptr<SceneObject::State> last_1 = pHistory()[ pHistory().size()-1 ];
	Ptr<SceneObject::State> last_2 = pHistory()[ pHistory().size()-2 ];
	Ptr<SceneObject::State> last_3 = pHistory()[ pHistory().size()-3 ];

	double velX_1 = (*last_1).x-(*last_2).x;
	double velX_2 = (*last_2).x-(*last_3).x;
	double accX = velX_1-velX_2;

	double velY_1 = (*last_1).y-(*last_2).y;
	double velY_2 = (*last_2).y-(*last_3).y;
	double accY = velY_1-velY_2;

	return Point( (int)(0.5*accX+velX_1+(*last_1).x), (int)(0.5*accY+velY_1+(*last_1).y) );
}

Point2f SimpleFreeMovement::predictCtrFPosition()
{
	if( pHistory().size()==0 ) return Point(0,0);
	else if( pHistory().size()==1 || pTimeSincePredictionReset()==0 ) return Point( (int)pHistory().back()->x, (int)pHistory().back()->y );

	// velocity based prediction
	else if( pHistory().size()==2 || pTimeSincePredictionReset()==1 )
	{
		Ptr<SceneObject::State> last = pHistory()[ pHistory().size()-1 ];
		Ptr<SceneObject::State> secondlast = pHistory()[ pHistory().size()-2 ];

		double velX = (*last).x-(*secondlast).x;
		double velY = (*last).y-(*secondlast).y;

		return Point( (int)((*last).x+velX), (int)((*last).y+velY) );
	}

	// acceleration based prediction
	Ptr<SceneObject::State> last_1 = pHistory()[ pHistory().size()-1 ];
	Ptr<SceneObject::State> last_2 = pHistory()[ pHistory().size()-2 ];
	Ptr<SceneObject::State> last_3 = pHistory()[ pHistory().size()-3 ];

	double velX_1 = (*last_1).x-(*last_2).x;
	double velX_2 = (*last_2).x-(*last_3).x;
	double accX = velX_1-velX_2;

	double velY_1 = (*last_1).y-(*last_2).y;
	double velY_2 = (*last_2).y-(*last_3).y;
	double accY = velY_1-velY_2;

	return Point2f( (0.5*accX+velX_1+(*last_1).x), (0.5*accY+velY_1+(*last_1).y) );
}

double SimpleFreeMovement::predictXCtrVelocity()
{
	if( pHistory().size()==0 ) return 0;
	else if( pHistory().size()==1 || pTimeSincePredictionReset()==0 ) return 0;

	// velocity based prediction
	else if( pHistory().size()==2 || pTimeSincePredictionReset()==1 )
	{
		Ptr<SceneObject::State> last = pHistory()[ pHistory().size()-1 ];
		Ptr<SceneObject::State> secondlast = pHistory()[ pHistory().size()-2 ];
		
		double velX =(*last).x-(*secondlast).x;

		return velX;
	}

	// acceleration based prediction
	Ptr<SceneObject::State> last_1 = pHistory()[ pHistory().size()-1 ];
	Ptr<SceneObject::State> last_2 = pHistory()[ pHistory().size()-2 ];
	Ptr<SceneObject::State> last_3 = pHistory()[ pHistory().size()-3 ];

	double velX_1 = (*last_1).x-(*last_2).x;
	double velX_2 = (*last_2).x-(*last_3).x;
	double accX = velX_1-velX_2;

	return accX+velX_1;
}

double SimpleFreeMovement::predictXCtrAcceleration()
{
	if( pHistory().size()==0 ) return 0;
	else if( pHistory().size()==1 || pTimeSincePredictionReset()==0 ) return 0;
	else if( pHistory().size()==2 || pTimeSincePredictionReset()==1 ) return 0;

	// acceleration based prediction
	Ptr<SceneObject::State> last_1 = pHistory()[ pHistory().size()-1 ];
	Ptr<SceneObject::State> last_2 = pHistory()[ pHistory().size()-2 ];
	Ptr<SceneObject::State> last_3 = pHistory()[ pHistory().size()-3 ];

	double velX_1 = (*last_1).x-(*last_2).x;
	double velX_2 = (*last_2).x-(*last_3).x;
	double accX = velX_1-velX_2;

	return accX;
}

double SimpleFreeMovement::predictYCtrVelocity()
{
	if( pHistory().size()==0 ) return 0;
	else if( pHistory().size()==1 || pTimeSincePredictionReset()==0 ) return 0;

	// velocity based prediction
	else if( pHistory().size()==2 || pTimeSincePredictionReset()==1 )
	{
		Ptr<SceneObject::State> last = pHistory()[ pHistory().size()-1 ];
		Ptr<SceneObject::State> secondlast = pHistory()[ pHistory().size()-2 ];

		double velY_1 = (*last).y-(*secondlast).y;

		return velY_1;
	}

	// acceleration based prediction
	Ptr<SceneObject::State> last_1 = pHistory()[ pHistory().size()-1 ];
	Ptr<SceneObject::State> last_2 = pHistory()[ pHistory().size()-2 ];
	Ptr<SceneObject::State> last_3 = pHistory()[ pHistory().size()-3 ];

	double velY_1 = (*last_1).y-(*last_2).y;
	double velY_2 = (*last_2).y-(*last_3).y;
	double accY = velY_1-velY_2;

	return accY+velY_1;
}

double SimpleFreeMovement::predictYCtrAcceleration()
{
	if( pHistory().size()==0 ) return 0;
	else if( pHistory().size()==1 || pTimeSincePredictionReset()==0 ) return 0;
	else if( pHistory().size()==2 || pTimeSincePredictionReset()==1 ) return 0;

	// acceleration based prediction
	Ptr<SceneObject::State> last_1 = pHistory()[ pHistory().size()-1 ];
	Ptr<SceneObject::State> last_2 = pHistory()[ pHistory().size()-2 ];
	Ptr<SceneObject::State> last_3 = pHistory()[ pHistory().size()-3 ];

	double velY_1 = (*last_1).y-(*last_2).y;
	double velY_2 = (*last_2).y-(*last_3).y;
	double accY = velY_1-velY_2;

	return accY;
}

double SimpleFreeMovement::predictAgl()
{
	if( pHistory().size()==0 ) return 0;
	else if( pHistory().size()==1 || pTimeSincePredictionReset()==0 ) return pHistory().back()->angle;

	// velocity based prediction
	else if( pHistory().size()==2 || pTimeSincePredictionReset()==1 )
	{
		Ptr<SceneObject::State> last = pHistory()[ pHistory().size()-1 ];
		Ptr<SceneObject::State> secondlast = pHistory()[ pHistory().size()-2 ];

		double velA = calcAngleVelocity( (*secondlast).angle , (*last).angle );

		return (*last).angle + velA;
	}

	// acceleration based prediction
	Ptr<SceneObject::State> last_1 = pHistory()[ pHistory().size()-1 ];
	Ptr<SceneObject::State> last_2 = pHistory()[ pHistory().size()-2 ];
	Ptr<SceneObject::State> last_3 = pHistory()[ pHistory().size()-3 ];

	double velA_1 = calcAngleVelocity( (*last_2).angle , (*last_1).angle );
	double velA_2 = calcAngleVelocity( (*last_3).angle , (*last_2).angle );
	double accA = velA_1-velA_2;

	return (*last_1).angle+velA_1+0.5*accA;
}

double SimpleFreeMovement::predictAglVelocity()
{
	if( pHistory().size()==0 ) return 0;
	else if( pHistory().size()==1 || pTimeSincePredictionReset()==0 ) return 0;

	// velocity based prediction
	else if( pHistory().size()==2 || pTimeSincePredictionReset()==1 )
	{
		Ptr<SceneObject::State> last = pHistory()[ pHistory().size()-1 ];
		Ptr<SceneObject::State> secondlast = pHistory()[ pHistory().size()-2 ];
		double angSp_1 = calcAngleVelocity( (*secondlast).angle , (*last).angle );

		return angSp_1;
	}

	// acceleration based prediction
	Ptr<SceneObject::State> last_1 = pHistory()[ pHistory().size()-1 ];
	Ptr<SceneObject::State> last_2 = pHistory()[ pHistory().size()-2 ];
	Ptr<SceneObject::State> last_3 = pHistory()[ pHistory().size()-3 ];
	
	double angSp_1 = calcAngleVelocity( (*last_2).angle , (*last_1).angle );
	double angSp_2 = calcAngleVelocity( (*last_3).angle , (*last_2).angle );
	double angAcc = angSp_1-angSp_2;

	return angAcc+angSp_1;
}

double SimpleFreeMovement::predictAglAcceleration()
{
	if( pHistory().size()==0 ) return 0;
	else if( pHistory().size()==1 || pTimeSincePredictionReset()==0 ) return 0;
	else if( pHistory().size()==2 || pTimeSincePredictionReset()==1 ) return 0;

	// acceleration based prediction
	Ptr<SceneObject::State> last_1 = pHistory()[ pHistory().size()-1 ];
	Ptr<SceneObject::State> last_2 = pHistory()[ pHistory().size()-2 ];
	Ptr<SceneObject::State> last_3 = pHistory()[ pHistory().size()-3 ];

	double angleVel_1 = calcAngleVelocity( (*last_2).angle , (*last_1).angle );
	double angleVel_2 = calcAngleVelocity( (*last_3).angle , (*last_2).angle );
	double angAcc = angleVel_1-angleVel_2;

	return angAcc;
}

double SimpleFreeMovement::predictArea()
{
	if( pHistory().size()==0 ) return 0;
	else if( pHistory().size()==1 || pTimeSincePredictionReset()==0 ) return pHistory().back()->area;

	// velocity based prediction
	else if( pHistory().size()==2 || pTimeSincePredictionReset()==1 )
	{
		Ptr<SceneObject::State> last = pHistory()[ pHistory().size()-1 ];
		Ptr<SceneObject::State> secondlast = pHistory()[ pHistory().size()-2 ];

		double areaChange = (*last).area-(*secondlast).area;

		return (*last).area+areaChange;
	}

	// acceleration based prediction
	Ptr<SceneObject::State> last_1 = pHistory()[ pHistory().size()-1 ];
	Ptr<SceneObject::State> last_2 = pHistory()[ pHistory().size()-2 ];
	Ptr<SceneObject::State> last_3 = pHistory()[ pHistory().size()-3 ];

	double areaCh_1 = (*last_1).area-(*last_2).area;
	double areaCh_2 = (*last_2).area-(*last_3).area;
	double areaAcc = areaCh_1-areaCh_2;

	return 0.5*areaAcc+areaCh_1+(*last_1).area;
}




int SimpleFreeMovement::registerFactoryFunction()
{
	string info = "Unfiltered, free movement dynamics, 3 independent degrees of freedom (position x and y, angle). The prediction is based on a constant acceleration assumption. Tends to overshoot.";
	return Dynamics::registerDynamics( DynamicsFacEntry( "SimpleFreeMovement", &sfmCreator, info ) );
}



int SimpleFreeMovement::classId = registerFactoryFunction();
