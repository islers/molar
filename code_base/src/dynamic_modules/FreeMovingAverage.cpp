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

#include "FreeMovingAverage.h"


FreeMovingAverage::FreeMovingAverage( Ptr<GenericObject> _objectPointer ):SimpleFreeMovement(_objectPointer)
{
	// standard alpha values
	pPosAlpha=1; // no filtering for position
	pAngAlpha=1;
	pVelAlpha=0.08;
	pVelAngAlpha=0.08;
	pAccAlpha=0.08;
	pAccAngAlpha=0.08;
}


FreeMovingAverage::~FreeMovingAverage(void)
{
}


Ptr<GenericObject::Dynamics> FreeMovingAverage::fmaCreator( Ptr<GenericObject> _objectPointer )
{
	return new FreeMovingAverage(_objectPointer);
}


void FreeMovingAverage::setOptions( GenericMultiLevelMap<string>& _dynamicsOptions )
{
	if( _dynamicsOptions.hasKey("position_alpha") ) pPosAlpha=_dynamicsOptions["position_alpha"].as<double>();
	if( _dynamicsOptions.hasKey("angle_alpha") ) pAngAlpha=_dynamicsOptions["angle_alpha"].as<double>();
	if( _dynamicsOptions.hasKey("velocity_alpha") ) pVelAlpha=_dynamicsOptions["velocity_alpha"].as<double>();
	if( _dynamicsOptions.hasKey("angle_velocity_alpha") ) pVelAngAlpha=_dynamicsOptions["angle_velocity_alpha"].as<double>();
	if( _dynamicsOptions.hasKey("acceleration_alpha") ) pAccAlpha=_dynamicsOptions["acceleration_alpha"].as<double>();
	if( _dynamicsOptions.hasKey("angle_acceleration_alpha") ) pAccAngAlpha=_dynamicsOptions["angle_acceleration_alpha"].as<double>();
	
	return;
}


void FreeMovingAverage::getOptions( GenericMultiLevelMap<string>& _optionLink )
{
	_optionLink["position_alpha"].as<double>()=pPosAlpha;
	_optionLink["angle_alpha"].as<double>()=pAngAlpha;
	_optionLink["velocity_alpha"].as<double>()=pVelAlpha;
	_optionLink["angle_velocity_alpha"].as<double>()=pVelAngAlpha;
	_optionLink["acceleration_alpha"].as<double>()=pAccAlpha;
	_optionLink["angle_acceleration_alpha"].as<double>()=pAccAngAlpha;

	return;
}



void FreeMovingAverage::setStandardOptions( GenericMultiLevelMap<string>& _optLink )
{
	_optLink["position_alpha"].as<double>()=1;
	_optLink["angle_alpha"].as<double>()=1;
	_optLink["velocity_alpha"].as<double>()=0.08;
	_optLink["angle_velocity_alpha"].as<double>()=0.08;
	_optLink["acceleration_alpha"].as<double>()=0.08;
	_optLink["angle_acceleration_alpha"].as<double>()=0.08;

	return;
}



string FreeMovingAverage::info()
{
	return "Free movement dynamics, 3 independent degrees of freedom (position x and y, angle). Applies multiple exponential moving average filtering, possible with different alpha values for position, velocity and acceleration. Those values are used for the prediction as well.";
}


Ptr<SceneObject::State> FreeMovingAverage::newState( Mat& _state, RectangleRegion& /*_region*/, vector<Point>& /*_contour*/, Mat& /*_invImg*/, double _time )
{
	State* newState = new State();
	
	newState->raw_x = _state.at<float>(0);
	newState->raw_y = _state.at<float>(1);
	newState->raw_angle = SceneObject::moveAngleIntoRange( _state.at<float>(2) );
	newState->time = _time;
	newState->type = pObjectPointer->type();
	

	if( pHistory().size()==0 )
	{
		newState->x = newState->raw_x;
		newState->y = newState->raw_y;
		newState->angle = newState->raw_angle;
		newState->vel_x = 0;
		newState->vel_y = 0;
		newState->vel_angle = 0;
		newState->acc_x = 0;
		newState->acc_y = 0;
		newState->acc_angle = 0;
	}
	else if( pHistory().size()==1 )
	{
		Ptr<SceneObject::State> lastState = pHistory().back();
		newState->x = pPosAlpha*newState->raw_x + (1-pPosAlpha)*lastState->x;
		newState->y = pPosAlpha*newState->raw_y + (1-pPosAlpha)*lastState->y;
		newState->angle = pAngAlpha*newState->raw_angle + (1-pAngAlpha)*lastState->angle;
		newState->vel_x = newState->raw_x - lastState->x;
		newState->vel_y = newState->raw_y - lastState->y;
		newState->vel_angle = SceneObject::calcAngleVelocity(lastState->angle, newState->raw_angle);
		newState->acc_x = 0;
		newState->acc_y = 0;
		newState->acc_angle = 0;
	}
	else
	{
		Ptr<SceneObject::State> lastState = pHistory()[ pHistory().size()-1 ];
		Ptr<SceneObject::State> secondLastState = pHistory()[ pHistory().size()-2 ];

		newState->x = pPosAlpha*newState->raw_x + (1-pPosAlpha)*lastState->x;
		newState->y = pPosAlpha*newState->raw_y + (1-pPosAlpha)*lastState->y;
		newState->angle = pAngAlpha*newState->raw_angle + (1-pAngAlpha)*lastState->angle;
		
		if( lastState->type == newState->type )
		{
			Ptr<State> lastConv(lastState);

			double rawLastVelX, rawLastVelY, rawLastVelAngle;
			rawLastVelX = newState->raw_x - lastConv->raw_x;
			rawLastVelY = newState->raw_y - lastConv->raw_y;
			rawLastVelAngle = SceneObject::calcAngleVelocity(lastConv->raw_angle, newState->raw_angle);

			newState->vel_x = pVelAlpha*rawLastVelX + (1-pVelAlpha)*lastConv->vel_x;
			newState->vel_y = pVelAlpha*rawLastVelY + (1-pVelAlpha)*lastConv->vel_y;
			newState->vel_angle = pVelAngAlpha*rawLastVelAngle + (1-pVelAngAlpha)*lastConv->vel_angle;

			if( secondLastState->type == newState->type )
			{
				Ptr<State> secondLastConv(secondLastState);
				
				newState->acc_x = pAccAlpha*( rawLastVelX - (lastConv->raw_x - secondLastConv->raw_x) ) + (1-pAccAlpha)*lastConv->acc_x;
				newState->acc_y = pAccAlpha*( rawLastVelY - (lastConv->raw_y - secondLastConv->raw_y) ) + (1-pAccAlpha)*lastConv->acc_y;
				newState->acc_angle = pAccAngAlpha*( rawLastVelAngle - SceneObject::calcAngleVelocity(secondLastConv->raw_angle,lastConv->raw_angle) ) + (1-pAccAngAlpha)*lastConv->acc_angle;
			}
			else
			{
				newState->acc_x = pAccAlpha*( rawLastVelX - (lastConv->raw_x - secondLastState->x) ) + (1-pAccAlpha)*lastConv->acc_x;
				newState->acc_y = pAccAlpha*( rawLastVelY - (lastConv->raw_y - secondLastState->y) ) + (1-pAccAlpha)*lastConv->acc_y;
				newState->acc_angle = pAccAngAlpha*( rawLastVelAngle - SceneObject::calcAngleVelocity(secondLastState->angle,lastConv->raw_angle) ) + (1-pAccAngAlpha)*lastConv->acc_angle;
			}
		}
		else
		{
			newState->vel_x = newState->raw_x - lastState->x;
			newState->vel_y = newState->raw_y - lastState->y;
			newState->vel_angle = SceneObject::calcAngleVelocity(lastState->angle,newState->raw_angle);
		
			if( secondLastState->type == newState->type )
			{
				Ptr<State> secondLastConv(secondLastState);
				
				newState->acc_x = (newState->raw_x - lastState->x) - (lastState->x - secondLastConv->raw_x);
				newState->acc_y = (newState->raw_y - lastState->y) - (lastState->y - secondLastConv->raw_y);
				newState->acc_angle = newState->vel_angle - (lastState->angle - secondLastConv->raw_angle);			
			}
			else
			{
				newState->acc_x = (newState->raw_x - lastState->x) - (lastState->x - secondLastState->x);
				newState->acc_y = (newState->raw_y - lastState->y) - (lastState->y - secondLastState->y);
				newState->acc_angle = newState->vel_angle - SceneObject::calcAngleVelocity(secondLastState->angle,lastState->angle);
			}
		}
	}

	return Ptr<State>(newState);
}


bool FreeMovingAverage::exportData( string _filePath, string _dataPointSeparator, string _timeStepSeparator )
{
	try
	{
		std::ofstream file;
        file.open( _filePath.c_str(),std::ios_base::trunc ); // old file is overwritten

		file<<"time"<<_dataPointSeparator<<"x_pos"<<_dataPointSeparator<<"y_pos"<<_dataPointSeparator<<"angle"<<_dataPointSeparator<<"x_pos_raw"<<_dataPointSeparator<<"y_pos_raw"<<_dataPointSeparator<<"angle_raw"<<_timeStepSeparator;

        for( size_t i=0;i<pHistory().size(); i++ )
		{
			file<<pHistory()[i]->time<<_dataPointSeparator<<pHistory()[i]->x<<_dataPointSeparator<<pHistory()[i]->y<<_dataPointSeparator<<pHistory()[i]->angle;
			if( pHistory()[i]->type == pObjectPointer->type() ) file << Ptr<State>( pHistory()[i] )->raw_x << _dataPointSeparator << Ptr<State>( pHistory()[i] )->raw_y << _dataPointSeparator << Ptr<State>( pHistory()[i] )->raw_angle;
			file<<_timeStepSeparator;
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

Mat FreeMovingAverage::predictState()
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

		double velX, velY, angSpeed;
		if( last->type==pObjectPointer->type() )
		{
			Ptr<State> convLast(last);
			velX = convLast->vel_x;
			velY = convLast->vel_y;
			angSpeed = convLast->vel_angle;
		}
		else
		{
			velX = (*last).x-(*secondlast).x;
			velY = (*last).y-(*secondlast).y;
			angSpeed = calcAngleVelocity( (*secondlast).angle , (*last).angle );
		}

		Mat prediction;
		prediction.create(1,3,CV_32FC1);
		prediction.at<float>(0) = (float)((*last).x+velX);
		prediction.at<float>(1) = (float)((*last).y+velY);
		prediction.at<float>(2) = SceneObject::moveAngleIntoRange( (float)((*last).angle+angSpeed) );
		return prediction;
	}

	// acceleration based prediction
	Ptr<SceneObject::State> last_1 = pHistory()[ pHistory().size()-1 ];

	double velX_1, velY_1, angSp_1, accX, accY, angAcc;

	if( last_1->type == pObjectPointer->type() )
	{
		Ptr<State> convLast(last_1);

		velX_1 = convLast->vel_x;
		velY_1 = convLast->vel_y;
		angSp_1 = convLast->vel_angle;

		accX = convLast->acc_x;
		accY = convLast->acc_y;
		angAcc = convLast->acc_angle;
	}
	else
	{
		Ptr<SceneObject::State> last_2 = pHistory()[ pHistory().size()-2 ];
		Ptr<SceneObject::State> last_3 = pHistory()[ pHistory().size()-3 ];

		velX_1 = (*last_1).x-(*last_2).x;
		double velX_2 = (*last_2).x-(*last_3).x;
		accX = velX_1-velX_2;

		velY_1 = (*last_1).y-(*last_2).y;
		double velY_2 = (*last_2).y-(*last_3).y;
		accY = velY_1-velY_2;
	
		angSp_1 = calcAngleVelocity( (*last_2).angle , (*last_1).angle );
		double angSp_2 = calcAngleVelocity( (*last_3).angle , (*last_2).angle );
		angAcc = angSp_1-angSp_2;
	}

	Mat prediction;
	prediction.create(1,3,CV_32FC1);
	prediction.at<float>(0) = (float)(0.5*accX+velX_1+(*last_1).x);
	prediction.at<float>(1) = (float)(0.5*accY+velY_1+(*last_1).y);
	prediction.at<float>(2) = SceneObject::moveAngleIntoRange( (float)(0.5*angAcc+angSp_1+(*last_1).angle) );
	return prediction;
}

Point FreeMovingAverage::predictCtrPosition()
{
	if( pHistory().size()==0 ) return Point(0,0);
	else if( pHistory().size()==1 || pTimeSincePredictionReset()==0 ) return Point( (int)pHistory().back()->x, (int)pHistory().back()->y );

	// velocity based prediction
	else if( pHistory().size()==2 || pTimeSincePredictionReset()==1 )
	{
		Ptr<SceneObject::State> last = pHistory()[ pHistory().size()-1 ];
		Ptr<SceneObject::State> secondlast = pHistory()[ pHistory().size()-2 ];

        double velX, velY;
		if( last->type==pObjectPointer->type() )
		{
			Ptr<State> convLast(last);
			velX = convLast->vel_x;
			velY = convLast->vel_y;
		}
		else
		{
			velX = (*last).x-(*secondlast).x;
			velY = (*last).y-(*secondlast).y;
		}

		return Point( (int)((*last).x+velX), (int)((*last).y+velY) );
	}

	// acceleration based prediction
	Ptr<SceneObject::State> last_1 = pHistory()[ pHistory().size()-1 ];

	double velX_1, velY_1, accX, accY;

	if( last_1->type == pObjectPointer->type() )
	{
		Ptr<State> convLast(last_1);

		velX_1 = convLast->vel_x;
		velY_1 = convLast->vel_y;

		accX = convLast->acc_x;
		accY = convLast->acc_y;
	}
	else
	{
		Ptr<SceneObject::State> last_2 = pHistory()[ pHistory().size()-2 ];
		Ptr<SceneObject::State> last_3 = pHistory()[ pHistory().size()-3 ];

		velX_1 = (*last_1).x-(*last_2).x;
		double velX_2 = (*last_2).x-(*last_3).x;
		accX = velX_1-velX_2;

		velY_1 = (*last_1).y-(*last_2).y;
		double velY_2 = (*last_2).y-(*last_3).y;
		accY = velY_1-velY_2;
	}

	return Point( (int)(0.5*accX+velX_1+(*last_1).x), (int)(0.5*accY+velY_1+(*last_1).y) );
}

Point2f FreeMovingAverage::predictCtrFPosition()
{
	if( pHistory().size()==0 ) return Point(0,0);
	else if( pHistory().size()==1 || pTimeSincePredictionReset()==0 ) return Point( (int)pHistory().back()->x, (int)pHistory().back()->y );

	// velocity based prediction
	else if( pHistory().size()==2 || pTimeSincePredictionReset()==1 )
	{
		Ptr<SceneObject::State> last = pHistory()[ pHistory().size()-1 ];
		Ptr<SceneObject::State> secondlast = pHistory()[ pHistory().size()-2 ];

        double velX, velY;
		if( last->type==pObjectPointer->type() )
		{
			Ptr<State> convLast(last);
			velX = convLast->vel_x;
			velY = convLast->vel_y;
		}
		else
		{
			velX = (*last).x-(*secondlast).x;
			velY = (*last).y-(*secondlast).y;
		}

		return Point( (int)((*last).x+velX), (int)((*last).y+velY) );
	}

	// acceleration based prediction
	Ptr<SceneObject::State> last_1 = pHistory()[ pHistory().size()-1 ];

	double velX_1, velY_1, accX, accY;

	if( last_1->type == pObjectPointer->type() )
	{
		Ptr<State> convLast(last_1);

		velX_1 = convLast->vel_x;
		velY_1 = convLast->vel_y;

		accX = convLast->acc_x;
		accY = convLast->acc_y;
	}
	else
	{
		Ptr<SceneObject::State> last_2 = pHistory()[ pHistory().size()-2 ];
		Ptr<SceneObject::State> last_3 = pHistory()[ pHistory().size()-3 ];

		velX_1 = (*last_1).x-(*last_2).x;
		double velX_2 = (*last_2).x-(*last_3).x;
		accX = velX_1-velX_2;

		velY_1 = (*last_1).y-(*last_2).y;
		double velY_2 = (*last_2).y-(*last_3).y;
		accY = velY_1-velY_2;
	}

	return Point2f( (0.5*accX+velX_1+(*last_1).x), (0.5*accY+velY_1+(*last_1).y) );
}

double FreeMovingAverage::predictXCtrVelocity()
{
	if( pHistory().size()==0 ) return 0;
	else if( pHistory().size()==1 || pTimeSincePredictionReset()==0 ) return 0;

	// velocity based prediction
	else if( pHistory().size()==2 || pTimeSincePredictionReset()==1 )
	{
		Ptr<SceneObject::State> last = pHistory()[ pHistory().size()-1 ];
		Ptr<SceneObject::State> secondlast = pHistory()[ pHistory().size()-2 ];

		double velX;
		if( last->type==pObjectPointer->type() )
		{
			Ptr<State> convLast(last);
			velX = convLast->vel_x;
		}
		else
		{
			velX = (*last).x-(*secondlast).x;
		}

		return velX;
	}

	// acceleration based prediction
	Ptr<SceneObject::State> last_1 = pHistory()[ pHistory().size()-1 ];

	double velX_1, accX;

	if( last_1->type == pObjectPointer->type() )
	{
		Ptr<State> convLast(last_1);

		velX_1 = convLast->vel_x;

		accX = convLast->acc_x;
	}
	else
	{
		Ptr<SceneObject::State> last_2 = pHistory()[ pHistory().size()-2 ];
		Ptr<SceneObject::State> last_3 = pHistory()[ pHistory().size()-3 ];

		velX_1 = (*last_1).x-(*last_2).x;
		double velX_2 = (*last_2).x-(*last_3).x;
		accX = velX_1-velX_2;
	}

	return accX+velX_1;
}

double FreeMovingAverage::predictXCtrAcceleration()
{
	if( pHistory().size()==0 ) return 0;
	else if( pHistory().size()==1 || pTimeSincePredictionReset()==0 ) return 0;
	else if( pHistory().size()==2 || pTimeSincePredictionReset()==1 ) return 0;

	// acceleration based prediction
	Ptr<SceneObject::State> last_1 = pHistory()[ pHistory().size()-1 ];

	double velX_1, accX;

	if( last_1->type == pObjectPointer->type() )
	{
		Ptr<State> convLast(last_1);

		accX = convLast->acc_x;
	}
	else
	{
		Ptr<SceneObject::State> last_2 = pHistory()[ pHistory().size()-2 ];
		Ptr<SceneObject::State> last_3 = pHistory()[ pHistory().size()-3 ];

		velX_1 = (*last_1).x-(*last_2).x;
		double velX_2 = (*last_2).x-(*last_3).x;
		accX = velX_1-velX_2;
	}

	return accX;
}

double FreeMovingAverage::predictYCtrVelocity()
{
	if( pHistory().size()==0 ) return 0;
	else if( pHistory().size()==1 || pTimeSincePredictionReset()==0 ) return 0;

	// velocity based prediction
	else if( pHistory().size()==2 || pTimeSincePredictionReset()==1 )
	{
		Ptr<SceneObject::State> last = pHistory()[ pHistory().size()-1 ];
		Ptr<SceneObject::State> secondlast = pHistory()[ pHistory().size()-2 ];

		double velY;
		if( last->type==pObjectPointer->type() )
		{
			Ptr<State> convLast(last);
			velY = convLast->vel_y;
		}
		else
		{
			velY = (*last).y-(*secondlast).y;
		}

		return velY;
	}

	// acceleration based prediction
	Ptr<SceneObject::State> last_1 = pHistory()[ pHistory().size()-1 ];

	double velY_1, accY;

	if( last_1->type == pObjectPointer->type() )
	{
		Ptr<State> convLast(last_1);

		velY_1 = convLast->vel_y;
		accY = convLast->acc_y;
	}
	else
	{
		Ptr<SceneObject::State> last_2 = pHistory()[ pHistory().size()-2 ];
		Ptr<SceneObject::State> last_3 = pHistory()[ pHistory().size()-3 ];

		velY_1 = (*last_1).y-(*last_2).y;
		double velY_2 = (*last_2).y-(*last_3).y;
		accY = velY_1-velY_2;
	}

	return accY+velY_1;
}

double FreeMovingAverage::predictYCtrAcceleration()
{
	if( pHistory().size()==0 ) return 0;
	else if( pHistory().size()==1 || pTimeSincePredictionReset()==0 ) return 0;
	else if( pHistory().size()==2 || pTimeSincePredictionReset()==1 ) return 0;

	// acceleration based prediction
	Ptr<SceneObject::State> last_1 = pHistory()[ pHistory().size()-1 ];

	double velY_1, accY;

	if( last_1->type == pObjectPointer->type() )
	{
		Ptr<State> convLast(last_1);

		accY = convLast->acc_y;
	}
	else
	{
		Ptr<SceneObject::State> last_2 = pHistory()[ pHistory().size()-2 ];
		Ptr<SceneObject::State> last_3 = pHistory()[ pHistory().size()-3 ];

		velY_1 = (*last_1).y-(*last_2).y;
		double velY_2 = (*last_2).y-(*last_3).y;
		accY = velY_1-velY_2;
	}

	return accY;
}

double FreeMovingAverage::predictAgl()
{
	if( pHistory().size()==0 ) return 0;
	else if( pHistory().size()==1 || pTimeSincePredictionReset()==0 ) return pHistory().back()->angle;

	// velocity based prediction
	else if( pHistory().size()==2 || pTimeSincePredictionReset()==1 )
	{
		Ptr<SceneObject::State> last = pHistory()[ pHistory().size()-1 ];
		Ptr<SceneObject::State> secondlast = pHistory()[ pHistory().size()-2 ];

		double angSpeed;
		if( last->type==pObjectPointer->type() )
		{
			Ptr<State> convLast(last);
			angSpeed = convLast->vel_angle;
		}
		else
		{
			angSpeed = calcAngleVelocity( (*secondlast).angle , (*last).angle );
		}

		return SceneObject::moveAngleIntoRange( (*last).angle + angSpeed );
	}

	// acceleration based prediction
	Ptr<SceneObject::State> last_1 = pHistory()[ pHistory().size()-1 ];

	double angSp_1, angAcc;

	if( last_1->type == pObjectPointer->type() )
	{
		Ptr<State> convLast(last_1);

		angSp_1 = convLast->vel_angle;
		angAcc = convLast->acc_angle;
	}
	else
	{
		Ptr<SceneObject::State> last_2 = pHistory()[ pHistory().size()-2 ];
		Ptr<SceneObject::State> last_3 = pHistory()[ pHistory().size()-3 ];

		angSp_1 = calcAngleVelocity( (*last_2).angle , (*last_1).angle );
		double angSp_2 = calcAngleVelocity( (*last_3).angle , (*last_2).angle );
		angAcc = angSp_1-angSp_2;
	}

	return SceneObject::moveAngleIntoRange( (*last_1).angle+angSp_1+0.5*angAcc );
}

double FreeMovingAverage::predictAglVelocity()
{
	if( pHistory().size()==0 ) return 0;
	else if( pHistory().size()==1 || pTimeSincePredictionReset()==0 ) return 0;

	// velocity based prediction
	else if( pHistory().size()==2 || pTimeSincePredictionReset()==1 )
	{
		Ptr<SceneObject::State> last = pHistory()[ pHistory().size()-1 ];
		Ptr<SceneObject::State> secondlast = pHistory()[ pHistory().size()-2 ];
		double angSpeed;
		if( last->type==pObjectPointer->type() )
		{
			Ptr<State> convLast(last);
			angSpeed = convLast->vel_angle;
		}
		else
		{
			angSpeed = calcAngleVelocity( (*secondlast).angle , (*last).angle );
		}

		return angSpeed;
	}

	// acceleration based prediction
	Ptr<SceneObject::State> last_1 = pHistory()[ pHistory().size()-1 ];

	double angSp_1, angAcc;

	if( last_1->type == pObjectPointer->type() )
	{
		Ptr<State> convLast(last_1);

		angSp_1 = convLast->vel_angle;
		angAcc = convLast->acc_angle;
	}
	else
	{
		Ptr<SceneObject::State> last_2 = pHistory()[ pHistory().size()-2 ];
		Ptr<SceneObject::State> last_3 = pHistory()[ pHistory().size()-3 ];

		angSp_1 = calcAngleVelocity( (*last_2).angle , (*last_1).angle );
		double angSp_2 = calcAngleVelocity( (*last_3).angle , (*last_2).angle );
		angAcc = angSp_1-angSp_2;
	}

	return angAcc+angSp_1;
}

double FreeMovingAverage::predictAglAcceleration()
{
	if( pHistory().size()==0 ) return 0;
	else if( pHistory().size()==1 || pTimeSincePredictionReset()==0 ) return 0;
	else if( pHistory().size()==2 || pTimeSincePredictionReset()==1 ) return 0;

	// acceleration based prediction
	Ptr<SceneObject::State> last_1 = pHistory()[ pHistory().size()-1 ];

	double angSp_1, angAcc;

	if( last_1->type == pObjectPointer->type() )
	{
		Ptr<State> convLast(last_1);

		angAcc = convLast->acc_angle;
	}
	else
	{
		Ptr<SceneObject::State> last_2 = pHistory()[ pHistory().size()-2 ];
		Ptr<SceneObject::State> last_3 = pHistory()[ pHistory().size()-3 ];

		angSp_1 = calcAngleVelocity( (*last_2).angle , (*last_1).angle );
		double angSp_2 = calcAngleVelocity( (*last_3).angle , (*last_2).angle );
		angAcc = angSp_1-angSp_2;
	}

	return angAcc;
}


int FreeMovingAverage::registerFactoryFunction()
{
	string info = "Free movement dynamics, 3 independent degrees of freedom (position x and y, angle). Applies multiple exponential moving average filtering, possible with different alpha values for position, velocity and acceleration. Those values are used for the prediction as well.";
	GenericMultiLevelMap<string> options;
	setStandardOptions(options);
	int _classId = Dynamics::registerDynamics( DynamicsFacEntry( "FreeMovingAverage", &fmaCreator, info, options ) );
	return _classId;
}



int FreeMovingAverage::classId = FreeMovingAverage::registerFactoryFunction();




FreeMovingAverage::State::State(){}
FreeMovingAverage::State::~State(){}
FreeMovingAverage::State::State( double _x, double _y, double time, double _angle, double _area, int _type ):SceneObject::State( _x, _y, time, _angle, _area, _type ){}
FreeMovingAverage::State::State( Point _pos, double time, double _angle, double _area, int _type ):SceneObject::State( _pos, time, _angle, _area, _type ){}
