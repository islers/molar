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

#include "DirectedRodEMA.h"


DirectedRodEMA::DirectedRodEMA( Ptr<GenericObject> _objectPointer ):FreeMovingAverage(_objectPointer)
{
	// standard alpha values
	pHLOAlpha=1;
	pVelHLOAlpha=0.08;
	pAccHLOAlpha=0.02;

	pPosAlpha=1; // no filtering for position
	pAngAlpha=1;
	pVelAlpha=0.1;
	pVelAngAlpha=0.08;
	pAccAlpha=0.08;
	pAccAngAlpha=0.08;

	pEMAAngleFac=0.3;
	pEMAAngleInit=false;
}


DirectedRodEMA::~DirectedRodEMA(void)
{
}


Ptr<GenericObject::Dynamics> DirectedRodEMA::dremaCreator( Ptr<GenericObject> _objectPointer )
{
	return new DirectedRodEMA(_objectPointer);
}


void DirectedRodEMA::setOptions( GenericMultiLevelMap<string>& _dynamicsOptions )
{
	FreeMovingAverage::setOptions(_dynamicsOptions);

	if( _dynamicsOptions.hasKey("half_length_offset_alpha") ) pHLOAlpha = _dynamicsOptions["half_length_offset_alpha"].as<double>();
	if( _dynamicsOptions.hasKey("half_length_velocity_alpha") ) pVelHLOAlpha = _dynamicsOptions["half_length_velocity_alpha"].as<double>();
	if( _dynamicsOptions.hasKey("half_length_acceleration_alpha") ) pAccHLOAlpha = _dynamicsOptions["half_length_acceleration_alpha"].as<double>();

    if( _dynamicsOptions.hasKey("position_alpha") ) pPosAlpha=_dynamicsOptions["position_alpha"].as<double>();
    if( _dynamicsOptions.hasKey("angle_alpha") ) pAngAlpha=_dynamicsOptions["angle_alpha"].as<double>();
    if( _dynamicsOptions.hasKey("velocity_alpha") ) pVelAlpha=_dynamicsOptions["velocity_alpha"].as<double>();
    if( _dynamicsOptions.hasKey("angle_velocity_alpha") ) pVelAngAlpha=_dynamicsOptions["angle_velocity_alpha"].as<double>();
    if( _dynamicsOptions.hasKey("acceleration_alpha") ) pAccAlpha=_dynamicsOptions["acceleration_alpha"].as<double>();
    if( _dynamicsOptions.hasKey("angle_acceleration_alpha") ) pAccAngAlpha=_dynamicsOptions["angle_acceleration_alpha"].as<double>();

	return;
}


void DirectedRodEMA::getOptions( GenericMultiLevelMap<string>& _optionLink )
{
	_optionLink["half_length_offset_alpha"].as<double>()=pHLOAlpha;
	_optionLink["half_length_velocity_alpha"].as<double>()=pHLOAlpha;
	_optionLink["half_length_acceleration_alpha"].as<double>()=pAccHLOAlpha;

	_optionLink["position alpha"].as<double>()=pPosAlpha;
	_optionLink["angle alpha"].as<double>()=pAngAlpha;
	_optionLink["velocity alpha"].as<double>()=pVelAlpha;
	_optionLink["angle velocity alpha"].as<double>()=pVelAngAlpha;
	_optionLink["acceleration alpha"].as<double>()=pAccAlpha;
	_optionLink["angle acceleration alpha"].as<double>()=pAccAngAlpha;



	return;
}



void DirectedRodEMA::setStandardOptions( GenericMultiLevelMap<string>& _optionLink )
{
	
	_optionLink["half_length_offset_alpha"].as<double>()=1;
	_optionLink["half_length_velocity_alpha"].as<double>()=0.08;
	_optionLink["half_length_acceleration_alpha"].as<double>()=0.02;

	_optionLink["position alpha"].as<double>()=1;
	_optionLink["angle alpha"].as<double>()=1;
	_optionLink["velocity alpha"].as<double>()=0.1;
	_optionLink["angle velocity alpha"].as<double>()=0.08;
	_optionLink["acceleration alpha"].as<double>()=0.08;
	_optionLink["angle acceleration alpha"].as<double>()=0.08;
	return;
}


string DirectedRodEMA::info()
{
	return "Free movement dynamics, 4 independent degrees of freedom (position x and y, angle, half length offset), targeted at rod like objects with a directed movement with the direction being interfered and constantly checked (adjustment of the angle). Applies multiple exponential moving average filtering, possible with different alpha values for position, velocity and acceleration. Those values are used for the prediction as well. The half length offset (length-diameter)/2 is used as a measure to estimate and predict movement orthogonal to the view plane, targeted at tracking the direction through 'in place rotations'.";
}


Ptr<SceneObject::State> DirectedRodEMA::newState( Mat& _state, RectangleRegion& _region, vector<Point>& /*_contour*/, Mat& _invImg, double _time )
{

	State* newState = new State();
	
	newState->raw_x = _state.at<float>(0);
	newState->raw_y = _state.at<float>(1);
	newState->raw_angle = SceneObject::moveAngleIntoRange( _state.at<float>(2) );
	newState->time = _time;
	newState->type = pObjectPointer->type();

	//adjust the angle
	double pi = ObjectHandler::pi;
	double pi_half = ObjectHandler::pi_half;

	double predictedAngle = predictAgl();

	double angleUpwardBoundary = predictedAngle+pi_half;
	double angleLowerBoundary = predictedAngle-pi_half;

	// choose the direction that lies closer to the predicted direction
    if( !( (newState->raw_angle<angleUpwardBoundary && newState->raw_angle>angleLowerBoundary) || (newState->raw_angle<angleUpwardBoundary+ObjectHandler::two_pi && newState->raw_angle>angleLowerBoundary+ObjectHandler::two_pi) || (newState->raw_angle<angleUpwardBoundary-ObjectHandler::two_pi && newState->raw_angle>angleLowerBoundary-ObjectHandler::two_pi) ) )
	{
		if( newState->raw_angle<pi ) newState->raw_angle = newState->raw_angle+pi;
		else newState->raw_angle = newState->raw_angle-pi;
	}


	// calculate half length offset

	if( !_invImg.empty() ) // the inverse image is empty if the state was infered from a group image
	{
		newState->raw_halfLengthOffset = ( _region.width() - pDiameter ) / 2;
	}
	else if( pHistory().size()>=1 ) // for states infered in a group image additional parameters are very unsure. For simplification, it is assumed here that they are constant during this time
	{
		if( pHistory().back()->type==pObjectPointer->type() ) newState->raw_halfLengthOffset = dynamic_cast<State*>( pHistory().back().obj )->halfLengthOffset;
		else  newState->raw_halfLengthOffset = 0;
	}
	else newState->raw_halfLengthOffset = 0;


	if( pHistory().size()==0 )
	{
		newState->x = newState->raw_x;
		newState->y = newState->raw_y;
		newState->halfLengthOffset = newState->raw_halfLengthOffset;
		newState->angle = newState->raw_angle;
		newState->vel_x = 0;
		newState->vel_y = 0;
		newState->vel_hlo = 0;
		newState->vel_angle = 0;
		newState->acc_x = 0;
		newState->acc_y = 0;
		newState->acc_hlo = 0;
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
		newState->acc_hlo = 0;
		
		if( lastState->type==newState->type )
		{
			Ptr<State> lastConv(lastState);
			newState->halfLengthOffset = pHLOAlpha*newState->raw_halfLengthOffset + (1-pHLOAlpha)*lastConv->halfLengthOffset;
			newState->vel_hlo = newState->raw_halfLengthOffset - lastConv->raw_halfLengthOffset;
		}
		else
		{
			newState->halfLengthOffset = newState->raw_halfLengthOffset;
			newState->vel_hlo = 0;
		}
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

			newState->halfLengthOffset = pHLOAlpha*newState->raw_halfLengthOffset + (1-pHLOAlpha)*lastConv->halfLengthOffset;

			double rawLastVelX, rawLastVelY, rawLastVelAngle, rawLastVelHLO;
			rawLastVelX = newState->raw_x - lastConv->raw_x;
			rawLastVelY = newState->raw_y - lastConv->raw_y;
			rawLastVelAngle = SceneObject::calcAngleVelocity(lastConv->raw_angle, newState->raw_angle);
			rawLastVelHLO = newState->raw_halfLengthOffset - lastConv->raw_halfLengthOffset;

			newState->vel_x = pVelAlpha*rawLastVelX + (1-pVelAlpha)*lastConv->vel_x;
			newState->vel_y = pVelAlpha*rawLastVelY + (1-pVelAlpha)*lastConv->vel_y;
			newState->vel_angle = pVelAngAlpha*rawLastVelAngle + (1-pVelAngAlpha)*lastConv->vel_angle;
			newState->vel_hlo = pVelHLOAlpha*rawLastVelHLO + (1-pVelHLOAlpha)*lastConv->vel_hlo;

			if( secondLastState->type == newState->type )
			{
				Ptr<State> secondLastConv(secondLastState);
				
				newState->acc_x = pAccAlpha*( rawLastVelX - (lastConv->raw_x - secondLastConv->raw_x) ) + (1-pAccAlpha)*lastConv->acc_x;
				newState->acc_y = pAccAlpha*( rawLastVelY - (lastConv->raw_y - secondLastConv->raw_y) ) + (1-pAccAlpha)*lastConv->acc_y;
				newState->acc_angle = pAccAngAlpha*( rawLastVelAngle - SceneObject::calcAngleVelocity(secondLastConv->raw_angle,lastConv->raw_angle) ) + (1-pAccAngAlpha)*lastConv->acc_angle;
				newState->acc_hlo = pAccHLOAlpha*( rawLastVelHLO - (lastConv->raw_halfLengthOffset-secondLastConv->raw_halfLengthOffset) ) + (1-pAccHLOAlpha)*lastConv->acc_hlo;
			}
			else
			{
				newState->acc_x = pAccAlpha*( rawLastVelX - (lastConv->raw_x - secondLastState->x) ) + (1-pAccAlpha)*lastConv->acc_x;
				newState->acc_y = pAccAlpha*( rawLastVelY - (lastConv->raw_y - secondLastState->y) ) + (1-pAccAlpha)*lastConv->acc_y;
				newState->acc_angle = pAccAngAlpha*( rawLastVelAngle - SceneObject::calcAngleVelocity(secondLastState->angle,lastConv->raw_angle) ) + (1-pAccAngAlpha)*lastConv->acc_angle;
				newState->acc_hlo = 0;
			}
		}
		else
		{
			newState->vel_x = newState->raw_x - lastState->x;
			newState->vel_y = newState->raw_y - lastState->y;
			newState->vel_angle = SceneObject::calcAngleVelocity(lastState->angle,newState->raw_angle);
			newState->vel_hlo = 0;
			newState->acc_hlo = 0;
		
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


bool DirectedRodEMA::addState( Ptr<SceneObject::State> _newState, RectangleRegion& _regionOfInterest, vector<Point>& _contour )
{
	bool imageBorderTouched = pObjectPointer->touchesImageBorder();

	if( _newState->type != pObjectPointer->type() ) return false; // reject to add the state if it isn't of the appropriate type
	
	DirectedRodEMA::State* statePointer = dynamic_cast<DirectedRodEMA::State*>( _newState.obj );
	_newState.addref(); // prevent the _newState pointer from releasing the underlying data structure which is with the next line handled by newState
	Ptr<DirectedRodEMA::State> newState( statePointer );
	
	//adjust angle range
    //double angle = newState->raw_angle;

	// update infered geometric properties of the abf
	double currentLength = _regionOfInterest.width();
	double lengthWidthRatio = currentLength/pDiameter;

	if( imageBorderTouched ) // freeze geometric properties of the abf if the image border is touched (for obvious reasons...)
	{
		if( pHistory().size()!=0 )
		{
			if( pHistory().back()->type==pObjectPointer->type() ) newState->halfLengthOffset = dynamic_cast<DirectedRodEMA::State*>( pHistory().back().obj )->halfLengthOffset;
			else newState->halfLengthOffset = 0.5*RectangleRegion( pROI()[0],pROI()[1],pROI()[2],pROI()[3] ).width();
		}
	}
	else
	{
		if( lengthWidthRatio < 1.1 ) pUprightDirectionCount++;
		else pUprightDirectionCount = 0;

		pDiameter = _regionOfInterest.height();
		if( currentLength>pLength ) pLength = currentLength;

	}

		
	if( pUprightDirectionCount > 3 )	//test if the abf was too long in an upright position for the direction still to be known. An alternative version would be to consider the direction unknown if the halfLengthOffset prediction is 0 (test needed to see if this is as valuable as it theoretically seems)
	{
		pDirectionConfidenceLevel = 0;
		/*cout<<endl<<"Reset for object "<<id()<<" with length width ratio: "<<lengthWidthRatio;
		cout<<endl<<"Current length: "<<currentLength<<"px, current width: "<<_regionOfInterest.height()<<"px";
		cout<<endl<<"Overall length: "<<pLength<<"px, overall diameter: "<<pDiameter<<"px";*/
		//waitKey(0);
	}
		
	// infer angle and direction
	if( pHistory().size()>=1 )
	{
		bool performDirectionSwitch = false;


		/*// moving average velocity calculation _______________________________
		Point2f newVelocity = newState->pos()-pHistory().back()->pos();
		pLastVelocities.push( newVelocity );
		pMovingAverageVelocity += newVelocity;

		Point2f averageVelocity;
		if( pLastVelocities.size()>velocity_moving_average_width ) // FIFO queue is full
		{
			pMovingAverageVelocity -= pLastVelocities.front();
			pLastVelocities.pop();
			averageVelocity.x = (int)pMovingAverageVelocity.x >> velocity_moving_average_power; // division by shift
			averageVelocity.y = (int)pMovingAverageVelocity.y >> velocity_moving_average_power;
		}
		else
		{
			averageVelocity = 1.0/pLastVelocities.size()*pMovingAverageVelocity;
		}*/

        double pi = ObjectHandler::pi;
        //double pi_half = ObjectHandler::pi_half;

        /*double predictedAngle = predictAgl();//predictAglVelocity()+pHistory.back()->angle;
		while( predictedAngle<0 ) predictedAngle += ObjectHandler::two_pi;
		while( predictedAngle>ObjectHandler::two_pi ) predictedAngle -= ObjectHandler::two_pi;*/

        /*
		double angleUpwardBoundary = predictedAngle+pi_half;
		double angleLowerBoundary = predictedAngle-pi_half;


		// choose the direction that lies closer to the predicted direction
		if( !( angle<angleUpwardBoundary && angle>angleLowerBoundary || angle<angleUpwardBoundary+ObjectHandler::two_pi && angle>angleLowerBoundary+ObjectHandler::two_pi || angle<angleUpwardBoundary-ObjectHandler::two_pi && angle>angleLowerBoundary-ObjectHandler::two_pi ) )
		{
			if( newState->angle<pi ) newState->angle = newState->angle+pi;
			else newState->angle = newState->angle-pi;
		}
		*/

		// moving average angle calculation ____________________________________________________________
		// handle break at 2 pi
		/*double averageAngle = pMovingAverageAngle/pLastAngles.size();
		double angleDiff = abs( averageAngle - newState->angle );
		double biggerAngleOccurance = newState->angle + ObjectHandler::two_pi;
		double smallerAngleOccurance = newState->angle - ObjectHandler::two_pi;

		double adjustedAngle = newState->angle+pAverageAngleAdjustment;

		// adjust angle before adding it to moving average filter:
		if( abs(averageAngle-biggerAngleOccurance)<angleDiff ) // positive step over angle range limit
		{
			double newAngle = adjustedAngle+ObjectHandler::two_pi;
			pLastAngles.push(newAngle);
			pMovingAverageAngle += newAngle;
		}
		else if( abs(averageAngle-smallerAngleOccurance)<angleDiff ) // negative step over angle range limit
		{
			double newAngle = adjustedAngle-ObjectHandler::two_pi;
			pLastAngles.push(newAngle);
			pMovingAverageAngle += newAngle;
		}
		else // regular case
		{
			pLastAngles.push( adjustedAngle );
			pMovingAverageAngle += adjustedAngle;
		}

		if( pLastAngles.size()>angle_moving_average_width ) // FIFO queue is full
		{
			pMovingAverageAngle -= pLastAngles.front();
			pLastAngles.pop();
		}
		
		// check if average angle steps over angle range limit
		if( pMovingAverageAngle >= ObjectHandler::two_pi*pLastAngles.size() ) // steps over angle range limit
		{
			pMovingAverageAngle -= ObjectHandler::two_pi*pLastAngles.size(); // adjust it accordingly, back inside limits
			pAverageAngleAdjustment += ObjectHandler::two_pi; // necessary adjustment of input values if the sum is changed
		}
		else if( pMovingAverageAngle < 0 ) // average steps below angle range limit
		{
			pMovingAverageAngle += ObjectHandler::two_pi*pLastAngles.size();
			pAverageAngleAdjustment -= ObjectHandler::two_pi;
		}
		averageAngle = pMovingAverageAngle/pLastAngles.size();*/
		double averageAngle = pEMAAngle;//newState->angle; // exponential moving average

		// check if the filtered averaged velocities and angle directions are pointing in the same direction, inverse the angle direction if not
		vector<double> angleDirection = direction( averageAngle );
		double scalarVelAngle = newState->vel_x*angleDirection[0]+newState->vel_y*angleDirection[1];
		
		double easyScalarLimit = 0.095; //squared
		double hardScalarLimit = 3;
		double confidence = abs( scalarVelAngle/hardScalarLimit );

		// check for halfLengthOffset switches and adjust those variables if necessary, but only if the object doesn't touch the image border
		if( pHistory().back()->type==pObjectPointer->type() && !imageBorderTouched )
		{
			double predictedHLO = predictHalfLengthOffset();
			double lastHLO = dynamic_cast<DirectedRodEMA::State*>( pHistory().back().obj )->halfLengthOffset;
			int predictedSign = (0 < predictedHLO) - ( predictedHLO < 0 );
			int lastSign = (0 < lastHLO ) - ( lastHLO < 0 );
			if( predictedSign!=lastSign && predictedSign!=0 ) // rotatation through perpendicular axis is assumed
			{
				newState->halfLengthOffset *= -1; // switch halfLengthOffset
				performDirectionSwitch = true; // turn direction
				pDirectionConfidenceLevel *= 0.5;
				/*cout<<endl<<"HLO based direction switch performed for object "<<id()<<" (type "<<type()<<")";
				cout<<endl<<"last HLO: "<<lastHLO<<", predicted HLO: "<<predictedHLO<<", actual HLO: "<<newState->halfLengthOffset;
				cout<<endl<<"second last HLO (state type "<<dynamic_cast<ABFSpiral::State*>( pHistory[pHistory.size()-2].obj )->type<<"): "<<dynamic_cast<ABFSpiral::State*>( pHistory[pHistory.size()-2].obj )->halfLengthOffset;
				*/
			}
		}

		// if velocity and angle directions contradict each other clearly enough or another reason exist for switching the direction
        if( (scalarVelAngle<-easyScalarLimit && pDirectionConfidenceLevel<confidence) || scalarVelAngle<-hardScalarLimit || performDirectionSwitch )
		{
			newState->angle = SceneObject::moveAngleIntoRange( newState->angle+pi ); // turn current average
			newState->raw_angle = SceneObject::moveAngleIntoRange( newState->raw_angle+pi ); // turn current raw
			if( pHistory().back()->type == pObjectPointer->type() ) // turn old raws
			{
				Ptr<State>( pHistory().back() )->raw_angle = SceneObject::moveAngleIntoRange( Ptr<State>( pHistory().back() )->raw_angle+pi );
			}
			if( pHistory().size()>1 )
			{
				if( pHistory()[ pHistory().size()-2 ]->type == pObjectPointer->type() )
				{
					Ptr<State>( pHistory()[ pHistory().size()-2 ] )->raw_angle = SceneObject::moveAngleIntoRange( Ptr<State>( pHistory()[ pHistory().size()-2 ] )->raw_angle+pi );
				}
			}

			//adjust confidence level
			if( scalarVelAngle<-hardScalarLimit ) pDirectionConfidenceLevel = 1.0;
			else pDirectionConfidenceLevel = confidence;
		}
		else if( pDirectionConfidenceLevel != 0.0 && confidence > pDirectionConfidenceLevel && pDirectionConfidenceLevel!=1.0 )
		{
			pDirectionConfidenceLevel = confidence;
			if( pDirectionConfidenceLevel>1.0 ) pDirectionConfidenceLevel=1.0;
		}
		
	}

	if( pEMAAngleInit ) pEMAAngle = pEMAAngleFac*newState->angle + (1-pEMAAngleFac)*pEMAAngle;
	else
	{
		pEMAAngleInit=true;
		pEMAAngle = newState->angle;
	}

	pHistory().push_back( newState );

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

double DirectedRodEMA::predictHalfLengthOffset() // since halfLengthOffset is a proprietary ABFSpiral property, it has to be checked for each state if it was built when the object was considered an ABFSpiral or not
{
	if( pHistory().size()==0 ) return pLength/2-pDiameter;
	else
	{
		if( pHistory().back()->type == pObjectPointer->type() )
		{
			Ptr<State> lastConv( pHistory().back() );
			return lastConv->halfLengthOffset + lastConv->vel_hlo + 0.5*lastConv->acc_hlo;
		}
		else return pLength/2-pDiameter;
	}
}


void DirectedRodEMA::drawDirection( Mat& _img, Scalar _color )
{
	Ptr<SceneObject::State> current = pHistory().back();
	vector<double> myDirection = direction( current->angle );
	vector<double> myOrthDirection(2);
	myOrthDirection[0]=myDirection[1];
	myOrthDirection[1]=-myDirection[0];

	Point eins( current->x-30*myDirection[0], current->y-30*myDirection[1] );
	Point zwei( current->x+30*myDirection[0], current->y+30*myDirection[1] );
	Point drei( zwei.x-5*myOrthDirection[0]-8*myDirection[0], zwei.y-5*myOrthDirection[1]-8*myDirection[1] );
	Point vier( zwei.x+5*myOrthDirection[0]-8*myDirection[0], zwei.y+5*myOrthDirection[1]-8*myDirection[1] );

	line( _img, eins, zwei, _color, 2, 8 );
	line( _img, zwei, drei, _color, 2, 8 );
	line( _img, zwei, vier, _color, 2, 8 );

	return;
}


int DirectedRodEMA::registerFactoryFunction()
{
	string info = "Free movement dynamics, 4 independent degrees of freedom (position x and y, angle, half length offset), targeted at rod like objects with a directed movement with the direction being interfered and constantly checked (adjustment of the angle). Applies multiple exponential moving average filtering, possible with different alpha values for position, velocity and acceleration. Those values are used for the prediction as well. The half length offset (length-diameter)/2 is used as a measure to estimate and predict movement orthogonal to the view plane, targeted at tracking the direction through 'in place rotations'.";
	GenericMultiLevelMap<string> options;
	setStandardOptions(options);
	int _classId = Dynamics::registerDynamics( DynamicsFacEntry( "DirectedRodEMA", &dremaCreator, info,options ) );
	return _classId;
}



int DirectedRodEMA::classId = DirectedRodEMA::registerFactoryFunction();




DirectedRodEMA::State::State(){}
DirectedRodEMA::State::~State(){}
DirectedRodEMA::State::State( double _x, double _y, double time, double _angle, double _area, int _type ):FreeMovingAverage::State( _x, _y, time, _angle, _area, _type ){}
DirectedRodEMA::State::State( Point _pos, double time, double _angle, double _area, int _type ):FreeMovingAverage::State( _pos, time, _angle, _area, _type ){}
