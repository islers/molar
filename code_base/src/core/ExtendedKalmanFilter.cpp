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

#include "ExtendedKalmanFilter.h"
#include <iostream>
using namespace std;

ExtendedKalmanFilter::ExtendedKalmanFilter(void)
{
	tpnInit = false;
	tmnInit = false;
}


ExtendedKalmanFilter::~ExtendedKalmanFilter(void)
{
}


void ExtendedKalmanFilter::predict( bool _processNoiseModelUnchanged )
{
	if( !_processNoiseModelUnchanged || !tpnInit ) // L*Q*L^T
	{
		Mat LQ;
        gemm(processNoiseTransMatrix,processNoiseCov,1,Mat(),0,LQ );
        gemm(LQ,processNoiseTransMatrix,1,Mat(),0,tempProcessNoise, GEMM_2_T );

		tpnInit = true;
	}

    gemm(transitionMatrix,errorCovPost,1,Mat(),0,tempAPm); // A*Pm
	gemm(tempAPm,transitionMatrix,1,tempProcessNoise,1,errorCovPre,GEMM_2_T); // Pp = A*Pm*A^T + L*Q*L^T

	return;
}


void ExtendedKalmanFilter::correct( const Mat& _measurements, const Mat& _predictedMeasurements, bool _measurementNoiseModelUnchanged )
{
	if( !_measurementNoiseModelUnchanged || !tmnInit )
	{
		Mat MR;
        gemm(measurementNoiseTransMatrix,measurementNoiseCov,1,Mat(),0,MR);
        gemm(MR,measurementNoiseTransMatrix,1,Mat(),0,tempMeasNoise,GEMM_2_T);

		tmnInit = true;
	}

    gemm(errorCovPre,measurementMatrix,1,Mat(),0,tempPHt,GEMM_2_T); // P_p*H^T
	gemm(measurementMatrix,tempPHt,1,tempMeasNoise,1,tempToInv); // H*Pp*H^T+M*R*M^T
	invert(tempToInv,tempInv); // (H*Pp*H^T+M*R*M^T)^(-1)
    gemm(tempPHt,tempInv,1,Mat(),0,tempK); // K=P_p*H^T*(H*Pp*H^T+M*R*M^T)^(-1)
	tempDiff = _measurements-_predictedMeasurements; // z(k)-h_k(x_p(k))

	gemm(tempK,tempDiff,1,statePre,1,statePost); // x_m = x_p + K(k)*[z(k)-h_k(x_p(k))]

    gemm(tempK,measurementMatrix,1,Mat(),0,tempKH); // K*H
	gemm(tempKH,errorCovPre,-1,errorCovPre,1,errorCovPost); // P_m = [I-K*H]*P_p

	return;
}
