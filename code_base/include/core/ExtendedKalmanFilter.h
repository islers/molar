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

/** provides some functionality for an extended Kalman filter implementation
Updates of the state need to be carried out separately and are not included

No control matrix is included either.
*/

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
using namespace cv;

class ExtendedKalmanFilter
{
public:
	ExtendedKalmanFilter(void);
	~ExtendedKalmanFilter(void);

	Mat statePre; // x_p
	Mat statePost; // x_m
	Mat transitionMatrix; // A
	Mat measurementMatrix; // H
	Mat processNoiseCov; // Q
	Mat measurementNoiseCov; // R
	Mat errorCovPre; // P_p
	Mat errorCovPost; // P_m

	Mat processNoiseTransMatrix; // L
	Mat measurementNoiseTransMatrix; // M

	/** runs the prediction step for the estimated state covariance matrix P, but not for the state*/
	void predict( bool processNoiseModelUnchanged=true );
	/** carries out the update step for state and state covariance, takes as input the measured values and the predicted values, based on the predicted state*/
	void correct( const Mat& _measurements, const Mat& _predictedMeasurements, bool measurementNoiseModelUnchanged=true );

private:
	Mat tempProcessNoise; // L*Q*L^T
	Mat tempMeasNoise; // M*R*M^T
	bool tpnInit, tmnInit;

	Mat tempK;
	Mat tempPHt; // P_p*H^T temporary matrix
	Mat tempAPm;
	Mat tempToInv;
	Mat tempInv;
	Mat tempDiff;
	Mat tempKH;
};

