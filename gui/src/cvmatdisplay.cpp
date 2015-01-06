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

#include "cvmatdisplay.h"

CVMatDisplay::CVMatDisplay():QQuickImageProvider(QQuickImageProvider::Image)
{

	_tmp = Mat();
	updateImage(_tmp);
	_tOriginal = Mat();
	updateOriginal(_tOriginal);

	_tThresholdImage = Mat();
	updateThresholdImage( _tThresholdImage );

	_tPreProcessImage = Mat();
	updatePreProcessImage( _tPreProcessImage );

	_tPreThresholdImage = Mat();
	updatePreThresholdImage( _tPreThresholdImage );
}

CVMatDisplay::~CVMatDisplay()
{

}


QImage CVMatDisplay::requestImage(const QString &id, QSize */*size*/, const QSize &/*requestedSize*/)
{
	QImage ret;
	pLock.lock();

	if( id=="edit" ) ret = _qimage;
	if( id=="original" ) ret = _qOriginal;
	if( id=="thresholdimage" ) ret = _qThresholdImage;
	if( id=="preprocessimage" ) ret = _qPreProcessImage;
	if( id=="prethresholdimage" ) ret = _qPreThresholdImage;

	//ret.bits(); qt's implicit sharing detaches shared memory before it is edited, thus no manual deep copies are necessary
	pLock.unlock();
	return ret;
}


void CVMatDisplay::updateImage( cv::Mat _newImage )
{
	convertImage( _newImage, _tmp, _qimage );
	return;
}


void CVMatDisplay::updateOriginal( cv::Mat _newImage )
{
	convertImage( _newImage, _tOriginal, _qOriginal );
	return;
}



void CVMatDisplay::updateThresholdImage( cv::Mat _newImage )
{
	convertImage( _newImage, _tThresholdImage, _qThresholdImage );
	return;
}


void CVMatDisplay::updatePreProcessImage( cv::Mat _newImage )
{
	convertImage( _newImage, _tPreProcessImage, _qPreProcessImage );
	return;
}


void CVMatDisplay::updatePreThresholdImage( cv::Mat _newImage )
{
	convertImage( _newImage, _tPreThresholdImage, _qPreThresholdImage );
	return;
}

void CVMatDisplay::updatesDone()
{
	emit imageUpdated();
	return;
}


void CVMatDisplay::convertImage( cv::Mat& _cvImage, cv::Mat& _cvTemp, QImage& _qImage )
{
	 // Convert the image to the RGB888 format
    switch (_cvImage.type()) {
    case CV_8UC1:
        cvtColor(_cvImage, _cvTemp, CV_GRAY2RGB);
        break;
    case CV_8UC3:
        cvtColor(_cvImage, _cvTemp, CV_BGR2RGB);
        break;
    }

    // QImage needs the data to be stored continuously in memory
    assert(_cvTemp.isContinuous());
    // Assign OpenCV's image buffer to the QImage. Note that the bytesPerLine parameter
    // (http://qt-project.org/doc/qt-4.8/qimage.html#QImage-6) is 3*width because each pixel
    // has three bytes.
	pLock.lock();
    _qImage = QImage(_cvTemp.data, _cvTemp.cols, _cvTemp.rows, _cvTemp.cols*3, QImage::Format_RGB888);
	pLock.unlock();
	return;
}
