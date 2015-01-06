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

#include <QApplication>
#include <QQmlApplicationEngine>
#include <qdeclarativeview.h>
#include <qicon.h>
#include <qthread.h>
#include "cvmatdisplay.h"

#include "Runner.h"

#include <iostream>
#include <string>
#include <list>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "SceneHandler.h"
#include "ContrastBrightnessAdjustment.h"


using namespace std;
using namespace cv;


#include <QWidget>
#include <QImage>
#include <QPainter>


#include <qqmlcontext.h>

int main(int argc, char *argv[])
{

	Runner molar;
	
    QApplication app(argc, argv);
    QQmlApplicationEngine engine;
	engine.addImageProvider(QLatin1String("opencv"), molar.getCVStream() );
    engine.rootContext()->setContextProperty("cvstream", molar.getCVStream() );
	
	engine.rootContext()->setContextProperty("p_inter", molar.getInterface() );
    engine.load(QUrl("qrc:/qml/main.qml"));
    app.setWindowIcon(QIcon("qrc:/img/logo.png"));

	molar.getInterface()->initialize();


	molar.start();

	app.exec();
	molar.stopThread();

    return 0;
}
