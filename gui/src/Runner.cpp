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

#include "Runner.h"

Runner::Runner()
{
	pThreadRunning = false;
	pRun = false;
	pRecord = false;
	pVideo = NULL;
	pVideoWriter = NULL;

	pScene = new SceneHandler(false);
	pInterface = new QtInterface(this);
	pStream = new CVMatDisplay();

	if( (*Options::General)["general_settings"]["restore_last_state_at_startup"].as<bool>() )
	{
		pScene->loadFromFile("lastproject.swsc");

		if( (*Options::General)["automatically_updated_program_data"]["last_stream_source"]["exists"].as<bool>() )
		{
			if( (*Options::General)["automatically_updated_program_data"]["last_stream_source"]["is_cam"].as<bool>() )
			{
				openNewStream( "", (*Options::General)["automatically_updated_program_data"]["last_stream_source"].as<int>() );
			}
			else
			{
				openNewStream( (*Options::General)["automatically_updated_program_data"]["last_stream_source"].as<string>() );
			}
		}
	}

}


Runner::~Runner()
{
	if( pScene!=NULL ) delete pScene;
	if( pInterface!=NULL ) delete pInterface;
	//if( pStream!=NULL ) delete pStream; // qt application takes control over object, this will throw an error if it is destroyed here
	if( pVideo!=NULL ) delete pVideo;
	if( pVideoWriter!= NULL ) delete pVideoWriter;
}


void Runner::putStream( CVMatDisplay* _stream )
{
	pStream=_stream;
}


CVMatDisplay* Runner::getCVStream()
{
	return pStream;
}


Runner::QtInterface* Runner::getInterface()
{
	return pInterface;
}


bool Runner::openNewStream( string _filePath, int _cameraId )
{
	//string videoFolder = "D:\\Benutzer\\stewess\\Documents\\ETH\\FS 2014\\Bachelorarbeit\\Matlab\\";
	//string videoName = "swarm1.avi";//"swarm1.avi";//"swarmClipEnd.avi";//"swarmCrossing1.avi";
	// more video:morevideo\\ crystals1.avi crystals2.avi crystals3.avi type2_1.avi type2_2.avi type2_3.avi
	//_filePath=videoFolder+videoName;
	//cout<<endl<<"filepath: "<<_filePath<<endl;
	if( pVideo!=NULL ) delete pVideo;

	if( _cameraId==-1 ) pVideo = new VideoCapture( _filePath );
	else pVideo = new VideoCapture( _cameraId );

	if( !pVideo->isOpened() )
	{
		cerr<<endl<<"Could not open camera or find the video"<<endl;
		return false;
	}
	else
	{
		(*Options::General)["automatically_updated_program_data"].eraseKey("last_stream_source");
		(*Options::General)["automatically_updated_program_data"]["last_stream_source"]["exists"].as<bool>() = true;

		if( _cameraId==-1 )
		{
			(*Options::General)["automatically_updated_program_data"]["last_stream_source"]["is_cam"].as<bool>() = false;
			(*Options::General)["automatically_updated_program_data"]["last_stream_source"].as<string>() = _filePath;
		}
		else
		{
			(*Options::General)["automatically_updated_program_data"]["last_stream_source"]["is_cam"].as<bool>() = true;
			(*Options::General)["automatically_updated_program_data"]["last_stream_source"].as<int>() = _cameraId;
		}

		pScene->setFrameRate(*pVideo);

		(*pScene) << (*pVideo);

		pStream->updateImage( (*pScene).last() );
		pStream->updateOriginal( (*pScene).getOriginalFrame() );
		pStream->updateThresholdImage( (*pScene).objects().thresholdImage() );
		pStream->updatePreProcessImage( (*pScene).preProcessImage() );
		pStream->updatePreThresholdImage( (*pScene).preThresholdImage() );
		pStream->updatesDone();
		pInterface->signalNewSource();
		return true;
	}
}


bool Runner::analyzeNewFrame()
{

	pSceneLock.lock(); // make sure SceneHandler values aren't altered while SceneHandler is active
	
	if( pScene==NULL || pVideo==NULL )
	{
		pSceneLock.unlock();	
		return false;
	}
	else if( !pVideo->isOpened() )
	{
		pSceneLock.unlock();	
		return false;
	}

	bool returnValue = true;
	if( !( (*pScene) << (*pVideo) ) ) returnValue = false;
			
	pStream->updateImage( (*pScene).last() );
	pStream->updateOriginal( (*pScene).getOriginalFrame() );
	pStream->updateThresholdImage( (*pScene).objects().thresholdImage() );
	pStream->updatePreProcessImage( (*pScene).preProcessImage() );
	pStream->updatePreThresholdImage( (*pScene).preThresholdImage() );
	pStream->updatesDone();

	if( pRecord )
	{
		if( pVideoWriter!=NULL ) (*pVideoWriter) << (*pScene).last();
		else record();
	}

	pSceneLock.unlock();

	return returnValue;
}


void Runner::read()
{
	pRun = true;
}


void Runner::stop()
{
	pRun = false;
	stopRecording();
}


void Runner::reinitialize( bool _withoutLock )
{
	if( !_withoutLock ) pSceneLock.lock();
	if( pVideo==NULL )
	{
		if( !_withoutLock ) pSceneLock.unlock();
		return;
	}
	stop();
	try
	{
		delete pScene;
		pScene = new SceneHandler(false); // don't initialize all classes
		pScene->loadFromFile("lastproject.swsc");
        boost::filesystem::remove("temp/tempsave.xml");

		if( (*Options::General)["general_settings"]["reset_object_count_on_reload"].as<bool>() ) SceneObject::resetObjectCount();

		if( (*Options::General)["automatically_updated_program_data"]["last_stream_source"]["exists"].as<bool>() )
		{
			if( (*Options::General)["automatically_updated_program_data"]["last_stream_source"]["is_cam"].as<bool>() )
			{
				openNewStream( "", (*Options::General)["automatically_updated_program_data"]["last_stream_source"].as<int>() );
			}
			else
			{
				openNewStream( (*Options::General)["automatically_updated_program_data"]["last_stream_source"].as<string>() );
			}
		}
	}
	catch(...)
	{
		pInterface->postError("Reinitializing failed, not sure in what state now. Be cautious.");
	}
	if( !_withoutLock ) pSceneLock.unlock();
	
	if( !_withoutLock ) pInterface->signalReload();
	
	read();
}


void Runner::record()
{
    /*if(!pRun)
    {
		pRecord = false;
		return;
    }*/
	if( pVideoWriter==NULL && pVideo->isOpened() )
    {
        string tempName = "temp/tempRec.avi";
        boost::filesystem::create_directory( "temp" );
		boost::filesystem::remove( tempName );


		pVideoWriter = new VideoWriter();
		pVideoWriter->open( tempName, pVideo->get(CV_CAP_PROP_FOURCC), 30, Size(640,480) );
		
		if( pVideoWriter->isOpened() )
		{
			pRecord = true;
        }
		return;
	}
	else
	{
        pRecord = true;
	}
}


void Runner::stopRecording()
{
	pRecord = false;
}


void Runner::clearVideoBuffer()
{
	pSceneLock.lock();
	stopRecording();
	if( pVideoWriter!=NULL )
	{
		pVideoWriter->release();
		delete pVideoWriter;
		pVideoWriter=NULL;
	}
	pSceneLock.unlock();
}


bool Runner::saveVideo( QString _path )
{
	pSceneLock.lock();
	if( pVideoWriter==NULL )
	{
		pSceneLock.unlock();
		return false;
	}

	stopRecording();
	pVideoWriter->release();
	//cout<<endl<<_path<<endl;
	QUrl someAnnoyingUnnecessaryStuff(_path);
		
	boost::filesystem::path myPath( someAnnoyingUnnecessaryStuff.toLocalFile().toStdString() );
	if( myPath.extension() != ".avi" ) myPath += ".avi";

	try
	{
        boost::filesystem::copy_file( "temp/tempRec.avi",myPath.string() , boost::filesystem::copy_option::overwrite_if_exists );
	}
	catch( const boost::filesystem::filesystem_error& e )
	{
		cout<<endl<<e.what()<<endl;
	}
	pSceneLock.unlock();

	return true;
}


void Runner::stopThread()
{
	pThreadRunning = false;
}


bool Runner::saveProject( QString _path )
{
	if( pScene==NULL ) return false;
	
	QUrl someAnnoyingUnnecessaryStuff(_path);		
	boost::filesystem::path myPath( someAnnoyingUnnecessaryStuff.toLocalFile().toStdString() );
	if( myPath.extension() != ".swsc" ) myPath += ".swsc";

	return pScene->saveProject( myPath.string() );
}


bool Runner::openProject( QString _path )
{
	pSceneLock.lock();
	
	if( pScene!=NULL ) delete pScene;
	if( pVideo!=NULL ) delete pVideo;
	if( pVideoWriter!= NULL ) delete pVideoWriter;

	pVideo = NULL;
	pVideoWriter = NULL;
	pScene = new SceneHandler(false);
	
	QUrl someAnnoyingUnnecessaryStuff(_path);	
	boost::filesystem::path myPath( someAnnoyingUnnecessaryStuff.toLocalFile().toStdString() );
	stop();
	try
	{
		pScene->loadFromFile( myPath.string() );

		SceneObject::resetObjectCount();

		if( (*Options::General)["automatically_updated_program_data"]["last_stream_source"]["exists"].as<bool>() )
		{
			if( (*Options::General)["automatically_updated_program_data"]["last_stream_source"]["is_cam"].as<bool>() )
			{
				openNewStream( "", (*Options::General)["automatically_updated_program_data"]["last_stream_source"].as<int>() );
			}
			else
			{
				openNewStream( (*Options::General)["automatically_updated_program_data"]["last_stream_source"].as<string>() );
			}
		}
	}
	catch(...)
	{
		pSceneLock.unlock();
		// and create empty project
		return false;
	}
	pSceneLock.unlock();

	pInterface->signalReload();
	return true;
}


void Runner::createNewProject()
{
	pSceneLock.lock();

	stop();
	if( pScene!=NULL ) delete pScene;
	if( pVideo!=NULL ) delete pVideo;
	if( pVideoWriter!= NULL ) delete pVideoWriter;

	pVideo = NULL;
	pVideoWriter = NULL;
	pScene = new SceneHandler(false);
	
	SceneObject::resetObjectCount();
	Options::resetUserSettings();
	
	
	pStream->updateImage( Mat() );
	pStream->updateOriginal(  Mat() );
	pStream->updateThresholdImage(  Mat() );
	pStream->updatePreProcessImage(  Mat() );
	pStream->updatePreThresholdImage(  Mat() );
	
	(*Options::General)["automatically_updated_program_data"]["last_stream_source"]["exists"].as<bool>() = false;

	pStream->updatesDone();
	pSnapshot.release();

	pSceneLock.unlock();

	pInterface->signalReload();
	pInterface->signalNewSource();
	

}


bool Runner::newCamSource( int _cameraId )
{
	bool success;
	pSceneLock.lock();
	stop();
	try
	{
		delete pScene;
		pScene = new SceneHandler(false); // don't initialize all classes
		pScene->loadFromFile("lastproject.swsc");
        boost::filesystem::remove("temp/tempsave.xml");

		if( (*Options::General)["general_settings"]["reset_object_count_on_reload"].as<bool>() ) SceneObject::resetObjectCount();

		success = openNewStream( "", _cameraId );

		if( !success && (*Options::General)["automatically_updated_program_data"]["last_stream_source"]["exists"].as<bool>() )
		{
			if( (*Options::General)["automatically_updated_program_data"]["last_stream_source"]["is_cam"].as<bool>() )
			{
				openNewStream( "", (*Options::General)["automatically_updated_program_data"]["last_stream_source"].as<int>() );
			}
			else
			{
				openNewStream( (*Options::General)["automatically_updated_program_data"]["last_stream_source"].as<string>() );
			}
		}
		
	}
	catch(...)
	{
		pInterface->postError("Unknown error occured while attempting to open camera source, not sure in what state now. Be cautious.");
	}

	pSceneLock.unlock();
	pInterface->signalReload();
	
	//read();

	return success;
}


bool Runner::newFileSource( QString _path )
{
	bool success;

	QUrl someAnnoyingUnnecessaryStuff(_path);		
	boost::filesystem::path myPath( someAnnoyingUnnecessaryStuff.toLocalFile().toStdString() );

	pSceneLock.lock();
	stop();
	try
	{
		delete pScene;
		pScene = new SceneHandler(false); // don't initialize all classes
		pScene->loadFromFile("lastproject.swsc");
        boost::filesystem::remove("temp/tempsave.xml");
	

		if( (*Options::General)["general_settings"]["reset_object_count_on_reload"].as<bool>() ) SceneObject::resetObjectCount();

		success = openNewStream( myPath.string() );
		
		if( !success && (*Options::General)["automatically_updated_program_data"]["last_stream_source"]["exists"].as<bool>() )
		{
			if( (*Options::General)["automatically_updated_program_data"]["last_stream_source"]["is_cam"].as<bool>() )
			{
				openNewStream( "", (*Options::General)["automatically_updated_program_data"]["last_stream_source"].as<int>() );
			}
			else
			{
				openNewStream( (*Options::General)["automatically_updated_program_data"]["last_stream_source"].as<string>() );
			}
		}
		
		if( success )
		{
			(*Options::General)["automatically_updated_program_data"].eraseKey("last_stream_source");
			(*Options::General)["automatically_updated_program_data"]["last_stream_source"]["is_cam"].as<bool>() = false;
			(*Options::General)["automatically_updated_program_data"]["last_stream_source"]["exists"].as<bool>() = true;
			(*Options::General)["automatically_updated_program_data"]["last_stream_source"].as<string>() = myPath.string();
		}
	}
	catch(...)
	{
		pInterface->postError("Unknown error occured while attempting to open video file, not sure in what state now. Be cautious.");
	}

	pSceneLock.unlock();
	pInterface->signalReload();
	
	//read();

	return success;
}


bool Runner::exportData( QList<QString> _objectsForExport, QString _timestepSeparator, QString _dataSeparator, QString _path )
{
	if( pScene==NULL ) return false;
	
	QUrl someAnnoyingUnnecessaryStuff(_path);		
	boost::filesystem::path myPath( someAnnoyingUnnecessaryStuff.toLocalFile().toStdString() );

	string tSeparator = _timestepSeparator.toStdString();
	string dSeparator = _dataSeparator.toStdString();

	bool success = true;

	QList<QString>::Iterator it, end;
	end = _objectsForExport.end();

	lock();
	for( it=_objectsForExport.begin(); it!=end; it++ )
	{
		int objId = (*it).toUInt();

		Ptr<SceneObject> objToExport = this->pScene->objects().getObj( objId );
		boost::filesystem::path currPath = myPath;

		if( objToExport==NULL )
		{
			success = false;
			continue;
		}
		stringstream conv; string id;
		conv<<objId; conv>>id;
		currPath /= "obj"+id+".txt";
		success = success && objToExport->exportData( currPath.string(), dSeparator, tSeparator );
	}
	unlock();

	return success;
}


void Runner::lock()
{
	pSceneLock.lock();
	return;
}


void Runner::unlock()
{
	pSceneLock.unlock();
	return;
}


void Runner::takeSnapshot()
{
	(*pScene).last().copyTo( pSnapshot );
	return;
}


bool Runner::saveSnapshot( QString _path )
{
	if( pSnapshot.empty() ) return false;

	vector<int> pngOpt(2); // option vector for png compression
	pngOpt[0] = CV_IMWRITE_PNG_COMPRESSION;
	pngOpt[1] = 3;

	QUrl someAnnoyingUnnecessaryStuff(_path);

	boost::filesystem::path myPath( someAnnoyingUnnecessaryStuff.toLocalFile().toStdString() );
	if( myPath.extension() != ".png" ) myPath += ".png";

	bool success = imwrite( myPath.string(), pSnapshot, pngOpt );

	pSnapshot.release();
	return success;
}


void Runner::run()
{
	pThreadRunning = true;

	//(*pScene).addPreProcessingAlgorithm("color range expansion");
	//(*pScene).typesInScene("ABFDrill","Unknown Type");
	
	//openNewStream("");
	//SH_1.setThreshold(115);
	/*
	vector<double> cbaOptions;
	cbaOptions.push_back( 1.2 );
	cbaOptions.push_back( -10 ); //options 2.7, 20 // second 1.8, 25 // 1.23,0
	SH_1.addInternPreProcessingAlgorithm( "contrast brightness adjustment", cbaOptions );

	SH_1.typesNotInScene( "ABFSpiral","ThinHelix" );*/
	//SH_1.setObservationArea(180,40,270,320);
	
	//SH_1.loadFromFile("test2.xml");

//	double tpf = 1000/(*pScene).getFrameRate(); // time per frame
//	cout<<endl<<"Time per frame: "<<tpf<<endl;

//	double msTimer_a;
//	double msTimer_b;

//	double timer;
	
	
//	double timeTarget = tpf;
//	double startTime = (*pScene).msTime();
//	double stepTarget=tpf;

//	double waitKeyTime;

//	int frameCount=0;
//	string outputName = "D:\\Benutzer\\stewess\\Documents\\ETH\\FS 2014\\Bachelorarbeit\\opencvoutput2\\extract";
//	vector<int> pngOpt(2);
//	pngOpt[0]=CV_IMWRITE_PNG_COMPRESSION;
//	pngOpt[1]=0;

    /*GenericMultiLevelMap<string> someOptions;
	someOptions["object_class_settings"]["random"].as<string>()="ABFDrill";
	someOptions["object_class_settings"]["random"]["dynamics_options"]["position_alpha"].as<double>()=0;
	someOptions["object_class_settings"]["random"]["dynamics_options"]["angle_alpha"].as<double>()=0;
	someOptions["object_class_settings"]["random"]["dynamics_options"]["velocity_alpha"].as<double>()=0;
	someOptions["object_class_settings"]["random"]["dynamics_options"]["angle_velocity_alpha"].as<double>()=0;
	someOptions["object_class_settings"]["random"]["dynamics_options"]["acceleration_alpha"].as<double>()=0;
	someOptions["object_class_settings"]["random"]["dynamics_options"]["angle_acceleration_alpha"].as<double>()=0;
    */
    /*
	bool writeToVideo = false;
	bool record = false;

	VideoWriter output1, output2;
	if( record )
	{
		//output1.open( videoFolder+"colorRangeIn1234.avi", video.get(CV_CAP_PROP_FOURCC), 30, Size(640,480) );
		//output2.open( videoFolder+"ABFswarm2345.avi", video.get(CV_CAP_PROP_FOURCC), 30, Size(640,480) );
	}
	//SH_1.pushFrame(image,0);
	
	//imshow("original video", SH_1.getOriginalFrame() );
	//imshow("edited video", SH_1.last() );
	record=false;
	int lastWaitKey = -1;

	Ptr<DescriptorCreator> creator = (*pScene).objects().newDescriptorCreator(0,6,8);
    */

    //int i=0;
    //double sTime,dTime,minTime=100,maxTime=0,und23Time=0,over33Time=0,over30Time=0,totTime=0,firstItTime;
    //double timeBefore=(*pScene).msTime(),timeAfter;

	while(pThreadRunning)
    {

        if(pRun)
		{
//			sTime=(*pScene).msTime();

			if( analyzeNewFrame() )
			{
//				dTime=(*pScene).msTime()-sTime;
//				if(dTime<minTime) minTime=dTime;
//				if(dTime>maxTime) maxTime=dTime;
//				if(dTime<23) und23Time++;
//				if(dTime>30) over30Time++;
//				if(dTime>33) over33Time++;
//				if(i==0) firstItTime=dTime;
//				if( waitKey(1)>=0 ) if( waitKey(0)==27 ) break;
//				i++;
//				totTime+=dTime;
			}
			else stop();
        }
	}
    /*timeAfter=(*pScene).msTime();

    cout<<endl<<"Averaged time with waitKey(1) operation per iteration is: "<<(timeAfter-timeBefore)/i<<"ms.";
	cout<<endl<<"Averaged time without waitKey(1) operation per iteration is: "<<(totTime)/i<<"ms.";
	cout<<endl<<"Min iteration time is: "<<minTime<<"ms.";
	cout<<endl<<"Max iteration time is: "<<maxTime<<"ms.";
	cout<<endl<<"First iteration time is: "<<firstItTime<<"ms.";
	cout<<endl<<"Iterations under 23ms: "<<und23Time<<"";
	cout<<endl<<"Iterations over 30ms: "<<over30Time<<"";
	cout<<endl<<"Iterations over 33ms: "<<over33Time<<"";
    cout<<endl<<"Iterations in total: "<<i<<"";*/
	
	stop();
	/*
	cout<<endl<<"Number of recorded data: "<<creator->dataEntrySize()<<endl;

	// create new feature set with the recorded data
	creator->name = "Drawn_Negatives";
	creator->info = "Set of various handdrawn shapes on almost uniform white background intended to be used as a negative if no others are available. ";
	if( !creator->create() ) cout<<endl<<"something went wrong"<<endl;
	else cout<<endl<<"it appears to have worked!"<<endl;

	// create new generic class
	vector<string> abfFeatureSets;
	abfFeatureSets.push_back("fd1406851828");
	GenericObject::GOData genericABFClass( "Unknown Type",Scalar(130,130,130),"SimpleFreeMovement",abfFeatureSets );
	genericABFClass.info = "Unknown type, everything that is not of any known sort, can be used as a second type for classification reasons.";
	genericABFClass.save();

	*/
	//SH_1.saveProject("test2.xml");
	//Options::save_options();
	waitKey(0);
	
}
