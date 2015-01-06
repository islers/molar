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
#include "Dynamics.h"
#include <qurl.h>

Runner::QtInterface::QtInterface( Runner* _link )
{

	pLink = _link;
}

Runner::QtInterface::~QtInterface()
{

}


void Runner::QtInterface::initialize()
{
	if( pLink->pVideo!=NULL )
	{
		pLink->pStream->updatesDone();
		signalNewSource();
	}
	emit generalSettingsChange();
	if( pLink->pScene!=NULL )
	{
		emit preProcessAlgorithmChange();
		emit internPreProcessAlgorithmChange();
		emit sceneTypeChange();
	}
}


Q_INVOKABLE int Runner::QtInterface::intSignal( int id, bool _set, int _param )
{
	int ret;
	switch(id)
	{
		case 2:
			if(_set) (*Options::General)["display"]["objects"]["path_length"].as<int>() = _param;
			ret = (*Options::General)["display"]["objects"]["path_length"].as<int>();
			break;
		case 3:
			if(_set) (*Options::General)["display"]["objects"]["path_fadeout_speed"].as<int>() = _param;
			ret = (*Options::General)["display"]["objects"]["path_fadeout_speed"].as<int>();
			break;
		case 13:
			if(_set) (*Options::General)["runtime"]["memory"]["max_video_ram_usage"].as<int>() = _param;
			ret = (*Options::General)["runtime"]["memory"]["max_video_ram_usage"].as<int>();
			break;
		case 18:
			if(_set) (*Options::General)["runtime"]["compression"]["png_compression_level"].as<int>() = _param;
			ret = (*Options::General)["runtime"]["compression"]["png_compression_level"].as<int>();
			break;
		case 19:
			if(_set) (*Options::General)["runtime"]["compression"]["jpeg_quality"].as<int>() = _param;
			ret = (*Options::General)["runtime"]["compression"]["jpeg_quality"].as<int>();
			break;
		case 25:
			if(_set) (*Options::General)["object_detection"]["missing_state_bridging"].as<int>() = _param;
			ret = (*Options::General)["object_detection"]["missing_state_bridging"].as<int>();
			break;
		case 50:
			if(_set) (*Options::General)["classification"]["dynamic_feature_adaption_step"].as<int>() = _param;
			ret = (*Options::General)["classification"]["dynamic_feature_adaption_step"].as<int>();
			break;
		default: ret = 0;
	}

	SceneHandler::setupOptions();
	ObjectHandler::setupOptions();
	return ret;
}


Q_INVOKABLE unsigned int Runner::QtInterface::uintSignal( int id, bool _set, unsigned int _param )
{
	
	bool updateStatics = true;
	unsigned int ret;
	switch(id)
	{
		case 4:
			if(_set) (*Options::General)["display"]["objects"]["path_step_length"].as<unsigned int>() = _param;
			ret = (*Options::General)["display"]["objects"]["path_step_length"].as<unsigned int>();
			break;
		case 24:
			if(_set) (*Options::General)["object_detection"]["recalculation_interval"].as<unsigned int>() = _param;
			ret = (*Options::General)["object_detection"]["recalculation_interval"].as<unsigned int>();
			break;
		case 26:
			if(_set) (*Options::General)["object_detection"]["max_object_missing_time"].as<unsigned int>() = _param;
			ret = (*Options::General)["object_detection"]["max_object_missing_time"].as<unsigned int>();
			break;
		case 30:
			if(_set) (*Options::General)["object_detection"]["static_threshold"].as<unsigned int>() = _param;
			ret = (*Options::General)["object_detection"]["static_threshold"].as<unsigned int>();
			break;
		case 31:
			if(_set) (*Options::General)["object_detection"]["window_border_range"].as<unsigned int>() = _param;
			ret = (*Options::General)["object_detection"]["window_border_range"].as<unsigned int>();
			break;
		case 45: // observation area x
			{
				pLink->lock();
				updateStatics = false;
				Rect area = pLink->pScene->getObservationArea();
				if(_set)
				{
					Mat theLast = pLink->pScene->last();
                    unsigned int width = (unsigned int)theLast.cols;
					if( _param+area.width >= width )
					{
						_param = width - area.width;
					}
					pLink->pScene->setObservationArea(_param,area.y,area.width,area.height);
				}
				ret = area.x;
				pLink->unlock();
				break;
			}
		case 46: // observation area y
			{
				pLink->lock();
				updateStatics = false;
				Rect area = pLink->pScene->getObservationArea();
				if(_set)
				{
					Mat theLast = pLink->pScene->last();
                    unsigned int height = (unsigned int)theLast.rows;
					if( _param+area.height >= height )
					{
						_param = height - area.height;
					}
					pLink->pScene->setObservationArea(area.x,_param,area.width,area.height);
				}
				ret = area.y;
				pLink->unlock();
				break;
			}
		case 47: // observation area width
			{
				pLink->lock();
				updateStatics = false;
				Rect area = pLink->pScene->getObservationArea();
				if(_set)
				{
					Mat theLast = pLink->pScene->last();
                    unsigned int width = theLast.cols;
					if( _param+area.x >= width )
					{
						_param = width - area.x;
					}
					pLink->pScene->setObservationArea(area.x,area.y,_param,area.height);
					pLink->reinitialize(true);
				}
				area = pLink->pScene->getObservationArea();
				ret = area.width;
				pLink->unlock();
				break;
			}
		case 48: // observation area height
			{
				pLink->lock();
				updateStatics = false;
				Rect area = pLink->pScene->getObservationArea();
				if(_set)
				{
					Mat theLast = pLink->pScene->last();
                    unsigned int height = theLast.rows;
					if( _param+area.y >= height )
					{
						_param = height - area.y;
					}
					pLink->pScene->setObservationArea(area.x,area.y,area.width,_param);
					pLink->reinitialize(true);
				}
				area = pLink->pScene->getObservationArea();
				ret = area.height;
				pLink->unlock();
				break;
			}
		default: return 0;
	}

	if( updateStatics )
	{
		SceneHandler::setupOptions();
		ObjectHandler::setupOptions();
	}
	return ret;
}


Q_INVOKABLE double Runner::QtInterface::doubleSignal( int id, bool _set, double _param )
{

	double ret;
	bool updateStatics = true;

	switch(id)
	{
		case 10:
			if(_set) (*Options::General)["display"]["general"]["draw_observation_area"]["R"].as<double>() = _param;
			ret = (*Options::General)["display"]["general"]["draw_observation_area"]["R"].as<double>();
			break;
		case 11:
			if(_set) (*Options::General)["display"]["general"]["draw_observation_area"]["G"].as<double>() = _param;
			ret = (*Options::General)["display"]["general"]["draw_observation_area"]["G"].as<double>();
			break;
		case 12:
			if(_set) (*Options::General)["display"]["general"]["draw_observation_area"]["B"].as<double>() = _param;
			ret = (*Options::General)["display"]["general"]["draw_observation_area"]["B"].as<double>();
			break;
		case 20:
			if(_set) (*Options::General)["video_content_descriptions"]["min_area"].as<double>() = _param;
			ret = (*Options::General)["video_content_descriptions"]["min_area"].as<double>();
			break;
		case 21:
			if(_set) (*Options::General)["video_content_descriptions"]["max_area"].as<double>() = _param;
			ret = (*Options::General)["video_content_descriptions"]["max_area"].as<double>();
			break;
		case 22:
			if(_set) (*Options::General)["video_content_descriptions"]["contour_length_switch"].as<double>() = _param;
			ret = (*Options::General)["video_content_descriptions"]["contour_length_switch"].as<double>();
			break;
		case 27:
			if(_set) (*Options::General)["object_detection"]["max_matching_distance_mismatch"].as<double>() = _param;
			ret = (*Options::General)["object_detection"]["max_matching_distance_mismatch"].as<double>();
			break;
		case 29:
			if(_set) (*Options::General)["object_detection"]["max_object_gridpoint_distance"].as<double>() = _param;
			ret = (*Options::General)["object_detection"]["max_object_gridpoint_distance"].as<double>();
			break;
		case 41:
			if(_set) (*Options::General)["object_detection"]["max_object_gridpoint_distance"].as<double>() = _param;
			ret = (*Options::General)["object_detection"]["max_object_gridpoint_distance"].as<double>();
			break;
		case 44: // internal object threshold
			if(_set) pLink->pScene->setThreshold( _param );
			ret = pLink->pScene->getThreshold();
			updateStatics = false;
			break;
		default:
			ret = 0.0;
	}

	if(updateStatics)
	{
		SceneHandler::setupOptions();
		ObjectHandler::setupOptions();
		SceneObject::setupOptions();
	}
	return ret;
}


Q_INVOKABLE bool Runner::QtInterface::boolSignal( int id, bool _set, bool _param )
{

	bool ret;
	switch(id)
	{
		case 1:
			if(_set) (*Options::General)["general_settings"]["restore_last_state_at_startup"].as<bool>() = _param;
			ret = (*Options::General)["general_settings"]["restore_last_state_at_startup"].as<bool>();
			break;
		case 5:
			if(_set) (*Options::General)["display"]["objects"]["create_threshold_detection_image"].as<bool>() = _param;
			ret = (*Options::General)["display"]["objects"]["create_threshold_detection_image"].as<bool>();
			break;
		case 6:
			if(_set) (*Options::General)["display"]["objects"]["draw_predicted_regions"].as<bool>() = _param;
			ret = (*Options::General)["display"]["objects"]["draw_predicted_regions"].as<bool>();
			break;
		case 7:
			if(_set) (*Options::General)["display"]["objects"]["create_preprocess_filter_image"].as<bool>() = _param;
			ret = (*Options::General)["display"]["objects"]["create_preprocess_filter_image"].as<bool>();
			break;
		case 8:
			if(_set) (*Options::General)["display"]["objects"]["create_prethreshold_filter_image"].as<bool>() = _param;
			ret = (*Options::General)["display"]["objects"]["create_prethreshold_filter_image"].as<bool>();
			break;
		case 9:
			if(_set) (*Options::General)["display"]["general"]["draw_observation_area"].as<bool>() = _param;
			ret = (*Options::General)["display"]["general"]["draw_observation_area"].as<bool>();
			break;
		case 15:
			if(_set) (*Options::General)["runtime"]["buffer"]["activated"].as<bool>() = _param;
			ret = (*Options::General)["runtime"]["buffer"]["activated"].as<bool>();
			break;
		case 16:
			if(_set) (*Options::General)["runtime"]["buffer"]["record_input"].as<bool>() = _param;
			ret = (*Options::General)["runtime"]["buffer"]["record_input"].as<bool>();
			break;
		case 23:
			if(_set) (*Options::General)["object_detection"]["trace_states"].as<bool>() = _param;
			ret = (*Options::General)["object_detection"]["trace_states"].as<bool>();
			break;
		case 28:
			if(_set) (*Options::General)["object_detection"]["use_old_state_fallback"].as<bool>() = _param;
			ret = (*Options::General)["object_detection"]["use_old_state_fallback"].as<bool>();
			break;
		case 40:
			if(_set) (*Options::General)["classification"]["time_awareness"].as<bool>() = _param;
			ret = (*Options::General)["classification"]["time_awareness"].as<bool>();
			break;
		case 43:
			if(_set) (*Options::General)["general_settings"]["reset_object_count_on_reload"].as<bool>() = _param;
			ret = (*Options::General)["general_settings"]["reset_object_count_on_reload"].as<bool>();
			break;
		case 49:
			if(_set) (*Options::General)["classification"]["dynamic_feature_extraction_threshold_adaption"].as<bool>() = _param;
			ret = (*Options::General)["classification"]["dynamic_feature_extraction_threshold_adaption"].as<bool>();
			break;
		default: ret = false;
	}

	SceneHandler::setupOptions();
	ObjectHandler::setupOptions();
	return ret;
}


Q_INVOKABLE QString Runner::QtInterface::stringSignal( int id, bool _set, QString _param )
{
	string newContent = _param.toStdString();

	QString ret;
	switch(id)
	{
		case 0:
			if(_set) (*Options::General)["general_settings"]["program_folder"].as<string>() = newContent;
			ret = QString::fromStdString( (*Options::General)["general_settings"]["program_folder"].as<string>() );
			break;
		case 14:
			if(_set) (*Options::General)["runtime"]["memory"]["temporary_folder_path"].as<string>() = newContent;
			ret = QString::fromStdString( (*Options::General)["runtime"]["memory"]["temporary_folder_path"].as<string>() );
			break;
		case 17:
			if(_set) (*Options::General)["runtime"]["compression"]["format"].as<string>() = newContent;
			ret = QString::fromStdString( (*Options::General)["runtime"]["compression"]["format"].as<string>() );
			break;
		case 42:
			if(_set) (*Options::General)["classification"]["standard_negative_class"].as<string>() = newContent;
			ret = QString::fromStdString( (*Options::General)["classification"]["standard_negative_class"].as<string>() );
			break;
		default: ret="";
	}

	SceneHandler::setupOptions();
	ObjectHandler::setupOptions();
	return ret;
}


Q_INVOKABLE void Runner::QtInterface::resetSettingsToApplication()
{
	Options::resetGeneralSettings();
	emit generalSettingsChange();
}


Q_INVOKABLE void Runner::QtInterface::resetSettingsToUser()
{
	Options::resetUserSettings();
	emit generalSettingsChange();
}


Q_INVOKABLE void Runner::QtInterface::saveSettings()
{
	Options::save_options();
}


Q_INVOKABLE void Runner::QtInterface::read()
{
    pLink->read();
}


Q_INVOKABLE void Runner::QtInterface::stop()
{
	pLink->stop();
}


Q_INVOKABLE void Runner::QtInterface::reinitialize()
{
	removeDescriptorCreator();
	pLink->reinitialize();
}


Q_INVOKABLE void Runner::QtInterface::record()
{
	pLink->record();
}


Q_INVOKABLE void Runner::QtInterface::stopRecording()
{
	pLink->stopRecording();
}


Q_INVOKABLE void Runner::QtInterface::clearVideoBuffer()
{
	pLink->clearVideoBuffer();
}


Q_INVOKABLE void Runner::QtInterface::saveVideo( QString path )
{
	if( pLink->saveVideo( path ) ) postMessage( "Molar message", "The recorded video was successfully saved." );
	else postMessage( "Molar message", "The video couldn't be saved. Did you actually record anything?" );
}


Q_INVOKABLE void Runner::QtInterface::takeSnapshot()
{
	pLink->takeSnapshot();
}


Q_INVOKABLE void Runner::QtInterface::saveSnapshot( QString path )
{
	if( pLink->saveSnapshot( path ) ) postMessage( "Molar message", "The snapshot has been successfully taken and saved." );
	else postMessage( "Molar message", "There was an error taking the snapshot and it couldn't be saved.");
}


Q_INVOKABLE void Runner::QtInterface::saveProject( QString path )
{
	if( pLink->saveProject(path) ) postMessage( "Molar message", "The project configuration has been successfully saved." );
	else postMessage( "Molar message", "For some reason the configuration could not be saved.");
}


Q_INVOKABLE void Runner::QtInterface::openProject( QString _path )
{
	if( pLink->openProject(_path) )  postMessage( "Molar message", "The project configuration was successfully loaded." );
	else postMessage( "Molar message", "The chosen configuration couldn't be loaded.");
	
}


Q_INVOKABLE void Runner::QtInterface::createNewProject()
{
	pLink->createNewProject();
}



Q_INVOKABLE void Runner::QtInterface::newCamSource( QString _cameraId )
{
	int cameraId = _cameraId.toInt();
	
	if( pLink->newCamSource(cameraId) )  postMessage( "Molar message", "Camera source was loaded successfully." );
	else postMessage( "Molar message", "Camera source couldn't be opened. Attempting to restore previous state.");
}


Q_INVOKABLE void Runner::QtInterface::newFileSource( QString _path )
{	
	if( pLink->newFileSource(_path) )  postMessage( "Molar message", "File was loaded successfully." );
	else postMessage( "Molar message", "File couldn't be loaded. Attempting to restore previous state.");
}


Q_INVOKABLE void Runner::QtInterface::exportData( QList<QString> _objectsForExport, int _timeSepType, QString _timestepSeparator, QString _dataSeparator, QString _path )
{
	bool success;

	switch( _timeSepType)
	{
		case 0: success = pLink->exportData( _objectsForExport, "\n", _dataSeparator, _path ); break;
		case 1: success = pLink->exportData( _objectsForExport, "\t", _dataSeparator, _path ); break;
		case 2: success = pLink->exportData( _objectsForExport, _timestepSeparator, _dataSeparator, _path ); break;
	}
	


	if( success ) postMessage( "Data export", "Data files have been created successfully." );
	else postMessage( "Data export", "A problem occured while exporting data. At least part of the operation has failed.");
}


Q_INVOKABLE QString Runner::QtInterface::frameSize()
{
	if( pLink->pScene==NULL ) return QString();
	Mat frame = pLink->pScene->last();
	stringstream width,height;
	width<<frame.cols;
	height<<frame.rows;
	return QString::fromStdString( width.str() ) + "x" + QString::fromStdString( height.str() );
}


Q_INVOKABLE int Runner::QtInterface::nrOfAlgorithms()
{
	return IPAlgorithm::numberOfAlgorithms();
}


Q_INVOKABLE QString Runner::QtInterface::algorithmName( int _idx )
{
	Ptr<IPAlgorithm> testInstance = IPAlgorithm::createAlgorithm( _idx );

	if( testInstance==NULL ) return "";

	return QString::fromStdString( testInstance->algorithmName() );
}


Q_INVOKABLE QString Runner::QtInterface::algorithmDescription( int _idx )
{
	Ptr<IPAlgorithm> testInstance = IPAlgorithm::createAlgorithm( _idx );

	if( testInstance==NULL ) return "";

	return QString::fromStdString( testInstance->description() );
}


Q_INVOKABLE int Runner::QtInterface::nrOfPreProcessAlgorithms()
{
	if( pLink->pScene==NULL ) return 0;
	return pLink->pScene->pPreProcessStack.size();
}


Q_INVOKABLE QString Runner::QtInterface::preProcessAlgorithmName( int _idx )
{
	if( pLink->pScene==NULL ) return "";

    list<Ptr<IPAlgorithm> >::iterator it,end;
	int i=0;
	for( it=pLink->pScene->pPreProcessStack.begin(); it!=pLink->pScene->pPreProcessStack.end(); it++,i++ )
	{
		if( i==_idx )
		{
			return QString::fromStdString( (*it)->algorithmName() );
		}
	}

	return "";
}


Q_INVOKABLE void Runner::QtInterface::addPreProcessAlgorithm( QString _algorithmName )
{
	if( pLink->pScene==NULL ) return;
	pLink->pSceneLock.lock();
	pLink->pScene->addPreProcessingAlgorithm( _algorithmName.toStdString() );
	pLink->pSceneLock.unlock();
	emit preProcessAlgorithmChange();
	return;
}


Q_INVOKABLE void Runner::QtInterface::removePreProcessAlgorithm( int _idx )
{
	if( pLink->pScene==NULL ) return;

	pLink->pSceneLock.lock();
	pLink->pScene->removePreProcessingAlgorithm( _idx );
	pLink->pSceneLock.unlock();
	emit preProcessAlgorithmChange();
	return;
}

Q_INVOKABLE void Runner::QtInterface::movePreProcessAlgorithmUp( int _idx )
{
	if( pLink->pScene==NULL ) return;

	pLink->pSceneLock.lock();
	pLink->pScene->movePreProcessAlgorithmUp(_idx);
	pLink->pSceneLock.unlock();
	emit preProcessAlgorithmChange();
	return;
}

Q_INVOKABLE void Runner::QtInterface::movePreProcessAlgorithmDown( int _idx )
{
	if( pLink->pScene==NULL ) return;

	pLink->pSceneLock.lock();
	pLink->pScene->movePreProcessAlgorithmDown(_idx);
	pLink->pSceneLock.unlock();
	emit preProcessAlgorithmChange();
	return;
}

Q_INVOKABLE void Runner::QtInterface::addInternPreProcessAlgorithm( QString _algorithmName )
{
	if( pLink->pScene==NULL ) return;

	pLink->pSceneLock.lock();
	pLink->pScene->addInternPreProcessingAlgorithm( _algorithmName.toStdString() );
	pLink->pSceneLock.unlock();
	emit internPreProcessAlgorithmChange();
	return;
}

Q_INVOKABLE void Runner::QtInterface::removeInternPreProcessingAlgorithm( int _idx )
{
	if( pLink->pScene==NULL ) return;

	pLink->pSceneLock.lock();
	pLink->pScene->removeInternPreProcessingAlgorithm(_idx);
	pLink->pSceneLock.unlock();
	emit internPreProcessAlgorithmChange();
	return;
}

Q_INVOKABLE void Runner::QtInterface::moveInternPreProcessAlgorithmUp( int _idx )
{
	if( pLink->pScene==NULL ) return;

	pLink->pSceneLock.lock();
	pLink->pScene->moveInternPreProcessAlgorithmUp(_idx);
	pLink->pSceneLock.unlock();
	emit internPreProcessAlgorithmChange();
	return;
}

Q_INVOKABLE void Runner::QtInterface::moveInternPreProcessAlgorithmDown( int _idx )
{
	if( pLink->pScene==NULL ) return;

	pLink->pSceneLock.lock();
	pLink->pScene->moveInternPreProcessAlgorithmDown(_idx);
	pLink->pSceneLock.unlock();
	emit internPreProcessAlgorithmChange();
	return;
}


Q_INVOKABLE int Runner::QtInterface::nrOfInternPreProcessAlgorithms()
{
	if( pLink->pScene==NULL ) return 0;
	return pLink->pScene->pInternPreProcessStack.size();
}


Q_INVOKABLE QString Runner::QtInterface::internPreProcessAlgorithmName( int _idx )
{
	if( pLink->pScene==NULL ) return "";

    list<Ptr<IPAlgorithm> >::iterator it,end;
	int i=0;
	for( it=pLink->pScene->pInternPreProcessStack.begin(); it!=pLink->pScene->pInternPreProcessStack.end(); it++,i++ )
	{
		if( i==_idx )
		{
			return QString::fromStdString( (*it)->algorithmName() );
		}
	}

	return "";
}


Ptr<IPAlgorithm> Runner::QtInterface::getAlgorithm( int _algorithmIdx, bool _fromPreProcess )
{
	if( pLink->pScene==NULL || _algorithmIdx<0 ) return NULL;

	list< Ptr<IPAlgorithm> >* processStack;
	if( _fromPreProcess ) processStack = &(pLink->pScene->pPreProcessStack);
	else processStack = &(pLink->pScene->pInternPreProcessStack);

    if( (unsigned int)_algorithmIdx>=(*processStack).size() || _algorithmIdx<0 ) return NULL;

	list< Ptr<IPAlgorithm> >::iterator it, end;
	it = (*processStack).begin();
	end = (*processStack).end();
	int i = 0;

	for( ; it!=end ; it++, i++ )
	{
		if( i==_algorithmIdx ) return (*it);
	}
	return NULL;
}


Q_INVOKABLE void Runner::QtInterface::setParameterValue( int _algorithmIdx, bool _isPreProcess, QString _parameterValue, QString _parameterName )
{
	pLink->pSceneLock.lock();

	Ptr<IPAlgorithm> algorithm = getAlgorithm( _algorithmIdx, _isPreProcess );
	string parameterName = _parameterName.toStdString();

	if( algorithm==NULL )
	{
		pLink->pSceneLock.unlock();
		emit algorithmParameterChange( _algorithmIdx, _isPreProcess, _parameterName );
		return;
	}

	GenericMultiLevelMap<string> oldOptions;
	algorithm->getOptions( oldOptions );
	
	if( !oldOptions.hasKey(parameterName) )
	{
		pLink->pSceneLock.unlock();
		emit algorithmParameterChange( _algorithmIdx, _isPreProcess, _parameterName );
		return;
	}
	GenericMultiLevelMap<string> oldParam = oldOptions[parameterName];
	GenericMultiLevelMap<string> newParam;
	bool* ok = new bool(true);

	if( oldParam.is<string>() ) newParam[parameterName].as<string>() = _parameterValue.toStdString();
	else if( oldParam.is<bool>() ) newParam[parameterName].as<bool>() = _parameterValue.toInt(ok);
	else if( oldParam.is<int>() ) newParam[parameterName].as<int>() = _parameterValue.toInt(ok);
	else if( oldParam.is<unsigned int>() ) newParam[parameterName].as<unsigned int>() = _parameterValue.toUInt(ok);
	else if( oldParam.is<double>() ) newParam[parameterName].as<double>() = _parameterValue.toDouble(ok);
	else if( oldParam.is<float>() ) newParam[parameterName].as<float>() = _parameterValue.toFloat(ok);
	else if( oldParam.is<char>() ) newParam[parameterName].as<char>() = _parameterValue.toInt(ok);

	if( !*ok )
	{
		pLink->pSceneLock.unlock();
		emit algorithmParameterChange( _algorithmIdx, _isPreProcess, _parameterName );
		return;
	}
	algorithm->setOptions( newParam );

	delete ok;

	pLink->pSceneLock.unlock();
	emit algorithmParameterChange( _algorithmIdx, _isPreProcess, _parameterName );
	return;
}

Q_INVOKABLE QString Runner::QtInterface::getParameterValue( int _algorithmIdx, bool _isPreProcess, QString _parameterName )
{
	Ptr<IPAlgorithm> algorithm = getAlgorithm( _algorithmIdx, _isPreProcess );
	string parameterName = _parameterName.toStdString();

	if( algorithm==NULL ) return QString();

	GenericMultiLevelMap<string> options;
	algorithm->getOptions( options );
	
	if( !options.hasKey(parameterName) ) return QString();
	
	GenericMultiLevelMap<string> param = options[parameterName];

	if( param.is<string>() ) return QString::fromStdString( param.as<string>() );
	else if( param.is<bool>() ) return QString::number( param.as<bool>() );
	else if( param.is<int>() ) return QString::number( param.as<int>() );
	else if( param.is<unsigned int>() ) return QString::number( param.as<unsigned int>() );
	else if( param.is<double>() ) return QString::number( param.as<double>() );
	else if( param.is<float>() ) return QString::number( param.as<float>() );
	else if( param.is<char>() ) return QString::number( param.as<char>() );

	return QString();
}

Q_INVOKABLE bool Runner::QtInterface::filterActive( int _algorithmIdx, bool _isPreProcess, bool _set, bool _newValue )
{
	Ptr<IPAlgorithm> algorithm = getAlgorithm( _algorithmIdx, _isPreProcess );
	if( algorithm==NULL ) return false;

	if( _set ) algorithm->active = _newValue;

	return algorithm->active;
}

Q_INVOKABLE QList<QString> Runner::QtInterface::parameterNames( int _algorithmIdx, bool _isPreProcess )
{
	Ptr<IPAlgorithm> algorithm = getAlgorithm( _algorithmIdx, _isPreProcess );
	
	QList<QString> list;
	if( algorithm==NULL ) return list;

	GenericMultiLevelMap<string> options;
	algorithm->getOptions( options );

	GenericMultiLevelMap<string>::iterator it, end;
	it = options.begin();
	end = options.end();

	for( ; it!=end; it++ )
	{
		list.push_back( QString::fromStdString( (*it).first ) );
	}

	return list;
}


Q_INVOKABLE QList<QString> Runner::QtInterface::parameterValues( int _algorithmIdx, bool _isPreProcess )
{
	Ptr<IPAlgorithm> algorithm = getAlgorithm( _algorithmIdx, _isPreProcess );

	QList<QString> list;
	if( algorithm==NULL ) return list;

	GenericMultiLevelMap<string> options;
	algorithm->getOptions( options );

	GenericMultiLevelMap<string>::iterator it, end;
	it = options.begin();
	end = options.end();

	for( ; it!=end; it++ )
	{
		if( (*it).second.is<std::string>() ) list.push_back( QString::fromStdString( (*it).second.as<string>() ) );
		else if( (*it).second.is<bool>() ) list.push_back( QString::number( (*it).second.as<bool>() ) );
		else if( (*it).second.is<int>() ) list.push_back( QString::number( (*it).second.as<int>() ) );
		else if( (*it).second.is<unsigned int>() ) list.push_back( QString::number( (*it).second.as<unsigned int>() ) );
		else if( (*it).second.is<double>() ) list.push_back( QString::number( (*it).second.as<double>() ) );
		else if( (*it).second.is<float>() ) list.push_back( QString::number( (*it).second.as<float>() ) );
		else if( (*it).second.is<char>() ) list.push_back( QString( (*it).second.as<char>() ) );
	}
	return list;
}


Q_INVOKABLE QList<QString> Runner::QtInterface::parameterDescriptions( int _algorithmIdx, bool _isPreProcess )
{
	
	Ptr<IPAlgorithm> algorithm = getAlgorithm( _algorithmIdx, _isPreProcess );

	QList<QString> list;
	if( algorithm==NULL ) return list;

	GenericMultiLevelMap<string> options;
	algorithm->getOptions( options );

	GenericMultiLevelMap<string>::iterator it, end;
	it = options.begin();
	end = options.end();

	for( ; it!=end; it++ )
	{
		if( (*it).second.hasKey("info") ) list.push_back( QString::fromStdString( (*it).second["info"].as<string>() ) );
		else list.push_back( QString::fromStdString("") );
	}
	return list;
}


Q_INVOKABLE QList<bool> Runner::QtInterface::parameterIsBool( int _algorithmIdx, bool _isPreProcess )
{
	Ptr<IPAlgorithm> algorithm = getAlgorithm( _algorithmIdx, _isPreProcess );

	QList<bool> list;
	if( algorithm==NULL ) return list;

	GenericMultiLevelMap<string> options;
	algorithm->getOptions( options );

	GenericMultiLevelMap<string>::iterator it, end;
	it = options.begin();
	end = options.end();

	for( ; it!=end; it++ )
	{
		list.push_back( (*it).second.is<bool>() );
	}
	return list;
}


Q_INVOKABLE QList<bool> Runner::QtInterface::parameterIsInt( int _algorithmIdx, bool _isPreProcess )
{
	Ptr<IPAlgorithm> algorithm = getAlgorithm( _algorithmIdx, _isPreProcess );

	QList<bool> list;
	if( algorithm==NULL ) return list;

	GenericMultiLevelMap<string> options;
	algorithm->getOptions( options );

	GenericMultiLevelMap<string>::iterator it, end;
	it = options.begin();
	end = options.end();

	for( ; it!=end; it++ )
	{
		list.push_back( (*it).second.is<int>() );
	}
	return list;
}


Q_INVOKABLE QList<bool> Runner::QtInterface::parameterIsDouble( int _algorithmIdx, bool _isPreProcess )
{
	Ptr<IPAlgorithm> algorithm = getAlgorithm( _algorithmIdx, _isPreProcess );

	QList<bool> list;
	if( algorithm==NULL ) return list;

	GenericMultiLevelMap<string> options;
	algorithm->getOptions( options );

	GenericMultiLevelMap<string>::iterator it, end;
	it = options.begin();
	end = options.end();

	for( ; it!=end; it++ )
	{
		list.push_back( (*it).second.is<double>() );
	}
	return list;
}


Q_INVOKABLE QList<QString> Runner::QtInterface::typesOccuringInScene()
{
	QList<QString> list;

    for( size_t i=0; i<SceneObject::objectList->size(); i++ )
	{
		if( pLink->pScene->pObjectSet.typeIsInScene(i) ) list.push_back( QString::fromStdString((*SceneObject::objectList)[i]) );
	}
	
	return list;
}


Q_INVOKABLE QList<QString> Runner::QtInterface::getNotAddedTypeList()
{
	QList<QString> list;

    for( size_t i=0; i<SceneObject::objectList->size(); i++ )
	{
		if( !pLink->pScene->pObjectSet.typeIsInScene(i) ) list.push_back( QString::fromStdString((*SceneObject::objectList)[i]) );
	}
	return list;
}


Q_INVOKABLE QList<QString> Runner::QtInterface::getNotAddedTypeIds()
{
	QList<QString> list;

    for( size_t i=0; i<SceneObject::objectList->size(); i++ )
	{
		if( !pLink->pScene->pObjectSet.typeIsInScene(i) ) list.push_back( QString::number(i) );
	}
	return list;
}


Q_INVOKABLE QList<QString> Runner::QtInterface::getNotAddedTypeInfo()
{
	QList<QString> list;

    for( size_t i=0; i<SceneObject::objectList->size(); i++ )
	{
		if( !pLink->pScene->pObjectSet.typeIsInScene(i) )
		{
			if( GenericObject::isGenericType(i) )
			{
				list.push_back( QString::fromStdString( (*GenericObject::genericObjectClasses[ GenericObject::genericId(i) ]).info ) );
			}
			else list.push_back( "There is no information available for the chosen type." );
		}
	}
	return list;
}


Q_INVOKABLE QList<QString> Runner::QtInterface::getNotAddedDynamicsType()
{
	QList<QString> list;

    for( size_t i=0; i<SceneObject::objectList->size(); i++ )
	{
		if( !pLink->pScene->pObjectSet.typeIsInScene(i) )
		{
			if( GenericObject::isGenericType(i) )
			{
				list.push_back( QString::fromStdString( (*GenericObject::genericObjectClasses[ GenericObject::genericId(i) ]).dynamicsType ) );
			}
			else list.push_back( "No information available." );
		}
	}
	return list;
}


Q_INVOKABLE QList<QString> Runner::QtInterface::getNotAddedCreationDate()
{
	QList<QString> list;

    for( size_t i=0; i<SceneObject::objectList->size(); i++ )
	{
		if( !pLink->pScene->pObjectSet.typeIsInScene(i) )
		{
			if( GenericObject::isGenericType(i) )
			{
				list.push_back( QString::fromStdString( (*GenericObject::genericObjectClasses[ GenericObject::genericId(i) ]).creationDate ) );
			}
			else list.push_back( "No information available." );
		}
	}
	return list;
}


Q_INVOKABLE QList<bool> Runner::QtInterface::getNotAddedIsGeneric()
{
	QList<bool> list;

    for( size_t i=0; i<SceneObject::objectList->size(); i++ )
	{
		if( !pLink->pScene->pObjectSet.typeIsInScene(i) )
		{
			list.push_back( GenericObject::isGenericType(i) );
		}
	}
	return list;
}


Q_INVOKABLE QList<QString> Runner::QtInterface::getTypeFeatureSetNames( int _classId )
{
	QList<QString> list;

	if( GenericObject::isGenericType(_classId) )
	{
		Ptr<GenericObject::GOData> typeInfo = GenericObject::genericObjectClasses[ GenericObject::genericId(_classId) ];
        for( size_t i=0; i<(*typeInfo).featureSets.size(); i++ )
		{
			Ptr<GenericObject::FeatureData> featureInfo = GenericObject::getFeatureInfo( (*typeInfo).featureSets[i] );
			if( featureInfo==NULL ) continue;
			list.push_back( QString::fromStdString( featureInfo->name ) );
		}
	}

	return list;
}


Q_INVOKABLE QList<QString> Runner::QtInterface::getTypeFeatureSetIds( int _classId )
{
	QList<QString> list;

	if( GenericObject::isGenericType(_classId) )
	{
		Ptr<GenericObject::GOData> typeInfo = GenericObject::genericObjectClasses[ GenericObject::genericId(_classId) ];
        for( size_t i=0; i<(*typeInfo).featureSets.size(); i++ )
		{
			Ptr<GenericObject::FeatureData> featureInfo = GenericObject::getFeatureInfo( (*typeInfo).featureSets[i] );
			if( featureInfo==NULL ) continue; // not the optimal way, but ensures that it is only added when found
			list.push_back( QString::fromStdString( (*typeInfo).featureSets[i] ) );
		}
	}

	return list;
}


Q_INVOKABLE QList<QString> Runner::QtInterface::getTypeFeatureSetInfos( int _classId )
{
	QList<QString> list;

	if( GenericObject::isGenericType(_classId) )
	{
		Ptr<GenericObject::GOData> typeInfo = GenericObject::genericObjectClasses[ GenericObject::genericId(_classId) ];
        for( size_t i=0; i<(*typeInfo).featureSets.size(); i++ )
		{
			Ptr<GenericObject::FeatureData> featureInfo = GenericObject::getFeatureInfo( (*typeInfo).featureSets[i] );
			if( featureInfo==NULL ) continue;
			list.push_back( QString::fromStdString( featureInfo->info ) );
		}
	}

	return list;
}


Q_INVOKABLE QList<QString> Runner::QtInterface::getTypeFeatureSetCreationDates( int _classId )
{
	QList<QString> list;

	if( GenericObject::isGenericType(_classId) )
	{
		Ptr<GenericObject::GOData> typeInfo = GenericObject::genericObjectClasses[ GenericObject::genericId(_classId) ];
        for( size_t i=0; i<(*typeInfo).featureSets.size(); i++ )
		{
			Ptr<GenericObject::FeatureData> featureInfo = GenericObject::getFeatureInfo( (*typeInfo).featureSets[i] );
			if( featureInfo==NULL ) continue;
			list.push_back( QString::fromStdString( featureInfo->creationDate ) );
		}
	}

	return list;
}


Q_INVOKABLE QList<QString> Runner::QtInterface::getTypeFeatureSetSize( int _classId )
{
	QList<QString> list;

	if( GenericObject::isGenericType(_classId) )
	{
		Ptr<GenericObject::GOData> typeInfo = GenericObject::genericObjectClasses[ GenericObject::genericId(_classId) ];
        for( size_t i=0; i<(*typeInfo).featureSets.size(); i++ )
		{
			Ptr<GenericObject::FeatureData> featureInfo = GenericObject::getFeatureInfo( (*typeInfo).featureSets[i] );
			if( featureInfo==NULL ) continue;
			list.push_back( QString::number( featureInfo->numberOfFeaturePoints ) );
		}
	}

	return list;
}


Q_INVOKABLE QList<QString> Runner::QtInterface::getAllFeatureSetNames()
{
	QList<QString> list;

    for( size_t i=0; i<GenericObject::availableFeatureSets.size(); i++ )
	{
		list.push_back( QString::fromStdString( GenericObject::availableFeatureSets[i]->name ) );
	}


	return list;
}


Q_INVOKABLE QList<QString> Runner::QtInterface::getAllFeatureSetIds()
{
	QList<QString> list;

    for( size_t i=0; i<GenericObject::availableFeatureSets.size(); i++ )
	{
		list.push_back( QString::fromStdString( GenericObject::availableFeatureSets[i]->fileName ) );
	}

	return list;
}


Q_INVOKABLE QList<QString> Runner::QtInterface::getAllFeatureSetInfos()
{
	QList<QString> list;

    for( size_t i=0; i<GenericObject::availableFeatureSets.size(); i++ )
	{
		list.push_back( QString::fromStdString( GenericObject::availableFeatureSets[i]->info ) );
	}

	return list;
}


Q_INVOKABLE QList<QString> Runner::QtInterface::getAllFeatureSetCreationDates()
{
	QList<QString> list;

    for( size_t i=0; i<GenericObject::availableFeatureSets.size(); i++ )
	{
		list.push_back( QString::fromStdString( GenericObject::availableFeatureSets[i]->creationDate ) );
	}

	return list;
}


Q_INVOKABLE QList<QString> Runner::QtInterface::getAllFeatureSetSize()
{
	QList<QString> list;

    for( size_t i=0; i<GenericObject::availableFeatureSets.size(); i++ )
	{
		list.push_back( QString::number( GenericObject::availableFeatureSets[i]->numberOfFeaturePoints ) );
	}

	return list;
}


Q_INVOKABLE QList<QString> Runner::QtInterface::availableDynamics()
{
	QList<QString> list;
	vector<string> dynamics;
	GenericObject::Dynamics::availableDynamics( dynamics );

    for( size_t i=0; i<dynamics.size(); i++ )
	{
		list.push_back( QString::fromStdString( dynamics[i] ) );
	}

	return list;
}


Q_INVOKABLE QList<QString> Runner::QtInterface::dynamicsInfo()
{
	QList<QString> list;
	vector<string> dynamics;
	GenericObject::Dynamics::dynamicsInfo( dynamics );

    for( size_t i=0; i<dynamics.size(); i++ )
	{
		list.push_back( QString::fromStdString( dynamics[i] ) );
	}

	return list;
}


Q_INVOKABLE bool Runner::QtInterface::createType( QString _name, QString _colR, QString _colG, QString _colB, QString _dynamicsModule , QList<QString> _featureSet , QString _info )
{
	string name = _name.toStdString();
	Scalar color( _colB.toInt(), _colG.toInt(), _colR.toInt() );
	string dynamics = _dynamicsModule.toStdString();
	vector<string> featureSets;
	for( int i=0; i<_featureSet.size(); i++ )
	{
		featureSets.push_back( _featureSet[i].toStdString() );
	}
	string info = _info.toStdString();

	GenericObject::GOData* newTypeDefinition = new GenericObject::GOData( name, color, dynamics ,featureSets ); // control over the dynamic memory is given to a openCV Ptr<> object on call of the registerClass() function
	newTypeDefinition->info = info;

	pLink->lock();
	bool success = newTypeDefinition->registerClass();
	pLink->unlock();

	emit sceneTypeChange();
	return success;
}


Q_INVOKABLE bool Runner::QtInterface::addTypeToScene( int _classId, QList<QString> _activeFeatures )
{
	bool isGeneric = GenericObject::isGenericType(_classId );

	if( isGeneric && _activeFeatures.size()==0 ) return false;

	vector<string> activeFeatures;

	for( int i=0; i<_activeFeatures.size(); i++ )
	{
        activeFeatures.push_back( _activeFeatures[i].toStdString() );
	}

	vector<string> types;
    for( size_t i=0; i<SceneObject::objectList->size(); i++ )
	{
        if( pLink->pScene->pObjectSet.typeIsInScene(i) || (i==(unsigned int)_classId && _classId>=0) ) types.push_back( (*SceneObject::objectList)[i] );
	}

	pLink->lock();
	
	if( isGeneric) GenericObject::genericObjectClasses[ GenericObject::genericId(_classId) ]->setFeaturesActive( activeFeatures );
	pLink->pScene->typesInScene( types );

	pLink->reinitialize(true);
	pLink->unlock();
	emit sceneTypeChange();

	return pLink->pScene->pObjectSet.typeIsInScene(_classId);
}



Q_INVOKABLE void Runner::QtInterface::removeTypeFromScene( QString _name )
{
	vector<string> types;

	string toRemove = _name.toStdString();

    for( size_t i=0; i<SceneObject::objectList->size(); i++ )
	{
		if( pLink->pScene->pObjectSet.typeIsInScene(i) && (*SceneObject::objectList)[i]!=toRemove ) types.push_back( (*SceneObject::objectList)[i] );
	}
	
	pLink->lock();
	pLink->pScene->typesInScene( types );
	pLink->reinitialize(true);
	pLink->unlock();
	emit sceneTypeChange();

	return;
}


Q_INVOKABLE void Runner::QtInterface::recordFeatures()
{
	createDescriptorCreator();

	pDescriptorCreator->record(); // should already be threadsafe

	return;
}

Q_INVOKABLE void Runner::QtInterface::stopRecordingFeatures()
{
	createDescriptorCreator();

	pDescriptorCreator->stop(); // should already be threadsafe

	return;
}


Q_INVOKABLE void Runner::QtInterface::clearFeatures()
{
	removeDescriptorCreator();
	createDescriptorCreator();

	return;
}


Q_INVOKABLE unsigned int Runner::QtInterface::nrOfRecordedFeatures()
{
	createDescriptorCreator();

	return pDescriptorCreator->dataEntrySize();
}


Q_INVOKABLE unsigned int Runner::QtInterface::getLowestObjId()
{
	pLink->lock();
	unsigned int lowestId = pLink->pScene->objects().lowestActiveId();
	pLink->unlock();
	return lowestId;
}


Q_INVOKABLE unsigned int Runner::QtInterface::getHighestObjId()
{
	pLink->lock();
	unsigned int highestId = pLink->pScene->objects().highestActiveId();
	pLink->unlock();
	return highestId;
}


Q_INVOKABLE void Runner::QtInterface::recordObjFeatures( unsigned int _objId )
{
	createDescriptorCreator();
	pDescriptorCreator->recordObject( _objId );
}


Q_INVOKABLE void Runner::QtInterface::stopRecordingObjFeatures( unsigned int _objId )
{
	createDescriptorCreator();
	pDescriptorCreator->removeObject( _objId );
}


Q_INVOKABLE void Runner::QtInterface::clearRecordingObj()
{
	createDescriptorCreator();
	pDescriptorCreator->clear();
}


Q_INVOKABLE bool Runner::QtInterface::createFeatureSet( QString _featureName, QString _featureDescription )
{
	createDescriptorCreator();
	pDescriptorCreator->stop();

	pDescriptorCreator->name = _featureName.toStdString();
	pDescriptorCreator->info = _featureDescription.toStdString();

	return pDescriptorCreator->create();
}


void Runner::QtInterface::postError( string _message )
{
	emit error( QString::fromStdString(_message) );
}


Q_INVOKABLE void Runner::QtInterface::postMessage( string _title, string _message )
{
	emit message( QString::fromStdString(_title), QString::fromStdString(_message) );
}

void Runner::QtInterface::signalNewSource()
{

	string source;
	if( !(*Options::General)["automatically_updated_program_data"]["last_stream_source"]["exists"].as<bool>() )
	{
		source = "No source";
	}
	else
	{
		if( (*Options::General)["automatically_updated_program_data"]["last_stream_source"]["is_cam"].as<bool>() )
		{
			stringstream converter;
			converter<<(*Options::General)["automatically_updated_program_data"]["last_stream_source"].as<int>();
			source = "Camera device "+converter.str();
		}
		else
		{
			boost::filesystem::path videoPath( (*Options::General)["automatically_updated_program_data"]["last_stream_source"].as<string>() );
			source = "Video "+videoPath.filename().string();
		}
	}
	emit newSource( QString::fromStdString(source) );
}


Q_INVOKABLE void Runner::QtInterface::signalReload()
{
	if( pLink->pVideo!=NULL )
	{
		pLink->pStream->updatesDone();
		signalNewSource();
	}
	emit generalSettingsChange();
	if( pLink->pScene!=NULL )
	{
		emit preProcessAlgorithmChange();
		emit internPreProcessAlgorithmChange();
		emit sceneTypeChange();
	}
}


void Runner::QtInterface::createDescriptorCreator()
{
	if( pDescriptorCreator!=NULL ) return;

	pLink->lock();

    Ptr<vector<unsigned int> > objToRecord = new vector<unsigned int>();
	pDescriptorCreator = pLink->pScene->objects().newDescriptorCreator( objToRecord );

	pLink->unlock();
	return;
}


void Runner::QtInterface::removeDescriptorCreator()
{
	if( pDescriptorCreator==NULL ) return;

	pLink->lock();

	pDescriptorCreator->stop();
	pDescriptorCreator->unlink();

	pLink->unlock();

	pDescriptorCreator = NULL;
	return;
}
