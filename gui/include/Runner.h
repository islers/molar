#pragma once

#include <QApplication>
#include <QQmlApplicationEngine>
#include <qdeclarativeview.h>
#include <qicon.h>
#include <qthread.h>
#include "cvmatdisplay.h"
#include "SceneHandler.h"
#include "cvmatdisplay.h"
#include <iostream>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

class Runner: public QThread
{
public:
	friend class CVMatDisplay;

	class QtInterface;

	Runner();
	~Runner();

	void putStream( CVMatDisplay* _stream );

	/** returns the cvmatdisplay object - which offers the video stream as provider to qt */
	CVMatDisplay* getCVStream();

	/** returns the qt interface to molar */
	QtInterface* getInterface();

	/** opens a new stream, video or camera */
	bool openNewStream( string _filePath, int _cameraId=-1 );

	/** loads and analyzes a new frame from pVideo */
	bool analyzeNewFrame();

	/** starts reading the stream and operating on it */
	void read();

	/** stops reading the stream */
	void stop();

	/** reinitializes the scene using the current settings, old trajectories are lost */
	void reinitialize( bool _withoutLock=false );

	/** records the edited frames as video */
	void record();

	/** stops recording */
	void stopRecording();

	/** clears the video buffer */
	void clearVideoBuffer();

	/** save video to file */
	bool saveVideo( QString _path );

	/** takes a new snapshot */
	void takeSnapshot();

	/** saves the previously taken snapshot at the given path */
	bool saveSnapshot( QString _path );

	/** stops the thread regularly */
	void stopThread();

	/** saves all project settings */
	bool saveProject( QString _path );

	/** opens and loads a new configuration */
	bool openProject( QString _path );

	/** creates a whole new project */
	void createNewProject();
	
	/** self explanatory...*/
	bool newCamSource( int _cameraId );

	/** new file source */
	bool newFileSource( QString _path );

	/** exports the data of the objects given in the list, output files are named "obj*.txt" */
	bool exportData( QList<QString> _objectsForExport, QString _timestepSeparator, QString _dataSeparator, QString _path );

	/** lock the mutex */
	void lock();

	/** unlock the mutex */
	void unlock();


private:
	void run() Q_DECL_OVERRIDE;

	CVMatDisplay* pStream;
	SceneHandler* pScene;
	VideoCapture* pVideo;
	Mat pSnapshot;
	QtInterface* pInterface;
	VideoWriter* pVideoWriter;
	QMutex pSceneLock;
	bool pRun, pThreadRunning, pRecord;
};



class Runner::QtInterface : public QObject
{
	Q_OBJECT

public:
	QtInterface( Runner* _link );
	~QtInterface();

	void initialize();
	
	Q_INVOKABLE int intSignal( int id, bool _set, int _param=0 );
	Q_INVOKABLE unsigned int uintSignal( int id, bool _set, unsigned int _param=0 );
	Q_INVOKABLE double doubleSignal( int id, bool _set, double _param=0 );
	Q_INVOKABLE bool boolSignal( int id, bool _set, bool _param=false );
	Q_INVOKABLE QString stringSignal( int id, bool _set, QString _param="" );

	Q_INVOKABLE void resetSettingsToApplication();
	Q_INVOKABLE void resetSettingsToUser();
	Q_INVOKABLE void saveSettings();
	Q_INVOKABLE void read();
	Q_INVOKABLE void stop();
	Q_INVOKABLE void reinitialize();
	Q_INVOKABLE void record();
	Q_INVOKABLE void stopRecording();
	Q_INVOKABLE void clearVideoBuffer();
	Q_INVOKABLE void saveVideo( QString path );
	Q_INVOKABLE void takeSnapshot();
	Q_INVOKABLE void saveSnapshot( QString path );
	Q_INVOKABLE void saveProject( QString path );
	Q_INVOKABLE void openProject( QString _path );
	Q_INVOKABLE void createNewProject();
	Q_INVOKABLE void newCamSource( QString _cameraId );
	Q_INVOKABLE void newFileSource( QString _path );
	Q_INVOKABLE void exportData( QList<QString> _objectsForExport, int _timeSepType, QString _timestepSeparator, QString _dataSeparator, QString _path );
	
	Q_INVOKABLE QString frameSize();

	Q_INVOKABLE int nrOfAlgorithms();
	Q_INVOKABLE QString algorithmName( int _idx );
	Q_INVOKABLE QString algorithmDescription( int _idx );

	Q_INVOKABLE int nrOfPreProcessAlgorithms();
	Q_INVOKABLE QString preProcessAlgorithmName( int _idx );
	Q_INVOKABLE void addPreProcessAlgorithm( QString _algorithmName );
	Q_INVOKABLE void removePreProcessAlgorithm( int _idx );
	Q_INVOKABLE void movePreProcessAlgorithmUp( int _idx );
	Q_INVOKABLE void movePreProcessAlgorithmDown( int _idx );
	Q_INVOKABLE int nrOfInternPreProcessAlgorithms();
	Q_INVOKABLE QString internPreProcessAlgorithmName( int _idx );
	Q_INVOKABLE void addInternPreProcessAlgorithm( QString _algorithmName );
	Q_INVOKABLE void removeInternPreProcessingAlgorithm( int _idx );
	Q_INVOKABLE void moveInternPreProcessAlgorithmUp( int _idx );
	Q_INVOKABLE void moveInternPreProcessAlgorithmDown( int _idx );


	/** returns a pointer to the algorithm at position _algorithmIdx of either the preprocess stack (if _fromPreProcess=true) or the intern preprocess stack( _fromPreProcess=false) */
	Ptr<IPAlgorithm> getAlgorithm( int _algorithmIdx, bool _fromPreProcess );

	Q_INVOKABLE void setParameterValue( int _algorithmIdx, bool _isPreProcess, QString _parameterValue, QString _parameterName );
	Q_INVOKABLE QString getParameterValue( int _algorithmIdx, bool _isPreProcess, QString _parameterName );
	Q_INVOKABLE bool filterActive( int _algorithmIdx, bool _isPreProcess, bool _set, bool _newValue=false ); 

	Q_INVOKABLE QList<QString> parameterNames( int _algorithmIdx, bool _isPreProcess );
	Q_INVOKABLE QList<QString> parameterValues( int _algorithmIdx, bool _isPreProcess );
	Q_INVOKABLE QList<QString> parameterDescriptions( int _algorithmIdx, bool _isPreProcess );
	Q_INVOKABLE QList<bool> parameterIsBool( int _algorithmIdx, bool _isPreProcess );
	Q_INVOKABLE QList<bool> parameterIsInt( int _algorithmIdx, bool _isPreProcess );
	Q_INVOKABLE QList<bool> parameterIsDouble( int _algorithmIdx, bool _isPreProcess );

	/** returns a list of the names of all types that are set to be expected to occur in the scene */
	Q_INVOKABLE QList<QString> typesOccuringInScene();
	/** returns a list of all types that are registered with the type factory which are not yet set as occuring in the scene*/
	Q_INVOKABLE QList<QString> getNotAddedTypeList();
	/** returns a list of the class ids corresponding to the names return by getNotAddedTypeList() */
	Q_INVOKABLE QList<QString> getNotAddedTypeIds();
	/** returns a list with info texts corresponding to the names returned by getNotAddedTypeList() */
	Q_INVOKABLE QList<QString> getNotAddedTypeInfo();
	/** returns a list with the dynamics type of the class types corresponding to the names returned by getNotAddedTypeList() */
	Q_INVOKABLE QList<QString> getNotAddedDynamicsType();
	/** returns a list with the creation dates of the class types corresponding to the names returned by getNotAddedTypeList() */
	Q_INVOKABLE QList<QString> getNotAddedCreationDate();
	/** returns if it is a generic type or not */
	Q_INVOKABLE QList<bool> getNotAddedIsGeneric();

	/** returns a list with the names of the features available for a given class */
	Q_INVOKABLE QList<QString> getTypeFeatureSetNames( int _classId );
	/** returns the ids (fileName) of those feature sets */
	Q_INVOKABLE QList<QString> getTypeFeatureSetIds( int _classId );
	/** returns the infos for those feature sets */
	Q_INVOKABLE QList<QString> getTypeFeatureSetInfos( int _classId );
	/** returns the creation dates of those feature sets */
	Q_INVOKABLE QList<QString> getTypeFeatureSetCreationDates( int _classId );
	/** returns the number of included feature points of those feature sets */
	Q_INVOKABLE QList<QString> getTypeFeatureSetSize( int _classId );

	/** returns a list with the names of all features available*/
	Q_INVOKABLE QList<QString> getAllFeatureSetNames();
	/** returns the ids (fileName) of those feature sets */
	Q_INVOKABLE QList<QString> getAllFeatureSetIds();
	/** returns the infos for those feature sets */
	Q_INVOKABLE QList<QString> getAllFeatureSetInfos();
	/** returns the creation dates of those feature sets */
	Q_INVOKABLE QList<QString> getAllFeatureSetCreationDates();
	/** returns the number of included feature points of those feature sets */
	Q_INVOKABLE QList<QString> getAllFeatureSetSize();

	/** returns a list of all available dynamics types */
	Q_INVOKABLE QList<QString> availableDynamics();
	/** returns the information texts of all available dynamic types */
	Q_INVOKABLE QList<QString> dynamicsInfo();


	/** creates a new type definition */
	Q_INVOKABLE bool createType( QString _name, QString _colR, QString _colG, QString _colB, QString _dynamicsModule , QList<QString> _featureSet , QString _info );


	/** adds a new type to the scene with the given options */
	Q_INVOKABLE bool addTypeToScene( int _classId, QList<QString> _activeFeatures );

	/** remove the indicated type from the set of expected types and reinitialize the scene */
	Q_INVOKABLE void removeTypeFromScene( QString _name );

	/** starts recording features */
	Q_INVOKABLE void recordFeatures();
	/** stops recording features */
	Q_INVOKABLE void stopRecordingFeatures();
	/** clear all features */
	Q_INVOKABLE void clearFeatures();
	/** returns the number of so far recorded feature points */
	Q_INVOKABLE unsigned int nrOfRecordedFeatures();
	Q_INVOKABLE unsigned int getLowestObjId();
	Q_INVOKABLE unsigned int getHighestObjId();
	/** add the object with the id _objId to those whose features are to be recorded */
	Q_INVOKABLE void recordObjFeatures( unsigned int _objId );
	/** removes the object with the id _objId from those whose features are to be recorded */
	Q_INVOKABLE void stopRecordingObjFeatures( unsigned int _objId );
	/** clears the list with objects whose features are to be recorded */
	Q_INVOKABLE void clearRecordingObj();
	/** creates the new feature set */
	Q_INVOKABLE bool createFeatureSet( QString _featureName, QString _featureDescription );


	void postError( string _message );
	Q_INVOKABLE void postMessage( string _title, string _message );
	void signalNewSource();
	// signals that a whole new scene has been loaded and thus potentially all settings have changed
	Q_INVOKABLE void signalReload();

private:
	Runner* pLink;
	Ptr<DescriptorCreator> pDescriptorCreator;

	void createDescriptorCreator();
	void removeDescriptorCreator();

signals:
	void generalSettingsChange();
	void error( QString message );
	void message( QString messagetitle, QString message );
	void newSource( QString newsource );
	void preProcessAlgorithmChange();
	void internPreProcessAlgorithmChange();
	void algorithmParameterChange( int sAlgorithmIdx, bool sIsPreProcess, QString sParameterName );
	void sceneTypeChange();
};
