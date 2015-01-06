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
#include "sceneobject.h"
#include "boost/filesystem.hpp"



/** class providing means for flexibly building generic classes with the entities wished and newly trained classifiers */

class GenericObject: public SceneObject
{
	friend class DescriptorCreator;

public:
	class Dynamics; // provides interfaces for dynamics, organizes the state history and provides functions to add new states, for prediction etc
	class GOData; // class to hold information about a specific generic object class
	class FeatureData; // class that holds information about features

protected:
	class ClassifierData;

public:
	GenericObject( ObjectHandler* _environmentControl, int _genericClassId );
	GenericObject( SceneObject* _toCopy, int _genericClassId );
	~GenericObject(void);

	/** returns the class name and color */
	virtual void classProperties( string& _className, Scalar& _classColor );
	virtual Scalar color();
	
	static Ptr<SceneObject> createGenericObject( SceneObject* ); // factory function
	/** finds and loads the classifiers for each class fitting their respective positive/negative feature set descriptions */
	static bool initializeClass( int _classId, ObjectHandler* _environment );
	static void setClassOptions( GenericMultiLevelMap<string>& _options, int _classId );
	static void getClassOptions( GenericMultiLevelMap<string>& _options, int _classId );

	virtual Ptr<SceneObject::State> newState( Mat& _state, RectangleRegion& _region, vector<Point>& _contour, Mat& _invImg, double _time );
	virtual bool addState( Ptr<SceneObject::State> _newState, RectangleRegion& _regionOfInterest, vector<Point>& _contour );

	// STATE ACCESS

	/** prints all time data to the given file, using the given separators, returns true if no exception was thrown inside the function. If the file already exists, then old content is overwritten. */
	virtual bool exportData( string _filePath, string _dataPointSeparator=" ", string _timeStepSeparator="\n" );

	virtual Ptr<SceneObject::State> state(); // see SceneObject class for function descriptions
	virtual Mat history();
	virtual Mat currentState();
	virtual Point pos();
	virtual double angle();
	virtual Point direction();
	virtual double area();
	virtual double velX();
	virtual double velY();
	virtual double velAng();
	virtual double accX();
	virtual double accY();
	virtual double accAng();
	virtual RectangleRegion lastROI();
	virtual void lastContour( vector<Point>& _contour );

	// PREDICTION
	virtual void predictROI( vector<Point>& _roi );
	virtual Mat predictState();
	virtual void resetPrediction();
	virtual Point predictPosition( Point _pt );
	virtual Point predictCtrPosition();
	virtual Point2f predictCtrFPosition();
	virtual double predictXCtrVelocity();
	virtual double predictXCtrAcceleration();
	virtual double predictYCtrVelocity();
	virtual double predictYCtrAcceleration();
	virtual double predictAgl();
	virtual double predictAglVelocity();
	virtual double predictAglAcceleration();
	virtual double predictArea();

	// DRAWING
	virtual bool drawLayer( Mat& _img, unsigned int _layerLevel, Scalar _color );

	// CLASSIFICATION
	static double isType( Mat& _img, RectangleRegion& _boundingRect, Mat& _descriptors, int _id, int _classId );
	
	/** extracts the descriptor set in the area _boundingRect (extended by 5px in all directions -> _boundingRect is altered!) in the image _img
	*	and stores it in _descriptors.
	*
	*	@return: returns true if it created descriptors, false if not (empty descriptor set)
	*/
	static bool createDescriptors( Mat& _img, RectangleRegion& _boundingRect, Mat& _descriptors );
	
	/** recalculates the type of the object based on its class likelihood vector (called after new likelihoods were loaded)
	*
	*	@arg	Ptr<SceneObject> _oldPointer	is added in order to keep the smart pointer alive if the object stays the same
	*
	*	@return		pointer to the object itself -> new instance of child type if the objects' type has changed
	*/
	virtual Ptr<SceneObject> recalculateClass( Ptr<SceneObject> _oldPointer ); // added here because if the object class changes, generic objects need to unregister themselves

	/** finds the object _old in the object registration list and erases the entry */
	static void unregisterObject( Ptr<SceneObject> _old );

	/** initializes a vector with the standard type likelihood function values */
	static void initializeStdTypeLikelihood( vector<double>& _likelihoodVector );

	/** checks if the object is a generic object type */
	static bool isGeneric( Ptr<SceneObject> _obj );
	static bool isGenericType( int _classId );


	// UTILITY FUNCTIONS
	static string timeString(); // returns the actual time in ms - used for unique name creation
	static string readableTime(); // returns the actual time as "Tue Jun 10 02:34:05 2014"

protected:
	// interface to private static object handler elements:
	static const int& path_length();
	static const int& path_fadeout_speed();

protected:
	Ptr<Dynamics> pDynamics;

	Ptr<GOData> pGenericClass; // pointer to the GOData object that describes the generic objects' current type (class)
	int pGenericClassId; // used for a simple check


	// STATIC CLASS METHODS AND MEMBERS ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	static bool isSetup;
	static bool setupClass();

	// classifier objects
    static vector<Ptr<CvBoost> > classifier; // index equivalent to generic class id
	static FastFeatureDetector detector;
	static BRISK extractor;

	
	// generic class factory and information functions


	static void registerClasses();
	
	/** looks for the indicated classifier in the set of existing classifiers and returns it if found, builds and trains it new if it is not found (it then also is stored to hard drive)
	*	
	*	@return		pointer to a trained classifier according to the specifications, NULL pointer if there was an error
	*/
    static Ptr<CvBoost> getClassifier( const vector<Ptr<vector<string> > >& _positiveSets, const vector<Ptr<vector<string> > >& _negativeSets, const string _type="CvBoost");

public:
	/** returns the generic class id of the class with name _gName, -1 if no class with the given name is found
	*/
	static int getGenericClassId( const string _gcName );

	/** transforms the overall class id to the generic class id */
	static int genericId( const int _overallClassId, bool _suppressMessages=false );

	/** transform the generic class id to the overall class id */
	static int overallId( const int _genericId );

    static vector<Ptr<GOData> > genericObjectClasses; // set of user created generic object classes

	//static vector<string> chosenClassifiers; // not implemented: other classifiers but the standard classifiers for the class may be chosen at runtime, not part of GOData object since this is stored with the SceneHandler settings
	
    static vector<Ptr<FeatureData> > availableFeatureSets; // set of available feature descriptors
    static vector<Ptr<ClassifierData> > availableClassifiers; // sot of previously calculated (trained) classifiers

	/** returns the information data that corresponds to the given id, a NULL pointer if no feature set with the given _featureId was found */
	static Ptr<FeatureData> getFeatureInfo( string _featureId );

	static vector<int> genericToOverallClassIds; // registers overall class ids for each generic class: genericToOverallClassIds[overallId]=genId (this more complicated version than its inverse is chosen because this conversion needs to be done more often)
    static vector<list<Ptr<GenericObject> > > genericObjectLists; // lists of the objects that are currently instantiated for each generic object class
    static vector<vector<double> > classTypeLikelihoodFunctions; // describes for each class how likely it is for an object to be of the class type if a certain percentage of descriptors were classified as class descriptors


	static bool factoryInitialized;
	static bool initializeGenericObjectFactory();

	static void setupOptions();
private:
	static bool dynamic_feature_extraction_threshold_adaption; // True: If the FAST keypoint extractor finds no keypoints for an image, it lowers the threshold as long as no keypoints are found
	static int dynamic_feature_adaption_step;// By how much the threshold is lowered for each stelp during dynamic threshold adaption
};



class GenericObject::GOData
{
public:
	/** create new GenericObject class data object */
	GOData( const string _name, const Scalar _color, const string _dynamicsType, const vector<string>& _featureSet, const string _preferredClassifierType="CvBoost" );
	/** load GenericObject class data from file */
	GOData( const string _filePath );

	/** returns the set of all features that are active, if none then the vector returned will be an empty one */
    Ptr<vector<string> > activeFeatureSet();

	/** set feature sets active: if this function isn't called, then all features are set to being active */
	void setFeaturesActive( vector<string>& _activeFeatures );

	/** saves the information to the file: If GOData object was created from file then this file will be overwritten if the name hasn't been altered.
	*	However if the GOData object was newly created (not from file), then it will be checked if the name already exists and if so an automated incrementer is appended to the name.
	*/
	bool save();

	/** registers and saves class in actual setting  (that is, first save() is called, then the class is being registered)*/
	bool registerClass();

	string name;
	Scalar color;
	string info;
	string creationDate;
	string dynamicsType;
	GenericMultiLevelMap<string> dynamicsOptions; // initializing will have to be done over the dynamics own interface
	
	string imagePath; // path to the image
	Mat img; // if set this contains a picture of the object the class represents

	vector<string> featureSets; // feature set filenames (since those should be unique) that describe the object class
	vector<bool> featureActive;

	string preferredClassifierType; // currently unused since no other classifier type but CvBoost is supported
private:
	bool pFromFile;
};



class GenericObject::FeatureData
{
public:
	FeatureData( const string _name, const Mat _featureSet, const Mat _sceneImage ); // constructor for new feature data
	FeatureData( const string _filePath ); // constructor to load old feature data from file

	bool save(); // saves the feature info file
	Mat getFeatureSet(); // loads the actual feature set from file

	string name;
	string info;
	string fileName; // without data type (e.g. ".img" or ".xml") -> is used as identifier
	string creationDate;
	double numberOfFeaturePoints; // number of feature points contained in the set
	string detectorType; // currently always FastFeatureDetector: no functions for using other OpenCV options have been implemented
	string extractorType; // currently always BRISK: no function for using other OpenCV options have been implemented

	Mat img;
private:
	Mat pFeatureSet; // the corresponding feature set: is only actually loaded if "Mat getFeatureSet()" is called, but then kept since one call to the function likely means that another will follow
	
	
};



class GenericObject::ClassifierData
{
public:
	/** creates data for a new classifier (it actually trains a new classifier): the indicated feature sets need to already be added to the "availableFeatureSets" vector of the GenericObject class */
    ClassifierData( const string _classifierType, const vector<Ptr<vector<string> > > & _positiveFeatureSets, const vector<Ptr<vector<string> > >& _negativeFeatureSets );
	ClassifierData( const string _filePath ); // constructor to load old classifier info from file

	/** saves the classifier data to file */
	bool save();

	/** function examines if the classifier is a classifier meeting the given specifications */
    bool isMatch( const string _classifierType, const vector<Ptr<vector<string> > >& _positiveFeatureSets, const vector<Ptr<vector<string> > >& _negativeFeatureSets );

	Ptr<CvBoost> getInitializedClassifier(); // returns an initialized classifier with the correct settings

	string fileName; // without data type (e.g. ".img" or ".xml") -> is used as identifier
	string classifierType;
	vector<string> positiveFeatureSets; // filenames of feature descriptor sets used for positive training
	vector<string> negativeFeatureSets; // filenames of feature descriptor sets used for negative training
private:
	Ptr<CvBoost> pClassifier; // is empty (NULL) if it hasn't been used
};
