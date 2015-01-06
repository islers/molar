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

#include "GenericObject.h"
#include "objecthandler.h"
#include "Dynamics.h"


GenericObject::GenericObject( ObjectHandler* _environmentControl, int _genericClassId ): SceneObject(_environmentControl), pGenericClassId( _genericClassId )
{
	
}


GenericObject::GenericObject( SceneObject* _toCopy, int _genericClassId ): SceneObject( _toCopy ), pGenericClassId( _genericClassId )
{
	pGenericClass = genericObjectClasses[ _genericClassId ];
	pDynamics = Dynamics::createDynamics( pGenericClass->dynamicsType, this );
	pDynamics->setOptions( pGenericClass->dynamicsOptions );
}


GenericObject::~GenericObject(void)
{

}


void GenericObject::classProperties( string& _className, Scalar& _classColor )
{
	_className = pGenericClass->name;
	_classColor = pGenericClass->color;
}


Scalar GenericObject::color()
{
	return pGenericClass->color;
}


Ptr<SceneObject> GenericObject::createGenericObject( SceneObject* _toCopy )
{
	int genId = genericId( _toCopy->type() );
	Ptr<SceneObject> newGO = new GenericObject( _toCopy, genId );
	Ptr<GenericObject> copy(newGO);
	copy.addref();
	genericObjectLists[ genId ].push_back( copy ); // register new generic object with correct class
	
	return newGO;
}


bool GenericObject::initializeClass( int _classId, ObjectHandler* _environment )
{
	int genId = genericId( _classId );

	try
	{

        vector<Ptr<vector<string> > > positiveFeatureSets, negativeFeatureSets;

		// positive feature sets
		positiveFeatureSets.push_back( genericObjectClasses[genId]->activeFeatureSet() );
		// negative feature sets
        for( size_t i=0; i<genericObjectClasses.size(); i++ )
		{
            //(this equals i!=genId && _environment...)
            if( ( i!=(unsigned int)genId || genId<0 ) && _environment->pObjectTypesInScene[ overallId(i) ] ) negativeFeatureSets.push_back( genericObjectClasses[i]->activeFeatureSet() );
		}
		
		// count the number of feature sets
		int nrOfPos = 0;
		int nrOfNeg = 0;
        for( size_t i=0; i<positiveFeatureSets.size(); i++ )
		{
			nrOfPos += positiveFeatureSets[i]->size();
		}
        for( size_t i=0; i<negativeFeatureSets.size(); i++ )
		{
			nrOfNeg += negativeFeatureSets[i]->size();
		}

		//cout<<endl<<nrOfPos<<" positive feature sets were found for class with id "<<_classId<<"."<<endl;
		//cout<<endl<<nrOfNeg<<" negative feature sets were found for class with id "<<_classId<<"."<<endl;

		if( (nrOfPos!=0) && (nrOfNeg==0) )
		{
			// use the standard negative class feature sets as negative sets if only one class is indicated as occuring in the image
			int standardId = getGenericClassId( (*Options::General)["classification"]["standard_negative_class"].as<string>() );
			if( standardId!=-1 && standardId!=genId ) //if the class was found and the standard negative set is not the current class
			{
				negativeFeatureSets.push_back( genericObjectClasses[standardId]->activeFeatureSet() );
				nrOfNeg += negativeFeatureSets.back()->size();
			}
			else if( standardId==-1 && standardId!=genId )
			{
				cerr<<endl<<"GenericObject::initializeClass: classifier for single occuring class "<<genericObjectClasses[genId]->name<<" couldn't be created because the standard negative class "<<(*Options::General)["classification"]["standard_negative_class"].as<string>()<<" wasn't found.";
				throw 1;
			}
			else if( standardId!=-1 && standardId==genId )
			{
				cerr<<endl<<"GenericObject::initializeClass: classifier for single occuring class "<<genericObjectClasses[genId]->name<<" couldn't be created because this class acts as standard negative class as well.";
				throw 1;
			}
			else throw 1;
		}
		if( (nrOfPos==0) || (nrOfNeg==0) )
		{
			cerr<<endl<<"GenericObject::initializeClass: classifier for single occuring class "<<genericObjectClasses[genId]->name<<" couldn't be created because the positive or negative feature sets are empty.";
			throw 1;
		}
		
		classifier[genId] = getClassifier( positiveFeatureSets, negativeFeatureSets );

		if( classifier[genId] == NULL ) throw 1; // if getClassifier method failed

		return true;
	}
	catch(...)
	{
		cerr<<endl<<"Warning: GenericObject::initializeClass( int _classId ): Failed to get a classifier for generic object class " << genericObjectClasses[genId]->name << ". Thus the class cannot be used and is deactivated.";
		return false;
	}
}



void GenericObject::setClassOptions( GenericMultiLevelMap<string>& _options, int _classId )
{
	int genId = genericId(_classId);
	
	if( _options.hasKey("active_features") )
	{
		vector<string> activeFeatures;
		for( GenericMultiLevelMap<string>::iterator it = _options["active_features"].begin(); it != _options["active_features"].end() ; it++ )
		{
			activeFeatures.push_back( (*it).first );
		}

		genericObjectClasses[genId]->setFeaturesActive(activeFeatures);
	}

	if( _options.hasKey("dynamics_options") )
	{
		genericObjectClasses[genId]->dynamicsOptions = _options["dynamics_options"];
		
		// update dynamics options for already instantiated objects of the class
        for( list<Ptr<GenericObject> >::iterator it = genericObjectLists[genId].begin(); it != genericObjectLists[genId].end(); it++ )
		{
			(*it)->pDynamics->setOptions( _options["dynamics_options"] );
		}
	}

	if( _options.hasKey("type_likelihood_function") )
	{
		vector<double> likelihoodFunction;
		double count=0;
		for( GenericMultiLevelMap<string>::iterator it = _options["type_likelihood_function"].begin(); it != _options["type_likelihood_function"].end(); it++ )
		{
			likelihoodFunction.push_back( (*it).second.as<double>() );
			if( likelihoodFunction.back()>1 )
			{
				cerr<<endl<<"Attempting to load invalid likelihood function (likelihood>1) for class "<<genericObjectClasses[genId]->name<<". Likelihood function is not loaded.";
				return;
			}
			count += likelihoodFunction.back();
		}
		if( count== 0 )
		{
			cerr<<endl<<"Attempting to load invalid likelihood function for class "<<genericObjectClasses[genId]->name<<": All likelihoods are zero. Likelihood function is not loaded.";
			return;
		}
		// load new likelihood function
		classTypeLikelihoodFunctions[genId].swap( likelihoodFunction );

	}

	return;
}



void GenericObject::getClassOptions( GenericMultiLevelMap<string>& _options, int _classId )
{
	int genId = genericId(_classId);

	// save active feature sets
    for( size_t i=0; i<genericObjectClasses[genId]->featureSets.size(); i++ )
	{
		if( genericObjectClasses[genId]->featureActive[i] ) _options["active_features"][ genericObjectClasses[genId]->featureSets[i] ].as<string>() = "";
	}

	_options["dynamics_options"] = genericObjectClasses[genId]->dynamicsOptions;
	
	// save type likelihood function
    for( size_t i=0; i<classTypeLikelihoodFunctions[genId].size(); i++ )
	{
		stringstream converter; string conv;
		converter<<i; converter>>conv;
		if(i<10) _options["type_likelihood_function"]["d00"+conv].as<double>() = classTypeLikelihoodFunctions[genId][i]; // leading are necessary because the used xml functionality orders the entries alphabetically when writing to a file: without the order of the entries thus won't be preserved since "d24" comes before "d3" with the used primitve ordering mechanism
		else if(i<100) _options["type_likelihood_function"]["d0"+conv].as<double>() = classTypeLikelihoodFunctions[genId][i];
		else _options["type_likelihood_function"]["d"+conv].as<double>() = classTypeLikelihoodFunctions[genId][i];
	}


	return;
}



Ptr<SceneObject::State> GenericObject::newState( Mat& _state, RectangleRegion& _region, vector<Point>& _contour, Mat& _invImg, double _time )
{
	return pDynamics->newState( _state, _region, _contour, _invImg, _time );
}


bool GenericObject::addState( Ptr<SceneObject::State> _newState, RectangleRegion& _regionOfInterest, vector<Point>& _contour )
{
	return pDynamics->addState( _newState, _regionOfInterest, _contour );
}


bool GenericObject::exportData( string _filePath, string _dataPointSeparator, string _timeStepSeparator )
{
	return pDynamics->exportData( _filePath, _dataPointSeparator, _timeStepSeparator );
}


Ptr<SceneObject::State> GenericObject::state()
{
	return pDynamics->state();
}


Mat GenericObject::history()
{
	return pDynamics->history();
}


Mat GenericObject::currentState()
{
	return pDynamics->currentState();
}


Point GenericObject::pos()
{
	return pDynamics->pos();
}


double GenericObject::angle()
{
	return pDynamics->angle();
}


Point GenericObject::direction()
{
	return pDynamics->direction();
}


double GenericObject::area()
{
	return pDynamics->area();
}


double GenericObject::velX()
{
	return pDynamics->velX();
}


double GenericObject::velY()
{
	return pDynamics->velY();
}


double GenericObject::velAng()
{
	return pDynamics->velAng();
}


double GenericObject::accX()
{
	return pDynamics->accX();
}


double GenericObject::accY()
{
	return pDynamics->accY();
}


double GenericObject::accAng()
{
	return pDynamics->accAng();
}


RectangleRegion GenericObject::lastROI()
{
	return pDynamics->lastROI();
}


void GenericObject::lastContour( vector<Point>& _contour )
{
	return pDynamics->lastContour( _contour );
}


void GenericObject::predictROI( vector<Point>& _roi )
{
	return pDynamics->predictROI( _roi );
}


Mat GenericObject::predictState()
{
	return pDynamics->predictState();
}


void GenericObject::resetPrediction()
{
	return pDynamics->resetPrediction();
}


Point GenericObject::predictPosition( Point _pt )
{
	return pDynamics->predictPosition( _pt );
}


Point GenericObject::predictCtrPosition()
{
	return pDynamics->predictCtrPosition();
}


Point2f GenericObject::predictCtrFPosition()
{
	return pDynamics->predictCtrFPosition();
}


double GenericObject::predictXCtrVelocity()
{
	return pDynamics->predictXCtrVelocity();
}


double GenericObject::predictXCtrAcceleration()
{
	return pDynamics->predictXCtrAcceleration();
}


double GenericObject::predictYCtrVelocity()
{
	return pDynamics->predictYCtrVelocity();
}


double GenericObject::predictYCtrAcceleration()
{
	return pDynamics->predictYCtrAcceleration();
}


double GenericObject::predictAgl()
{
	return pDynamics->predictAgl();
}


double GenericObject::predictAglVelocity()
{
	return pDynamics->predictAglVelocity();
}


double GenericObject::predictAglAcceleration()
{
	return pDynamics->predictAglAcceleration();
}


double GenericObject::predictArea()
{
	return pDynamics->predictArea();
}


bool GenericObject::drawLayer( Mat& _img, unsigned int _layerLevel, Scalar _color )
{
	return pDynamics->drawLayer( _img, _layerLevel, _color );
}


double GenericObject::isType( Mat& /*_img*/, RectangleRegion& /*_boundingRect*/, Mat& _descriptors, int /*_id*/, int _classId )
{
	
	Mat descriptors =_descriptors;

	if( descriptors.empty() )
	{
		return -1;
		//if( !createDescriptors( _img, _boundingRect, descriptors ) ) return -1;
	}
	
	int genId = genericId(_classId);
	
	if( classifier[genId]==NULL ) return 0.0;

	Mat predictionResults;
	for( int i=0; i<descriptors.size().height; i++ ) predictionResults.push_back( classifier[genId]->predict( descriptors.row(i) ) );
	

	double percentageOfPositivelyClassifiedKeypoints = ((double) norm( predictionResults, NORM_L1 ) )/predictionResults.size().height;

	/** match from percentage to probability: statistically determined value using test samples (200 train samples, 200 test samples - thus not yet very well adjusted) - but somewhat adjusted: Extended statistics of experiments would be needed to get statistically more sound data. However, so far this already seems to perform well, suggesting that maybe even a 0/1 decision would work*/
	
	int nrOfBins = classTypeLikelihoodFunctions[genId].size();

	int bin = ( percentageOfPositivelyClassifiedKeypoints+0.5/(nrOfBins-1) )*(nrOfBins-1); // parting the percentage space into 21 integer bins, each with a width of 0.05: ( 0-0.0.024 , 0.025-0.074 ,... )
	
	double likelihood;
	if( bin<0 || bin >= nrOfBins ) return 0;
	else likelihood = classTypeLikelihoodFunctions[genId][bin];

	return likelihood;
}


bool GenericObject::createDescriptors( Mat& _img, RectangleRegion& _boundingRect, Mat& _descriptors )
{
	double rectHeight = _boundingRect.height();
	double rectWidth = _boundingRect.width();
	_boundingRect.setLengths( rectHeight+10, rectWidth+10 );
	
	Mat mask = ObjectHandler::imageMask( _img, _boundingRect );
	
	vector<KeyPoint> keypoints;
	detector.detect( _img, keypoints, mask );

	if( keypoints.empty() && dynamic_feature_extraction_threshold_adaption )
	{
		for( int dynamicThreshold=55; dynamicThreshold>0 && keypoints.empty(); dynamicThreshold -= dynamic_feature_adaption_step )
		{
			FastFeatureDetector dynamicDetector(dynamicThreshold);
			dynamicDetector.detect( _img, keypoints, mask );
		}
	}
	
	Mat descriptors;
	extractor.compute( _img, keypoints, descriptors );
	


	if( descriptors.empty() ) return false; // no descriptors were found
	
	descriptors.convertTo( _descriptors,CV_32F );
	return true;
}


Ptr<SceneObject> GenericObject::recalculateClass( Ptr<SceneObject> _oldPointer )
{
	ensureClassLikelihoodSize();

	double maxLikelihood=0;
	int maxLikelihoodId;


	
	/*std::ofstream file2;
	stringstream converter; string id_str; converter<<id(); converter>>id_str;
	string filename2 = id_str + "_step.txt";
	file2.open( filename2,std::ios_base::app );
	file2<<" "<<endl;
	file2.close();

	std::ofstream file;
	string filename = id_str + ".txt";

	file.open( filename,std::ios_base::app );
	*/
    for( size_t i=0;i<pClassLikelihood.size();i++ )
	{
		//file << pClassLikelihood[i] << " ";
		if( pClassLikelihood[i]>maxLikelihood )
		{
			maxLikelihoodId=i;
			maxLikelihood=pClassLikelihood[i];
		}
	}
	/*file << endl;
	
	file.close();*/

	Ptr<SceneObject> objectPointer;
	
	if( maxLikelihood<0.5 ) // unknown object
	{
		unregisterObject(_oldPointer);
		pType = -1;
		objectPointer = new SceneObject(_oldPointer);
	}
	else if( pType==maxLikelihoodId ) // object stays the same
	{
		return _oldPointer;
	}
	else // new object is built
	{
		unregisterObject(_oldPointer);
		pType = maxLikelihoodId; // "must" be set before factory call, since for GenericObject class object otherwhise the maxLikelihoodId would have to be calculated again
		SceneObject* actualObj = dynamic_cast<SceneObject*>(this);
		objectPointer = (*factoryList)[pType]( actualObj );
	}
	return objectPointer;
}


void GenericObject::unregisterObject( Ptr<SceneObject> _old )
{
	int genId = genericId( _old->type() );

	Ptr<GenericObject> toUR(_old);
	
    for( list<Ptr<GenericObject> >::iterator it = genericObjectLists[genId].begin(); it != genericObjectLists[genId].end() ; it++ )
	{
		if( (*it) == toUR )
		{
			genericObjectLists[genId].erase(it);
			break;
		}
	}
	
	return;
}


void GenericObject::initializeStdTypeLikelihood( vector<double>& _likelihoodVector )
{
	_likelihoodVector.clear();
	
	/** match from percentage to probability: using linear mapping */
	_likelihoodVector.push_back( 0.00 ); // 0,0-0,024 (percentage of keypoints classified as abf keypoints)
	_likelihoodVector.push_back( 0.05 ); // 0,025-0,074
	_likelihoodVector.push_back( 0.1 ); // 0,075-0,124
	_likelihoodVector.push_back( 0.15 ); // 0,125-0,174
	_likelihoodVector.push_back( 0.2 ); // 0,175-0,224
	_likelihoodVector.push_back( 0.25 ); // 0,225-0,274
	_likelihoodVector.push_back( 0.30 ); // 0,275-0,324
	_likelihoodVector.push_back( 0.35 ); // 0,325-0,374
	_likelihoodVector.push_back( 0.40 ); // 0,375-0,424
	_likelihoodVector.push_back( 0.45 ); // 0,425-0,474
	_likelihoodVector.push_back( 0.50 ); // 0,475-0,524
	_likelihoodVector.push_back( 0.55 ); // 0,525-0,574
	_likelihoodVector.push_back( 0.60 ); // 0,575-0,624
	_likelihoodVector.push_back( 0.65 ); // 0,625-0,674
	_likelihoodVector.push_back( 0.70 ); // 0,675-0,724
	_likelihoodVector.push_back( 0.75 ); // 0,725-0,774
	_likelihoodVector.push_back( 0.80 ); // 0,775-0,824
	_likelihoodVector.push_back( 0.85 ); // 0,825-0,874
	_likelihoodVector.push_back( 0.90 ); // 0,875-0,924
	_likelihoodVector.push_back( 0.95 ); // 0,925-0,974
	_likelihoodVector.push_back( 1.0 ); // 0,975-1,000
	return;
}


string GenericObject::timeString()
{
	time_t currentTime;
	time(&currentTime);

	stringstream fileName;
	fileName << currentTime;
	string timeString;
	fileName >> timeString;

	return timeString;
}


string GenericObject::readableTime()
{
	time_t currentTime;
	time(&currentTime);
	return asctime(localtime(&currentTime));
}


const int& GenericObject::path_length()
{
	return ObjectHandler::path_length;
}


const int& GenericObject::path_fadeout_speed()
{
	return ObjectHandler::path_fadeout_speed;
}


bool GenericObject::setupClass()
{
    initializeGenericObjectFactory();
	
	// initialize size of vector for CvBoost pointers
	classifier.resize( genericObjectClasses.size(), NULL );

	// register all generic classes
    registerClasses();

    setupOptions();

	return true;
}


int GenericObject::getGenericClassId( string _gcName )
{
    for( size_t i=0 ; i < genericObjectClasses.size(); i++ )
	{
		if( genericObjectClasses[i]->name == _gcName ) return i;
	}
	return -1;
}


int GenericObject::genericId( const int _overallClassId, bool _suppressMessages )
{
    if( _overallClassId<0 || (unsigned int)_overallClassId >= genericToOverallClassIds.size() )
	{
		stringstream conv; string out; conv<<_overallClassId; conv>>out;
		if( !_suppressMessages ) cerr << endl << "Fatal error in GenericObject::genericId: _overallId '"<<_overallClassId<<"' is out of range and thus no genericId exists. Throwing error.";
		throw std::logic_error( "Fatal error in GenericObject::genericId: _overallId "+out+" is out of range and thus no genericId exists." );
	}
	int genId = genericToOverallClassIds[_overallClassId];
	if( genId < 0 )
	{
		stringstream conv; string out; conv<<_overallClassId; conv>>out;
		if( !_suppressMessages ) cerr << endl << "Fatal error in GenericObject::genericId: Attempting to calculate generic id for overall id "<<_overallClassId<<" but there is no match, which suggests that this class is not a generic one. Throwing error." ;
		throw std::logic_error("Fatal error in GenericObject::genericId: Attempting to calculate generic id for overall id "+out+" but there is no match, which suggests that this class is not a generic one. Throwing error.");
	}
	return genId;
}


int GenericObject::overallId( const int _genericId )
{
    for( size_t i=0; i<genericToOverallClassIds.size() ;i++ )
	{
		if( genericToOverallClassIds[i] == _genericId ) return i;
	}
	cerr << endl << "GenericObject::overallId: Attempted to calculate overallId for _genericId "<<_genericId<<" which couldn't be found";
	return -1;
}


Ptr<GenericObject::FeatureData> GenericObject::getFeatureInfo( string _featureId )
{
    for( size_t i=0; i<availableFeatureSets.size(); i++ )
	{
		if( availableFeatureSets[i]->fileName==_featureId ) return availableFeatureSets[i];
	}
	return NULL;
}


bool GenericObject::isGeneric( Ptr<SceneObject> _obj )
{
	try
	{
		genericId(  _obj->id(), true );
		return true;
	}
	catch(...)
	{
		return false;
	}
}


bool GenericObject::isGenericType( int _classId )
{
	try
	{
		genericId( _classId, true );
		return true;
	}
	catch(...)
	{
		return false;
	}
}


void GenericObject::registerClasses()
{
    for( size_t genericId=0; genericId<genericObjectClasses.size() ; genericId++ )
	{
        size_t overallId;
		overallId = SceneObject::registerObject( genericObjectClasses[genericId]->name, &createGenericObject, &isType, &initializeClass, &setClassOptions, &getClassOptions );

		if( genericToOverallClassIds.size() <= (overallId+1) ) genericToOverallClassIds.resize(overallId+1,-1);
		genericToOverallClassIds[overallId] = genericId;
	}
}


Ptr<CvBoost> GenericObject::getClassifier( const vector<Ptr<vector<string> > >& _positiveSets, const vector<Ptr<vector<string> > >& _negativeSets, const string _type )
{
	try
	{
		// check if a classifier for the given sets has already been created and trained before (training takes a little, thus it makes sense to save the trained states of classifiers and reinitialize from those whenever possible)
        for( size_t clId=0; clId < availableClassifiers.size(); clId++ )
		{
			if( availableClassifiers[clId]->isMatch( _type, _positiveSets, _negativeSets ) )
			{
				//cout<<endl<<"found match for set with "<<_positiveSets.size()<<" positive obj types and "<<_negativeSets.size()<<" negative obj types. The found classifier was "<<availableClassifiers[clId]->fileName<<"."<<endl;
				return availableClassifiers[clId]->getInitializedClassifier();
			}
		}
	}
	catch(std::exception& e)
	{
		cerr<<endl<<"Warning: GenericObject::getClassifier: There was an error when attempting to find a matching classifier." << e.what();
		return NULL;
	}
	catch(...)
	{
		cerr<<endl<<"Warning: GenericObject::getClassifier: There was an unspecified error when attempting to find a matching classifier.";
		return NULL;
	}

	Ptr<ClassifierData> newClassifier;
	try
	{
		// if no matching classifier is found, it is attempted to create a new one
		newClassifier = new ClassifierData( _type, _positiveSets, _negativeSets );
	}
	catch(std::exception& e)
	{
		cerr<<endl<<"Warning: GenericObject::getClassifier: There was an error when attempting to create a new classifier classifier:" << e.what();
		return NULL;
	}
	catch(...)
	{
		cerr<<endl<<"Warning: GenericObject::getClassifier: There was an unspecified error when attempting to find a matching classifier.";
		return NULL;
	}

	// new classifier has been successfully built

	// save its info file and trained state and add it to the list of existing classifiers
	newClassifier->save();
	availableClassifiers.push_back( newClassifier );

	// return actual classifier
	return newClassifier->getInitializedClassifier();
}

vector<Ptr<CvBoost> > GenericObject::classifier;
FastFeatureDetector GenericObject::detector(60); //keypoint extraction using the AGAST detector used in BRISK
BRISK GenericObject::extractor;
vector<Ptr<GenericObject::GOData> > GenericObject::genericObjectClasses;
//vector<string> GenericObject::chosenClassifiers; (not implemented)
vector<Ptr<GenericObject::FeatureData> > GenericObject::availableFeatureSets;
vector<Ptr<GenericObject::ClassifierData> > GenericObject::availableClassifiers;

vector<int> GenericObject::genericToOverallClassIds;
vector<list<Ptr<GenericObject> > > GenericObject::genericObjectLists;
vector<vector<double> > GenericObject::classTypeLikelihoodFunctions;


bool GenericObject::initializeGenericObjectFactory()
{
	// folder structure for generic objects is created if it does not already exist
	boost::filesystem::create_directory( "generic classes" );

    boost::filesystem::create_directory( "generic classes/descriptions" );
    boost::filesystem::create_directory( "generic classes/descriptions/inf" );
    boost::filesystem::create_directory( "generic classes/descriptions/img" );

    boost::filesystem::create_directory( "generic classes/features" );
    boost::filesystem::create_directory( "generic classes/features/sets" );
    boost::filesystem::create_directory( "generic classes/features/img" );

    boost::filesystem::create_directory( "generic classes/classifiers" );
    boost::filesystem::create_directory( "generic classes/classifiers/sets" );
    boost::filesystem::create_directory( "generic classes/classifiers/inf" );

	// build info vectors **********
	boost::filesystem::directory_iterator it_end; // default constructor yields end iterator

	// load available classes
    boost::filesystem::directory_iterator cfile_it("generic classes/descriptions/inf");
	for( ; cfile_it!=it_end ; cfile_it++ )
	{
		genericObjectClasses.push_back( new GOData((*cfile_it).path().string()) );
    }
	// load available feature sets
    boost::filesystem::directory_iterator fsets_it("generic classes/features/sets");
	for( ; fsets_it!=it_end ; fsets_it++ )
	{
		availableFeatureSets.push_back( new FeatureData((*fsets_it).path().string()) );
    }
	// load available existing classifiers
    boost::filesystem::directory_iterator cdata_it("generic classes/classifiers/inf");
	for( ; cdata_it!=it_end ; cdata_it++ )
	{
		availableClassifiers.push_back( new ClassifierData((*cdata_it).path().string()) );
    }

	if( genericObjectClasses.empty() ) cerr<<endl<<"GenericObject::initializeGenericObjectFactory(): No generic object definitions have been found."<<endl;
    SceneObject::setupObjectList();

	genericToOverallClassIds.resize( (*SceneObject::objectList).size()+genericObjectClasses.size(), -1 );
	genericObjectLists.resize( genericObjectClasses.size() );
	classTypeLikelihoodFunctions.resize( genericObjectClasses.size() );

    for( uint i=0; i<genericObjectClasses.size(); i++ ) // standard likelihood function
	{   /** match from percentage to probability: statistically determined value using test samples (200 train samples, 200 test samples - thus not yet very well adjusted) - but somewhat adjusted: Extended statistics of experiments would be needed to get statistically more sound data. However, so far this already seems to perform well, suggesting that maybe even a 0/1 decision would work*/
		initializeStdTypeLikelihood( classTypeLikelihoodFunctions[i] );
    }
	
	return true;
}


void GenericObject::setupOptions()
{
    Options::load_options();
	dynamic_feature_extraction_threshold_adaption = (*Options::General)["classification"]["dynamic_feature_extraction_threshold_adaption"].as<bool>();
	dynamic_feature_adaption_step = (*Options::General)["classification"]["dynamic_feature_adaption_step"].as<int>();
}

bool GenericObject::isSetup = setupClass();

bool GenericObject::dynamic_feature_extraction_threshold_adaption;
int GenericObject::dynamic_feature_adaption_step;
