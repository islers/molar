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

#include "GOData.h"
#include "SceneHandler.h"
#include "Dynamics.h"

GenericObject::GOData::GOData( const string _name, const Scalar _color, const string _dynamicsType, const vector<string>& _featureSet, const string _preferredClassifierType )
{
	name = _name;
	color = _color;
	creationDate = readableTime();
    //creationDate.pop_back(); // (only for c+11) the asci type function add a newline at the end... :)
    creationDate = creationDate.substr(0,creationDate.size()-1);
	dynamicsType = _dynamicsType;
	Dynamics::setStandardSettings( dynamicsType, dynamicsOptions ); // initializes dynamic settings of new classes with standard values
    for( size_t i=0; i<_featureSet.size(); i++ ) featureSets.push_back( _featureSet[i] );
    for( size_t i=0; i<_featureSet.size(); i++ ) featureActive.push_back( true );
	preferredClassifierType = _preferredClassifierType;
	pFromFile=false;
}

GenericObject::GOData::GOData( const string _filePath )
{
	try
	{
		// as structure and content types of general dynamic class options are not known and we want to make no assumptions to avoid complicating saving and loading of dynamic parameters, not OpenCV's FileStorage class is used but the more powerful and versatile GenericMultiLevelMap class
		GenericMultiLevelMap<string> tempXML;
		tempXML.initFromXML( _filePath );

		name = tempXML["Name"].as<string>();
		color = Scalar( tempXML["Color"]["B"].as<double>(), tempXML["Color"]["G"].as<double>(), tempXML["Color"]["R"].as<double>() );
		info = tempXML["Info"].as<string>();
		creationDate = tempXML["CreationDate"].as<string>();
		dynamicsType = tempXML["DynamicsType"].as<string>();
		dynamicsOptions = tempXML["DynamicsOptions"]; // copies complete sub tree of xml file
		preferredClassifierType = tempXML["PreferredClassifierType"].as<string>();

		GenericMultiLevelMap<string>::iterator it=tempXML["FeatureSets"].begin(), it_end=tempXML["FeatureSets"].end();

		for( ; it!=it_end; it++ )
		{
			featureSets.push_back( (*it).first );
			featureActive.push_back( true );
		}
		
        imagePath = "generic classes/descriptions/img/"+name+".png";
		img = imread( imagePath, CV_LOAD_IMAGE_COLOR );

	}
	catch(...)
	{
		cerr<<endl<<"Creating GOData object from file failed for file at "<<_filePath;
	}
	pFromFile = true;
}


Ptr<vector<string> > GenericObject::GOData::activeFeatureSet()
{
    Ptr<vector<string> > actFeat = new vector<string>();

	if( featureSets.size() != featureActive.size() )
	{
		cerr<<endl<<"GenericObject::GOData::activeFeatureSet(): feature set vectors have been corrupted - class "<<name<<" thus is invalid";
		return actFeat; // arrays have been corrupted, set can't be built - thus the class is invalid (after all they are not protected)
	}
	
    for( size_t i=0 ; i<featureSets.size(); i++ )
	{
		if( featureActive[i] ) actFeat->push_back( featureSets[i] );
	}
	return actFeat;
}


void GenericObject::GOData::setFeaturesActive( vector<string>& _activeFeatures )
{
    for( size_t i=0; i<featureActive.size(); i++ )
	{
		featureActive[i] = false;

        for( size_t j=0; j<_activeFeatures.size() ; j++ )
		{
			if( featureSets[i]==_activeFeatures[j] )
			{
				featureActive[i] = true;
				break;
			}
		}
	}
	return;
}


bool GenericObject::GOData::save()
{
	if( name=="" ) return false;
	if( featureSets.empty() ) return false;

	// build XML structure
	GenericMultiLevelMap<string> tempXML;

	tempXML["Name"].as<string>() = name;
	tempXML["Color"]["R"].as<double>() = color[2];
	tempXML["Color"]["G"].as<double>() = color[1];
	tempXML["Color"]["B"].as<double>() = color[0];
	tempXML["Info"].as<string>() = info;
	tempXML["CreationDate"].as<string>() = creationDate;
	tempXML["DynamicsType"].as<string>() = dynamicsType;
	tempXML["DynamicsOptions"] = dynamicsOptions;
	tempXML["PreferredClassifierType"].as<string>() = preferredClassifierType;

    for( size_t i=0; i<featureSets.size(); i++ )
	{
		tempXML["FeatureSets"][featureSets[i]].as<string>()="";
	}

    string filePath = "generic classes/descriptions/inf/"+name;
	
	// test if name is unique: if it is not, append a number to it to make it unique
	bool fileAlreadyExists = !pFromFile && boost::filesystem::exists( filePath+".xml" );
	if( fileAlreadyExists )
	{
		int versionCount=0;
		stringstream converter;
		string count;
		while( fileAlreadyExists )
		{
			converter<<versionCount;
			converter>>count;
			fileAlreadyExists = boost::filesystem::exists( filePath+"_"+count+".xml" );
			versionCount++;
			converter.clear();
		}
		name += "_"+count;
		tempXML["Name"].as<string>() = name;
	}

    tempXML.saveToXML("generic classes/descriptions/inf/"+name+".xml");

	// write image to disk
	if( !img.empty() )
	{
		vector<int> pngOpt(2); // option vector for png compression
		pngOpt[0] = CV_IMWRITE_PNG_COMPRESSION;
		pngOpt[1] = 3;
        imwrite( "generic classes/descriptions/img/"+name+".png", img, pngOpt );
	}
	return true;
}


bool GenericObject::GOData::registerClass()
{
	if( !save() ) return false;

	// register class with sceneobject factory
    size_t overallClassId = SceneObject::registerObject( name, &createGenericObject, &isType, &initializeClass, &setClassOptions, &getClassOptions );

	// register class as generic object type
	genericObjectClasses.push_back( this );
	int genericId = genericObjectClasses.size()-1;
	genericObjectLists.resize( genericObjectClasses.size() );

	classTypeLikelihoodFunctions.resize( genericObjectClasses.size() );
	initializeStdTypeLikelihood( classTypeLikelihoodFunctions.back() );

	if( genericToOverallClassIds.size() <= (overallClassId+1) ) genericToOverallClassIds.resize(overallClassId+1,-1);
	genericToOverallClassIds[overallClassId] = genericId;

	classifier.resize( genericObjectClasses.size(), NULL );
	return true;
}


GenericObject::FeatureData::FeatureData( const string _name, const Mat _featureSet, const Mat _sceneImage )
{
	name = _name;
	fileName = "fd"+timeString();
	creationDate = readableTime();
	numberOfFeaturePoints = _featureSet.rows;
	detectorType = "FastFeatureDetector";
	extractorType = "BRISK";
	pFeatureSet = _featureSet;
	img = _sceneImage;
}


GenericObject::FeatureData::FeatureData( const string _filePath )
{
	FileStorage FDFile( _filePath, FileStorage::READ );

	FDFile["Name"] >> name;
	FDFile["Info"] >> info;
	FDFile["FileName"] >> fileName;
	FDFile["CreationDate"] >> creationDate;
	FDFile["NumberOfFeaturePoints"] >> numberOfFeaturePoints;
	FDFile["DetectorType"] >> detectorType;
	FDFile["ExtractorType"] >> extractorType;

	FDFile.release();
	
	// feature set is only loaded if needed
    img = imread( "generic classes/features/img/"+fileName+".png", CV_LOAD_IMAGE_COLOR );

}


bool GenericObject::FeatureData::save()
{
	try{

        FileStorage FDFile( "generic classes/features/sets/"+fileName+".xml", FileStorage::WRITE );

		// store information
		FDFile<<"Name"<<name;
		FDFile<<"Info"<<info;
		FDFile<<"FileName"<<fileName; // directly added for convenience
		FDFile<<"CreationDate"<<creationDate;
		FDFile<<"NumberOfFeaturePoints"<<numberOfFeaturePoints;
		FDFile<<"DetectorType"<<detectorType;
		FDFile<<"ExtractorType"<<extractorType;

		// save feature set
		if( !pFeatureSet.empty() )
		{
			 FDFile<<"FeatureSet"<<pFeatureSet;
		}
		
		FDFile.release();

		// save scene image
		if( !img.empty() )
		{
			vector<int> pngOpt(2); // option vector for png compression
			pngOpt[0] = CV_IMWRITE_PNG_COMPRESSION;
			pngOpt[1] = 3;
            imwrite( "generic classes/features/img/"+fileName+".png", img, pngOpt );
		}


		return true;
	}
	catch(...)
	{
		return false;
	}
}



Mat GenericObject::FeatureData::getFeatureSet()
{
	if( pFeatureSet.empty() )
	{
        FileStorage FDFile( "generic classes/features/sets/"+fileName+".xml", FileStorage::READ );
		FDFile["FeatureSet"] >> pFeatureSet;
		FDFile.release();
	}
	return pFeatureSet;
}



GenericObject::ClassifierData::ClassifierData( const string /*_classifierType*/, const vector<Ptr<vector<string> > >& _positiveFeatureSets, const vector<Ptr<vector<string> > >& _negativeFeatureSets )
{
	fileName = "cd"+timeString();
	classifierType = "CvBoost";
	
	vector<int> positiveMapping, negativeMapping;
    for( size_t i=0; i<_positiveFeatureSets.size(); i++ )
	{
        for( size_t j=0; j<_positiveFeatureSets[i]->size(); j++ )
		{
			positiveFeatureSets.push_back( (*_positiveFeatureSets[i])[j] );
			positiveMapping.push_back(i);
		}
	}
    for( size_t i=0; i<_negativeFeatureSets.size(); i++ )
	{
        for( size_t j=0; j<_negativeFeatureSets[i]->size(); j++ )
		{
			negativeFeatureSets.push_back( (*_negativeFeatureSets[i])[j] );
			negativeMapping.push_back(i);
		}
	}


	// create and train classifier
	pClassifier = new CvBoost();

	//get largest set size
	double largestFeatureSetSize = 0;
	vector<double> positiveSetSizes, negativeSetSizes;

	double ratioThreshold = 1.6; // maximal difference between two sets before the smalle is duplicated
    for( size_t posId=0; posId<_positiveFeatureSets.size(); posId++ )
	{
		double featureSetSize=0;
        for( size_t subId=0; subId<_positiveFeatureSets[posId]->size(); subId++ )
		{
            for( size_t testId=0; testId<availableFeatureSets.size(); testId++ )
			{
				if( availableFeatureSets[testId]->fileName == (*_positiveFeatureSets[posId])[subId] ) featureSetSize += availableFeatureSets[testId]->numberOfFeaturePoints;	
			}
		}
		positiveSetSizes.push_back( featureSetSize );
		if( featureSetSize > largestFeatureSetSize ) largestFeatureSetSize = featureSetSize;
	}
    for( size_t negId=0; negId<_negativeFeatureSets.size(); negId++ )
	{
		double featureSetSize=0;
        for( size_t subId=0; subId<_negativeFeatureSets[negId]->size(); subId++ )
		{
            for( size_t testId=0; testId<availableFeatureSets.size(); testId++ )
			{
				if( availableFeatureSets[testId]->fileName == (*_negativeFeatureSets[negId])[subId] ) featureSetSize += availableFeatureSets[testId]->numberOfFeaturePoints;	
			}
		}
		negativeSetSizes.push_back( featureSetSize );
		if( featureSetSize > largestFeatureSetSize ) largestFeatureSetSize = featureSetSize;
	}

	cout<<endl<<"Larger data set has a size of "<<largestFeatureSetSize<<" features."<<endl;

	Mat positiveSet, negativeSet, totalSet, posLabels, negLabels, dataLabels;

	// The CvBoost performs better if the number of positive and negative training data is about the same order. In order to guarantee so, data is duplicated if this is not the case
    for( size_t testId=0; testId<availableFeatureSets.size(); testId++ ) // pick out the featureSets needed and build Mats
	{
        for( size_t posId=0; posId<positiveFeatureSets.size(); posId++ )
		{
			if( availableFeatureSets[testId]->fileName == positiveFeatureSets[posId] )
			{
				if( availableFeatureSets[testId]->numberOfFeaturePoints==0 )
				{
					cout<<"Attempted to add feature set "<<availableFeatureSets[testId]->fileName<<" which was empty. Feature set was skipped."<<endl;
					break;
				}
				double relativeSize = largestFeatureSetSize / positiveSetSizes[positiveMapping[posId]]; //positive mapping maps the position in positiveFeatureSets to the first level of _positiveFeatureSets, which indicates the object it comes from

				for( int i=0; i<=(relativeSize-ratioThreshold+1); i++ )
				{
					cout<<endl<<"adding feature set of size "<<availableFeatureSets[testId]->numberOfFeaturePoints<<" to positive set.";
					positiveSet.push_back( availableFeatureSets[testId]->getFeatureSet() );
				}

				break; // since fileName is unique no other positiveFeatureSet will be matched
			}
		}
        for( size_t negId=0; negId<negativeFeatureSets.size(); negId++ )
		{
			if( availableFeatureSets[testId]->fileName == negativeFeatureSets[negId] )
			{
				if( availableFeatureSets[testId]->numberOfFeaturePoints==0 )
				{
					cout<<"Attempted to add feature set "<<availableFeatureSets[testId]->fileName<<" which was empty. Feature set was skipped."<<endl;
					break;
				}
				double relativeSize = largestFeatureSetSize / negativeSetSizes[negativeMapping[negId]]; //negative mapping maps the position in negativeFeatureSets to the first level of _negativeFeatureSets, which indicates the object it comes from

				for( int i=0; i<=(relativeSize-ratioThreshold+1); i++ )
				{
					cout<<endl<<"adding feature set of size "<<availableFeatureSets[testId]->numberOfFeaturePoints<<" to negative set.";
					negativeSet.push_back( availableFeatureSets[testId]->getFeatureSet() );
				}

				break;
			}
		}
	}
	
	/*Mat finalPos, finalNeg;
	finalPos.push_back( positiveSet );
	finalNeg.push_back( negativeSet );*/

	// The CvBoost performs better if the number of positive and negative training data is about the same order. In order to guarantee so, data is duplicated if this is not the case
	if( negativeSet.rows==0 || positiveSet.rows==0 ) throw std::logic_error("Division by zero occured when attempting to create new classifier");
	
	/*
	double posToNeg = positiveSet.rows/negativeSet.rows;
	double negToPos = negativeSet.rows/positiveSet.rows;
	if( posToNeg>=1.6 )
	{
		for( int i=0; i<=(posToNeg-1.6) ; i++ )
		{
			finalNeg.push_back( negativeSet );
		}
	}
	else
	{
		for( int i=0; i<=(negToPos-1.6) ; i++ )
		{
			finalPos.push_back( positiveSet );
		}
	}
	*/

	// build corresponding data labels
	posLabels = Mat::ones( positiveSet.size().height, 1, CV_32F );
	negLabels = Mat::zeros( negativeSet.size().height, 1, CV_32F );

	// build final training sets
	totalSet.push_back( positiveSet );
	totalSet.push_back( negativeSet );
	dataLabels.push_back( posLabels );
	dataLabels.push_back( negLabels );

	// train classifier
	pClassifier->clear();
	double time = SceneHandler::msTime();
	pClassifier->train( totalSet, CV_ROW_SAMPLE, dataLabels );
	cout<<endl<<"It took the classifier "<<SceneHandler::msTime()-time<<"ms to train with "<<positiveSet.size()<<" positive labeled features and "<<negativeSet.size()<<" negative labeled features yielding a total feature size of "<<dataLabels.rows<<" features."<<endl;
}
	


GenericObject::ClassifierData::ClassifierData( const string _filePath )
{
	FileStorage CDFile( _filePath, FileStorage::READ );

	CDFile["FileName"] >> fileName;
	CDFile["ClassifierType"] >> classifierType;

	FileNode positiveFeatures = CDFile["PositiveFeatureSets"];
	FileNodeIterator it_p = positiveFeatures.begin(), it_p_end = positiveFeatures.end();

	for( ; it_p!=it_p_end ; it_p++ )
	{
		string newName;
		(*it_p) >> newName;
		positiveFeatureSets.push_back( newName );
	}

	FileNode negativeFeatures = CDFile["NegativeFeatureSets"];
	FileNodeIterator it_n = negativeFeatures.begin(), it_n_end = negativeFeatures.end();

	for( ; it_n!=it_n_end ; it_n++ )
	{
		string newName;
		(*it_n) >> newName;
		negativeFeatureSets.push_back( newName );
	}

	CDFile.release();

}


bool GenericObject::ClassifierData::save()
{
	try{

        FileStorage CDFile( "generic classes/classifiers/inf/"+fileName+".xml", FileStorage::WRITE );

		// store information
		CDFile<<"FileName"<<fileName; // directly added for convenience
		CDFile<<"ClassifierType"<<classifierType;
		
		CDFile<<"PositiveFeatureSets" << "[";
        for( size_t i=0; i<positiveFeatureSets.size(); i++ ) CDFile<<positiveFeatureSets[i];
		CDFile<<"]";

		CDFile<<"NegativeFeatureSets" << "[";
        for( size_t i=0; i<negativeFeatureSets.size(); i++ ) CDFile<<negativeFeatureSets[i];
		CDFile<<"]";
		
		CDFile.release();

		// save classifier content
		if( pClassifier!=NULL )
		{
            string filePath = "generic classes/classifiers/sets/"+fileName+".xml.gz";
			pClassifier->save( filePath.c_str() );
		}

		return true;
	}
	catch(...)
	{
		return false;
	}
}


bool GenericObject::ClassifierData::isMatch( const string _classifierType, const vector<Ptr<vector<string> > >& _positiveFeatureSets, const vector<Ptr<vector<string> > >& _negativeFeatureSets )
{
	if( classifierType!=_classifierType ) return false;
	
    size_t posFSize=0, negFSize=0;
    for( size_t m=0; m<_positiveFeatureSets.size(); m++ ){ posFSize += _positiveFeatureSets[m]->size(); }
    for( size_t m=0; m<_negativeFeatureSets.size(); m++ ){ negFSize += _negativeFeatureSets[m]->size(); }

	if( positiveFeatureSets.size()!=posFSize || negativeFeatureSets.size()!=negFSize ) return false;

	// check if all positive feature sets of the classifier are part of the set
    for( size_t i=0; i<positiveFeatureSets.size(); i++ )
	{
		bool isInSet=false;
        for( size_t m=0; m<_positiveFeatureSets.size(); m++ )
		{
            for( size_t n=0; n<_positiveFeatureSets[m]->size(); n++ )
			{
				if( positiveFeatureSets[i] == (*_positiveFeatureSets[m])[n] )
				{
					//cout<<endl<<"The found positive set for classifier "<<fileName<<" is "<<positiveFeatureSets[i]<<"."<<endl;
					isInSet = true;
					break;
				}
			}
			if( isInSet ) break;
		}
		if( !isInSet ) return false;
	}

	// check if all negative feature sets of the classifier are part of the set
    for( size_t i=0; i<negativeFeatureSets.size(); i++ )
	{
		bool isInSet=false;
        for( size_t m=0; m<_negativeFeatureSets.size(); m++ )
		{
            for( size_t n=0; n<_negativeFeatureSets[m]->size(); n++ )
			{
				if( negativeFeatureSets[i] == (*_negativeFeatureSets[m])[n] )
				{
					//cout<<endl<<"The found negative set for classifier "<<fileName<<" is "<<negativeFeatureSets[i]<<"."<<endl;
					isInSet = true;
					break;
				}
			}
			if( isInSet ) break;
		}
		if( !isInSet ) return false;
	}

	return true;
}


Ptr<CvBoost> GenericObject::ClassifierData::getInitializedClassifier()
{
	if( pClassifier==NULL ) // initialize it with data from file
	{
		pClassifier = new CvBoost();
        string filePath = "generic classes/classifiers/sets/"+fileName+".xml.gz";
		pClassifier->load( filePath.c_str() );
	}
	return pClassifier;
}
