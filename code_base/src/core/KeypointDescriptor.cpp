#include "KeypointDescriptor.h"


KeypointDescriptor::KeypointDescriptor(void)
{
}


KeypointDescriptor::~KeypointDescriptor(void)
{
}

extern double msTime();

void KeypointDescriptor::calculate()
{
	string imageFolder = "D:\\Benutzer\\stewess\\Documents\\ETH\\FS 2014\\Bachelorarbeit\\single robot set\\";
	
	vector<Mat> keypointDescriptors;

	stringstream convert;
	string temp;

	SURF surfAlgorithm(300,1,4,0,0);

	int totalNumberOfKeypoints=0;

	double measureTime1 = msTime();

	for( int folder = 1; folder<=2; folder++ )
	{
		convert<<folder;
		convert>>temp;
		string imgFolder = imageFolder+temp+"\\";

		for( int i=0;i<50;i++)
		{
			// read images and calculate descriptors

			convert<<i;
			convert>>temp;
			convert.clear();

			string eImageName = imgFolder+"extract"+temp+".png";
			Mat image = imread( eImageName, CV_LOAD_IMAGE_COLOR );

			if(! image.data )
			{
				cerr<<endl<<"Could not open or find the image "<<eImageName<<endl;
				return;
			}
			vector<KeyPoint> keypoints;
			Mat descriptors;

			surfAlgorithm(image,Mat(),keypoints,descriptors);

			totalNumberOfKeypoints+=keypoints.size();
			keypointDescriptors.push_back(descriptors.clone());
		}


	}

	double measureTime2 = msTime();

	cout<<endl<<"Extracting and calculating keypoint descriptors of "<<totalNumberOfKeypoints<<" keypoints using SURF used "<<measureTime2-measureTime1<<"ms of cpu time."<<endl;
	
	//write keypoint descriptors to file
	string kdFileName = "ABFdescriptors.xml.gz";
	FileStorage out(kdFileName, FileStorage::WRITE );

	out << "keypointDescriptors" << "[";
	for(int i=0;i<keypointDescriptors.size();i++) out<<keypointDescriptors[i];
	out << "]";

	out.release();

	//drawKeypoints(surfimage,points1,surfimage, Scalar::all(-1),4);
	//drawKeypoints(siftimage,points2,siftimage, Scalar::all(-1),4);



	return;
}


void KeypointDescriptor::fastBriskProperties()
{
	string trainingSetPath = "D:\\Benutzer\\stewess\\Documents\\ETH\\FS 2014\\Bachelorarbeit\\single robot set\\";
	
	vector<Mat> images;
	vector<vector<KeyPoint>> keypoints;
	vector<Mat> descriptors;
	
	//keypoint extraction using the AGAST detector used in BRISK
	FastFeatureDetector detector(60);
	Mat extractionTimes;

	BRISK extractor;//(true,true,12.0);
	Mat descriptorTimes;

	Mat abfDescriptorSet1;
	Mat abfDescriptorSet2;
	vector<Mat> abfDescriptors;
	// load training set images with keypoints , creating abf descriptor set
	stringstream convert; string nameTemp, folTemp;

	int minNrOfDescriptorsForRobots=100;
	Mat nrOfDescriptors;

	for( int fol=1;fol<3;fol++)
	{
		convert<<fol;
		convert>>folTemp;
		convert.clear();

		for( int i=0;i<50;i++ )
		{
			convert<<i;
			convert>>nameTemp;
			convert.clear();
			
			Mat img = imread( trainingSetPath+folTemp+"\\extract"+nameTemp+".png", CV_LOAD_IMAGE_COLOR);
			vector<KeyPoint> imgKeypoints;
			double time = msTime();
			detector.detect( img,imgKeypoints );
			extractionTimes.push_back( msTime()-time );

			Mat imgDescriptors(0,0,CV_32F);
			double time2 = msTime();
			extractor.compute(img,imgKeypoints,imgDescriptors);
			descriptorTimes.push_back( msTime()-time2 );
			

			if( !imgDescriptors.empty() )
			{
				if( imgDescriptors.size().height<minNrOfDescriptorsForRobots ) minNrOfDescriptorsForRobots=imgDescriptors.size().height;
				nrOfDescriptors.push_back( imgDescriptors.size().height );

				images.push_back(img);
				keypoints.push_back(imgKeypoints);
				descriptors.push_back(imgDescriptors);
				abfDescriptorSet1.push_back(imgDescriptors);
			}
		}
	}
	for( int fol=3;fol<5;fol++)
	{
		convert<<fol;
		convert>>folTemp;
		convert.clear();

		for( int i=0;i<50;i++ )
		{
			convert<<i;
			convert>>nameTemp;
			convert.clear();
			
			Mat img = imread( trainingSetPath+folTemp+"\\extract"+nameTemp+".png", CV_LOAD_IMAGE_COLOR);
			vector<KeyPoint> imgKeypoints;
			detector.detect( img,imgKeypoints );

			Mat imgDescriptors;
			extractor.compute(img,imgKeypoints,imgDescriptors);
			
			abfDescriptorSet2.push_back(imgDescriptors);
			if( !imgDescriptors.empty() ) 
			{
				if( imgDescriptors.size().height<minNrOfDescriptorsForRobots ) minNrOfDescriptorsForRobots=imgDescriptors.size().height;
				nrOfDescriptors.push_back( imgDescriptors.size().height );

				imgDescriptors.convertTo(imgDescriptors,CV_32F);
				abfDescriptors.push_back(imgDescriptors);
			}
		}
	}
	//geometric objects descriptor set
	Mat geoDescriptorSet1;
	Mat geoDescriptorSet2;
	string geoSetPath = "D:\\Benutzer\\stewess\\Documents\\ETH\\FS 2014\\Bachelorarbeit\\negativ set\\";
	
	for( int i=0;i<100;i++ )
	{
		convert<<i;
		convert>>nameTemp;
		convert.clear();
		Mat img = imread( geoSetPath+"noabf - Kopie ("+nameTemp+").png", CV_LOAD_IMAGE_COLOR );

		vector<KeyPoint> myKeypoints;
		Mat myDescriptor;
		detector.detect( img, myKeypoints );
		extractor.compute( img, myKeypoints, myDescriptor );

		geoDescriptorSet1.push_back( myDescriptor );
	}
	vector<Mat> geoDescriptors;
	for( int i=100;i<200;i++ )
	{
		convert<<i;
		convert>>nameTemp;
		convert.clear();
		Mat img = imread( geoSetPath+"noabf - Kopie ("+nameTemp+").png", CV_LOAD_IMAGE_COLOR );

		vector<KeyPoint> myKeypoints;
		Mat myDescriptor;
		detector.detect( img, myKeypoints );
		extractor.compute( img, myKeypoints, myDescriptor );

		geoDescriptorSet2.push_back( myDescriptor );
		if( !myDescriptor.empty() )
		{
			myDescriptor.convertTo(myDescriptor,CV_32F);
			geoDescriptors.push_back( myDescriptor );
		}
	}

	
	Mat posAnswer = Mat::ones( abfDescriptorSet1.size().height, 1,CV_32F );
	Mat negAnswer = Mat::zeros( geoDescriptorSet1.size().height, 1,CV_32F );
	

	Mat trainingData = abfDescriptorSet1.clone();
	trainingData.push_back(geoDescriptorSet1);

	
	trainingData.convertTo( trainingData, CV_32F );

	Mat dataLabel = posAnswer;
	dataLabel.push_back(negAnswer);

	// using bayes classifier (assuming gaussian distribution of data)
	CvNormalBayesClassifier bayes;
	// k nearest classifier
	CvKNearest knearest;
	// svm classifier
	CvSVM svm;
	// decision trees
	CvDTree dtree;
	// boosting
	CvBoost boost;
	//random tree
	CvRTrees rtree;
	
	bayes.train( trainingData, dataLabel );
	knearest.train( trainingData, dataLabel );
	svm.train( trainingData, dataLabel );
	dtree.train( trainingData, CV_ROW_SAMPLE, dataLabel );
	boost.train( trainingData, CV_ROW_SAMPLE, dataLabel );
	rtree.train( trainingData, CV_ROW_SAMPLE, dataLabel );

	//boost.save("ABFSpiralBoostClassifier_std.xml.gz","standard_boost_classifier");

	//test images
	abfDescriptorSet2.convertTo( abfDescriptorSet2,CV_32F );

	geoDescriptorSet1.convertTo( geoDescriptorSet1,CV_32F );
	geoDescriptorSet2.convertTo( geoDescriptorSet2,CV_32F );

	negDescriptors.push_back( geoDescriptorSet1 );
	negDescriptors.push_back( geoDescriptorSet2 );

	return;
	vector<Mat> positiveABFdecisions(6), positiveGEOdecisions(6);

	//calculate answer for abf set
	Mat bayesT, kNearestT, dTreeT, boostT, rTreeT;
	for( int im=0;im<abfDescriptors.size();im++ )
	{
		vector<Mat> predictionABF(6);
		double t1=msTime();
		bayes.predict( abfDescriptors[im], &predictionABF[0] );
		double t2=msTime();
		knearest.find_nearest( abfDescriptors[im],3, &predictionABF[1]);
		double t3=msTime();
		svm.predict(abfDescriptors[im], predictionABF[2]);
		double t4=msTime();
		for( int i=0;i<abfDescriptors[im].size().height;i++ ) predictionABF[3].push_back(dtree.predict( abfDescriptors[im].row(i) )->value);
		double t5=msTime();
		for( int i=0;i<abfDescriptors[im].size().height;i++ ) predictionABF[4].push_back( boost.predict( abfDescriptors[im].row(i) ));
		double t6=msTime();
		for( int i=0;i<abfDescriptors[im].size().height;i++ ) predictionABF[5].push_back( rtree.predict( abfDescriptors[im].row(i) ));
		double t7=msTime();
		bayesT.push_back(t2-t1); kNearestT.push_back(t3-t2); dTreeT.push_back(t5-t4); boostT.push_back(t6-t5); rTreeT.push_back(t7-t6);

		for( int i=0;i<6; i++ ) positiveABFdecisions[i].push_back( ( (double)norm( predictionABF[i],NORM_L1 ) )/ predictionABF[i].size().height );
	}

	//calculate answer for geo set
	for( int im=0;im<geoDescriptors.size();im++ )
	{
		vector<Mat> predictionGEO(6);

		bayes.predict( geoDescriptors[im], &predictionGEO[0] );
		knearest.find_nearest( geoDescriptors[im],3, &predictionGEO[1]);
		svm.predict(geoDescriptors[im], predictionGEO[2]);
		for( int i=0;i<geoDescriptors[im].size().height;i++ ) predictionGEO[3].push_back(dtree.predict( geoDescriptors[im].row(i) )->value);
		for( int i=0;i<geoDescriptors[im].size().height;i++ ) predictionGEO[4].push_back( boost.predict( geoDescriptors[im].row(i) ));
		for( int i=0;i<geoDescriptors[im].size().height;i++ ) predictionGEO[5].push_back( rtree.predict( geoDescriptors[im].row(i) ));

		for( int i=0;i<6; i++ ) positiveGEOdecisions[i].push_back( ( (double)norm( predictionGEO[i],NORM_L1 ) )/ predictionGEO[i].size().height );
	}

	string names[6] = {"Bayesian","KNearestK3","SVM","DTree","Boosting","RTree"};

	for( int i=0;i<6;i++ )
	{
		ofstream tempstream( "ABFSet"+names[i]+".txt", ios::out );
		tempstream<<positiveABFdecisions[i];
		tempstream.close();

		tempstream.open( "GEOSet"+names[i]+".txt", ios::out );
		tempstream<<positiveGEOdecisions[i];
		tempstream.close();
	}

	cout<<endl<<"Minimal number of descriptors calculated for robots:"<<minNrOfDescriptorsForRobots;
	cout<<endl<<"Maximal number of descriptors calculated for robots:"<<norm(nrOfDescriptors,NORM_INF);
	cout<<endl<<"Average number of descriptors per robot image:"<<norm( nrOfDescriptors,NORM_L1 )/nrOfDescriptors.size().height;
	
	cout<<endl;
	cout<<endl<<"Average keypoint extraction time with FAST is: "<< norm(extractionTimes,NORM_L1 )/extractionTimes.size().height<<"ms.";
	cout<<endl<<"Average descriptor computation time with BRISK is: "<< norm(descriptorTimes,NORM_L1 )/descriptorTimes.size().height<<"ms.";
	cout<<endl<<"Average time used for prediction for the different algorithms:";
	cout<<endl<<"Bayes: "<<norm( bayesT,NORM_L1)/bayesT.size().height<<"ms";
	cout<<endl<<"k-nearest (k=3): "<<norm( kNearestT,NORM_L1 )/kNearestT.size().height<<"ms";
	cout<<endl<<"decision trees: "<<norm( dTreeT,NORM_L1)/dTreeT.size().height<<"ms";
	cout<<endl<<"boost: "<<norm( boostT,NORM_L1)/boostT.size().height<<"ms";
	cout<<endl<<"random trees: "<<norm( rTreeT,NORM_L1)/rTreeT.size().height<<"ms";

	/*

	BFMatcher matcher(NORM_HAMMING,true);

	Mat myImage = imread("D:\\Benutzer\\stewess\\Documents\\ETH\\FS 2014\\Bachelorarbeit\\opencvoutput\\extract15.png",CV_LOAD_IMAGE_COLOR);
	vector<KeyPoint> myKeys;
	Mat myDescriptors;

	int comp=12;

	double time1=msTime();
	detector.detect( myImage, myKeys );
	extractor.compute( myImage, myKeys, myDescriptors );
	cout<<endl<<"BRISK keypoint detection and descriptor computation time for whole image: "<<msTime()-time1<<" ms."<<endl;

	vector<DMatch> matches;
	double time3=msTime();
	matcher.match( myDescriptors, descriptors[comp], matches );
	cout<<endl<<"Matching uses "<<msTime()-time3<<" ms of time."<<endl;
	cout<<endl<<"Descriptors for robot image: "<<descriptors[comp].size().height<<", descriptors for scene: "<<myDescriptors.size().height;
	cout<<endl<<"Number of matches: "<<matches.size()<<endl;
	
	Mat matchingImage;
	drawMatches( myImage, myKeys,images[comp], keypoints[comp],  matches, matchingImage );

	imshow("Matches", matchingImage);
	waitKey(0);
	*/

	return;
}



void KeypointDescriptor::load()
{
	string kdFileName = "ABFdescriptors.xml.gz";
	FileStorage out(kdFileName, FileStorage::READ );

	FileNode descriptor = out["keypointDescriptors"];
	if( descriptor.type()!=FileNode::SEQ )
	{
		cerr << "keypointDescriptors is not a sequence. Fail." << endl;
		return;
	}

	SURF surfAlgorithm(300,1,4,0,0);

	
	Mat descriptorSet;
	vector<Mat> keypointDescriptors;
	vector<int> lookuptable;


	for( FileNodeIterator it = descriptor.begin(); it!= descriptor.end(); it++ )
	{
		Mat loadeddescriptor;
		(*it)>>loadeddescriptor;
		keypointDescriptors.push_back( loadeddescriptor );
		descriptorSet.push_back(loadeddescriptor);

		//if( keypointDescriptors.size()!=30) descriptorSet.push_back(loadeddescriptor);
		
		/*for(int i=0;i<loadeddescriptor.size().height;i++)
		{
			lookuptable.push_back( keypointDescriptors.size()-1 );
		}*/
	}
	Mat positiveTrainSet=descriptorSet;

	string nSampleFolder = "D:\\Benutzer\\stewess\\Documents\\ETH\\FS 2014\\Bachelorarbeit\\negativ set\\";
	string nSampleName = "noabf - Kopie (";

	Mat negativeTrainSet;
	vector<Mat> nTSdescriptors;

	stringstream convert;
	string temp;

	for( int i=0;i<100;i++ )
	{
		convert<<i;
		convert>>temp;
		convert.clear();
		string sampleName= nSampleFolder+nSampleName+temp+").png";
		
		Mat image = imread( sampleName,CV_LOAD_IMAGE_COLOR );
		vector<KeyPoint> keypoints;
		Mat descriptors;
		surfAlgorithm(image,Mat(),keypoints,descriptors);
		negativeTrainSet.push_back(descriptors);
	}
	Mat posAnswer = Mat::ones( positiveTrainSet.size().height, 1,CV_32F );
	Mat negAnswer = Mat::zeros( negativeTrainSet.size().height, 1,CV_32F );

	Mat trainingData = positiveTrainSet.clone();
	trainingData.push_back(negativeTrainSet);


	Mat dataLabel = posAnswer;
	dataLabel.push_back(negAnswer);


	cout<<"Number of positive descriptors: "<<trainingData.size()<<endl;
	cout<<"Number of negative descriptors: "<<dataLabel.size()<<endl;


	// calculating number of keypoints classified as possible ABF keypoint using different classification methods
	// using bayes classifier (assuming gaussian distribution of data)
	CvNormalBayesClassifier bayes;

	// k nearest classifier
	CvKNearest knearest;

	// svm classifier
	CvSVM svm;

	// decision trees
	CvDTree dtree;

	// boosting
	CvBoost boost;

	//random tree
	CvRTrees rtree;
	
	bayes.train( trainingData, dataLabel );
	knearest.train( trainingData, dataLabel );
	svm.train( trainingData, dataLabel );
	dtree.train( trainingData, CV_ROW_SAMPLE, dataLabel );
	boost.train( trainingData, CV_ROW_SAMPLE, dataLabel );
	rtree.train( trainingData, CV_ROW_SAMPLE, dataLabel );
	
	Mat percentageOfPositivePredictions;
	string temp2;

	for( int n=3; n<4; n++ )
	{
		convert<<n;
		convert>>temp2;
		convert.clear();

		for(int i =100;i<200;i++)
		{
			string testImageLocN = "D:\\Benutzer\\stewess\\Documents\\ETH\\FS 2014\\Bachelorarbeit\\negativ set\\";
			string testImageLocP = "D:\\Benutzer\\stewess\\Documents\\ETH\\FS 2014\\Bachelorarbeit\\single robot set\\"+temp2+"\\";
		
			convert<<i;
			convert>>temp;
			convert.clear();

			string testImageName = "noabf - Kopie (";
			string testImageNameP = "extract";

			Mat testImage = imread( testImageLocN+testImageName+temp+").png",CV_LOAD_IMAGE_COLOR );
			vector<KeyPoint> keypoints;
			Mat descriptors;
			surfAlgorithm(testImage,Mat(),keypoints,descriptors);

			Mat predictionResult;
			double t1 = msTime();
			if( !descriptors.empty() ) bayes.predict( descriptors, &predictionResult );
			//if( !descriptors.empty() ) knearest.find_nearest( descriptors,10, &predictionResult);
			//if( !descriptors.empty() ) svm.predict( descriptors, predictionResult);
			/*for( int d=0;d<descriptors.size().height;d++ )
			{
				predictionResult.push_back( dtree.predict( descriptors.row(d) )->value ); //svm.predict( descriptors, predictionResult);
			}*/
			/*for( int d=0;d<descriptors.size().height;d++ )
			{
				predictionResult.push_back( boost.predict( descriptors.row(d) ));
			}*/
			/*for( int d=0;d<descriptors.size().height;d++ )
			{
				predictionResult.push_back( rtree.predict( descriptors.row(d) ));
			}*/

			double t2 = msTime();
			cout<<endl<<"Number of keypoints in test picture "<<i<<": "<<predictionResult.size()<<", time used: "<<t2-t1<<"ms."<<endl;
			cout<<endl<<"Number of keypoints predicted as abf keypoints: "<< norm(predictionResult,NORM_L1)/predictionResult.size().height<<endl;
			cout<<"Descriptor test result with bayesian classifier: "<< predictionResult <<endl;
			if( !predictionResult.empty() ) percentageOfPositivePredictions.push_back( norm(predictionResult,NORM_L1)/predictionResult.size().height );
		}
	}

	ofstream tempstream( "positivePredForNegativeSetBayesian.txt", ios::out );
	tempstream<<percentageOfPositivePredictions;
	tempstream.close();

	//cout<<endl<<"descriptor set size: "<<descriptorSet.size()<<endl;

	/*Mat kdLengths;

	//create abs values of all keypointdescriptors (trying to find out informations about the area they cover)
	for(int i=0;i<keypointDescriptors.size();i++)
	{
		for(int j=0;j<keypointDescriptors[i].size().height;j++)
		{
			kdLengths.push_back( norm( keypointDescriptors[i].col(j) ) );
		}
	}
	cout<<endl<<"Number of calculated keypoint descriptor lengths: "<<kdLengths.size()<<endl;

	double minLength,maxLength;
	minMaxIdx(kdLengths,&minLength,&maxLength);
	cout<<endl<<"Min. length of keypoint descriptor: "<<minLength;
	cout<<endl<<"Max. length of keypoint descriptor: "<<maxLength;

	ofstream tempstream( "kdlength.txt", ios::out );
	tempstream<<kdLengths;
	tempstream.close();*/

	//matching
	BFMatcher bruteForce(NORM_L2,true);
	vector<DMatch> matches;

	double minDistance=10000;
	double maxDistance=0;
	// calculate minimal and maximal distance in hyperspace for all nearest pairs of keypoint descriptors
	for( int set=0; set<keypointDescriptors.size();set++ )
	{
		Mat keydescriptorset;
		for( int kd=0; kd<keypointDescriptors.size();kd++ )
		{
			if( kd!=set ) keydescriptorset.push_back( keypointDescriptors[kd] );
		}
		bruteForce.match( keypointDescriptors[set], keydescriptorset, matches );

		for( int match=0; match<matches.size();match++ )
		{
			if( matches[match].distance<minDistance && matches[match].distance!=0 ) minDistance=matches[match].distance;
			if( matches[match].distance>maxDistance ) maxDistance=matches[match].distance;
		}
	}

	//compare test sets against fixed distance boundaries
	

	
	string imageFolder = "D:\\Benutzer\\stewess\\Documents\\ETH\\FS 2014\\Bachelorarbeit\\negativ set\\";
	string imageName = "noabf - Kopie (";

	Mat thresholdResponse;

	/* calculate how many images have all of their descriptors inside of a fixed boundary, the boundary depending on an alternating threshold
	string temp1;
	string temp2;

	for( double threshold=0; threshold<3; threshold+=0.025 )
	{
		double upperBoundary = threshold*maxDistance;
		vector<double> positiveKeyPointMatches;

		int allDescriptorsMatch=0;

		cout<<endl<<upperBoundary;
		for( int fold=4; fold<5; fold++ )
		{
			for( int im=0; im<200; im++ )
			{
				convert<<im;
				convert>>temp1;
				convert.clear();

				convert<<fold;
				convert>>temp2;
				convert.clear();

				string imgname=imageFolder+imageName+temp1+").png";
				//cout<<endl<<imgname;
				Mat testImage = imread(imgname,CV_LOAD_IMAGE_COLOR);

				double t_1=msTime();
				Mat testDescriptors;
				vector<KeyPoint> testKeypoints;
				surfAlgorithm(testImage,Mat(),testKeypoints,testDescriptors); //extract descriptors

				vector<DMatch> testMatches;
				bruteForce.match(testDescriptors,descriptorSet,testMatches); //find nearest neighbours

				int positiveCount=0;
				for(int i=0;i<testMatches.size();i++)
				{
					if( testMatches[i].distance<=upperBoundary ) positiveCount++;
					if( testMatches[i].distance<0 ) cout<<endl<<"WTF IS GOING ON???!!!"<<endl;
				}
				double t_2=msTime();
				
				if( positiveCount==testKeypoints.size() && testKeypoints.size()!=0 ) allDescriptorsMatch++;
				
			}
		}
		double posKeyPntMatchAverage=0.0;
		for( int i=0; i<positiveKeyPointMatches.size();i++ ) posKeyPntMatchAverage+=positiveKeyPointMatches[i];
		cout<<endl<<allDescriptorsMatch;
		thresholdResponse.push_back(allDescriptorsMatch);
	}

	ofstream file("negativeThresholdAnswerAbsolute100.txt",ios::out);
	file<<thresholdResponse;
	file.close();
	*/

	/*cout<<endl<<positiveCount<<"/"<<testKeypoints.size()<<" keypoints positively matched in "<<t_2-t_1<<" ms."<<endl;

	Mat surfimage;
	drawKeypoints(testImage,testKeypoints,surfimage, Scalar::all(-1),4);
	namedWindow( "SURF output", WINDOW_AUTOSIZE );
	imshow( "SURF output", surfimage);
	waitKey(0);*/

	return;
}

Mat KeypointDescriptor::negDescriptors;