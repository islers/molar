// (c) May 2014, Stefan Isler, ETH Zürich, IRIS Institute of Robotics and Intelligent Systems

#pragma once


#include <iostream>
#include <string>
#include <list>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"

#include "SceneHandler.h"
#include "Options.h"

#include "stis/genericmultilevelmap.hpp"
#include "opencv2/nonfree/nonfree.hpp"



class KeypointDescriptor
{
public:
	KeypointDescriptor(void);
	~KeypointDescriptor(void);
	static void calculate();
	static void load();
	static void fastBriskProperties();
	static Mat negDescriptors;
};

