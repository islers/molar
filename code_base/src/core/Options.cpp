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

#include "Options.h"



Options::Options(void)
{
}


Options::~Options(void)
{
}

void Options::load_options()
{
	if( General==NULL )
	{
		General = Options::setupGeneral();
	}
	return;
}

void Options::save_options()
{
	General->saveToXML( generalSettingsFilename );
}

GenericMultiLevelMap<string>* Options::setupGeneral()
{
	if( General!=NULL ) return General;
	
	//built in standard settings
	General = generalSettingsStandard();

	GenericMultiLevelMap<string> fromFile;

	try
	{
		fromFile.initFromXML( "generalsettings.xml" );
	}
	catch(...)
	{
		//built in standard settings
        cerr<<endl<<"Couldn't load generalsettings.xml file";
		return General;
	}
	// load settings from file -> only those entries are read that already exist in the standard setting map
	
    (*General).mirrorTwins( fromFile,GenericMultiLevelMap<string>::IGNOREEMPTYSTRING );
	return General;
}

void Options::resetGeneralSettings()
{
	if( General==NULL )
	{
		General = generalSettingsStandard();
	}
	else
	{
		GenericMultiLevelMap<string>* applicationStandard = generalSettingsStandard();
		(*General).mirrorTwins( *applicationStandard, GenericMultiLevelMap<string>::IGNOREEMPTYSTRING );
		(*applicationStandard).mirrorTwins( *General, GenericMultiLevelMap<string>::IGNOREEMPTYSTRING );
		delete General;
		General = applicationStandard;
	}

	return;
}

void Options::resetUserSettings()
{
	if( General!=NULL ) delete General;

	//built in standard settings
	General = generalSettingsStandard();

	GenericMultiLevelMap<string> fromFile;

	try
	{
		fromFile.initFromXML( "generalsettings.xml" );
	}
	catch(...)
	{
		//built in standard settings
		cerr<<endl<<"Couldn't load generalsettings.xml file";
	}
	// load settings from file -> only those entries are read that already exist in the standard setting map
	
	(*General).mirrorTwins( fromFile,GenericMultiLevelMap<string>::IGNOREEMPTYSTRING );
	
	return;
}


// built in standard settings
GenericMultiLevelMap<string>* Options::generalSettingsStandard()
{
	GenericMultiLevelMap<string>* General = new GenericMultiLevelMap<string>();

        (*General)["general_settings"]["program_folder"].as<string>()=""; // [0] ((unused currently))
	(*General)["general_settings"]["restore_last_state_at_startup"].as<bool>() = false; // [1]
	(*General)["general_settings"]["reset_object_count_on_reload"].as<bool>() = true; // [43] if set then the object id numbering will restart at zero if a new video or camera is chosen or the current scene is being reinitialized
    (*General)["general_settings"]["user_set_framerate"].as<int>() = 30; // [fps] if opencv's framerate method returns NaN, this framerate is used {affects:SceneHandler}

	(*General)["display"]["objects"]["path_length"].as<int>()=-2; // [2]  -2: draw all full paths into central image (faster than opt -1, about 0.5ms per object in the scene, paths of objects vanished stay in the image), -1: draw full paths per object (paths of vanished objects disappear with the object), 0: don't draw paths, >0: length of the drawn paths [frames back] (becomes slower with length, will only work if trace_states is true) {affects: ObjectHandler}
	(*General)["display"]["objects"]["path_fadeout_speed"].as<int>()=12; // [3]  for path_length options -1 and -2 a fadeout can be set: the higher path_fadeout_speed is, the slower the fadeout, negative values lead to no fadeout at all. No fadeout is computationally the fastest, if a fadeout is set then the slower the fadeout, the fewer computation time is needed {affects: ObjectHandler}
	(*General)["display"]["objects"]["path_step_length"].as<unsigned int>()=8; // [4]  if a path length is specified for path_length and not the whole paths are drawn, path drawing can be accelerated by specifying path_step_length to be different from 1: not to draw every history step but every i-th {affects: SceneObject}
	(*General)["display"]["objects"]["create_threshold_detection_image"].as<bool>()=false; // [5]  if true then the output of the last threshold operation with contours of detected objects is created and can be accessed through thresholdImage() in ObjectHandler {affects: ObjectHandler}
	(*General)["display"]["objects"]["draw_predicted_regions"].as<bool>()=false; // [6]  if true then the predicted region rectangles are drawn into the threshold detection image (if the latter is created, that is) {affects: ObjectHandler}
	(*General)["display"]["objects"]["create_preprocess_filter_image"].as<bool>()=false; // [7]  if true then the output after the first filter stage is buffered and can be accessed through preProcessImage() {affects: SceneHandler}
	(*General)["display"]["objects"]["create_prethreshold_filter_image"].as<bool>()=false; // [8]  if true then the output after the second, internal filter stage (right before thresholding) is buffered and can be accessed through preThresholdImage() {affects: SceneHandler}
	(*General)["display"]["general"]["draw_observation_area"].as<bool>()=true; // [9]  self explanatory {affects::SceneHandler}
	(*General)["display"]["general"]["draw_observation_area"]["R"].as<double>()=180; // [10]  color {affects::SceneHandler }
	(*General)["display"]["general"]["draw_observation_area"]["G"].as<double>()=55; // [11] 
	(*General)["display"]["general"]["draw_observation_area"]["B"].as<double>()=55; // [12] 
	
	(*General)["runtime"]["memory"]["max_video_ram_usage"].as<int>()=0; // [13] MB: maximal size of memory used for video frames {affects: VideoBuffer }
	(*General)["runtime"]["memory"]["temporary_folder_path"].as<string>()="temp"; // [14] {affects: VideoBuffer }

	(*General)["runtime"]["buffer"]["activated"].as<bool>()=true; // [15] ((leave as is! - not yet completely implemented)) if deactivated then no buffering at all takes place, only last frame is saved {affects: SceneHandler}
	(*General)["runtime"]["buffer"]["record_input"].as<bool>() = false; // [16] if set then everything that is put into the VideoBuffer gets recorded (lossless) until the program exits -> takes vast amount of hard disk space! {affects: VideoBuffer }
	
	(*General)["runtime"]["compression"]["format"].as<string>()=".PNG"; // [17] format used for compressing the temporary saved video stream on runtime, options: currently only .PNG, .JPEG (->openCV would support more)
	(*General)["runtime"]["compression"]["png_compression_level"].as<int>()=1; // [18] 0 to 9: openCV default is 3, higher compression levels take more time for computing
	(*General)["runtime"]["compression"]["jpeg_quality"].as<int>()=100; // [19] 0 to 100

	(*General)["video_content_descriptions"]["min_area"].as<double>()=100; // [20] [px^2], objects in image with smaller areas are not considered, unless their contour length is long enough (see contour_length_switch) {affects: ObjectHandler}
	(*General)["video_content_descriptions"]["max_area"].as<double>()=2000; // [21] [px^2], objects in image with larger areas are not considered {affects: ObjectHandler}
	(*General)["video_content_descriptions"]["contour_length_switch"].as<double>()=100; // [22] [px], objects in image with smaller areas than min_area but larger contour length than contour_length_switch will still be considered {affects: ObjectHandler}
	
	(*General)["object_detection"]["trace_states"].as<bool>()=true; // [23] defines whether states of detected objects are kept or not {affects: SceneObject and child classes}
	(*General)["object_detection"]["recalculation_interval"].as<unsigned int>()=30; // [24] [nr of frames] ((unused)) how often object positions are recalculated for the whole image (not just the predicted areas - every i-th frame) {affects: ObjectHandler}
	(*General)["object_detection"]["missing_state_bridging"].as<int>()=1; // [25] indicates if "artificial" states shall be added to objects at instants in time when they are missing, possible values are 0:no bridging (leads to 'holes' in state list of objects whenever it was missing), 1: successively keep predicting next state, based on previous prediction and use these to fill the void , 2: use the last known (old) state for bridging {affects: ObjectHandler}
	(*General)["object_detection"]["max_object_missing_time"].as<unsigned int>()=5; // [26] [nr of frames] maximal time an object that went missing is being searched before considered lost {affects: ObjectHandler}
	(*General)["object_detection"]["max_matching_distance_mismatch"].as<double>()=25; // [27] [px] maximal distance an objects center can be away from its predicted center before the match is refused {affects:ObjectHandler}
	(*General)["object_detection"]["use_old_state_fallback"].as<bool>()=true; // [28] if true then before calculating an overlap estimate between predicted object position and found contours, the old position will be checked if nothing was found at the predicted center position (this is one fast fallback option to handle possible prediction overshoots for the center point) {affects: ObjectHandler/SceneObject}
	(*General)["object_detection"]["max_object_gridpoint_distance"].as<double>()=5.0; // [29] [px] max distance between grid points in grid point generation over predicted object surface used to calculate surface overlap area estimation if using the center point position prediction (and old position) failed to produce a match for a known object {affects:ObjectHandler, SceneObject}
	(*General)["object_detection"]["static_threshold"].as<unsigned int>()=220; // [30] [color] determines which color value will be used in the object detection algorithm when thresholding the image to obtain a binary black and white {affects: ObjectHandler}
	(*General)["object_detection"]["window_border_range"].as<unsigned int>()=20; // [31] [px] defines the distance from the window border which is considered as the leaving area for objects: if an object is lost in this area, it is considered to have left the visible area {affects: ObjectHandler}

	(*General)["object_detection"]["ABFSpiral"]["velocity_moving_average_width"].as<unsigned int>()=16; // [32] number of samples used for moving average filter of velocity, must be a power of two (for speed purposes, shift operations are used instead of division...) {affects: ABFSpiral}
	(*General)["object_detection"]["ABFSpiral"]["angle_moving_average_width"].as<unsigned int>()=10; // [33] number of samples used for moving average filter of angle {affects: ABFSpiral}
	
	(*General)["object_detection"]["ThickHelix"]["velocity_moving_average_width"].as<unsigned int>()=16; // [34] number of samples used for moving average filter of velocity, must be a power of two (for speed purposes, shift operations are used instead of division...) {affects: ThickHelix}
	(*General)["object_detection"]["ThickHelix"]["angle_moving_average_width"].as<unsigned int>()=10; // [35] number of samples used for moving average filter of angle {affects: ABFSpiral}
		
	(*General)["object_detection"]["ThinHelix"]["velocity_moving_average_width"].as<unsigned int>()=16; // [36] number of samples used for moving average filter of velocity, must be a power of two (for speed purposes, shift operations are used instead of division...) {affects: ThinHelix}
	(*General)["object_detection"]["ThinHelix"]["angle_moving_average_width"].as<unsigned int>()=10; // [37] number of samples used for moving average filter of angle {affects: ABFSpiral}
			
	(*General)["object_detection"]["Crystal"]["velocity_moving_average_width"].as<unsigned int>()=16; // [38] number of samples used for moving average filter of velocity, must be a power of two (for speed purposes, shift operations are used instead of division...) {affects: Crystal}
	(*General)["object_detection"]["Crystal"]["angle_moving_average_width"].as<unsigned int>()=10; // [39] number of samples used for moving average filter of angle {affects: ABFSpiral}

	(*General)["classification"]["time_awareness"].as<bool>()=true; // [40] If true, then the classification functions keeps track of the remaining time postpones further classifications if the estimated classification time exceeds the actually available time. If in a feature point creation procedure all descriptors of all states during a recording time span are to be recorded, this should be deactivated. {affects:ObjectHandler}
	(*General)["classification"]["time_overhead"].as<double>()=5; // [41] [ms] How much time should be left after the classification processes for further processing {affects:ObjectHandler}
	(*General)["classification"]["standard_negative_class"].as<string>()="Unknown Type"; // [42] This is the class whose features will be used as negative descriptor set to train classifiers if only one generic object type is set to occur in a scene {affects: is directly used in GenericObject, thus no update is necessary on change}
	(*General)["classification"]["dynamic_feature_extraction_threshold_adaption"].as<bool>()=true; // [49] True: If the FAST keypoint extractor finds no keypoints for an image, it lowers the threshold as long as no keypoints are found {affects: GenericObject}
	(*General)["classification"]["feature_extraction_threshold"].as<int>()=60; // (currently needs a restart) Sets the standard feature extraction for the FAST algorithm  {affects: GenericObject}
	(*General)["classification"]["dynamic_feature_adaption_step"].as<int>()=5; // [50] By how much the threshold is lowered for each stelp during dynamic threshold adaption  {affects: GenericObject}

	(*General)["automatically_updated_program_data"]["classification_update_time_estimate"].as<double>() = 3; // [ms] time estimate for a classification update step - is automatically updated by the program and stored back to the variable when an ObjectHandler object is destroyed. The variable estimate is thus kept over several runs if the generalsettings.xml file is stored and loaded. {affects: ObjectHandler}
	(*General)["automatically_updated_program_data"]["last_stream_source"].as<int>() = -1;
	(*General)["automatically_updated_program_data"]["last_stream_source"]["is_cam"].as<bool>() = true;
	(*General)["automatically_updated_program_data"]["last_stream_source"]["exists"].as<bool>() = false;
	
	return General;
}

const std::string Options::generalSettingsFilename = "generalsettings.xml";
GenericMultiLevelMap<string>* Options::General = Options::setupGeneral();
