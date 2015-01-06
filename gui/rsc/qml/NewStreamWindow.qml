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

import QtQuick 2.3
import QtQuick.Window 2.1
import QtQuick.Controls 1.2
import QtQuick.Dialogs 1.1

Window
{
    id: stream_choice_window
	title: "Choose new video source"
	height: 150
    width: 620
	minimumHeight: 150
	minimumWidth: 620
                        
    onVisibleChanged:
    {
        if( visible ) types_to_export.refresh();
        else types_to_export.clear();
    }

	Rectangle
	{
		anchors.fill: parent
        gradient: Gradient {
                GradientStop { position: 0.0; color: "#eff5f8" }
                GradientStop { position: 0.7; color: "#cadde8" }
        }
	}

    ScrollView
	{
		anchors.fill:parent
        contentItem: all_content
			                

		Column
        {
            spacing:0
                                
            Item{ width:10; height:10 }
            Text
			{
				text: "Camera source:"
				wrapMode: Text.Wrap
				font.family: "Helvetica"
				font.pointSize: 8
                font.bold: true
				color: "black"
                anchors.left: parent.left
                anchors.leftMargin: 10
			}
            Rectangle
            {
                height:1;width:580;color:"black"
                anchors.left: parent.left
                anchors.leftMargin: 10
            }
            Item{ width:10; height:10 }
            Row
            {
                Item{ width:10; height:10 }
                Text
				{
					text: "device id (standard:0): "
					wrapMode: Text.Wrap
					font.family: "Helvetica"
					font.pointSize: 8
					color: "black"
				}
                Item{ width:10; height:10 }
                TextField
                {
                    width: 40
                    text: "0"
                    id: camera_id
                    validator: IntValidator {bottom: 0; top: 10000;}
                }
                Item{ width:30; height:10 }
                ModarButton
				{
					label: "open camera stream"
					width: 150
					height: 25

						
					MouseArea
					{
						anchors.fill:parent;
						onClicked:
						{
                            assure.open();
						}                    
                        property MessageDialog assure: MessageDialog
                        {
                            title: "Open camera stream"
                            text: "Opening a new camera stream will create a new scene with the current settings. Old trajectory and video data will be lost though. Continue?"
                            standardButtons: StandardButton.Yes | StandardButton.No
                            onYes:
                            {
                                p_inter.newCamSource(camera_id.text);
                                stream_choice_window.visible = false;
                            }
                            onNo:{}
                        }
                                            
					}
				}
            }               
            Item{height:20;width:10;}
            Text
			{
				text: "Video file source:"
				wrapMode: Text.Wrap
				font.family: "Helvetica"
				font.pointSize: 8
                font.bold: true
				color: "black"
                anchors.left: parent.left
                anchors.leftMargin: 10
			}
            Rectangle
            {
                height:1;width:580;color:"black"
                anchors.left: parent.left
                anchors.leftMargin: 10
            }
            Item{height:10;width:10;}
            Row // buttons
            {
                Item{height:10;width:10;}

                ModarButton
				{
					label: "choose file"
					width: 100
					height: 25

						
					MouseArea
					{
						anchors.fill:parent;
						onClicked:
						{
                            video_file_choice.open();
						}
                                            
					}

                    FileDialog
                    {
                        id: video_file_choice
                        title: "Choose video file to load"
                        selectExisting: true
                        selectMultiple: false
                        onAccepted: {
                            assure.open();
                        }
                        onRejected: {

                        }
                                            
                        property MessageDialog assure: MessageDialog
                        {
                            title: "Load new video"
                            text: "Loading a new video will create a new scene with the current settings. Old trajectory and video data will be lost though. Continue?"
                            standardButtons: StandardButton.Yes | StandardButton.No
                            onYes:
                            {
                                p_inter.newFileSource( video_file_choice.fileUrl );
                                stream_choice_window.visible = false;
                            }
                            onNo:{}
                        }
                    }
				}
                Item{height:10;width:10;}
                Text
				{
					text: "(file format support is platform dependent)"
					wrapMode: Text.Wrap
					font.family: "Helvetica"
					font.pointSize: 8
					color: "grey"
				}
            }
            Item{height:10;width:10;}
        }
    }
}
