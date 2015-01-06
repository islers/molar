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
	id: add_type_window
	title: "Create new feature set"
	height: 480
    width: 620
	minimumHeight: 480
	minimumWidth: 620

    onVisibleChanged:
    {
        if( visible ) type_features.refresh();
        else
        {
            type_features.clear();
            p_inter.clearRecordingObj();
        }
    }

	Rectangle
	{
		anchors.fill: parent
        gradient: Gradient {
                GradientStop { position: 0.0; color: "#eff5f8" }
                GradientStop { position: 0.7; color: "#cadde8" }
        }
	}

	Item
	{
		anchors.fill: parent
		anchors.topMargin: 10
		anchors.rightMargin: 10
		anchors.bottomMargin: 10
		anchors.leftMargin: 10

		ScrollView
		{
			anchors.fill:parent
            contentItem: all_content
			
			Column
            {
                spacing:0
                id:all_content

                Text
				{
					text: "Number of recorded feature points: 0"
					wrapMode: Text.Wrap
					font.family: "Helvetica"
					font.pointSize: 8
					color: "black"
                    id: feature_number_text

                    function update()
                    {
                        var nrOfFeaturePoints = p_inter.nrOfRecordedFeatures();
                        feature_number_text.text = "Number of recorded feature points: "+nrOfFeaturePoints;
                    }

                    Timer
                    {
                        interval:100;
                        running: read_features_button.isRecording && add_type_window.visible;
                        repeat:true;
                        onTriggered:feature_number_text.update();
                    }
				}
                Row{ Item{height:2;width:10;}}
                Row
                {
                    Item{height:30;width:10;}
                    ModarButton
                    {
                        label: "Record"
                        property bool isRecording:false
                        id: read_features_button
                        height: 30
                        width: 105
                        MouseArea
                        {
                            anchors.fill:parent;
                            onClicked:
                            {
                                p_inter.recordFeatures();
                                read_features_button.isRecording = true;
                            }
                        }
                    }
                    ModarButton
                    {
                        label: "Stop"
                        id: stop_features_button
                        height: 30
                        width: 105
                        MouseArea
                        {
                            anchors.fill:parent;
                            onClicked:
                            {
                                p_inter.stopRecordingFeatures();
                                read_features_button.isRecording = false;
                            }
                        }
                    }
                    ModarButton
                    {
                        label: "Clear"
                        id: clear_features_button
                        height: 30
                        width: 105
                        MouseArea
                        {
                            anchors.fill:parent;
                            onClicked:
                            {
                                p_inter.clearFeatures();
                                read_features_button.isRecording = false;
                            }
                        }
                    }
                    Item{height:30;width:10;}
                }
                Item{height:15;width:10;}
                Row
                {
                    Text
					{
						text: "Feature set name: "
						wrapMode: Text.Wrap
						font.family: "Helvetica"
						font.pointSize: 8
						color: "black"
					}
                    Item{ width:10; height:10 }
                    TextField
                    {
                        width: 300
                        text: ""
                        id: feature_name
                    }
                }
                Item{height:15;width:10;}
                Row
                {
                    Text
					{
						text: "Description: "
						wrapMode: Text.Wrap
						font.family: "Helvetica"
						font.pointSize: 8
						color: "black"
					}
                    Item{ width:10; height:10 }
                    TextField
                    {
                        width: 300
                        text: ""
                        id: feature_description
                    }
                }
                Item{height:50;width:10;}
                Text
				{
					text: "Objects whose features are to be recorded (once recorded, object features can't be removed from set):"
					wrapMode: Text.Wrap
					font.family: "Helvetica"
					font.pointSize: 8
					color: "black"
				}
                Rectangle
                {
                    height:1;width:580;color:"black"
                }
                Item
                {
                    height:180; width:580;
                    ScrollView
		            {
			            anchors.fill: parent
                        anchors.topMargin: 10
			
			            ListView
			            {
				            anchors.fill: parent

                            id: type_features
                            spacing: 10

				            model: ListModel{ id:object_list }
                            property int currentMaxId: -1;

				            delegate: Row
				            {
                                property string objectid: objectId
                                CheckBox
                                {
                                    checked: false
                                    onClicked:
                                    {
                                        if( checked ) p_inter.recordObjFeatures(objectId);
                                        else p_inter.stopRecordingObjFeatures(objectid);
                                    }
                                }
						        Text
						        {
							        text: "object id#" + objectId
							        wrapMode: Text.Wrap
							        font.family: "Helvetica"
							        font.pointSize: 8
							        color: "black"
						        }
				            }
                            function refresh()
		                    {
                                var lowestId = p_inter.getLowestObjId();
			                    var highestId = p_inter.getHighestObjId();
                                var lowest = ( (type_features.currentMaxId+1)>lowestId ) ? (type_features.currentMaxId+1) : lowestId;
			                    for( var i=lowest; i<=highestId; i++ )
			                    {
				                    object_list.append( {"objectId":i } );
			                    }
                                type_features.currentMaxId = highestId;
		                    }
                            function clear()
                            {
                                object_list.clear();
                                type_features.currentMaxId = -1;
                            }

                            Connections
                            {
                                target: p_inter
                                onGeneralSettingsChange: type_features.clear()
                                onNewSource: type_features.clear();
                            }
                                                        
                            Timer
                            {
                                interval:1000;
                                running: add_type_window.visible;
                                repeat:true;
                                onTriggered:type_features.refresh();
                            }
			            }
		            }
                }
                Item{height:50;width:10;}
                Row // buttons
                {
                    ModarButton
					{
						label: "Create"
						width: 50
						height: 25
						
						MouseArea
						{
							anchors.fill:parent;
							onClicked:
							{
                                if( feature_name.text=="" ) nameMissing.open();
                                else assure.open();
							}
                            property MessageDialog nameMissing: MessageDialog
                            {
                                title: "New feature must have a name"
                                text: "Cannot create a new feature set without a name."
                                standardButtons: StandardButton.Ok
                            }

                            property MessageDialog assure: MessageDialog
                            {
                                title: "New feature set"
                                text: "Create a new feature set?"
                                standardButtons: StandardButton.Yes | StandardButton.No
                                onYes: {
                                    var success = p_inter.createFeatureSet( feature_name.text, feature_description.text);
								    if( success )
                                    {
                                        add_type_window.visible = false;
                                        successfullyCreated.open();

                                        type_features.clear();
                                        p_inter.clearRecordingObj();
                                        feature_name.text = "";
                                        feature_description.text = "";
                                    }
                                    else problemOccured.open();
                                }
                                onNo: {}

                                property MessageDialog successfullyCreated: MessageDialog
                                {
                                    title: "New feature set"
                                    text: "The new feature set has been created successfully and is now available for further use."
                                    standardButtons: StandardButton.Ok
                                }
                                property MessageDialog problemOccured: MessageDialog
                                {
                                    title: "Feature set creation failed"
                                    text: "A problem occured when MOLAR attempted to create the new feature set. It was either empty (contained no feature points) or there was an issue with writing to your hard disk (Insufficient permissions? Hard disk full?)."
                                    standardButtons: StandardButton.Ok
                                }

                            }
						}
					}
                }
            }
		}

	}
}
