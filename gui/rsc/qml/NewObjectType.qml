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
	title: "Define new object type"
	height: 520
    width: 600
	minimumHeight: 520
	minimumWidth: 600

    onVisibleChanged: type_features.refresh();

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

                Row
                {
                    Text
					{
						text: "Name: "
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
                        id: type_name
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
                        id: type_description
                    }
                }
                Item{height:15;width:10;}
                Row
                {
                    Text
					{
						text: "Dynamics module: "
						wrapMode: Text.Wrap
						font.family: "Helvetica"
						font.pointSize: 8
						color: "black"
					}
                    Column
                    {
                        ComboBox
                        {
                            id: module_chooser
                            width: 400
                            model: ListModel
                                   {
                                        id: module_list 
                                        ListElement
                                        {
                                            text: "none";
                                            info: "";
                                        }
                                   }

                            Component.onCompleted: load();

                            function load()
                            {
                                module_list.clear();
                                var dynamics = p_inter.availableDynamics();
                                var info = p_inter.dynamicsInfo();

                                for( var i=0; i < dynamics.length ; i++ )
                                {
                                    module_list.append( { "text":dynamics[i], "info":info[i] } );
                                }
                            }
                        }
                        Item{height:5;width:10;}
                        TextEdit
                        {
                            width: 400
                            text: module_list.get(module_chooser.currentIndex).info
                            readOnly: true
                            color: "grey"
                            wrapMode: TextEdit.WordWrap
                            textFormat: TextEdit.RichText
                        }
                    }
                }
                Item{height:15;width:10;}
                Row
                {
                    Text
					{
						text: "Type color: "
						wrapMode: Text.Wrap
						font.family: "Helvetica"
						font.pointSize: 8
						color: "black"
					}
                    Item{ width:10; height:10 }
                    Text
					{
						text: "R: "
						wrapMode: Text.Wrap
						font.family: "Helvetica"
						font.pointSize: 8
						color: "black"
					}
                    Item{ width:10; height:10 }
                    TextField
                    {
                        width: 30
                        text: ""
                        validator: IntValidator {bottom: 0; top: 255;}
                        id: type_color_r
                    }
                    Item{ width:10; height:10 }
                    Text
					{
						text: "G: "
						wrapMode: Text.Wrap
						font.family: "Helvetica"
						font.pointSize: 8
						color: "black"
					}
                    Item{ width:10; height:10 }
                    TextField
                    {
                        width: 30
                        text: ""
                        validator: IntValidator {bottom: 0; top: 255;}
                        id: type_color_g
                    }
                    Item{ width:10; height:10 }
                    Text
					{
						text: "B: "
						wrapMode: Text.Wrap
						font.family: "Helvetica"
						font.pointSize: 8
						color: "black"
					}
                    Item{ width:10; height:10 }
                    TextField
                    {
                        width: 30
                        text: ""
                        validator: IntValidator {bottom: 0; top: 255;}
                        id: type_color_b
                    }
                }

                Item{height:50;width:10;}
                Text
				{
					text: "Feature sets that define the new type \""+type_name.text+"\":"
					wrapMode: Text.Wrap
					font.family: "Helvetica"
					font.pointSize: 8
					color: "black"
				}
                Rectangle
                {
                    height:1;width:550;color:"black"
                }
                Item
                {
                    height:200;width:550;
                    ScrollView
		            {
			            anchors.fill: parent
                        anchors.topMargin: 10
			
			            ListView
			            {
				            anchors.fill: parent

                            id: type_features
                            spacing: 10

				            model: ListModel{ id:feature_list }

                            property var featuresToUse : []

				            delegate: Row
				            {
                                property string featureid: featureId
                                CheckBox
                                {
                                    checked: false
                                    onClicked:
                                    {
                                        if( checked ) type_features.featuresToUse.push(featureid);
                                        else
                                        {
                                            for( var i=0; i<type_features.featuresToUse.length; i++ )
                                            {
                                                if( type_features.featuresToUse[i]==featureid )
                                                {
                                                    type_features.featuresToUse.splice(i,1);
                                                }
                                            }
                                        }
                                    }
                                    Component.onCompleted:
                                    {
                                    }
                                }
						        Text
						        {
							        text: name
							        wrapMode: Text.Wrap
							        font.family: "Helvetica"
							        font.pointSize: 8
							        color: "black"
						        }
                                Item{height:5;width:10;}
						        TextEdit
						        {
							        text: info + "<br /><i>date of creation</i>: " + creationDate + "<br /><i>number of contained features</i>: " + featureSize;
                                    width: 400
                                    readOnly: true
                                    wrapMode: TextEdit.WordWrap
                                    textFormat: TextEdit.RichText
							        font.family: "Helvetica"
							        font.pointSize: 8
							        color: "black"
						        }
				            }
                            function refresh()
		                    {
                                feature_list.clear();
                                while( featuresToUse.length>0 ){ featuresToUse.pop();}
                                
			                    var featureName = p_inter.getAllFeatureSetNames();
                                var featureId = p_inter.getAllFeatureSetIds();
                                var featureInfo = p_inter.getAllFeatureSetInfos();
                                var featureCreationDate = p_inter.getAllFeatureSetCreationDates();
                                var featureSize = p_inter.getAllFeatureSetSize();

			                    for( var i=0; i<featureName.length; i++ )
			                    {
				                    feature_list.append( {"name":featureName[i], "info":featureInfo[i], "creationDate":featureCreationDate[i], "featureSize": featureSize[i], "featureId": featureId[i] } );
			                    }
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
                                if( type_name.text=="" || type_color_r.text=="" || type_color_g.text=="" || type_color_b.text==""  ) nameMissing.open();
                                else if( type_features.featuresToUse.length==0 ) featuresMissing.open();
                                else assure.open();
							}
                            property MessageDialog nameMissing: MessageDialog
                            {
                                title: "Information missing"
                                text: "Cannot create your new type definition: Name or color is missing."
                                standardButtons: StandardButton.Ok
                            }
                            property MessageDialog featuresMissing: MessageDialog
                            {
                                title: "Type definition requires features"
                                text: "Cannot create your new type definition without any feature sets that describe the type."
                                standardButtons: StandardButton.Ok
                            }

                            property MessageDialog assure: MessageDialog
                            {
                                title: "New type definition"
                                text: "Create a new type definition?"
                                standardButtons: StandardButton.Yes | StandardButton.No
                                onYes: {
                                    var name = type_name.text.split(' ').join('_');
                                    var success = p_inter.createType( name, type_color_r.text, type_color_g.text, type_color_b.text, module_list.get(module_chooser.currentIndex).text, type_features.featuresToUse, type_description.text );
								    if( success )
                                    {
                                        add_type_window.visible = false;
                                        successfullyCreated.open();

                                        type_name.text = "";
                                        type_color_r.text = "";
                                        type_color_g.text = "";
                                        type_color_b.text = "";
                                        module_chooser.currentIndex = 0;
                                        type_features.refresh();
                                        type_description.text = "";
                                    }
                                    else problemOccured.open();
                                }
                                onNo: {}

                                property MessageDialog successfullyCreated: MessageDialog
                                {
                                    title: "New type definition"
                                    text: "The new type definition has been created successfully and is now available for further use."
                                    standardButtons: StandardButton.Ok
                                }
                                property MessageDialog problemOccured: MessageDialog
                                {
                                    title: "Type definition creation failed"
                                    text: "A problem occured when MOLAR attempted to create the new type definition. There probably was an issue with writing to your hard disk (Insufficient permissions? Hard disk full?)."
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
