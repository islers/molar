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
	title: "Choose new type to be expected in scene"
	height: 400
    width: 600
	minimumHeight: 300
	minimumWidth: 600

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
					text: "New type to be expected in scene:"
					wrapMode: Text.Wrap
					font.family: "Helvetica"
					font.pointSize: 8
					color: "black"
				}

                ComboBox
                {
                    id: type_chooser
                    width: 400
                    model: ListModel
                           {
                                id: type_list 
                                ListElement
                                {
                                    text: "none";
                                    classid: "-1";
                                    infotext:"";
                                    dynamicsModule:"";
                                    creationDate:"";
                                    isGeneric:false;
                                    image:"";
                                }
                           }
                    property bool isSetup: false

                    Component.onCompleted: load();

                    Connections
                    {
                        target: p_inter
                        onGeneralSettingsChange: {type_chooser.load(); type_chooser.currentIndex=0;}
                        onSceneTypeChange: {type_chooser.load(); type_chooser.currentIndex=0;}
                    }

                    function load()
                    {
                        type_list.clear();
                        var types = p_inter.getNotAddedTypeList();
                        var typeIds = p_inter.getNotAddedTypeIds();
                        var typeInfos = p_inter.getNotAddedTypeInfo();
                        var typeDynamics = p_inter.getNotAddedDynamicsType();
                        var typeCreationDate = p_inter.getNotAddedCreationDate();
                        var typeIsGeneric = p_inter.getNotAddedIsGeneric();

                        for( var i=0; i < types.length ; i++ )
                        {
                            type_list.append( {"classid":typeIds[i], "text":types[i]+" ("+typeCreationDate[i]+")", "infotext":typeInfos[i], "dynamicsModule":typeDynamics[i], "creationDate":typeCreationDate[i], "isGeneric":typeIsGeneric[i] } );
                        }
                        type_infotext.refresh();
                        type_features.refresh();
                        isSetup = true
                    }
                    onCurrentIndexChanged:
                    {
                        if( isSetup )
                        {
                            type_infotext.refresh();
                            type_features.refresh();
                         }
                    }
                }

                Row
                {
                    TextEdit
                    {
                        id: type_infotext
                        width: 400
                        height: 50
                        text: ""
                        readOnly: true
                        wrapMode: TextEdit.WordWrap
                        textFormat: TextEdit.RichText
                        
                        function refresh()
                        {
                            var currentType = type_list.get(type_chooser.currentIndex);
                            type_infotext.text="<br /><b>Info:</b><br/>"+currentType.infotext+"<br/><b>Dynamics Module:</b> "+currentType.dynamicsModule+"<br/><b>Date of creation:</b> "+currentType.creationDate;
                        }
                    }
                }
                Item{height:50;width:10;}
                Text
				{
					text: "Feature sets of the type that are to be included:"
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

                            id: type_features
                            spacing: 10

				            model: ListModel{ id:feature_list }

                            property var featuresToUse : []

				            delegate: Row
				            {
                                property string featureid: featureId
                                CheckBox
                                {
                                    checked: true
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
                                        type_features.featuresToUse.push(featureid);
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

                                var currentId = type_list.get(type_chooser.currentIndex).classid

			                    var featureName = p_inter.getTypeFeatureSetNames( currentId );
                                var featureId = p_inter.getTypeFeatureSetIds( currentId );
                                var featureInfo = p_inter.getTypeFeatureSetInfos( currentId );
                                var featureCreationDate = p_inter.getTypeFeatureSetCreationDates( currentId );
                                var featureSize = p_inter.getTypeFeatureSetSize( currentId );

			                    for( var i=0; i<featureName.length; i++ )
			                    {
				                    feature_list.append( {"name":featureName[i], "info":featureInfo[i], "creationDate":featureCreationDate[i], "featureSize": featureSize[i], "featureId": featureId[i] } );
			                    }
		                    }
			            }
		            }
                }
                Row // buttons
                {
                    ModarButton
					{
						label: "Add"
						width: 50
						height: 25
						
						MouseArea
						{
							anchors.fill:parent;
							onClicked:
							{
                                var currentType = type_list.get(type_chooser.currentIndex)
                                if( type_features.featuresToUse.length==0 && currentType.isGeneric || currentType.classid=="-1" ) return;

                                assure.open();
								/*if( add_algorithm_window.isPreProcess ) p_inter.addPreProcessAlgorithm( algorithmName );
								else  p_inter.addInternPreProcessAlgorithm( algorithmName );*/
							}
                            property MessageDialog assure: MessageDialog
                            {
                                title: "Add new type?"
                                text: "Do you really want to add this type to the list of types expected to occur in the scene?"
                                informativeText: "Please be aware that if the given feature point configurations have never been used before, new classifiers will be trained. Depending on the number of included feature points this might take a while (several minutes...) and since no progression information is obtained to date, patience is advised. Doing so will also reinitialize the scene. Reinitializing will restart videos from the start, previous trajectory data will be lost. For camera sources tracking will restart with the same effects. Continue?"
                                standardButtons: StandardButton.Yes | StandardButton.No
                                onYes: {
                                    var currentType = type_list.get(type_chooser.currentIndex)
                                    waitScreen.open();
                                    var success = p_inter.addTypeToScene( currentType.classid, type_features.featuresToUse );
                                    waitScreen.close();
								    if( success )
                                    {
                                        add_type_window.visible = false;
                                        successfullyAdded.open();
                                    }
                                    else problemOccured.open();
                                }
                                onNo: {}
                                
                                property MessageDialog waitScreen: MessageDialog
                                {
                                    text: "New type is being added and classifiers are being trained if necessary. Have patience..."
                                    title: "New type addition... Have patience..."
                                }
                                property MessageDialog successfullyAdded: MessageDialog
                                {
                                    title: "New type was successfully added"
                                    text: "The addition of the new type has concluded successfully."
                                    standardButtons: StandardButton.Ok
                                }
                                property MessageDialog problemOccured: MessageDialog
                                {
                                    title: "New type could not be added"
                                    text: "A problem occured when MOLAR attempted to add your chosen type. One possible cause are missing dependency files."
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
