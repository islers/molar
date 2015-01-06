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
import QtQuick.Controls 1.2
import QtGraphicalEffects 1.0
import QtQuick.Dialogs 1.1

ScrollView
{
                            
    anchors.fill:parent
    frameVisible:true
    contentItem: scene_control_content

    id: scene_control_content_scroll
    property int textSize: 8
    property int headerSize: 8

    property alias showPreprocessed: show_preprocessed_image.checked
    property alias showInternPreprocessed: show_internpreprocessed_image.checked
    property alias showDetectionThresholded: show_detection_image.checked

    Column
    {
        spacing:1
        id: scene_control_content
        property int arrowposition: 180

        Row{ Item{height:10;width:10;}}
        
        
        // INFO /////////////////////////////////////////////////////////////////////                         
        Row
        {
            Item{height:10;width:10;}
            Text
            {
                text: "Info"
                property bool active: true
                wrapMode: Text.Wrap
                font.family: "Helvetica"
                font.bold: true
                font.pointSize: 8
                color: "black"
                id: info_text
                MouseArea
                {
                    anchors.fill:parent
                    hoverEnabled:true
                    onEntered:
                    {
                        info_img.source = info_text.active?"../img/arrow_up_w.png":"../img/arrow_w.png"
                        info_text.color = "white"
                    }
                    onExited:
                    {
                        info_img.source = info_text.active?"../img/arrow_up_b.png":"../img/arrow_b.png"
                        info_text.color = "black"
                    }
                    onClicked:
                    {
                        info_text.active = info_text.active?false:true
                        info_img.source = info_text.active?"../img/arrow_up_w.png":"../img/arrow_w.png"
                    }
                }
            }
            Item{height:10;width:scene_control_content.arrowposition-info_text.width;}
            Image
            {
                id: info_img
                fillMode: Image.PreserveAspectFit
                source: info_text.active?"../img/arrow_up_b.png":"../img/arrow_b.png"
            }
            Item{height:10;width:5;}
        }
        Row
        {
            Item{height:1;width:10;}
            Rectangle
            {
                height:1;width:210;color:"black"
            }
            Item{height:1;width:10;}
        }
        Row{ Item{height:2;width:10;}}
        Row
        {
            visible: info_text.active
            Item{height:10;width:10;}
            Text
            {
                text: "Current source: None"
                id: current_source_title
                wrapMode: Text.Wrap
                font.family: "Helvetica"
                font.pointSize: 8
                color: "black"

                Connections
                {
                    target: p_inter
                    onNewSource: current_source_title.text = "Current source: "+newsource
                }
            }
            Item{height:10;width:10;}
        }
        Row{ Item{height:2;width:10;}}
        Row
        {
            visible: info_text.active
            Item{height:30;width:10;}
            Text
            {
                text: "Frame size:"
                id: info_frame_size
                wrapMode: Text.Wrap
                font.family: "Helvetica"
                font.pointSize: 8
                color: "black"

                Connections
                {
                    target: p_inter
                    onNewSource: info_frame_size.text = "Frame size: "+p_inter.frameSize()+" [px]";
                }
            }
            Item{height:30;width:10;}
        }
        Row{ Item{height:10;width:10;}}
        
        
        
        // TYPE SETTINGS /////////////////////////////////////////////////////////////////////                         
        Row
        {
            Item{height:10;width:10;}
            Text
            {
                text: "Type Configuration"
                property bool active: true
                wrapMode: Text.Wrap
                font.family: "Helvetica"
                font.bold: true
                font.pointSize: 8
                color: "black"
                id: typeconfiguration_text
                MouseArea
                {
                    anchors.fill:parent
                    hoverEnabled:true
                    onEntered:
                    {
                        typeconfiguration_img.source = typeconfiguration_text.active?"../img/arrow_up_w.png":"../img/arrow_w.png"
                        typeconfiguration_text.color = "white"
                    }
                    onExited:
                    {
                        typeconfiguration_img.source = typeconfiguration_text.active?"../img/arrow_up_b.png":"../img/arrow_b.png"
                        typeconfiguration_text.color = "black"
                    }
                    onClicked:
                    {
                        typeconfiguration_text.active = typeconfiguration_text.active?false:true
                        typeconfiguration_img.source = typeconfiguration_text.active?"../img/arrow_up_w.png":"../img/arrow_w.png"
                    }
                }
            }
            Item{height:10;width:scene_control_content.arrowposition-typeconfiguration_text.width;}
            Image
            {
                id: typeconfiguration_img
                fillMode: Image.PreserveAspectFit
                source: transport_text.active?"../img/arrow_up_b.png":"../img/arrow_b.png"
            }
            Item{height:10;width:5;}
        }
        Row
        {
            Item{height:1;width:10;}
            Rectangle
            {
                height:1;width:210;color:"black"
            }
            Item{height:1;width:10;}
        }        
        Row{ Item{height:2;width:10;}}
        Row
        {
            visible: typeconfiguration_text.active
            Item{height:10;width:10;}
            Text
            {
                text: "types expected in scene:"
                wrapMode: Text.Wrap
                font.family: "Helvetica"
                font.pointSize: 7
                color: "black"
            }
            Item{height:10;width:10;}
        }
        Row{ Item{height:2;width:10;}}
        Row
        {
            visible: (types_in_scene.count==0) && typeconfiguration_text.active
            Item{height:10;width:10;}
            Rectangle
            {
                    width: 210
                    height: 15
                    color:"white"
                    border.color:"#9db9c9"
                    Text
                    {
                        text: "none"
                        wrapMode: Text.Wrap
                        font.family: "Helvetica"
                        font.pointSize: 8
                        color: "black"
                        anchors.fill:parent
                        anchors.leftMargin: 5
                    }
            }
        }
        Row
        {
            visible: typeconfiguration_text.active
            Item{height:10;width:10;}
            Item
            {
                //color: "white"
                height: types_in_scene.count*15+10
                width: 210
                //border.color: "black"
                //border.width: 1
                
                
                ListView
                {
                    id: types_in_scene
                    anchors.fill: parent
                    spacing: 0

                    model: ListModel{ id:types_list }

                    delegate: Row
                    {
                        property string type: name

                        Rectangle
                        {
                            color:"white"
                            border.color:"#9db9c9"
                            height: 15
                            width: 195//180
                            clip: true
                
                            Text
                            {
                                text: name
                                wrapMode: Text.Wrap
                                font.family: "Helvetica"
                                font.pointSize: 8
                                color: "black"
                                anchors.fill:parent
                                anchors.leftMargin: 5
                            }
                        }
                        /*WhiteButton
                        {
                            height: 15
                            width: 15

                            Image
                            {
                                anchors.horizontalCenter: parent.horizontalCenter
                                anchors.verticalCenter: parent.verticalCenter
                                fillMode: Image.PreserveAspectFit
                                source: "img/edit.png"
                            }
                            
                            MouseArea{ anchors.fill:parent; onClicked: algorithm_settings.visible = true }

                            AlgorithmSetting
                            {
                                id: algorithm_settings
                                visible: false
                                algorithmName: name
                                index: position
                                isPreProcess: true
                            }
                        }*/
                        WhiteButton
                        {
                            label: "x"
                            height: 15
                            width: 15

                            MouseArea{ anchors.fill:parent; onClicked: parent.assure.open() }

                            property MessageDialog assure: MessageDialog
                            {
                                title: "Remove type?"
                                text: "Do you really want to remove this type from the list of types expected to occur in the scene? This will also reinitialize the scene. Reinitializing will restart videos from the start, previous trajectory data will be lost. For camera sources tracking will restart with the same effects. Furthermore new classifier training might be necessary if the new type configuration hasn't been used before, which might require some time. Continue?"
                                standardButtons: StandardButton.Yes | StandardButton.No
                                onYes: {
                                    p_inter.removeTypeFromScene(name);
                                }
                                onNo: {}
                                //Component.onCompleted: visible = true
                            }
                        }
                    }
                    Connections
                    {
                        target: p_inter
                        onSceneTypeChange:
                        {
                            types_list.clear();
                            var typesInScene = p_inter.typesOccuringInScene();

			                for( var i=0; i<typesInScene.length; i++ )
			                {
                                types_list.append({ "name":typesInScene[i] });
			                }
                        }
                    }
                }
            }
            Item{height:10;width:10;}
        }
        Row
        {
            visible: typeconfiguration_text.active
            Item{height:10;width:200;}
            ModarButton
            {
                label: "+"
                id: add_type_button
                height: 20
                width: 20

                MouseArea{ anchors.fill:parent; onClicked: add_type_window.visible = true }

                TypeChooser
                {
                    id: add_type_window
                    visible: false
                }
            }
        }
        Row{ Item{height:10;width:10;}}

        // TRANSPORT /////////////////////////////////////////////////////////////////////                         
        Row
        {
            Item{height:10;width:10;}
            Text
            {
                text: "Transport"
                property bool active: true
                wrapMode: Text.Wrap
                font.family: "Helvetica"
                font.bold: true
                font.pointSize: 8
                color: "black"
                id: transport_text
                MouseArea
                {
                    anchors.fill:parent
                    hoverEnabled:true
                    onEntered:
                    {
                        transport_img.source = transport_text.active?"../img/arrow_up_w.png":"../img/arrow_w.png"
                        transport_text.color = "white"
                    }
                    onExited:
                    {
                        transport_img.source = transport_text.active?"../img/arrow_up_b.png":"../img/arrow_b.png"
                        transport_text.color = "black"
                    }
                    onClicked:
                    {
                        transport_text.active = transport_text.active?false:true
                        transport_img.source = transport_text.active?"../img/arrow_up_w.png":"../img/arrow_w.png"
                    }
                }
            }
            Item{height:10;width:scene_control_content.arrowposition-transport_text.width;}
            Image
            {
                id: transport_img
                fillMode: Image.PreserveAspectFit
                source: transport_text.active?"../img/arrow_up_b.png":"../img/arrow_b.png"
            }
            Item{height:10;width:5;}
        }
        Row
        {
            Item{height:1;width:10;}
            Rectangle
            {
                height:1;width:210;color:"black"
            }
            Item{height:1;width:10;}
        }
        Row{ Item{height:2;width:10;}}
        Row
        {
            visible: transport_text.active

            Item{height:30;width:10;}
            ModarButton
            {
                label: "Read stream"
                property bool isRecording:false
                id: read_stream_button
                height: 30
                width: 105
                MouseArea{ anchors.fill:parent; onClicked: {p_inter.read(); read_stream_button.isRecording=true;} }
            }
            ModarButton
            {
                label: "Stop"
                id: stop_stream_button
                height: 30
                width: 105
                MouseArea{ anchors.fill:parent; onClicked: {p_inter.stop(); recording_sign.active = false; read_stream_button.isRecording=false; } }
            }
            Item{height:30;width:10;}
        }
        Row
        {
            visible: transport_text.active
            Item{height:30;width:10;}
            ModarButton
            {
                label: "Reinitialize"
                height: 30
                width: 105
                MouseArea{ anchors.fill:parent; onClicked: reinitialize_check.open(); }

                MessageDialog 
                {
                    id: reinitialize_check
                    title: "Reinitialize scene?"
                    text: "Reinitializing will restart videos from the start, previous trajectory data will be lost. For camera sources tracking will restart with the same effects. Continue?"
                    standardButtons: StandardButton.Yes | StandardButton.No
                    onYes: {
                        p_inter.reinitialize();
                    }
                    onNo: {}
                    //Component.onCompleted: visible = true
                }
            }
            Item{height:30;width:10;}
        }
        Row
        {
            visible: transport_text.active
            Item{height:20;width:10;}
            ModarButton
            {
                label: "Record to video"
                height: 20
                width: 105
                MouseArea{ anchors.fill:parent; onClicked: {p_inter.record(); recording_sign.active = true;} }
            }
            ModarButton
            {
                label: "Stop recording"
                height: 20
                width: 105
                MouseArea{ anchors.fill:parent; onClicked: {p_inter.stopRecording(); recording_sign.active = false;} }
            }
            Item{height:20;width:10;}
        }
        Row
        {
            visible: transport_text.active
            Item{height:20;width:10;}
            ModarButton
            {
                label: "Clear video buffer"
                height: 20
                width: 105
                MouseArea{ anchors.fill:parent; onClicked: clear_video_buffer_check.open() }
                
                MessageDialog 
                {
                    id: clear_video_buffer_check
                    title: "Clear video buffer?"
                    text: "If you clear the video buffer the beforehand recorded data will be deleted. Continue?"
                    standardButtons: StandardButton.Yes | StandardButton.No
                    onYes: {
                        p_inter.clearVideoBuffer();
                        recording_sign.active = false;
                    }
                    onNo: {}
                    //Component.onCompleted: visible = true
                }
            }
            Item{height:20;width:10;}
        }
        Row{ Item{height:2;width:10;}}

        Row
        {
            Item{height:10;width:80;}
            Text
            {
                text: "Recording..."
                visible: active && visibilitySwitch
                property bool active: false
                property bool visibilitySwitch: false
                id: recording_sign
                wrapMode: Text.Wrap
                font.family: "Helvetica"
                font.pointSize: 8
                color: "red"

                Timer
                {
                    interval:1000; running: recording_sign.active; repeat:true;
                    onTriggered: recording_sign.visibilitySwitch = recording_sign.visibilitySwitch?false:true;
                }
            }
            Item{height:10;width:10;}
        }
        
        // PREPROCESSING /////////////////////////////////////////////////////////////////////                         
        Row
        {
            Item{height:10;width:10;}
            Text
            {
                text: "Preprocessing"
                property bool active: false
                wrapMode: Text.Wrap
                font.family: "Helvetica"
                font.bold: true
                font.pointSize: 8
                color: "black"
                id: preprocessing_text
                MouseArea
                {
                    anchors.fill:parent
                    hoverEnabled:true
                    onEntered:
                    {
                        preprocessing_img.source = preprocessing_text.active?"../img/arrow_up_w.png":"../img/arrow_w.png"
                        preprocessing_text.color = "white"
                    }
                    onExited:
                    {
                        preprocessing_img.source = preprocessing_text.active?"../img/arrow_up_b.png":"../img/arrow_b.png"
                        preprocessing_text.color = "black"
                    }
                    onClicked:
                    {
                        preprocessing_text.active = preprocessing_text.active?false:true
                        preprocessing_img.source = preprocessing_text.active?"../img/arrow_up_w.png":"../img/arrow_w.png"
                    }
                }
            }
            Item{height:10;width:scene_control_content.arrowposition-preprocessing_text.width;}
            Image
            {
                id: preprocessing_img
                fillMode: Image.PreserveAspectFit
                source: preprocessing_text.active?"../img/arrow_up_b.png":"../img/arrow_b.png"
            }
            Item{height:10;width:5;}
        }
        Row
        {
            Item{height:1;width:10;}
            Rectangle
            {
                height:1;width:210;color:"black"
            }
            Item{height:1;width:10;}
        }
        Row{ Item{height:2;width:10;}}
        Row
        {
            visible: preprocessing_text.active
            Item{height:10;width:10;}
            Text
            {
                text: "show preprocessed image"
                wrapMode: Text.Wrap
                font.family: "Helvetica"
                font.pointSize: 8
                color: "black"
            }
            Item{height:10;width:10;}
            CheckBox
            {
                id: show_preprocessed_image
                
                Connections
                {
                    target: p_inter
                    onGeneralSettingsChange: show_preprocessed_image.checked = p_inter.boolSignal(7, false);
                }
                onClicked: p_inter.boolSignal(7,true,show_preprocessed_image.checked);
            }
            Item{height:10;width:10;}
        }
        Row{ Item{height:2;width:10;}}
        Row
        {
            visible: preprocessing_text.active
            Item{height:10;width:10;}
            Text
            {
                text: "added algorithms:"
                wrapMode: Text.Wrap
                font.family: "Helvetica"
                font.pointSize: 7
                color: "black"
            }
            Item{height:10;width:10;}
        }
        Row{ Item{height:2;width:10;}}
        Row
        {
            visible: (preprocessing_algorithms.count==0) && preprocessing_text.active
            Item{height:10;width:10;}
            Rectangle
            {
                    width: 210
                    height: 15
                    color:"white"
                    border.color:"#9db9c9"
                    Text
                    {
                        text: "none"
                        wrapMode: Text.Wrap
                        font.family: "Helvetica"
                        font.pointSize: 8
                        color: "black"
                        anchors.fill:parent
                        anchors.leftMargin: 5
                    }
            }
        }
        Row
        {
            visible: preprocessing_text.active
            Item{height:10;width:10;}
            Item
            {
                //color: "white"
                height: preprocessing_algorithms.count*15+10
                width: 210
                //border.color: "black"
                //border.width: 1
                
                
                ListView
                {
                    id: preprocessing_algorithms
                    anchors.fill: parent
                    spacing: 0

                    model: AlgorithmListModel{ id:pre_process_list }

                    delegate: Row
                    {
                        property int pos: position

                        Rectangle
                        {
                            color:"white"
                            border.color:"#9db9c9"
                            height: 15
                            width: 150
                            clip: true
                
                            Text
                            {
                                text: name
                                wrapMode: Text.Wrap
                                font.family: "Helvetica"
                                font.pointSize: 8
                                color: "black"
                                anchors.fill:parent
                                anchors.leftMargin: 5
                            }
                        }
                        WhiteButton
                        {
                            height: 15
                            width: 15

                            Image
                            {
                                anchors.horizontalCenter: parent.horizontalCenter
                                anchors.verticalCenter: parent.verticalCenter
                                fillMode: Image.PreserveAspectFit
                                source: "../img/edit.png"
                            }
                            
                            MouseArea{ anchors.fill:parent; onClicked: algorithm_settings.visible = true }

                            AlgorithmSetting
                            {
                                id: algorithm_settings
                                visible: false
                                algorithmName: name
                                index: position
                                isPreProcess: true
                            }
                        }
                        WhiteButton
                        {
                            height: 15
                            width: 15

                            MouseArea{ anchors.fill:parent; onClicked: p_inter.movePreProcessAlgorithmDown(pos) }

                            Image
                            {
                                anchors.horizontalCenter: parent.horizontalCenter
                                anchors.verticalCenter: parent.verticalCenter
                                fillMode: Image.PreserveAspectFit
                                source: "../img/little_arrow_down.png"
                            }
                        }
                        WhiteButton
                        {
                            height: 15
                            width: 15

                            MouseArea{ anchors.fill:parent; onClicked: p_inter.movePreProcessAlgorithmUp(pos) }

                            Image
                            {
                                anchors.horizontalCenter: parent.horizontalCenter
                                anchors.verticalCenter: parent.verticalCenter
                                fillMode: Image.PreserveAspectFit
                                source: "../img/little_arrow_up.png"
                            }
                        }
                        WhiteButton
                        {
                            label: "x"
                            height: 15
                            width: 15

                            MouseArea{ anchors.fill:parent; onClicked: parent.assure.open() }

                            property MessageDialog assure: MessageDialog
                            {
                                title: "Remove algorithm?"
                                text: "Do you really want to remove the algorithm?"
                                standardButtons: StandardButton.Yes | StandardButton.No
                                onYes: {
                                    p_inter.removePreProcessAlgorithm(pos);
                                }
                                onNo: {}
                                //Component.onCompleted: visible = true
                            }
                        }
                    }
                    Connections
                    {
                        target: p_inter
                        onPreProcessAlgorithmChange:
                        {
                            pre_process_list.clear();
                            var nrOfAlgorithms = p_inter.nrOfPreProcessAlgorithms();
                            for( var i=0; i<nrOfAlgorithms; i++ )
                            {
                                pre_process_list.append({ "name":p_inter.preProcessAlgorithmName(i), "position":i });
                            }
                        }
                    }
                }
            }
            Item{height:10;width:10;}
        }
        Row
        {
            visible: preprocessing_text.active
            Item{height:10;width:200;}
            ModarButton
            {
                label: "+"
                id: add_algorithm_button
                height: 20
                width: 20

                MouseArea{ anchors.fill:parent; onClicked: add_algorithm_window.visible = true }

                AlgorithmChooser
                {
                    id: add_algorithm_window
                    visible: false
                    isPreProcess: true
                }
            }
        }
        Row{ Item{height:10;width:10;}}
        
        
        // INTERNAL PREPROCESSING /////////////////////////////////////////////////////////////////////                         
        Row
        {
            Item{height:10;width:10;}
            Text
            {
                text: "Prethresholdprocessing"
                property bool active: false
                wrapMode: Text.Wrap
                font.family: "Helvetica"
                font.bold: true
                font.pointSize: 8
                color: "black"
                id: internpreprocessing_text
                MouseArea
                {
                    anchors.fill:parent
                    hoverEnabled:true
                    onEntered:
                    {
                        internpreprocessing_img.source = internpreprocessing_text.active?"../img/arrow_up_w.png":"../img/arrow_w.png"
                        internpreprocessing_text.color = "white"
                    }
                    onExited:
                    {
                        internpreprocessing_img.source = internpreprocessing_text.active?"../img/arrow_up_b.png":"../img/arrow_b.png"
                        internpreprocessing_text.color = "black"
                    }
                    onClicked:
                    {
                        internpreprocessing_text.active = internpreprocessing_text.active?false:true
                        internpreprocessing_img.source = internpreprocessing_text.active?"../img/arrow_up_w.png":"../img/arrow_w.png"
                    }
                }
            }
            Item{height:10;width:scene_control_content.arrowposition-internpreprocessing_text.width;}
            Image
            {
                id: internpreprocessing_img
                fillMode: Image.PreserveAspectFit
                source: internpreprocessing_text.active?"../img/arrow_up_b.png":"../img/arrow_b.png"
            }
            Item{height:10;width:5;}
        }
        Row
        {
            Item{height:1;width:10;}
            Rectangle
            {
                height:1;width:210;color:"black"
            }
            Item{height:1;width:10;}
        }
        Row{ Item{height:2;width:10;}}
        Row
        {
            visible: internpreprocessing_text.active
            Item{height:10;width:10;}
            Text
            {
                text: "show prethreshold image"
                wrapMode: Text.Wrap
                font.family: "Helvetica"
                font.pointSize: 8
                color: "black"
            }
            Item{height:10;width:10;}
            CheckBox
            {
                id: show_internpreprocessed_image
                
                Connections
                {
                    target: p_inter
                    onGeneralSettingsChange: show_internpreprocessed_image.checked = p_inter.boolSignal(8, false);
                }
                onClicked: p_inter.boolSignal(8,true,show_internpreprocessed_image.checked);
            }
            Item{height:10;width:10;}
        }
        Row{ Item{height:2;width:10;}}
        Row
        {
            visible: internpreprocessing_text.active
            Item{height:10;width:10;}
            Text
            {
                text: "added algorithms:"
                wrapMode: Text.Wrap
                font.family: "Helvetica"
                font.pointSize: 7
                color: "black"
            }
            Item{height:10;width:10;}
        }
        Row{ Item{height:2;width:10;}}
        Row
        {
            visible: (internpreprocessing_algorithms.count==0) && internpreprocessing_text.active
            Item{height:10;width:10;}
            Rectangle
            {
                    width: 210
                    height: 15
                    color:"white"
                    border.color:"#9db9c9"
                    Text
                    {
                        text: "none"
                        wrapMode: Text.Wrap
                        font.family: "Helvetica"
                        font.pointSize: 8
                        color: "black"
                        anchors.fill:parent
                        anchors.leftMargin: 5
                    }
            }
        }
        Row
        {
            visible: internpreprocessing_text.active
            Item{height:10;width:10;}
            Item
            {
                //color: "white"
                height: internpreprocessing_algorithms.count*15+10
                width: 210
                //border.color: "black"
                //border.width: 1
                
                
                ListView
                {
                    id: internpreprocessing_algorithms
                    anchors.fill: parent
                    spacing: 0

                    model: AlgorithmListModel{ id:intern_pre_process_list }

                    delegate: Row
                    {
                        property int pos: position

                        Rectangle
                        {
                            color:"white"
                            border.color:"#9db9c9"
                            height: 15
                            width: 150
                            clip: true
                
                            Text
                            {
                                text: name
                                wrapMode: Text.Wrap
                                font.family: "Helvetica"
                                font.pointSize: 8
                                color: "black"
                                anchors.fill:parent
                                anchors.leftMargin: 5
                            }
                        }
                        WhiteButton
                        {
                            height: 15
                            width: 15

                            Image
                            {
                                anchors.horizontalCenter: parent.horizontalCenter
                                anchors.verticalCenter: parent.verticalCenter
                                fillMode: Image.PreserveAspectFit
                                source: "../img/edit.png"
                            }
                            
                            MouseArea{ anchors.fill:parent; onClicked: intern_algorithm_settings.visible = true }

                            AlgorithmSetting
                            {
                                id: intern_algorithm_settings
                                visible: false
                                algorithmName: name
                                index: position
                                isPreProcess: false
                            }
                        }
                        WhiteButton
                        {
                            height: 15
                            width: 15

                            MouseArea{ anchors.fill:parent; onClicked: p_inter.moveInternPreProcessAlgorithmDown(pos) }

                            Image
                            {
                                anchors.horizontalCenter: parent.horizontalCenter
                                anchors.verticalCenter: parent.verticalCenter
                                fillMode: Image.PreserveAspectFit
                                source: "../img/little_arrow_down.png"
                            }
                        }
                        WhiteButton
                        {
                            height: 15
                            width: 15

                            MouseArea{ anchors.fill:parent; onClicked: p_inter.moveInternPreProcessAlgorithmUp(pos) }

                            Image
                            {
                                anchors.horizontalCenter: parent.horizontalCenter
                                anchors.verticalCenter: parent.verticalCenter
                                fillMode: Image.PreserveAspectFit
                                source: "../img/little_arrow_up.png"
                            }
                        }
                        WhiteButton
                        {
                            label: "x"
                            height: 15
                            width: 15

                            MouseArea{ anchors.fill:parent; onClicked: parent.assure.open() }

                            property MessageDialog assure: MessageDialog
                            {
                                title: "Remove algorithm?"
                                text: "Do you really want to remove the algorithm?"
                                standardButtons: StandardButton.Yes | StandardButton.No
                                onYes: {
                                    p_inter.removeInternPreProcessingAlgorithm(pos);
                                }
                                onNo: {}
                                //Component.onCompleted: visible = true
                            }
                        }
                    }
                    Connections
                    {
                        target: p_inter
                        onInternPreProcessAlgorithmChange:
                        {
                            intern_pre_process_list.clear();
                            var nrOfAlgorithms = p_inter.nrOfInternPreProcessAlgorithms();
                            for( var i=0; i<nrOfAlgorithms; i++ )
                            {
                                intern_pre_process_list.append({ "name":p_inter.internPreProcessAlgorithmName(i), "position":i });
                            }
                        }
                    }
                }
            }
            Item{height:10;width:10;}
        }
        Row
        {
            visible: internpreprocessing_text.active
            Item{height:10;width:200;}
            ModarButton
            {
                label: "+"
                id: intern_add_algorithm_button
                height: 20
                width: 20

                MouseArea{ anchors.fill:parent; onClicked: intern_add_algorithm_window.visible = true }

                AlgorithmChooser
                {
                    id: intern_add_algorithm_window
                    visible: false
                    isPreProcess: false
                }
            }
        }
        Row{ Item{height:10;width:10;}}
        
        
        // THRESHOLDING AND DETECTION /////////////////////////////////////////////////////////////////////                         
        Row
        {
            Item{height:10;width:10;}
            Text
            {
                text: "Thresholding and Detection"
                property bool active: false
                wrapMode: Text.Wrap
                font.family: "Helvetica"
                font.bold: true
                font.pointSize: 8
                color: "black"
                id: thresholdingdetection_text
                MouseArea
                {
                    anchors.fill:parent
                    hoverEnabled:true
                    onEntered:
                    {
                        thresholdingdetection_img.source = thresholdingdetection_text.active?"../img/arrow_up_w.png":"../img/arrow_w.png"
                        thresholdingdetection_text.color = "white"
                    }
                    onExited:
                    {
                        thresholdingdetection_img.source = thresholdingdetection_text.active?"../img/arrow_up_b.png":"../img/arrow_b.png"
                        thresholdingdetection_text.color = "black"
                    }
                    onClicked:
                    {
                        thresholdingdetection_text.active = thresholdingdetection_text.active?false:true
                        thresholdingdetection_img.source = thresholdingdetection_text.active?"../img/arrow_up_w.png":"../img/arrow_w.png"
                    }
                }
            }
            Item{height:10;width:scene_control_content.arrowposition-thresholdingdetection_text.width;}
            Image
            {
                id: thresholdingdetection_img
                fillMode: Image.PreserveAspectFit
                source: thresholdingdetection_text.active?"../img/arrow_up_b.png":"../img/arrow_b.png"
            }
            Item{height:10;width:5;}
        }
        Row
        {
            Item{height:1;width:10;}
            Rectangle
            {
                height:1;width:210;color:"black"
            }
            Item{height:1;width:10;}
        }
        Row{ Item{height:2;width:10;}}
        Row
        {
            visible: thresholdingdetection_text.active
            Item{height:10;width:10;}
            Text
            {
                text: "show detection image"
                wrapMode: Text.Wrap
                font.family: "Helvetica"
                font.pointSize: 8
                color: "black"
            }
            Item{height:10;width:10;}
            CheckBox
            {
                id: show_detection_image
                
                Connections
                {
                    target: p_inter
                    onGeneralSettingsChange: show_detection_image.checked = p_inter.boolSignal(5, false);
                }
                onClicked: p_inter.boolSignal(5,true,show_detection_image.checked);
            }
            Item{height:10;width:10;}
        }
        Row{ Item{height:2;width:10;}}
        Row
        {
            visible: thresholdingdetection_text.active
            Item{height:10;width:10;}
            Text
            {
                text: "Threshold [0...255]"
                wrapMode: Text.Wrap
                font.family: "Helvetica"
                font.pointSize: 8
                color: "black"
            }
            Item{height:10;width:10;}
            TextField
            {
                id: param_44
                width: 35
                validator: IntValidator {bottom: 0; top: 255;}
                text:""
                onEditingFinished: p_inter.doubleSignal(44,true,param_44.text);
                Connections
                {
                    target: p_inter
                    onGeneralSettingsChange: param_44.text = p_inter.doubleSignal(44,false);
                }
            }
            Item{height:10;width:10;}
        }
        Row{ Item{height:2;width:10;}}
        Row
        {
            visible: thresholdingdetection_text.active
            Item{height:10;width:10;}
            Text
            {
                text: "Draw predicted regions"
                wrapMode: Text.Wrap
                font.family: "Helvetica"
                font.pointSize: 8
                color: "black"
            }
            Item{height:10;width:10;}
            CheckBox
            {
                id: param_6
                
                Connections
                {
                    target: p_inter
                    onGeneralSettingsChange: param_6.checked = p_inter.boolSignal(6, false);
                }
                onClicked: p_inter.boolSignal(6,true,param_6.checked);
            }
            Item{height:10;width:10;}
        }
        Row{ Item{height:2;width:10;}}
        Row
        {
            visible: thresholdingdetection_text.active
            Item{height:10;width:10;}
            Text
            {
                text: "Min object area [square px]"
                wrapMode: Text.Wrap
                font.family: "Helvetica"
                font.pointSize: 8
                color: "black"
            }
            Item{height:10;width:10;}
            TextField
            {
                id: param_20
                width: 40
                validator: IntValidator {bottom: 0; top: 100000;}
                text:""
                onEditingFinished: p_inter.doubleSignal(20,true,param_20.text);
                Connections
                {
                    target: p_inter
                    onGeneralSettingsChange: param_20.text = p_inter.doubleSignal(20,false);
                }
            }
            Item{height:10;width:10;}
        }
        Row{ Item{height:2;width:10;}}
        Row
        {
            visible: thresholdingdetection_text.active
            Item{height:10;width:10;}
            Text
            {
                text: "Max object area [square px]"
                wrapMode: Text.Wrap
                font.family: "Helvetica"
                font.pointSize: 8
                color: "black"
            }
            Item{height:10;width:10;}
            TextField
            {
                id: param_21
                width: 40
                validator: IntValidator {bottom: 0; top: 100000;}
                text:""
                onEditingFinished: p_inter.doubleSignal(21,true,param_21.text);
                Connections
                {
                    target: p_inter
                    onGeneralSettingsChange: param_21.text = p_inter.doubleSignal(21,false);
                }
            }
            Item{height:10;width:10;}
        }
        Row{ Item{height:2;width:10;}}
        Row
        {
            visible: thresholdingdetection_text.active
            Item{height:10;width:10;}
            Text
            {
                text: "Min contour (thin objects) [px]"
                wrapMode: Text.Wrap
                font.family: "Helvetica"
                font.pointSize: 8
                color: "black"
            }
            Item{height:10;width:10;}
            TextField
            {
                id: param_22
                width: 35
                validator: IntValidator {bottom: 0; top: 100000;}
                text:""
                onEditingFinished: p_inter.doubleSignal(22,true,param_22.text);
                Connections
                {
                    target: p_inter
                    onGeneralSettingsChange: param_22.text = p_inter.doubleSignal(22,false);
                }
            }
            Item{height:10;width:10;}
        }
        Row{ Item{height:2;width:10;}}
        Row
        {
            visible: thresholdingdetection_text.active
            Item{height:10;width:10;}
            Text
            {
                text: "Draw observation area border"
                wrapMode: Text.Wrap
                font.family: "Helvetica"
                font.pointSize: 8
                color: "black"
            }
            Item{height:10;width:10;}
            CheckBox
            {
                id: param_9
                
                Connections
                {
                    target: p_inter
                    onGeneralSettingsChange: param_9.checked = p_inter.boolSignal(9, false);
                }
                onClicked: p_inter.boolSignal(9,true,param_9.checked);
            }
            Item{height:10;width:10;}
        }
        Row{ Item{height:2;width:10;}}
        Row
        {
            visible: thresholdingdetection_text.active
            Item{height:10;width:10;}
            Text
            {
                text: "Observation area:"
                wrapMode: Text.Wrap
                font.family: "Helvetica"
                font.pointSize: 8
                color: "black"
            }
            Item{height:10;width:10;}
        }
        Row
        {
            visible: thresholdingdetection_text.active
            Item{height:10;width:10;}
            property int entrySize: 33

            Text
            {
                text: "x:"
                wrapMode: Text.Wrap
                font.family: "Helvetica"
                font.pointSize: 8
                color: "black"
            }
            Item{height:10;width:5;}
            TextField
            {
                id: param_45
                width: parent.entrySize
                validator: IntValidator {bottom: 0; top: 100000;}
                text:""
                onEditingFinished:
                {
                    p_inter.uintSignal(45,true,param_45.text);
                    //param_45.displayText = "20";
                }
                Connections
                {
                    target: p_inter
                    onGeneralSettingsChange: param_45.text = p_inter.uintSignal(45,false);
                }
            }
            Item{height:10;width:5;}
            Text
            {
                text: "y:"
                wrapMode: Text.Wrap
                font.family: "Helvetica"
                font.pointSize: 8
                color: "black"
            }
            Item{height:10;width:5;}
            TextField
            {
                id: param_46
                width: parent.entrySize
                validator: IntValidator {bottom: 0; top: 100000;}
                text:""
                onEditingFinished: p_inter.uintSignal(46,true,param_46.text);
                Connections
                {
                    target: p_inter
                    onGeneralSettingsChange: param_46.text = p_inter.uintSignal(46,false);
                }
            }
            Item{height:10;width:5;}
            Text
            {
                text: "w:"
                wrapMode: Text.Wrap
                font.family: "Helvetica"
                font.pointSize: 8
                color: "black"
            }
            Item{height:10;width:5;}
            TextField
            {
                id: param_47
                width: parent.entrySize
                validator: IntValidator {bottom: 0; top: 100000;}
                text:""
                property string oldtext:""
                onEditingFinished: 
                {
                    if( text != oldtext ) width_reinitialize.open();
                }
                Connections
                {
                    target: p_inter
                    onGeneralSettingsChange: { param_47.text = p_inter.uintSignal(47,false); param_47.oldtext = param_47.text; }
                }

                MessageDialog 
                {
                    id: width_reinitialize
                    title: "Reinitializing necessary"
                    text: "In order to change the width of the observation area, it is currently necessary to reinitialize the scene. Reinitializing will restart videos from the start, previous trajectory data will be lost. For camera sources tracking will restart with the same effects. Continue?"
                    standardButtons: StandardButton.Yes | StandardButton.No
                    onYes: {
                       p_inter.uintSignal(47,true,param_47.text);
                       p_inter.signalReload();
                    }
                    onNo: {}
                    //Component.onCompleted: visible = true
                }
            }
            Item{height:10;width:5;}
            Text
            {
                text: "h:"
                wrapMode: Text.Wrap
                font.family: "Helvetica"
                font.pointSize: 8
                color: "black"
            }
            Item{height:10;width:5;}
            TextField
            {
                id: param_48
                width: parent.entrySize
                validator: IntValidator {bottom: 0; top: 100000;}
                text:""
                property string oldtext:""
                onEditingFinished:
                {
                    if( text != oldtext ) height_reinitialize.open();
                }
                Connections
                {
                    target: p_inter
                    onGeneralSettingsChange: { param_48.text = p_inter.uintSignal(48,false); param_48.oldtext = param_48.text; }
                }

                MessageDialog 
                {
                    id: height_reinitialize
                    title: "Reinitializing necessary"
                    text: "In order to change the height of the observation area, it is currently necessary to reinitialize the scene. Reinitializing will restart videos from the start, previous trajectory data will be lost. For camera sources tracking will restart with the same effects. Continue?"
                    standardButtons: StandardButton.Yes | StandardButton.No
                    onYes: {
                       p_inter.uintSignal(48,true,param_48.text);
                       p_inter.signalReload();
                    }
                    onNo: {}
                    //Component.onCompleted: visible = true
                }
            }
            Item{height:10;width:10;}
        }
        Row{ Item{height:2;width:10;}}

    }
}
