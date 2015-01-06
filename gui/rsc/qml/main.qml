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
import Qt.labs.settings 1.0
import QtQuick.Dialogs 1.1
import QtQuick.Window 2.1

ApplicationWindow
{
    id: window

    property string applicationTitle: "Multiple Object Localization And Recognition v1.0"
    title: applicationTitle
    Connections
    {
        target: p_inter
        onNewSource:
        {
            if( newsource!="No source" ) window.title = newsource + " - " + applicationTitle;
            else window.title = applicationTitle;
        }
        onError:
        {
            message_dialog.title = "I'm sorry, an error occured";
            message_dialog.text = message;
            message_dialog.open();
        }
        onMessage:
        {
            message_dialog.title = messagetitle;
            message_dialog.text = message;
            message_dialog.open();
        }
    }

    visible:true
    opacity:1
    height:400
    width:600
    minimumHeight: 616
    minimumWidth: 1100 //968
    
    MessageDialog 
    {
        id: message_dialog
        title: "Some title"
        text: "And some text"
        standardButtons: StandardButton.Ok 
    }

    Settings
    {
        property alias x: window.x
        property alias y: window.y
        property alias width: window.width
        property alias height: window.height
    }

    Rectangle // background
    {
        id: windowbackground
        color: "#12DA35"
        anchors.fill: parent
        gradient: Gradient {
                GradientStop { position: 0.0; color: "#eff5f8" }
                GradientStop { position: 0.7; color: "#cadde8" }
            }
    }

    Item // main content : dark green container
    {
        id: all
        anchors.fill:parent
        anchors.leftMargin: 10
        anchors.topMargin: 8
        anchors.rightMargin: 20
        anchors.bottomMargin: 8


        Item // load and save column
        {
            id: left
            width:54
            //anchors.leftMargin: 20
            anchors.topMargin: 50
            anchors.top: parent.top
            anchors.bottom: parent.bottom
            //spacing:2
            Column
            {
                spacing:2

                ModarButton // load button
                {
                    id: video_source
                    width: 50
                    height: 60

                    Image
                    {
                        anchors.horizontalCenter: parent.horizontalCenter
                        anchors.verticalCenter: parent.verticalCenter
                        width:40
                        fillMode: Image.PreserveAspectFit
                        source: "../img/Movies-icon.png"
                    }

                    MouseArea
                    {
                        anchors.fill:parent
                        onPressed:
                        {
                            stream_choice_window.visible = true;
                        }
                    }

                    NewStreamWindow
                    {
                        id: stream_choice_window
                    }

                }
                ModarButton // new project button
                {
                    id: new_project
                    width: 50
                    height: 60

                    Image
                    {
                        anchors.horizontalCenter: parent.horizontalCenter
                        anchors.verticalCenter: parent.verticalCenter
                        width:40
                        fillMode: Image.PreserveAspectFit
                        source: "../img/Files-New-File-icon.png"
                    }

                    MouseArea
                    {
                        anchors.fill:parent
                        onPressed:
                        {
                            new_project_dialog.open();
                        }
                    }

                    MessageDialog 
                    {
                        id: new_project_dialog
                        title: "Create new project?"
                        text: "This will create a new, empty project with standard user settings. Everything that is not saved beforehand will be lost, including trajectory data, videos and settings. Continue?"
                        standardButtons: StandardButton.Yes | StandardButton.No
                        onYes: {
                            p_inter.createNewProject();
                        }
                        onNo: {}
                        //Component.onCompleted: visible = true
                    }
                }
                ModarButton // save project configuration button
                {
                    id: save_project
                    width: 50
                    height: 60
                    Image
                    {
                        anchors.horizontalCenter: parent.horizontalCenter
                        anchors.verticalCenter: parent.verticalCenter
                        width:40
                        fillMode: Image.PreserveAspectFit
                        source: "../img/Save-icon.png"
                    }

                    MouseArea
                    {
                        anchors.fill:parent
                        onPressed:
                        {
                            p_inter.takeSnapshot();
                            save_project_dialog.open();
                        }
                    }

                    FileDialog
                    {
                        id: save_project_dialog
                        title: "Save project configuration"
                        selectExisting: false
                        nameFilters: [ "Molar configuration files (*.swsc)", "All files (*)" ]
                        onAccepted: {
                            p_inter.saveProject( fileUrl );
                        }
                        onRejected: {

                        }
                    }
                }
                ModarButton // open project configuration button
                {
                    id: load_project
                    width: 50
                    height: 60

                    Image
                    {
                        anchors.horizontalCenter: parent.horizontalCenter
                        anchors.verticalCenter: parent.verticalCenter
                        width:40
                        fillMode: Image.PreserveAspectFit
                        source: "../img/Folder-Open-icon.png"
                    }

                    MouseArea
                    {
                        anchors.fill:parent
                        onPressed:
                        {
                            load_project_dialog.open();
                        }
                    }

                    FileDialog
                    {
                        id: load_project_dialog
                        title: "Load project configuration"
                        selectExisting: true
                        selectMultiple: false
                        nameFilters: [ "Molar configuration files (*.swsc)", "All files (*)" ]
                        onAccepted: {
                            p_inter.openProject( fileUrl );
                        }
                        onRejected: {

                        }
                    }
                }
                ModarButton // load button
                {
                    id: export_video
                    width: 50
                    height: 60

                    MouseArea
                    {
                        anchors.fill:parent
                        onPressed:
                        {
                            export_video_dialog.open();
                        }
                    }

                    Image
                    {
                        anchors.horizontalCenter: parent.horizontalCenter
                        anchors.verticalCenter: parent.verticalCenter
                        width:40
                        fillMode: Image.PreserveAspectFit
                        source: "../img/Save-icon.png"
                    }

                    Image
                    {
                        x: 20
                        y: 27
                        width:30
                        fillMode: Image.PreserveAspectFit
                        source: "../img/movie-icon.png"
                    }

                    FileDialog
                    {
                        id: export_video_dialog
                        title: "Save recorded video"
                        selectExisting: false
                        nameFilters: [ "*.avi", "All files (*)" ]
                        onAccepted: {
                            p_inter.saveVideo( fileUrl );
                        }
                        onRejected: {

                        }
                    }
                }
                ModarButton // save image button
                {
                    id: export_image
                    width: 50
                    height: 60

                    Image
                    {
                        anchors.horizontalCenter: parent.horizontalCenter
                        anchors.verticalCenter: parent.verticalCenter
                        width:40
                        fillMode: Image.PreserveAspectFit
                        source: "../img/Save-icon.png"
                    }

                    Image
                    {
                        x: 22
                        y: 32
                        width:22
                        fillMode: Image.PreserveAspectFit
                        source: "../img/Pictures-icon.png"
                    }

                    MouseArea
                    {
                        anchors.fill:parent
                        onPressed:
                        {
                            p_inter.takeSnapshot();
                            export_image_dialog.open();
                        }
                    }

                    FileDialog
                    {
                        id: export_image_dialog
                        title: "Save snapshot"
                        selectExisting: false
                        nameFilters: [ "*.png", "All files (*)" ]
                        onAccepted: {
                            p_inter.saveSnapshot( fileUrl );
                        }
                        onRejected: {

                        }
                    }
                }
                ModarButton // save data button
                {
                    id: export_data
                    width: 50
                    height: 60

                    Image
                    {
                        anchors.horizontalCenter: parent.horizontalCenter
                        anchors.verticalCenter: parent.verticalCenter
                        width:40
                        fillMode: Image.PreserveAspectFit
                        source: "../img/database-arrow-up-icon.png"
                    }

                    MouseArea
                    {
                        anchors.fill:parent
                        onPressed:
                        {
                            export_data_window.visible = true;
                        }
                    }

                    Window
                    {
                        id: export_data_window
	                    title: "Export data"
	                    height: 500
                        width: 620
	                    minimumHeight: 480
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
                                Row
                                {
                                    Item{ width:10; height:10 }
                                    Text
					                {
						                text: "Time step separator: "
						                wrapMode: Text.Wrap
						                font.family: "Helvetica"
						                font.pointSize: 8
                                        font.bold: true
						                color: "black"
					                }
                                    Item{ width:30; height:10 }
                                    CheckBox
                                    {
                                        checked: true
                                        id: newline_checked
                                        onClicked:
                                        {
                                            if( checked==false ) checked = true;
                                            tab_checked.checked = false;
                                            custom_checked.checked = false;
                                        }
                                    }
                                    Text
					                {
						                text: "newline"
						                wrapMode: Text.Wrap
						                font.family: "Helvetica"
						                font.pointSize: 8
						                color: "black"
					                }
                                    Item{ width:30; height:10 }
                                    CheckBox
                                    {
                                        checked: false
                                        id: tab_checked
                                        onClicked:
                                        {
                                            if( checked==false ) checked = true;
                                            newline_checked.checked = false;
                                            custom_checked.checked = false;
                                        }
                                    }
                                    Text
					                {
						                text: "tab"
						                wrapMode: Text.Wrap
						                font.family: "Helvetica"
						                font.pointSize: 8
						                color: "black"
					                }
                                    Item{ width:30; height:10 }
                                    CheckBox
                                    {
                                        checked: false
                                        id: custom_checked
                                        onClicked:
                                        {
                                            if( checked==false ) checked = true;
                                            tab_checked.checked = false;
                                            newline_checked.checked = false;
                                        }
                                    }
                                    Text
					                {
						                text: "custom:"
						                wrapMode: Text.Wrap
						                font.family: "Helvetica"
						                font.pointSize: 8
						                color: "black"
					                }
                                    Item{ width:10; height:10 }
                                    TextField
                                    {
                                        width: 40
                                        text: ""
                                        id: custom_timestep_separator
                                    }
                                }
                                Item{height:15;width:10;}
                                Row
                                {
                                    Item{ width:10; height:10 }
                                    Text
					                {
						                text: "Data point separator:"
						                wrapMode: Text.Wrap
						                font.family: "Helvetica"
						                font.pointSize: 8
                                        font.bold: true
						                color: "black"
					                }
                                    Item{ width:10; height:10 }
                                    TextField
                                    {
                                        width: 40
                                        text: " "
                                        id: data_separator
                                    }
                                }
                                
                                Item{height:50;width:10;}
                                Text
				                {
					                text: "Choose objects for data export:"
					                wrapMode: Text.Wrap
					                font.family: "Helvetica"
					                font.pointSize: 8
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
                                Item
                                {
                                    height:280; width:580;
                                    anchors.left: parent.left
                                    anchors.leftMargin: 10

                                    ScrollView
		                            {
			                            anchors.fill: parent
                                        anchors.topMargin: 10
			
			                            ListView
			                            {
				                            anchors.fill: parent

                                            id: types_to_export
                                            spacing: 10

				                            model: ListModel{ id:object_list }
                                            property int currentMaxId: -1;
                                            
                                            property var objectsForExport : []

				                            delegate: Row
				                            {
                                                property string objectid: objectId
                                                CheckBox
                                                {
                                                    checked: false
                                                    onClicked:
                                                    {
                                                        if( checked ) types_to_export.objectsForExport.push(objectId);
                                                        else
                                                        {
                                                            for( var i=0; i<types_to_export.objectsForExport.length; i++ )
                                                            {
                                                                if( types_to_export.objectsForExport[i]==objectId )
                                                                {
                                                                    types_to_export.objectsForExport.splice(i,1);
                                                                }
                                                            }
                                                        }
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
                                                var lowest = ( (types_to_export.currentMaxId+1)>lowestId ) ? (types_to_export.currentMaxId+1) : lowestId;
			                                    for( var i=lowest; i<=highestId; i++ )
			                                    {
				                                    object_list.append( {"objectId":i } );
			                                    }
                                                types_to_export.currentMaxId = highestId;
		                                    }
                                            function clear()
                                            {
                                                object_list.clear();
                                                types_to_export.currentMaxId = -1;
                                                types_to_export.objectsForExport = []
                                            }

                                            Connections
                                            {
                                                target: p_inter
                                                onGeneralSettingsChange: types_to_export.clear()
                                                onNewSource: types_to_export.clear();
                                            }
                                                        
                                            Timer
                                            {
                                                interval:1000;
                                                running: export_data_window.visible;
                                                repeat:true;
                                                onTriggered:types_to_export.refresh();
                                            }
			                            }
		                            }
                                }
                                Item{height:50;width:10;}
                                Row // buttons
                                {
                                    Item{height:10;width:10;}

                                    ModarButton
					                {
						                label: "Export"
						                width: 50
						                height: 25

						
						                MouseArea
						                {
							                anchors.fill:parent;
							                onClicked:
							                {
                                                if( types_to_export.objectsForExport.length==0 ) emptyList.open();
                                                else data_export_folder_choice.open();
							                }
                                            
                                            property MessageDialog emptyList: MessageDialog
                                            {
                                                title: "Choose an object"
                                                text: "Can't export data if you don't choose at least one object."
                                                standardButtons: StandardButton.Ok
                                            }
                                            
						                }

                                        FileDialog
                                        {
                                            id: data_export_folder_choice
                                            title: "Choose target folder for data files"
                                            selectFolder: true
                                            onAccepted: {
                                                if( newline_checked.checked ) p_inter.exportData( types_to_export.objectsForExport, 0, "", data_separator.text, fileUrl );
                                                else if( tab_checked.checked ) p_inter.exportData( types_to_export.objectsForExport, 1, "", data_separator.text, fileUrl );
                                                else if( custom_checked.checked ) p_inter.exportData( types_to_export.objectsForExport, 2, custom_timestep_separator.text, data_separator.text, fileUrl );
                                                export_data_window.visible = false;
                                            }
                                            onRejected: {

                                            }
                                        }
					                }
                                }
                                Item{height:10;width:10;}
                            }
                        }
                    }
                }
                
                Row
                {
                    //Item{ width:25; height:10 }
                    ModarButton // open project configuration button
                    {
                        id: info_button
                        width: 50
                        height: 20

                        MouseArea
                        {
                            anchors.fill:parent
                            onPressed:
                            {
                                info_window.visible = true;
                            }
                        }
					    Text
					    {
                            anchors.fill: parent
                            anchors.leftMargin: 10
						    text: "about"
						    wrapMode: Text.Wrap
						    font.family: "Helvetica"
						    font.pointSize: 7
                            font.italic: true
                            //font.bold:true
						    color: "black"
					    }

                        Window
                        {
                            id: info_window
	                        title: "About..."
	                        height: 100
                            width: 150
	                        minimumHeight: 250
	                        minimumWidth: 420

	                        Rectangle
	                        {
		                        anchors.fill: parent
                                gradient: Gradient {
                                        GradientStop { position: 0.0; color: "#eff5f8" }
                                        GradientStop { position: 0.7; color: "#cadde8" }
                                }
	                        }
                            Column
                            {
                                width:400;
					            TextEdit
					            {
						            text: "<b>MOLAR - Multiple Object Localization And Recognition v1.0</b><br /><br />(c) Stefan Isler, 2014 (<a href='mailto:islerstefan@bluewin.ch'>islerstefan@bluewin.ch</a>)<br /> Institute of Robotics and Intelligent Systems, ETH Zurich
                                    <br /><br />"
                                    onLinkActivated: Qt.openUrlExternally(link)
						            wrapMode: TextEdit.WordWrap
						            font.family: "Helvetica"
						            font.pointSize: 8
                                    readOnly:true
                                    textFormat: TextEdit.RichText
						            color: "black"
					            }
					            TextEdit
					            {
						            text: "
                                    <b>Graphics sources:</b><br />
                                    HADezign for video source choose icon<br />
                                    <a href='http://hadezign.com/'>http://hadezign.com/</a><br />
                                    Sean Poon for new project icon<br />
                                    <a href='http://gakuseisean.deviantart.com/'>http://gakuseisean.deviantart.com/</a><br />
                                    Double-J Design (Jack Cai) for <a href='http://www.iconarchive.com/show/ravenna-3d-icons-by-double-j-design/Save-icon.html'>save icon</a> (<a href='http://creativecommons.org/licenses/by/4.0/'>CC Attribution 4.0</a> license)<br />
                                    <a href='http://www.doublejdesign.co.uk/'>http://www.doublejdesign.co.uk/</a><br />
                                    Javier Aroche for <a href='http://www.iconarchive.com/show/delta-icons-by-aroche/Folder-Open-icon.html'>load project icon</a> (<a href='http://creativecommons.org/licenses/by-nc-nd/4.0/'>CC Attribution-Noncommercial-No Derivate 4.0</a> license)<br />
                                    <a href='http://www.javier-aroche.com/'>http://www.javier-aroche.com/</a><br />
                                    Arnaud Nelissen (Chromatix) for <a href='http://www.iconarchive.com/show/aerial-icons-by-chromatix/movie-icon.html'>video tape icon</a> (<a href='http://creativecommons.org/licenses/by-nc-nd/4.0/'>CC Attribution-Noncommercial-No Derivate 4.0</a> license)<br />
                                    <a href='http://chromatix.deviantart.com/'>http://chromatix.deviantart.com/</a><br />
                                    Photoshopedia for <a href='http://www.iconarchive.com/show/xedia-icons-by-photoshopedia/Pictures-icon.html'>camera icon</a><br />
                                    <a href='http://www.photoshopedia.com/'>http://www.photoshopedia.com/</a><br />
                                    Icojam for export data icon (Public domain license)<br />
                                    <a href='http://www.icojam.com/'>http://www.icojam.com/</a><br />
                                    "
                                    onLinkActivated: Qt.openUrlExternally(link)
						            wrapMode: TextEdit.WordWrap
						            font.family: "Helvetica"
						            font.pointSize: 6
                                    readOnly:true
                                    textFormat: TextEdit.RichText
						            color: "black"
					            }
                            }
                        }
                    }
                }
            }
        }


        Item // main content column
        {
            id:main
            anchors.fill:parent
            anchors.leftMargin:54

            Item // main content column
            {
                id: middle
                anchors.fill:parent


                Item // menü leiste: object tracking, settings etc
                {
                    id:middle_menu
                    anchors.left:parent.left
                    anchors.right:parent.right
                    height:20

                    Row
                    {
                        spacing: 0

                        ModarSwitchButton // load button
                        {
                            id: tracking_view

                            height: middle_menu.height
                            label: "Tracking View"
                            MouseArea
                            {
                                anchors.fill:parent
                                onPressed: parent.press()
                            }

                            active: true

                            function press()
                            {
                                original_stream.active=false
                                threshold_image_stream.active=false
                                tracking_view.active=true
                                preprocess_image_stream.active=false
                                prethreshold_image_stream.active=false
                                settings.active=false
                            }

                        }
                        ModarSwitchButton // load button
                        {
                            id: original_stream
                            //width: label.width+40
                            height: middle_menu.height
                            label:"Original"
                            MouseArea
                            {
                                anchors.fill:parent
                                onPressed:
                                {
                                    original_stream.active=true
                                    threshold_image_stream.active=false
                                    tracking_view.active=false
                                    preprocess_image_stream.active=false
                                    prethreshold_image_stream.active=false
                                    settings.active=false
                                }
                            }
                        }
                        ModarSwitchButton // pre process image view button
                        {
                            id: preprocess_image_stream
                            //width: label.width+40
                            height: middle_menu.height
                            label:"Preprocessed"
                            visible: scene_control.showPreprocessed
                            onVisibleChanged:
                            {
                                if(visible) press();
                                else if(active) tracking_view.press();
                            }

                            MouseArea
                            {
                                anchors.fill:parent
                                id: preprocess_image_stream_mouseArea
                                onPressed: parent.press();
                            }
                            function press()
                            {
                                preprocess_image_stream.active=true
                                prethreshold_image_stream.active=false
                                threshold_image_stream.active=false
                                original_stream.active=false
                                tracking_view.active=false
                                settings.active=false
                            }
                        }
                        ModarSwitchButton // pre threshold image view button
                        {
                            id: prethreshold_image_stream
                            //width: label.width+40
                            height: middle_menu.height
                            label:"Pre-Threshold"
                            visible: scene_control.showInternPreprocessed
                            onVisibleChanged:
                            {
                                if(visible) press();
                                else if(active) tracking_view.press();
                            }
                            MouseArea
                            {
                                anchors.fill:parent
                                onPressed: parent.press()
                            }
                            function press()
                            {
                                prethreshold_image_stream.active=true
                                preprocess_image_stream.active=false
                                threshold_image_stream.active=false
                                original_stream.active=false
                                tracking_view.active=false
                                settings.active=false
                            }
                        }
                        ModarSwitchButton // threshold image view button
                        {
                            id: threshold_image_stream
                            //width: label.width+40
                            height: middle_menu.height
                            label:"Threshold Detection"
                            visible: scene_control.showDetectionThresholded
                            onVisibleChanged:
                            {
                                if(visible) press();
                                else if(active) tracking_view.press();
                            }
                            MouseArea
                            {
                                anchors.fill:parent
                                onPressed: parent.press()
                            }
                            function press()
                            {
                                threshold_image_stream.active=true
                                prethreshold_image_stream.active=false
                                original_stream.active=false
                                tracking_view.active=false
                                preprocess_image_stream.active=false
                                settings.active=false
                            }
                        }
                        ModarSwitchButton // settings button
                        {
                            id: settings

                            height: middle_menu.height
                            label: "Settings"
                            MouseArea
                            {
                                anchors.fill:parent
                                onPressed: parent.press()
                            }
                            function press()
                            {
                                original_stream.active=false
                                tracking_view.active=false
                                threshold_image_stream.active=false
                                preprocess_image_stream.active=false
                                prethreshold_image_stream.active=false
                                settings.active=true
                            }
                        }
                    }
                }

                Item // content area
                {
                    id: video_display_area
                    anchors.top:parent.top
                    anchors.bottom:parent.bottom
                    anchors.left:parent.left
                    anchors.right:parent.right
                    anchors.topMargin: middle_menu.height
                    //anchors.bottomMargin: message_display.height

                    Rectangle
                    {
                        height:right_content.height
                        anchors.left:parent.left
                        anchors.right:parent.right
                        clip:true
                        border.color:"#9db9c9"
                        border.width:1
                        
                        Image
                        {
                            visible: tracking_view.active
                            id: tracking_view_content
                            x:1
                            y:1
                            source: ""
                            cache: false
                            function reload() 
                            {
                                source = "";
                                source = "image://opencv/edit";
                            }
                            Connections 
                            {
                                target: cvstream
                                onImageUpdated: tracking_view_content.reload(); 
                            }
                        }
                        Image
                        {
                            visible:original_stream.active
                            id:original_view_content
                            x:1
                            y:1
                            source: ""
                            cache: false
                            function reload() 
                            {
                                source = "";
                                source = "image://opencv/original";
                            }
                            Connections 
                            {
                                target: cvstream
                                onImageUpdated: original_view_content.reload(); 
                            }


                        }
                        Image
                        {
                            visible:preprocess_image_stream.active
                            id:preprocess_image_stream_content
                            x:1
                            y:1
                            source: ""
                            cache: false
                            onVisibleChanged:
                            {
                                if(!preprocess_image_stream.visible) source="";
                            }
                            function reload() 
                            {
                                source = "";
                                source = "image://opencv/preprocessimage";
                            }
                            Connections 
                            {
                                target: cvstream
                                onImageUpdated: if(preprocess_image_stream.visible) preprocess_image_stream_content.reload(); 
                            }


                        }
                        Image
                        {
                            visible:prethreshold_image_stream.active
                            id:prethreshold_image_stream_content
                            x:1
                            y:1
                            source: ""
                            cache: false
                            onVisibleChanged:
                            {
                                if(!prethreshold_image_stream.visible) source="";
                            }
                            function reload() 
                            {
                                source = "";
                                source = "image://opencv/prethresholdimage";
                            }
                            Connections 
                            {
                                target: cvstream
                                onImageUpdated: if(prethreshold_image_stream.visible) prethreshold_image_stream_content.reload(); 
                            }


                        }
                        Image
                        {
                            visible:threshold_image_stream.active
                            id:threshold_image_stream_content
                            x:1
                            y:1
                            source: ""
                            cache: false
                            onVisibleChanged:
                            {
                                if(!threshold_image_stream.visible) source="";
                            }
                            function reload() 
                            {
                                source = "";
                                source = "image://opencv/thresholdimage";
                            }
                            Connections 
                            {
                                target: cvstream
                                onImageUpdated: if(threshold_image_stream.visible) threshold_image_stream_content.reload();
                            }


                        }
                        Rectangle
                        {
                            visible:settings.active
                            id:settings_content
                            anchors.fill:parent

                            Rectangle
                            {
                                anchors.fill: parent
                                gradient: Gradient {
                                        GradientStop { position: 0.0; color: "#eff5f8" }
                                        GradientStop { position: 0.7; color: "#cadde8" }
                                    }
                            }

                            MolarSettings{}
                        }
                    }

                }
            }


            Item // scene and classes column
            {
                id:right

                anchors.right: parent.right
                anchors.bottom: parent.bottom
                anchors.top: parent.top
                width:250

                Rectangle // menü leiste: object tracking, settings etc
                {
                    id:right_menu
                    anchors.left:parent.left
                    anchors.right:parent.right
                    height:middle_menu.height
                    color: "#bfd6e3"
                    gradient: Gradient {
                        GradientStop { position: 0.0; color: "#bfd6e3" }
                        GradientStop { position: 0.7; color: "#d1e3ed" }
                    }
                    border.color:"#9db9c9"

                    Row
                    {
                        spacing: 0

                        ModarSwitchButton // load button
                        {
                            id: scene_info_options
                            height: middle_menu.height

                            label: "Scene Control"
                            active: true
                            MouseArea
                            {
                                anchors.fill:parent
                                onPressed:
                                {
                                    scene_info_options.active=true
                                    classification_options.active=false
                                }
                            }
                        }
                        ModarSwitchButton // load button
                        {
                            id: classification_options
                            height: middle_menu.height

                            label: "Classification"
                            MouseArea
                            {
                                anchors.fill:parent
                                onPressed:
                                {
                                    scene_info_options.active=false
                                    classification_options.active=true
                                }
                            }
                        }
                    }
                }
                Item // inhalt rechts
                {
                    id: right_content
                    anchors.fill:parent
                    anchors.topMargin: middle_menu.height

                    Rectangle
                    {
                        anchors.fill:parent
                        gradient: Gradient {
                            GradientStop { position: 0.0; color: "#bfd6e3" }
                            GradientStop { position: 0.7; color: "#d1e3ed" }
                        }
                        border.color:"#9db9c9"
                        border.width:1

                        Item
                        {
                            id: scene_control_content
                            visible: scene_info_options.active
                            anchors.fill:parent
                            SceneControlContent
                            {
                                id: scene_control
                            }
                        }
                        Item
                        {
                            id: classification_options_content
                            visible: classification_options.active
                            anchors.fill:parent
                            ClassificationContent
                            {
                                id: classification_content
                            }
                            /*Text
                            {
                                text: "create new feature point sets and type \ndefinitions\n\n\n new object class \n creation as well as \n new features creations\n creation as well as \n new features creations\n creation as well as \n new features creations\n creation as well as \n new features creations\n\n\n\n creation as well as \n new features creations\n creation as well as \n new features creations\n\n creation as well as \n new features creations"
                                anchors.fill:parent
                                wrapMode: Text.Wrap
                                font.family: "Helvetica"
                                font.pointSize: 8
                                color: "black"
                            }*/
                        }

                    }
                }
            }

        }



    }
    DropShadow
    {
        anchors.fill: source
        cached: true;
        horizontalOffset: 3
        verticalOffset: 3
        radius: 8.0
        samples: 16
        color: "#30000000"
        source: all
        smooth:true;
    }

}
