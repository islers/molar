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

ScrollView
{
                            
    anchors.fill:parent
    anchors.rightMargin: 250
    frameVisible:true
    contentItem: settings_1

    id: settings_scroll
    property int textSize: 8
    property int headerSize: 8

    Column
    {
        spacing:1
        id: settings_1

        Row{ Item{height:10;width:10;}}
           
        // GENERAL OPTIONS /////////////////////////////////////////////////////////////////////                         
        Rectangle{
            height:15;width:300;color:"white" 
            Row
            {
                Item{height:10;width:10;}
                Text
                {
                    text: "General Options"
                    wrapMode: Text.Wrap
                    font.capitalization: Font.AllUppercase 
                    font.family: "Helvetica"
                    font.pointSize: settings_scroll.headerSize
                    color: "black"
                }
                Item{height:10;width:10;}
            }
        }
        Row{ Item{height:2;width:10;}}

        Row
        {
            Item{height:10;width:10;}
            Text
            {
                text: "Restore last program state on startup"
                wrapMode: Text.Wrap
                font.family: "Helvetica"
                font.pointSize: settings_scroll.textSize
                color: "black"
            }
            Item{height:10;width:10;}
            CheckBox
            {
                id: param_1
                Connections
                {
                    target: p_inter
                    onGeneralSettingsChange: param_1.checked = p_inter.boolSignal(1, false);
                }
                onClicked: p_inter.boolSignal(1,true,param_1.checked);
            }
            Text
            {
                text: "Will only have an effect if saved as standard user settings."
                wrapMode: Text.Wrap
                font.family: "Helvetica"
                font.italic: true
                font.pointSize: settings_scroll.textSize
                color: "grey"
            }
        }
        Row
        {
            Item{height:10;width:10;}
            Text
            {
                text: "Reset object count on reload"
                wrapMode: Text.Wrap
                font.family: "Helvetica"
                font.pointSize: settings_scroll.textSize
                color: "black"
            }
            Item{height:10;width:10;}
            CheckBox
            {
                id: param_43
                Connections
                {
                    target: p_inter
                    onGeneralSettingsChange: param_43.checked = p_inter.boolSignal(43, false);
                }
                onClicked: p_inter.boolSignal(43,true,param_43.checked);
            }
        }
        // DISPLAY OPTIONS /////////////////////////////////////////////////////////////////////
        Row{ Item{height:15;width:10;}}
        Rectangle{
            height:15;width:300;color:"white" 
            Row
            {
                Item{height:10;width:10;}
                Text
                {
                    text: "Display Options"
                    wrapMode: Text.Wrap
                    font.capitalization: Font.AllUppercase 
                    font.family: "Helvetica"
                    font.pointSize: settings_scroll.headerSize
                    color: "black"
                }
                Item{height:10;width:10;}
            }
        }
        Row{ Item{height:2;width:10;}}

        Row
        {
            Item{height:10;width:10;}
            Text
            {
                text: "Object trajectories"
                wrapMode: Text.Wrap
                font.family: "Helvetica"
                font.pointSize: settings_scroll.textSize
                color: "black"
            }
            Item{height:10;width:10;}
            ComboBox
            {
                id: param_2
                model: [ "Full paths", "Full path per object", "No paths", "Specified frame number " ]
                Connections
                {
                    target: p_inter
                    onGeneralSettingsChange: param_2.reload();
                }
                function reload()
                {
                    var id = p_inter.intSignal(2,false);
                    if(id == -2) param_2.currentIndex = 0
                    else if(id == -1) param_2.currentIndex = 1
                    else if(id == 0) param_2.currentIndex = 2
                    else param_2.currentIndex = 3
                    param_2_1.text = (id<10)?"10":id;
                }
                onCurrentIndexChanged: param_2.safe()
                function safe()
                {
                    var id = currentIndex
                    switch(id)
                    {
                        case 0: p_inter.intSignal(2,true,-2); break;
                        case 1: p_inter.intSignal(2,true,-1); break;
                        case 2: p_inter.intSignal(2,true,0); break;
                        default: p_inter.intSignal(2,true,param_2_1.text); break;
                    }
                }
            }
            Item{height:10;width:10;}
            Text
            {
                text: "Frame number"
                wrapMode: Text.Wrap
                font.family: "Helvetica"
                font.pointSize: settings_scroll.textSize
                color: "black"
            }
            Item{height:10;width:10;}
            TextField
            {
                id: param_2_1
                width: 50
                validator: IntValidator {bottom: 10; top: 100000;}
                text:"100"
                onEditingFinished: param_2.safe()
            }
            Item{height:10;width:10;}
            Text
            {
                text: "Frame step size"
                wrapMode: Text.Wrap
                font.family: "Helvetica"
                font.pointSize: settings_scroll.textSize
                color: "black"
            }
            Item{height:10;width:10;}
            TextField
            {
                id: param_4
                width: 50
                validator: IntValidator {bottom: 2; top: 100000;}
                text:"8"
                onEditingFinished: p_inter.uintSignal(4,true,param_4.text);
                Connections
                {
                    target: p_inter
                    onGeneralSettingsChange: param_4.text = p_inter.uintSignal(4,false);
                }
            }
        }
        Item{height:4;width:10;}
        Row
        {
            Item{height:10;width:10;}
            Text
            {
                text: "Trajectory fadeout speed"
                wrapMode: Text.Wrap
                font.family: "Helvetica"
                font.pointSize: settings_scroll.textSize
                color: "black"
            }
            Item{height:10;width:10;}
            TextField
            {
                id: param_3
                width: 50
                validator: IntValidator {bottom: -1; top: 100000;}
                text:"8"
                onEditingFinished: p_inter.intSignal(3,true,param_3.text);
                Connections
                {
                    target: p_inter
                    onGeneralSettingsChange: param_3.text = p_inter.intSignal(3,false);
                }
            }
        }
        
        Row{ Item{height:2;width:10;}}

        Row
        {
            Item{height:10;width:10;}
            Text
            {
                text: "Observation area border color"
                wrapMode: Text.Wrap
                font.family: "Helvetica"
                font.pointSize: settings_scroll.textSize
                color: "black"
            }
            Item{height:10;width:10;}
            Text
            {
                text: "RGB"
                wrapMode: Text.Wrap
                font.family: "Helvetica"
                font.pointSize: settings_scroll.textSize
                color: "black"
            }
            Item{height:10;width:10;}
            TextField
            {
                id: param_10
                width: 35
                validator: IntValidator {bottom: 0; top: 255;}
                text:"8"
                onEditingFinished: p_inter.doubleSignal(10,true,param_10.text);
                Connections
                {
                    target: p_inter
                    onGeneralSettingsChange: param_10.text = p_inter.doubleSignal(10,false);
                }
            }
            TextField
            {
                id: param_11
                width: 35
                validator: IntValidator {bottom: 0; top: 255;}
                text:"8"
                onEditingFinished: p_inter.doubleSignal(11,true,param_11.text);
                Connections
                {
                    target: p_inter
                    onGeneralSettingsChange: param_11.text = p_inter.doubleSignal(11,false);
                }
            }
            TextField
            {
                id: param_12
                width: 35
                validator: IntValidator {bottom: 0; top: 255;}
                text:"8"
                onEditingFinished: p_inter.doubleSignal(12,true,param_12.text);
                Connections
                {
                    target: p_inter
                    onGeneralSettingsChange: param_12.text = p_inter.doubleSignal(12,false);
                }
            }
        }  
         
        // OBJECT DETECTION OPTIONS /////////////////////////////////////////////////////////////////////                         
        Row{ Item{height:15;width:10;}}
        Rectangle{
            height:15;width:300;color:"white" 
            Row
            {
                Item{height:10;width:10;}
                Text
                {
                    text: "Object Detection"
                    wrapMode: Text.Wrap
                    font.capitalization: Font.AllUppercase 
                    font.family: "Helvetica"
                    font.pointSize: settings_scroll.headerSize
                    color: "black"
                }
                Item{height:10;width:10;}
            }
        }
        Row{ Item{height:2;width:10;}}
        Row
        {
            Item{height:10;width:10;}
            Text
            {
                text: "Standard threshold"
                wrapMode: Text.Wrap
                font.family: "Helvetica"
                font.pointSize: settings_scroll.textSize
                color: "black"
            }
            Item{height:10;width:10;}
            TextField
            {
                id: param_30
                width: 35
                validator: IntValidator {bottom: 0; top: 255;}
                text:"8"
                onEditingFinished: p_inter.uintSignal(30,true,param_30.text);
                Connections
                {
                    target: p_inter
                    onGeneralSettingsChange: param_30.text = p_inter.uintSignal(30,false);
                }
            }
            Item{height:10;width:5;}
            Text
            {
                text: "[color: 0...255]"
                wrapMode: Text.Wrap
                font.family: "Helvetica"
                font.pointSize: settings_scroll.textSize
                color: "black"
            }
        }
        Row{ Item{height:2;width:10;}}

        Row
        {
            Item{height:10;width:10;}
            Text
            {
                text: "Store trajectories temporarily"
                wrapMode: Text.Wrap
                font.family: "Helvetica"
                font.pointSize: settings_scroll.textSize
                color: "black"
            }
            Item{height:10;width:10;}
            CheckBox
            {
                id: param_23
                Connections
                {
                    target: p_inter
                    onGeneralSettingsChange: param_23.checked = p_inter.boolSignal(23, false);
                }
                onClicked: p_inter.boolSignal(23,true,param_23.checked);
            }
        }
        Row{ Item{height:2;width:10;}}

        Row
        {
            Item{height:10;width:10;}
            Text
            {
                text: "Missing states"
                wrapMode: Text.Wrap
                font.family: "Helvetica"
                font.pointSize: settings_scroll.textSize
                color: "black"
            }
            Item{height:10;width:10;}
            ComboBox
            {
                id: param_25
                model: [ "No filling", "Fill with predictions", "Use old state for filling" ]
                Connections
                {
                    target: p_inter
                    onGeneralSettingsChange: param_25.reload();
                }
                function reload()
                {
                    param_25.currentIndex = p_inter.intSignal(25,false);
                }
                onCurrentIndexChanged: param_25.safe()
                function safe()
                {
                    p_inter.intSignal(25,true,currentIndex);
                }
            }
            Item{height:10;width:10;}
            Text
            {
                text: "Maximal object missing time"
                wrapMode: Text.Wrap
                font.family: "Helvetica"
                font.pointSize: settings_scroll.textSize
                color: "black"
            }
            Item{height:10;width:10;}
            TextField
            {
                id: param_26
                width: 25
                validator: IntValidator {bottom: 0; top: 1000;}
                text:"8"
                onEditingFinished: p_inter.uintSignal(26,true,param_26.text);
                Connections
                {
                    target: p_inter
                    onGeneralSettingsChange: param_26.text = p_inter.uintSignal(26,false);
                }
            }
            Item{height:10;width:10;}
            Text
            {
                text: "[frames]"
                wrapMode: Text.Wrap
                font.family: "Helvetica"
                font.pointSize: settings_scroll.textSize
                color: "black"
            }
        }
        Row{ Item{height:4;width:10;}}
        Row
        {
            Item{height:10;width:10;}
            Text
            {
                text: "Maximal matching distance"
                wrapMode: Text.Wrap
                font.family: "Helvetica"
                font.pointSize: settings_scroll.textSize
                color: "black"
            }
            Item{height:10;width:10;}
            TextField
            {
                id: param_27
                width: 25
                validator: IntValidator {bottom: 0; top: 1000;}
                text:"8"
                onEditingFinished: p_inter.doubleSignal(27,true,param_27.text);
                Connections
                {
                    target: p_inter
                    onGeneralSettingsChange: param_27.text = p_inter.doubleSignal(27,false);
                }
            }
            Item{height:10;width:5;}
            Text
            {
                text: "[px]"
                wrapMode: Text.Wrap
                font.family: "Helvetica"
                font.pointSize: settings_scroll.textSize
                color: "black"
            }
            Item{height:10;width:10;}
            Text
            {
                text: "Use old state as fallback if prediction fails"
                wrapMode: Text.Wrap
                font.family: "Helvetica"
                font.pointSize: settings_scroll.textSize
                color: "black"
            }
            Item{height:10;width:10;}
            CheckBox
            {
                id: param_28
                Connections
                {
                    target: p_inter
                    onGeneralSettingsChange: param_28.checked = p_inter.boolSignal(28, false);
                }
                onClicked: p_inter.boolSignal(28,true,param_28.checked);
            }
        }
        Row{ Item{height:4;width:10;}}
        Row
        {
            Item{height:10;width:10;}
            Text
            {
                text: "Grid density used for area overlap estimate"
                wrapMode: Text.Wrap
                font.family: "Helvetica"
                font.pointSize: settings_scroll.textSize
                color: "black"
            }
            Item{height:10;width:10;}
            TextField
            {
                id: param_29
                width: 25
                validator: IntValidator {bottom: 0; top: 1000;}
                text:"8"
                onEditingFinished: p_inter.doubleSignal(29,true,param_29.text);
                Connections
                {
                    target: p_inter
                    onGeneralSettingsChange: param_29.text = p_inter.doubleSignal(29,false);
                }
            }
            Item{height:10;width:5;}
            Text
            {
                text: "[px]"
                wrapMode: Text.Wrap
                font.family: "Helvetica"
                font.pointSize: settings_scroll.textSize
                color: "black"
            }
        }
        Row{ Item{height:4;width:10;}}
        Row
        {
            Item{height:10;width:10;}
            Text
            {
                text: "Regular window border disappearance range"
                wrapMode: Text.Wrap
                font.family: "Helvetica"
                font.pointSize: settings_scroll.textSize
                color: "black"
            }
            Item{height:10;width:10;}
            TextField
            {
                id: param_31
                width: 25
                validator: IntValidator {bottom: 0; top: 1000;}
                text:"8"
                onEditingFinished: p_inter.uintSignal(31,true,param_31.text);
                Connections
                {
                    target: p_inter
                    onGeneralSettingsChange: param_31.text = p_inter.uintSignal(31,false);
                }
            }
            Item{height:10;width:5;}
            Text
            {
                text: "[px]"
                wrapMode: Text.Wrap
                font.family: "Helvetica"
                font.pointSize: settings_scroll.textSize
                color: "black"
            }
        }  
         
        // CLASSIFICATION /////////////////////////////////////////////////////////////////////                         
        Row{ Item{height:15;width:10;}}
        Rectangle{
            height:15;width:300;color:"white" 
            Row
            {
                Item{height:10;width:10;}
                Text
                {
                    text: "Classification"
                    wrapMode: Text.Wrap
                    font.capitalization: Font.AllUppercase 
                    font.family: "Helvetica"
                    font.pointSize: settings_scroll.headerSize
                    color: "black"
                }
                Item{height:10;width:10;}
            }
        }
        Row{ Item{height:2;width:10;}}

        Row
        {
            Item{height:10;width:10;}
            Text
            {
                text: "Timing aware"
                wrapMode: Text.Wrap
                font.family: "Helvetica"
                font.pointSize: settings_scroll.textSize
                color: "black"
            }
            Item{height:10;width:10;}
            CheckBox
            {
                id: param_40
                Connections
                {
                    target: p_inter
                    onGeneralSettingsChange: param_40.checked = p_inter.boolSignal(40, false);
                }
                onClicked: p_inter.boolSignal(40,true,param_40.checked);
            }
            Item{height:10;width:10;}
            Text
            {
                text: "Targeted time overhead"
                wrapMode: Text.Wrap
                font.family: "Helvetica"
                font.pointSize: settings_scroll.textSize
                color: "black"
            }
            Item{height:10;width:10;}
            TextField
            {
                id: param_41
                width: 25
                validator: IntValidator {bottom: 0; top: 1000;}
                text:"8"
                onEditingFinished: p_inter.doubleSignal(41,true,param_41.text);
                Connections
                {
                    target: p_inter
                    onGeneralSettingsChange: param_41.text = p_inter.doubleSignal(41,false);
                }
            }
            Item{height:10;width:5;}
            Text
            {
                text: "[ms]"
                wrapMode: Text.Wrap
                font.family: "Helvetica"
                font.pointSize: settings_scroll.textSize
                color: "black"
            }
        }
        Row{ Item{height:2;width:10;}}

        Row
        {
            Item{height:10;width:10;}
            Text
            {
                text: "Standard negative object type"
                wrapMode: Text.Wrap
                font.family: "Helvetica"
                font.pointSize: settings_scroll.textSize
                color: "black"
            }
            Item{height:10;width:10;}
            TextField
            {
                id: param_42
                width: 300
                text:"object type"
                onEditingFinished: p_inter.stringSignal(42,true,param_42.text);
                Connections
                {
                    target: p_inter
                    onGeneralSettingsChange: param_42.text = p_inter.stringSignal(42,false);
                }
            }
        }
        Row{ Item{height:2;width:10;}}
        
        Row
        {
            Item{height:10;width:10;}
            Text
            {
                text: "Dynamic keypoint extraction threshold adaption"
                wrapMode: Text.Wrap
                font.family: "Helvetica"
                font.pointSize: settings_scroll.textSize
                color: "black"
            }
            Item{height:10;width:10;}
            CheckBox
            {
                id: param_49
                Connections
                {
                    target: p_inter
                    onGeneralSettingsChange: param_49.checked = p_inter.boolSignal(49, false);
                }
                onClicked: p_inter.boolSignal(49,true,param_49.checked);
            }
            Item{height:10;width:10;}
            Text
            {
                text: "Adaption step"
                wrapMode: Text.Wrap
                font.family: "Helvetica"
                font.pointSize: settings_scroll.textSize
                color: "black"
            }
            Item{height:10;width:10;}
            TextField
            {
                id: param_50
                width: 25
                validator: IntValidator {bottom: 0; top: 1000;}
                text:"7"
                onEditingFinished: p_inter.intSignal(50,true,param_50.text);
                Connections
                {
                    target: p_inter
                    onGeneralSettingsChange: param_50.text = p_inter.intSignal(50,false);
                }
            }
        }

        Row{ Item{height:50;width:10;}}

        Row
        {
            Item{height:10;width:10;}
            Text
            {
                text: "If changed the settings are automatically applied to the current scene."
                wrapMode: Text.Wrap
                font.family: "Helvetica"
                font.italic: true
                font.pointSize: settings_scroll.textSize
                color: "grey"
            }
        }
        Row{ Item{height:2;width:10;}}
        Row
        {
            Item{height:10;width:10;}
            Button
            {
                text: "Reset to user standard"
                iconName: "Reset to user standard"
                onClicked: p_inter.resetSettingsToUser();
            }
            Item{height:10;width:10;}
            Button
            {
                text: "Save as user standard"
                iconName: "Save as user standard"
                onClicked: p_inter.saveSettings();
            }
            Item{height:10;width:10;}
            Button
            {
                text: "Reset to application standard"
                iconName: "Reset to application standard"
                onClicked: p_inter.resetSettingsToApplication();
            }
        }
        Row{ Item{height:30;width:10;}}
    }
}
