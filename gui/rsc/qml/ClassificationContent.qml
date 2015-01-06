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
    contentItem: classification_content

    id: classification_content_scroll
    property int textSize: 8
    property int headerSize: 8

    Column
    {
        spacing:1
        id: classification_content
        property int arrowposition: 180

        Row{ Item{height:10;width:10;}}
        Row
        {
            Item{height:10;width:10;}
            TextEdit
			{
				text: "Create new feature point sets from video streams and use the sets of existing features to define new object types."
                width: 210
                readOnly: true
                wrapMode: TextEdit.WordWrap
                textFormat: TextEdit.RichText
				font.family: "Helvetica"
				font.pointSize: 8
				color: "black"
			}
            Item{height:10;width:5;}
        }
        Item{height:30;width:10;}
        Row
        {
            Item{height:30;width:10;}
            ModarButton
            {
                label: "Create new feature set"
                id: create_feature_button
                height: 30
                width: 210
                MouseArea
                {
                    anchors.fill:parent;
                    onClicked:
                    {
                        new_feature_window.visible = true
                    }
                    NewFeatureSet
                    {
                        id: new_feature_window
                        visible: false
                    }
                }
            }
            Item{height:30;width:10;}
        }
        Item{height:5;width:10;}
        Row
        {
            Item{height:30;width:10;}
            ModarButton
            {
                label: "Define new object type"
                id: define_object_type_button
                height: 30
                width: 210
                MouseArea
                {
                    anchors.fill:parent;
                    onClicked:
                    {
                        new_object_type_window.visible = true
                    }
                    NewObjectType
                    {
                        id: new_object_type_window
                        visible: false
                    }
                }
            }
            Item{height:30;width:10;}
        }
        
    }
}
