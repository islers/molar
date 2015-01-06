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


Rectangle // load button
{
    color: "#bfd6e3"
    border.color:"#9db9c9"
    clip: true

    property string regularUpColor: "#bfd6e3"
    property string regularDownColor: "#d1e3ed"
    
    property string mouseoverUpColor: "#bfd6e3"
    property string mouseoverDownColor: "white"
    
    property string clickUpColor: "#bfd6e3"
    property string clickDownColor: "#bfd6e3"

    property Gradient regularState: Gradient
    {
        GradientStop { position: 0.0; color: regularUpColor }
        GradientStop { position: 0.7; color: regularDownColor }
    }
    property Gradient mouseoverState: Gradient
    {
        GradientStop { position: 0.0; color: mouseoverUpColor }
        GradientStop { position: 0.7; color: mouseoverDownColor }
        }
    property Gradient clickState: Gradient
    {
        GradientStop { position: 0.0; color: clickUpColor }
        GradientStop { position: 0.7; color: clickDownColor }
    }

    property alias label: msblabel.text
    Text
    {
        id:msblabel
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.verticalCenter: parent.verticalCenter
        text: ""
        font.family: "Helvetica"
        font.pointSize: 8
        color: "black"
    }

    gradient: regularState

    MouseArea
    {
        id: modar_button_id
        property bool mouseOver: false;
        anchors.fill: parent
        hoverEnabled: true
        onEntered:{
            gradient = mouseoverState
            mouseOver = true
        }
        onReleased:
        {
            if(mouseOver) gradient = mouseoverState
        }
        onExited:
        {
            gradient = regularState
            mouseOver = false
        }
        onPressed:
        {
            gradient =clickState
        }
    }
}
