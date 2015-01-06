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

import QtQuick 2.0

ModarButton {
    property bool active: false


    property Gradient activeState: Gradient
    {
        GradientStop { position: 0.0; color: "#000000" }
        GradientStop { position: 0.7; color: "#05344f" }
    }
    property Gradient inactiveState: regularState

    width:msblabel.width+40
    property alias label: msblabel.text
    Text
    {
        id:msblabel
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.verticalCenter: parent.verticalCenter
        text: "Example Label"
        font.family: "Helvetica"
        font.pointSize: 8
        color: active?"white":"black"
    }

    gradient: active?activeState:inactiveState
    MouseArea
    {
        property bool mouseOver: false;
        anchors.fill: parent
        hoverEnabled: true
        onEntered:{
            inactiveState = mouseoverState
            mouseOver = true
        }
        onReleased:
        {
            if(mouseOver) inactiveState = mouseoverState
        }
        onExited:
        {
            inactiveState = regularState
            mouseOver = false
        }
        onPressed:
        {
            inactiveState = clickState
        }
    }

}

