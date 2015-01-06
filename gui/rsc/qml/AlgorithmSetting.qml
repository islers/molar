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

Window
{
	id: algorithm_setting
	title: (isPreProcess?"Preprocessing slot ":"Prethresholdprocessing slot ") + index + ": " + algorithmName + " settings"
	height: 300
    width: 600
	minimumHeight: 300
	minimumWidth: 600

    property int index: -1
    property string algorithmName: ""
    property bool isPreProcess: true


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
        height:20
        width:100
        anchors.top: parent.top
        anchors.right: parent.right
        anchors.topMargin:10
        anchors.rightMargin:20

        Row
        {
            Text
	        {
		        text: "filter is active "
		        wrapMode: Text.Wrap
		        font.family: "Helvetica"
		        font.pointSize: 8
		        color: "black"
	        }
            CheckBox
            {
                id: activate_algorithm
                
                Connections
                {
                    target: p_inter
                    onGeneralSettingsChange: activate_algorithm.checked = p_inter.filterActive( algorithm_setting.index, algorithm_setting.isPreProcess, false );
                }
                onClicked: p_inter.filterActive( algorithm_setting.index, algorithm_setting.isPreProcess, true, activate_algorithm.checked );
                Component.onCompleted: activate_algorithm.checked = p_inter.filterActive( algorithm_setting.index, algorithm_setting.isPreProcess, false );
            }
        }
    }

	Item
	{
		anchors.fill: parent
		anchors.topMargin: 10
		anchors.rightMargin: 10
		anchors.bottomMargin: 10
		anchors.leftMargin: 10

        
        Row
        {
            spacing: 0

            ModarSwitchButton // settings button
            {
                id: parameter_settings

                height: 20
                label: "Settings"
                MouseArea
                {
                    anchors.fill:parent
                    onPressed: parent.press()
                }

                active: true

                function press()
                {
                    parameter_settings.active=true
                    parameter_info.active=false
                }

            }
            ModarSwitchButton // info button
            {
                id: parameter_info
                //width: label.width+40
                height: 20
                label:"Info"
                MouseArea
                {
                    anchors.fill:parent
                    onPressed: parent.press()
                }

                function press()
                {
                    parameter_settings.active=false
                    parameter_info.active=true
                }
            }
        }
            
		ScrollView
		{
			anchors.fill: parent
            anchors.topMargin: 30
            visible: parameter_settings.active
			
			ListView
			{
				anchors.fill: parent

				model: ListModel{ id:settings_list }
				spacing: 20

				delegate: Row
				{
					property int pos: position
					property string name: parameterName
                    property string value: parameterValue
                    property bool isBool: isbool
                    property bool isInt: isint
                    property bool isDouble: isdouble

					Text
					{
						text: parameterName
						wrapMode: Text.Wrap
						font.family: "Helvetica"
						font.pointSize: 8
						color: "black"
					}
                    Item{ width:10; height:10 }
                    TextField
                    {
                        visible: !isBool
                        width: 50
                        text: parameterValue
                        onEditingFinished:
                        {
                            p_inter.setParameterValue( algorithm_setting.index, algorithm_setting.isPreProcess, text, parameterName );
                        }
                        Connections
                        {
                            target: p_inter
                            onAlgorithmParameterChange:
                            {
                                if( !isBool )
                                {
                                    if( sAlgorithmIdx==algorithm_setting.index && sIsPreProcess==algorithm_setting.isPreProcess && sParameterName==parameterName )
                                    {
                                        parent.text = p_inter.getParameterValue( algorithm_setting.index, algorithm_setting.isPreProcess, name );
                                    }
                                }
                            }
                        }
                    }
                    CheckBox
                    {
                        visible: isBool
                        Connections
                        {
                            target: p_inter
                            onAlgorithmParameterChange:
                            {
                                if( isBool )
                                {
                                    if( sAlgorithmIdx==algorithm_setting.index && sIsPreProcess==algorithm_setting.isPreProcess && sParameterName==parameterName )
                                    {
                                        parent.parent.checked = p_inter.getParameterValue( algorithm_setting.index, algorithm_setting.isPreProcess, name );
                                    }
                                }
                            }
                        }
                        onClicked: p_inter.setParameterValue( index, isPreProcess, parent.checked, parameterName );
                    }
				}
			}
		}
        
            
		ScrollView
		{
			anchors.fill: parent
            anchors.topMargin: 30
            visible: parameter_info.active
			
			ListView
			{
				anchors.fill: parent

				model: ListModel{ id:info_list }
				spacing: 20

				delegate: Row
				{
					property int pos: position
					property string name: parameterName

					Column
					{
						Text
						{
							text: parameterName
							wrapMode: Text.Wrap
							font.family: "Helvetica"
							font.pointSize: 8
							font.bold: true
							color: "black"
						}
						Text
						{
							text: parameterDescription
							wrapMode: Text.Wrap
							font.family: "Helvetica"
							font.pointSize: 8
							color: "black"
							width: 500
						}
					}
				}
			}
		}
		Component.onCompleted:
		{
			var parameterName = p_inter.parameterNames( index, isPreProcess );
            var parameterValue = p_inter.parameterValues( index, isPreProcess );
            var parameterDescription = p_inter.parameterDescriptions( index, isPreProcess );
            var parameterIsBool = p_inter.parameterIsBool( index, isPreProcess );
            var parameterIsInt = p_inter.parameterIsInt( index, isPreProcess );
            var parameterIsDouble = p_inter.parameterIsDouble( index, isPreProcess );

			for( var i=0; i<parameterName.length; i++ )
			{
				settings_list.append( {"parameterName":parameterName[i], "parameterValue": parameterValue[i], "parameterDescription":parameterDescription[i], "position":i, "isbool": parameterIsBool[i], "isint": parameterIsInt[i], "isdouble":parameterIsDouble[i] } );
                info_list.append( {"parameterName":parameterName[i], "parameterDescription":parameterDescription[i] } );
			}
		}
	}
}
