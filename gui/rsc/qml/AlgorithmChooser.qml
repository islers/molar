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
	id: add_algorithm_window
	title: "Choose algorithm to add"
	height: 300
    width: 600
    minimumHeight: 300
	minimumWidth: 600
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
		anchors.fill: parent
		anchors.topMargin: 10
		anchors.rightMargin: 10
		anchors.bottomMargin: 10
		anchors.leftMargin: 10

		ScrollView
		{
			anchors.fill:parent
			
			ListView
			{
				anchors.fill: parent

				model: ListModel{ id:algorithm_list }
				spacing: 20

				delegate: Row
				{
					property int pos: position
					property string name: algorithmName

					Column
					{
						Text
						{
							text: algorithmName
							wrapMode: Text.Wrap
							font.family: "Helvetica"
							font.pointSize: 8
							font.bold: true
							color: "black"
						}
						Text
						{
							text: algorithmDescription
							wrapMode: Text.Wrap
							font.family: "Helvetica"
							font.pointSize: 8
							color: "black"
							width: 500
						}
					}
					ModarButton
					{
						label: "Add"
						width: 50
						height: 50
						
						MouseArea
						{
							anchors.fill:parent;
							onClicked:
							{
								if( add_algorithm_window.isPreProcess ) p_inter.addPreProcessAlgorithm( algorithmName );
								else  p_inter.addInternPreProcessAlgorithm( algorithmName );
								add_algorithm_window.visible = false;
							}
						}
					}
				}
				Component.onCompleted:
				{
					var nrOfAlgorithms = p_inter.nrOfAlgorithms();
					for( var i=0; i<nrOfAlgorithms; i++ )
					{
						algorithm_list.append( {"algorithmName":p_inter.algorithmName(i), "algorithmDescription":p_inter.algorithmDescription(i), "position":i } );
					}
				}
			}
		}

	}
}
