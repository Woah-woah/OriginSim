import QtQuick 2.0
import QtQuick.Controls 2.0
import QtQuick.Layouts 1.15

Rectangle {
    color: "white"
    width: parent.width  // 不强制填充高度
    height: 50  // 固定高度
    Layout.minimumWidth: 100
    Layout.minimumHeight: 50
    // anchors.fill: parent
    // anchors.horizontalCenter: parent.horizontalCenter
    RowLayout {
        anchors.centerIn: parent
        width: parent.width * 0.95
        spacing: 20

        // 左侧名称标签
        Label {
            text: RefereeUI.label
            color: "black"
            font.pixelSize: 14
            Layout.preferredWidth: 150
        }

        // 滑条主体
        Slider {
            id: sliderControl
            Layout.fillWidth: true
            from: 0.0
            to: RefereeUI.maxValue ? RefereeUI.maxValue : 100.0
            stepSize: 1.0
            value : RefereeUI.defaultValue
            background: Rectangle {
                implicitHeight: 6
                radius: 3
                color: "#505050"
                
                Rectangle {
                    width: sliderControl.visualPosition * parent.width
                    height: parent.height
                    color: "#4CAF50"
                    radius: 3
                }
            }

            handle: Rectangle {
                x: sliderControl.visualPosition * (sliderControl.width - width)
                y: (sliderControl.height - height) / 2
                width: 16
                height: 16
                radius: 8
                color: sliderControl.pressed ? "#FFFFFF" : "#F5F5F5"
                border.color: "#BDBDBD"
            }

            onValueChanged: RefereeUI.OnSlider(value)
        }

        // 右侧上限值
        Label {
            text: sliderControl.value
            color: "black"
            font.pixelSize: 14
            Layout.preferredWidth: 30
        }
    }
}