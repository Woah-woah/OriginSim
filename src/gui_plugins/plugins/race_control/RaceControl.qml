import QtQuick 2.0
import QtQuick.Controls 2.0
import QtQuick.Layouts 1.15


Rectangle {
    color: "white"
    width: parent.width
    height: 100  // 稍微高一点
    Layout.minimumWidth: 100
    Layout.minimumHeight: 100

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 8
        spacing: 6
        RowLayout {
            width: parent.width * 0.95
            spacing: 20
            Layout.alignment: Qt.AlignHCenter
            // 顶部时间显示
            Label {
                text: RaceControl.time
                color: "black"
                font.pixelSize: 16
                horizontalAlignment: Text.AlignHCenter
                Layout.alignment: Qt.AlignHCenter
            }
            Label {
                text: RaceControl.bulletablesend
                color: "black"
                font.pixelSize: 16
                horizontalAlignment: Text.AlignHCenter
                Layout.alignment: Qt.AlignHCenter
            }
        }

        // 主体滑条 + 标签
        RowLayout {
            width: parent.width * 0.95
            spacing: 20
            Layout.alignment: Qt.AlignHCenter

            // // 左侧标签
            // Label {
            //     text: RaceControl.label
            //     color: "black"
            //     font.pixelSize: 14
            //     Layout.preferredWidth: 150
            // }

            // 滑条
            Slider {
                id: sliderControl
                Layout.fillWidth: true
                from: 0.0
                to: RaceControl.maxValue ? RaceControl.maxValue : 100.0
                stepSize: 1.0
                value: RaceControl.currentTime

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

                onValueChanged: RaceControl.OnSlider(value)
            }

            // // 数值标签
            // Label {
            //     text: sliderControl.value
            //     color: "black"
            //     font.pixelSize: 14
            //     Layout.preferredWidth: 30
            // }
            // 重置按钮

        }
        RowLayout {
            width: parent.width * 0.95
            spacing: 20
            Layout.alignment: Qt.AlignHCenter
            Button {
                text: "开始比赛"
                palette.button: "#4CAF50" // 绿色背景
                onClicked: RaceControl.startRace()
            }

            Button {
                text: "停止比赛"
                palette.button: "#F44336" // 红色背景
                onClicked: RaceControl.stopRace()
            }
            Button {
                text: "重置时间"
                Layout.alignment: Qt.AlignHCenter
                onClicked: RaceControl.resetSlider()
            }
            Button {
                text: "给予弹丸"
                Layout.alignment: Qt.AlignHCenter
                onClicked: RaceControl.sendBullet()
            }
        }
        
    }
}
