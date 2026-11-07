import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.3
import Vedder.vesc.commands 1.0

Item {
    id: root
    anchors.fill: parent
    anchors.margins: 12

    property Commands mCommands: VescIf.commands()

    // Keep every controller waypoint below 180 degrees so that the VESC
    // shortest-path position controller always moves in the requested direction.
    property real maxSegmentDeg: 120.0
    // Allow a small steady-state error at intermediate waypoints. The final
    // absolute target is still approached again by the last segment.
    property real toleranceDeg: 5.0
    property int requiredStableSamples: 5

    property bool positionValid: false
    property bool commandActive: false
    property bool motionComplete: false
    property real lastSingleTurnDeg: 0.0
    property real multiTurnDeg: 0.0
    property real finalTargetDeg: 0.0
    property real waypointDeg: 0.0
    property real waypointCommandDeg: 0.0
    property int stableSamples: 0

    function normalizeDeg(angle) {
        var result = angle % 360.0
        if (result < 0.0) {
            result += 360.0
        }
        return result
    }

    function clamp(value, minimum, maximum) {
        return Math.max(minimum, Math.min(maximum, value))
    }

    function updateWaypoint() {
        var remaining = finalTargetDeg - multiTurnDeg
        if (Math.abs(remaining) <= toleranceDeg) {
            waypointDeg = finalTargetDeg
            waypointCommandDeg = normalizeDeg(waypointDeg)
            motionComplete = true
            statusText.text = "已到达目标，保持位置"
        } else {
            waypointDeg = multiTurnDeg + clamp(remaining, -maxSegmentDeg, maxSegmentDeg)
            waypointCommandDeg = normalizeDeg(waypointDeg)
            motionComplete = false
            statusText.text = "运动中"
        }

        stableSamples = 0
        mCommands.setPos(waypointCommandDeg)
    }

    function startRelativeMove() {
        if (!positionValid) {
            statusText.text = "尚未收到位置数据"
            return
        }

        var delta = Number(relativeAngleInput.text)
        if (!isFinite(delta)) {
            statusText.text = "请输入有效的相对角度"
            return
        }

        finalTargetDeg = multiTurnDeg + delta
        commandActive = true
        motionComplete = false
        updateWaypoint()
    }

    function stopMotor() {
        commandActive = false
        motionComplete = false
        stableSamples = 0
        mCommands.setCurrent(0.0)
        statusText.text = "已停止并释放电机"
    }

    Timer {
        interval: 100
        running: true
        repeat: true

        onTriggered: {
            mCommands.getValues()

            // Position commands must be refreshed so that the VESC communication
            // timeout does not release the motor during a long multi-turn move.
            if (commandActive) {
                mCommands.setPos(waypointCommandDeg)
            }
        }
    }

    Connections {
        target: mCommands

        function onValuesReceived(values, mask) {
            // fault_code can be exposed as a non-numeric enum by some VESC Tool
            // versions. Use the stable fault string instead; otherwise NaN !== 0
            // incorrectly treats FAULT_CODE_NONE as a fault.
            var faultName = String(values.fault_str)
            if (faultName.length > 0 && faultName !== "FAULT_CODE_NONE") {
                commandActive = false
                motionComplete = false
                statusText.text = "控制器故障：" + faultName
                return
            }

            var singleTurn = Number(values.position)
            if (!isFinite(singleTurn)) {
                return
            }

            if (!positionValid) {
                positionValid = true
                lastSingleTurnDeg = singleTurn
                multiTurnDeg = singleTurn
                statusText.text = "位置数据已就绪"
            } else {
                var delta = singleTurn - lastSingleTurnDeg
                if (delta > 180.0) {
                    delta -= 360.0
                } else if (delta < -180.0) {
                    delta += 360.0
                }

                multiTurnDeg += delta
                lastSingleTurnDeg = singleTurn
            }

            if (!commandActive || motionComplete) {
                return
            }

            if (Math.abs(waypointDeg - multiTurnDeg) <= toleranceDeg) {
                stableSamples++
                if (stableSamples >= requiredStableSamples) {
                    updateWaypoint()
                }
            } else {
                stableSamples = 0
            }
        }
    }

    ColumnLayout {
        anchors.fill: parent
        spacing: 10

        Label {
            text: "AS5600 任意角度相对运动"
            font.pixelSize: 22
            font.bold: true
        }

        Label {
            text: "正数正转，负数反转；目标会自动拆分为不超过 " +
                  maxSegmentDeg.toFixed(0) + "° 的位置段。"
            wrapMode: Text.WordWrap
            Layout.fillWidth: true
        }

        RowLayout {
            Layout.fillWidth: true

            Label { text: "相对角度：" }

            TextField {
                id: relativeAngleInput
                Layout.fillWidth: true
                text: "720"
                placeholderText: "例如 1000 或 -720"
                selectByMouse: true
            }

            Label { text: "°" }
        }

        RowLayout {
            Button {
                text: "开始运动"
                enabled: positionValid
                onClicked: startRelativeMove()
            }

            Button {
                text: "停止 / 释放"
                onClicked: stopMotor()
            }

            Button {
                text: "当前位置设为0°"
                enabled: positionValid && !commandActive
                onClicked: {
                    multiTurnDeg = 0.0
                    lastSingleTurnDeg = Number(lastSingleTurnDeg)
                    finalTargetDeg = 0.0
                    waypointDeg = 0.0
                    statusText.text = "上位机累计位置已清零"
                }
            }
        }

        GridLayout {
            columns: 2
            columnSpacing: 15
            rowSpacing: 5

            Label { text: "AS5600单圈位置" }
            Label { text: positionValid ? lastSingleTurnDeg.toFixed(2) + "°" : "--" }

            Label { text: "上位机累计位置" }
            Label { text: positionValid ? multiTurnDeg.toFixed(2) + "°" : "--" }

            Label { text: "最终目标" }
            Label { text: commandActive ? finalTargetDeg.toFixed(2) + "°" : "--" }

            Label { text: "当前分段目标" }
            Label { text: commandActive ? waypointDeg.toFixed(2) + "°" : "--" }
        }

        Label {
            id: statusText
            text: "等待位置数据"
            font.bold: true
            Layout.fillWidth: true
        }

        Label {
            text: "注意：累计圈数只保存在本脚本运行期间。停止脚本、断开连接或设备断电后，必须重新设置零点。"
            color: "orange"
            wrapMode: Text.WordWrap
            Layout.fillWidth: true
        }

        Item { Layout.fillHeight: true }
    }
}
