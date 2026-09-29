import signal
import sys
import time
import rclpy

from python_qt_binding.QtCore import QTimer, Qt
from python_qt_binding.QtGui import QIntValidator
from python_qt_binding.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

from dec_tree.referee_fake import Panel_Publisher
from python_qt_binding.QtCore import QTimer, Qt, QRegularExpression
from python_qt_binding.QtGui import QIntValidator, QRegularExpressionValidator


UINT8_MAX = 255
UINT16_MAX = 65535
UINT32_MAX = 4294967295


GAME_PROGRESS_ITEMS = [
    ("GAME_UNSTARTED", 0),
    ("GAME_READY", 1),
    ("GAME_INITIAL", 2),
    ("GAME_START_COUNTDOWN", 3),
    ("GAME_RUNNING", 4),
    ("GAME_STOP", 5),
]

HEALTH_STATE_ITEMS = [
    ("NORMAL", 0),
    ("DAMAGED", 1),
    ("CRITICAL", 2),
    ("LOST", 3),
]

STATE_NOW_ITEMS = [
    ("UNKNOWN", 0),
    ("IDLE", 1),
    ("PATROL", 2),
    ("ATTACK", 3),
    ("DEFENSE", 4),
    ("RETREAT", 5),
]

RETURN_FORTRESS_ITEMS = [
    ("False / 0", 0),
    ("True / 1", 1),
]


class ControlPanelGui(QMainWindow):
    def __init__(self, title, publisher: Panel_Publisher):
        super().__init__()
        self.publisher = publisher

        self.setWindowTitle(title)
        self.resize(760, 820)

        self.fields = {}
        self.combo_fields = {}

        self._build_ui()
        self._load_defaults()
        self._setup_timers()

    # ---------------------------
    # UI 构建
    # ---------------------------
    def _build_ui(self):
        central = QWidget()
        root_layout = QVBoxLayout(central)
        root_layout.setContentsMargins(12, 12, 12, 12)
        root_layout.setSpacing(10)

        title_label = QLabel("Referee Fake Control Panel")
        title_label.setStyleSheet("font-size: 20px; font-weight: bold;")
        root_layout.addWidget(title_label)

        self.status_label = QLabel("状态：未发布")
        self.status_label.setStyleSheet("color: #1565C0; font-weight: bold;")
        root_layout.addWidget(self.status_label)

        self.live_label = QLabel("当前消息：stage_remain_time=0, game_progress=0")
        self.live_label.setStyleSheet("color: #666666;")
        self.live_label.setWordWrap(True)
        root_layout.addWidget(self.live_label)
        self.state_now_live_label = QLabel("当前 state_now：0")
        self.state_now_live_label.setStyleSheet("color: #444444;")
        root_layout.addWidget(self.state_now_live_label)

        self.pending_state_label = QLabel("待切换 state_now：无")
        self.pending_state_label.setStyleSheet("color: #AA6600;")
        root_layout.addWidget(self.pending_state_label)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_content = QWidget()
        scroll_layout = QVBoxLayout(scroll_content)
        scroll_layout.setSpacing(12)

        # 1) 战斗核心状态
        scroll_layout.addWidget(
            self._make_group_box(
                "核心状态",
                self._build_core_state_group()
            )
        )

        # 2) 枪口 / 热量 / 能量
        scroll_layout.addWidget(
            self._make_group_box(
                "武器与能量",
                self._build_weapon_group()
            )
        )

        # 3) 友方血量
        scroll_layout.addWidget(
            self._make_group_box(
                "友方单位血量",
                self._build_ally_group()
            )
        )

        # 4) 事件状态
        scroll_layout.addWidget(
            self._make_group_box(
                "事件与标志位",
                self._build_event_group()
            )
        )

        scroll_layout.addStretch(1)
        scroll.setWidget(scroll_content)
        root_layout.addWidget(scroll)

        # 按钮区
        btn_layout = QHBoxLayout()

        self.publish_btn = QPushButton("发布一次")
        self.publish_btn.clicked.connect(self.publish_now)
        btn_layout.addWidget(self.publish_btn)

        self.sync_btn = QPushButton("从当前消息回填")
        self.sync_btn.clicked.connect(self.sync_from_message)
        btn_layout.addWidget(self.sync_btn)

        self.reset_btn = QPushButton("恢复默认值")
        self.reset_btn.clicked.connect(self._load_defaults)
        btn_layout.addWidget(self.reset_btn)

        root_layout.addLayout(btn_layout)

        # 自动发布区
        auto_layout = QHBoxLayout()

        self.auto_publish_checkbox = QCheckBox("自动发布")
        self.auto_publish_checkbox.setChecked(True)
        auto_layout.addWidget(self.auto_publish_checkbox)

        auto_layout.addWidget(QLabel("周期(ms):"))
        self.auto_pub_interval = QLineEdit()
        self.auto_pub_interval.setFixedWidth(80)
        self.auto_pub_interval.setValidator(QIntValidator(20, 5000, self))
        self.auto_pub_interval.setText("100")
        auto_layout.addWidget(self.auto_pub_interval)

        self.apply_interval_btn = QPushButton("应用周期")
        self.apply_interval_btn.clicked.connect(self.apply_publish_interval)
        auto_layout.addWidget(self.apply_interval_btn)

        auto_layout.addStretch(1)
        root_layout.addLayout(auto_layout)

        self.setCentralWidget(central)

    def _make_group_box(self, title: str, child_widget: QWidget) -> QGroupBox:
        box = QGroupBox(title)
        box.setStyleSheet("QGroupBox { font-weight: bold; }")
        layout = QVBoxLayout(box)
        layout.addWidget(child_widget)
        return box

    def _build_core_state_group(self) -> QWidget:
        w = QWidget()
        layout = QFormLayout(w)
        layout.setLabelAlignment(Qt.AlignRight)

        self.combo_fields["game_progress"] = self._add_combo(
            layout, "game_progress", GAME_PROGRESS_ITEMS
        )
        self.fields["stage_remain_time"] = self._add_uint_line_edit(
            layout, "stage_remain_time", UINT16_MAX
        )
        self.fields["remain_hp"] = self._add_uint_line_edit(
            layout, "remain_hp", UINT16_MAX
        )
        self.fields["max_hp"] = self._add_uint_line_edit(
            layout, "max_hp", UINT16_MAX
        )
        self.combo_fields["health_state"] = self._add_combo(
            layout, "health_state", HEALTH_STATE_ITEMS
        )
        self.combo_fields["state_now"] = self._add_combo(
            layout, "state_now", STATE_NOW_ITEMS
        )

        return w

    def _build_weapon_group(self) -> QWidget:
        w = QWidget()
        layout = QFormLayout(w)
        layout.setLabelAlignment(Qt.AlignRight)

        self.fields["bullet_remaining_num_17mm"] = self._add_uint_line_edit(
            layout, "bullet_remaining_num_17mm", UINT16_MAX
        )
        self.fields["bullet_cooling_speed"] = self._add_uint_line_edit(
            layout, "bullet_cooling_speed", UINT16_MAX
        )
        self.fields["shooter_heat_limit"] = self._add_uint_line_edit(
            layout, "shooter_heat_limit", UINT16_MAX
        )
        self.fields["shooter_heat_now"] = self._add_uint_line_edit(
            layout, "shooter_heat_now", UINT16_MAX
        )
        self.fields["remain_energy"] = self._add_uint_line_edit(
            layout, "remain_energy", UINT8_MAX
        )

        return w

    def _build_ally_group(self) -> QWidget:
        w = QWidget()
        layout = QFormLayout(w)
        layout.setLabelAlignment(Qt.AlignRight)

        self.fields["ally_1_robot_hp"] = self._add_uint_line_edit(
            layout, "ally_1_robot_hp", UINT16_MAX
        )
        self.fields["ally_2_robot_hp"] = self._add_uint_line_edit(
            layout, "ally_2_robot_hp", UINT16_MAX
        )
        self.fields["ally_3_robot_hp"] = self._add_uint_line_edit(
            layout, "ally_3_robot_hp", UINT16_MAX
        )
        self.fields["ally_4_robot_hp"] = self._add_uint_line_edit(
            layout, "ally_4_robot_hp", UINT16_MAX
        )
        self.fields["ally_outpost_hp"] = self._add_uint_line_edit(
            layout, "ally_outpost_hp", UINT16_MAX
        )
        self.fields["ally_base_hp"] = self._add_uint_line_edit(
            layout, "ally_base_hp", UINT16_MAX
        )

        return w

    def _build_event_group(self) -> QWidget:
        w = QWidget()
        layout = QFormLayout(w)
        layout.setLabelAlignment(Qt.AlignRight)

        self.fields["rfid_status"] = self._add_uint_line_edit(
            layout, "rfid_status", UINT32_MAX
        )
        self.fields["event_type"] = self._add_uint_line_edit(
            layout, "event_type", UINT32_MAX
        )
        self.combo_fields["return_fortress"] = self._add_combo(
            layout, "return_fortress", RETURN_FORTRESS_ITEMS
        )

        return w

    def _add_uint_line_edit(self, layout: QFormLayout, name: str, max_value: int) -> QLineEdit:
        edit = QLineEdit()
        edit.setMinimumWidth(160)
        edit.setPlaceholderText(f"0 ~ {max_value}")

        # QIntValidator 只能处理 32 位有符号整数
        if max_value <= 2147483647:
            edit.setValidator(QIntValidator(0, max_value, self))
        else:
            # 对 uint32 字段，只限制输入为数字，具体范围发布前再检查
            regex = QRegularExpression(r"^\d{0,10}$")
            edit.setValidator(QRegularExpressionValidator(regex, self))

        layout.addRow(QLabel(name), edit)
        return edit

    def _add_combo(self, layout: QFormLayout, name: str, items) -> QComboBox:
        combo = QComboBox()
        for text, value in items:
            combo.addItem(f"{text} ({value})", value)
        combo.setMinimumWidth(220)
        layout.addRow(QLabel(name), combo)
        return combo

    # ---------------------------
    # 默认值
    # ---------------------------
    def _load_defaults(self):
        defaults = {
            "stage_remain_time": 420,
            "remain_hp": 600,
            "max_hp": 600,
            "bullet_remaining_num_17mm": 400,
            "bullet_cooling_speed": 40,
            "shooter_heat_limit": 200,
            "shooter_heat_now": 0,
            "remain_energy": 100,
            "ally_1_robot_hp": 600,
            "ally_2_robot_hp": 600,
            "ally_3_robot_hp": 600,
            "ally_4_robot_hp": 600,
            "ally_outpost_hp": 1000,
            "ally_base_hp": 5000,
            "rfid_status": 0,
            "event_type": 0,
        }

        for k, v in defaults.items():
            if k in self.fields:
                self.fields[k].setText(str(v))

        self._set_combo_value("game_progress", 0)
        self._set_combo_value("health_state", 0)
        self._set_combo_value("state_now", 0)
        self._set_combo_value("return_fortress", 0)

        self.status_label.setText("状态：已恢复默认值")

    # ---------------------------
    # 定时器
    # ---------------------------
    def _setup_timers(self):
        # ROS spin 定时器
        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(self._spin_ros_once)
        self.ros_timer.start(30)

        # 自动发布定时器
        self.publish_timer = QTimer(self)
        self.publish_timer.timeout.connect(self._auto_publish_tick)
        self.publish_timer.start(int(self.auto_pub_interval.text()))

    def apply_publish_interval(self):
        text = self.auto_pub_interval.text().strip()
        interval = int(text) if text else 100
        interval = max(20, interval)
        self.publish_timer.start(interval)
        self.status_label.setText(f"状态：自动发布周期已设置为 {interval} ms")

    def _spin_ros_once(self):
        try:
            rclpy.spin_once(self.publisher, timeout_sec=0.0)
        except Exception as e:
            self.status_label.setText(f"状态：ROS spin 异常：{e}")

        self._refresh_live_label()

    def _auto_publish_tick(self):
        if self.auto_publish_checkbox.isChecked():
            self.publish_now(silent=True)

    # ---------------------------
    # 读写控件
    # ---------------------------
    def _get_uint_value(self, field_name: str) -> int:
        text = self.fields[field_name].text().strip()
        if text == "":
            return 0

        value = int(text)

        field_max = {
            "remain_energy": UINT8_MAX,
            "health_state": UINT8_MAX,
            "state_now": UINT8_MAX,
            "game_progress": UINT8_MAX,
            "return_fortress": UINT8_MAX,

            "remain_hp": UINT16_MAX,
            "max_hp": UINT16_MAX,
            "bullet_remaining_num_17mm": UINT16_MAX,
            "bullet_cooling_speed": UINT16_MAX,
            "shooter_heat_limit": UINT16_MAX,
            "shooter_heat_now": UINT16_MAX,
            "stage_remain_time": UINT16_MAX,
            "ally_1_robot_hp": UINT16_MAX,
            "ally_2_robot_hp": UINT16_MAX,
            "ally_3_robot_hp": UINT16_MAX,
            "ally_4_robot_hp": UINT16_MAX,
            "ally_outpost_hp": UINT16_MAX,
            "ally_base_hp": UINT16_MAX,

            "rfid_status": UINT32_MAX,
            "event_type": UINT32_MAX,
        }

        max_value = field_max.get(field_name, UINT32_MAX)
        if not (0 <= value <= max_value):
            raise ValueError(f"{field_name} 超出范围: {value}, 应为 0 ~ {max_value}")

        return value
    def _get_combo_value(self, field_name: str) -> int:
        return int(self.combo_fields[field_name].currentData())

    def _set_combo_value(self, field_name: str, value: int):
        combo = self.combo_fields[field_name]
        for i in range(combo.count()):
            if int(combo.itemData(i)) == int(value):
                combo.setCurrentIndex(i)
                return

    # ---------------------------
    # 发布逻辑
    # ---------------------------
    def publish_now(self, silent: bool = False):
        try:
            msg = self.publisher.game_state

            msg.remain_hp = self._get_uint_value("remain_hp")
            msg.max_hp = self._get_uint_value("max_hp")
            msg.bullet_remaining_num_17mm = self._get_uint_value("bullet_remaining_num_17mm")
            msg.bullet_cooling_speed = self._get_uint_value("bullet_cooling_speed")
            msg.shooter_heat_limit = self._get_uint_value("shooter_heat_limit")
            msg.shooter_heat_now = self._get_uint_value("shooter_heat_now")
            msg.remain_energy = self._get_uint_value("remain_energy")
            msg.health_state = self._get_combo_value("health_state")
            # msg.state_now = self._get_combo_value("state_now")
            msg.game_progress = self._get_combo_value("game_progress")
            msg.ally_1_robot_hp = self._get_uint_value("ally_1_robot_hp")
            msg.ally_2_robot_hp = self._get_uint_value("ally_2_robot_hp")
            msg.ally_3_robot_hp = self._get_uint_value("ally_3_robot_hp")
            msg.ally_4_robot_hp = self._get_uint_value("ally_4_robot_hp")
            msg.ally_outpost_hp = self._get_uint_value("ally_outpost_hp")
            msg.ally_base_hp = self._get_uint_value("ally_base_hp")
            msg.rfid_status = self._get_uint_value("rfid_status")
            msg.event_type = self._get_uint_value("event_type")
            msg.return_fortress = self._get_combo_value("return_fortress")

            # 只有手动发布时才覆写倒计时
            if not silent:
                msg.stage_remain_time = self._get_uint_value("stage_remain_time")

            self.publisher.game_state_publisher_.publish(msg)

            if not silent:
                self.status_label.setText("状态：已发布一次")

            self._refresh_live_label()

        except Exception as e:
            self.status_label.setText(f"状态：发布失败：{e}")
    def sync_from_message(self):
        try:
            msg = self.publisher.game_state

            mapping_fields = [
                "remain_hp",
                "max_hp",
                "bullet_remaining_num_17mm",
                "bullet_cooling_speed",
                "shooter_heat_limit",
                "shooter_heat_now",
                "remain_energy",
                "stage_remain_time",
                "ally_1_robot_hp",
                "ally_2_robot_hp",
                "ally_3_robot_hp",
                "ally_4_robot_hp",
                "ally_outpost_hp",
                "ally_base_hp",
                "rfid_status",
                "event_type",
            ]

            for name in mapping_fields:
                if hasattr(msg, name) and name in self.fields:
                    self.fields[name].setText(str(getattr(msg, name)))

            if hasattr(msg, "game_progress"):
                self._set_combo_value("game_progress", msg.game_progress)
            if hasattr(msg, "health_state"):
                self._set_combo_value("health_state", msg.health_state)
            if hasattr(msg, "state_now"):
                self._set_combo_value("state_now", msg.state_now)
            if hasattr(msg, "return_fortress"):
                self._set_combo_value("return_fortress", msg.return_fortress)

            self.status_label.setText("状态：已从当前消息回填")
            self._refresh_live_label()

        except Exception as e:
            self.status_label.setText(f"状态：回填失败：{e}")
            self.publisher.get_logger().error(f"Sync from message failed: {e}")

    def _refresh_live_label(self):
        try:
            msg = self.publisher.game_state
            self.live_label.setText(
                "当前消息："
                f"stage_remain_time={getattr(msg, 'stage_remain_time', 0)}, "
                f"game_progress={getattr(msg, 'game_progress', 0)}, "
                f"remain_hp={getattr(msg, 'remain_hp', 0)}, "
                f"heat={getattr(msg, 'shooter_heat_now', 0)}, "
                f"energy={getattr(msg, 'remain_energy', 0)}"
            )

            self.state_now_live_label.setText(
                f"当前 state_now：{getattr(msg, 'state_now', 0)}"
            )

            pending = getattr(self.publisher, 'pending_state_now', None)
            apply_time = getattr(self.publisher, 'pending_apply_time', None)

            if pending is None or apply_time is None:
                self.pending_state_label.setText("待切换 state_now：无")
            else:
                remain = max(0.0, apply_time - time.monotonic())
                self.pending_state_label.setText(
                    f"待切换 state_now：{pending}，剩余延迟 {remain:.2f}s"
                )
        except Exception:
            pass
    # ---------------------------
    # 关闭
    # ---------------------------
    def closeEvent(self, event):
        self.ros_timer.stop()
        self.publish_timer.stop()
        super().closeEvent(event)


def main():
    rclpy.init()

    app = QApplication(sys.argv)
    panel_pub = Panel_Publisher()
    panel = ControlPanelGui("Control Panel", panel_pub)
    panel.show()

    signal.signal(signal.SIGINT, signal.SIG_DFL)
    ret = app.exec_()

    try:
        panel.destroy()
    except Exception:
        pass

    try:
        panel_pub.destroy_node()
    except Exception:
        pass

    rclpy.shutdown()
    sys.exit(ret)


if __name__ == "__main__":
    main()