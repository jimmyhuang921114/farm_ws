"""RQT panel for collection state, sensor health, and mapping coverage."""

import json
import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from collection_interfaces.msg import (
    CollectionEvent,
    CollectionState,
    SensorStatus,
)
from collection_interfaces.srv import (
    AddMarker,
    PreflightCollection,
    StartCollection,
    StopCollection,
)
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QFormLayout, QGroupBox, QLabel, QWidget
from qt_gui.plugin import Plugin
from std_msgs.msg import Bool, String


STATES = [
    'IDLE', 'PREFLIGHT', 'COUNTDOWN', 'RECORDING', 'FINALIZING',
    'VERIFYING', 'COMPLETED', 'FAILED',
]
STATUS = ['UNKNOWN', 'WAITING', 'OK', 'WARNING', 'ERROR']


class CollectionPanel(Plugin):
    """Display the UI-only manager plus mapping and coverage preview state."""

    def __init__(self, context):
        super().__init__(context)
        self.setObjectName('CollectionPanel')
        self.widget = QWidget()
        ui_path = os.path.join(
            get_package_share_directory('collection_rqt_panel'),
            'resource',
            'collection_panel.ui',
        )
        loadUi(ui_path, self.widget)
        ui_only_default = os.environ.get(
            'FARM_COLLECTION_UI_ONLY_DEFAULT', 'true').lower()
        self.widget.uiOnlyCheck.setChecked(
            ui_only_default in ('1', 'true', 'yes', 'on'))
        if context.serial_number() > 1:
            self.widget.setWindowTitle(
                f'Collection Panel ({context.serial_number()})')
        context.add_widget(self.widget)
        self.node = context.node
        self._add_mapping_group()

        self.clients = {
            name: self.node.create_client(service_type, f'/collection/{name}')
            for name, service_type in (
                ('preflight', PreflightCollection),
                ('start', StartCollection),
                ('stop', StopCollection),
                ('add_marker', AddMarker),
            )
        }
        self.node.create_subscription(
            CollectionState, '/collection/state', self.on_state, 10)
        self.node.create_subscription(
            SensorStatus, '/collection/sensor_status', self.on_sensor, 20)
        self.node.create_subscription(
            CollectionEvent, '/collection/events', self.on_event, 20)
        self.node.create_subscription(
            String, '/coverage/status_json', self.on_coverage, 10)
        self.node.create_subscription(
            Bool, '/glim/session_supervisor/ready',
            self.on_mapping_session_ready, 10)

        self.widget.preflightButton.clicked.connect(self.preflight)
        self.widget.startButton.clicked.connect(
            lambda: self.call_empty('start'))
        self.widget.stopButton.clicked.connect(
            lambda: self.call_empty('stop'))
        self.widget.markerButton.clicked.connect(self.marker)
        self.widget.openButton.clicked.connect(self.open_folder)
        self.path = ''

    def _add_mapping_group(self):
        group = QGroupBox('GLIM Mapping / Coverage Preview')
        layout = QFormLayout(group)
        self.mapping_state = QLabel('WAITING — mapping odometry unavailable')
        self.mapping_session = QLabel('Not supervised / not ready')
        self.coverage_cells = QLabel('0 cells / 0.00 m²')
        self.mapping_distance = QLabel('0.00 m')
        self.mapping_poses = QLabel('0')
        layout.addRow('Mode', QLabel('Mapping (not pure localization)'))
        layout.addRow('State', self.mapping_state)
        layout.addRow('GLIM Session', self.mapping_session)
        layout.addRow('Visited Cells', self.coverage_cells)
        layout.addRow('Travel Distance', self.mapping_distance)
        layout.addRow('Accepted Poses', self.mapping_poses)
        scroll_layout = self.widget.scrollContents.layout()
        scroll_layout.insertWidget(max(0, scroll_layout.count() - 2), group)

    def ready(self, name):
        if self.clients[name].service_is_ready():
            return True
        self.on_event_text(f'WAITING: /collection/{name} service')
        return False

    def preflight(self):
        if not self.ready('preflight'):
            return
        request = PreflightCollection.Request()
        request.session_name = self.widget.sessionEdit.text()
        request.location = self.widget.locationEdit.text()
        request.duration_sec = float(self.widget.durationSpin.value())
        request.profile = self.widget.profileEdit.text()
        request.note = self.widget.noteEdit.toPlainText()
        request.ui_only = self.widget.uiOnlyCheck.isChecked()
        self.clients['preflight'].call_async(request).add_done_callback(
            self.preflight_done)

    def preflight_done(self, future):
        try:
            response = future.result()
            self.path = response.session_path
            self.widget.pathValue.setText(self.path)
            self.on_event_text(response.message)
        except Exception as error:  # UI boundary: show service errors to operator.
            self.on_event_text(f'Preflight error: {error}')

    def call_empty(self, name):
        if not self.ready(name):
            return
        service_type = {'start': StartCollection, 'stop': StopCollection}[name]
        self.clients[name].call_async(service_type.Request())

    def marker(self):
        if not self.ready('add_marker'):
            return
        request = AddMarker.Request()
        request.label = 'UI marker'
        self.clients['add_marker'].call_async(request)

    def open_folder(self):
        if self.path:
            subprocess.Popen(['xdg-open', self.path])

    def on_state(self, message):
        self.path = message.session_path or self.path
        state = (
            STATES[message.state]
            if message.state < len(STATES)
            else 'UNKNOWN'
        )
        self.widget.stateValue.setText(state)
        self.widget.elapsedValue.setText(f'{message.elapsed_sec:.1f} s')
        self.widget.remainingValue.setText(f'{message.remaining_sec:.1f} s')
        self.widget.bagValue.setText(
            f'{message.bag_size_bytes / 1e6:.1f} MB')
        self.widget.diskValue.setText(f'{message.disk_free_gb:.1f} GB')
        self.widget.pathValue.setText(self.path)

    def on_sensor(self, message):
        labels = {
            'LiDAR Packets': 'packetsStatus',
            'LiDAR Points': 'pointsStatus',
            'IMU': 'imuStatus',
            'Camera': 'cameraStatus',
            'Camera 1280x720': 'cameraStatus',
            'CameraInfo': 'cameraInfoStatus',
            'GLIM TF': 'glimStatus',
            'Disk': 'diskStatus',
        }
        if message.name in labels:
            label = getattr(self.widget, labels[message.name])
            label.setText(f'{STATUS[message.status]} — {message.detail}')

    def on_coverage(self, message):
        try:
            status = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError) as error:
            self.mapping_state.setText(f'ERROR — invalid status JSON: {error}')
            return
        if status.get('pure_localization') is not False:
            self.mapping_state.setText('ERROR — unsupported mapping mode')
            return
        state = status.get('state', 'UNKNOWN')
        age = status.get('last_message_age_sec')
        age_text = '' if age is None else f'; age={float(age):.2f} s'
        self.mapping_state.setText(f'{state}{age_text}')
        self.coverage_cells.setText(
            f"{int(status.get('visited_cell_count', 0))} cells / "
            f"{float(status.get('visited_area_m2', 0.0)):.2f} m²")
        self.mapping_distance.setText(
            f"{float(status.get('travel_distance_m', 0.0)):.2f} m")
        self.mapping_poses.setText(str(int(status.get('pose_count', 0))))

    def on_mapping_session_ready(self, message):
        self.mapping_session.setText(
            'Ready' if message.data else 'Not ready / restarting')

    def on_event(self, message):
        text = f'[{message.level}] {message.message} {message.marker}'.strip()
        self.on_event_text(text)

    def on_event_text(self, text):
        self.widget.eventsEdit.appendPlainText(text)

    def shutdown_plugin(self):
        """No external processes are owned by the panel."""
