import dearpygui.dearpygui as dpg
import numpy as np
import cv2 as cv
import serial.tools.list_ports
import time

from challenge import Challenge
from config import Config
from lights import LightsController, StripEffect, TurnSignal

from path import HOME_PATH, Path
from perception.line_tracking import LineTrackingResult
from perception.sign_worker import DEBUG_HEIGHT, DEBUG_WIDTH, SignSnapshot
from input.sign_policy import PolicyState, SignConstraint
from cargo import CargoReading
from input.mission import LETTERS, Mission, MissionCommand, MissionState


_PATH_LABELS = {
    "Normal": Path.NORMAL,
    "Straight": Path.STRAIGHT,
    "Detour": Path.DETOUR,
}
_PATH_NAMES = {path: label for label, path in _PATH_LABELS.items()}
_PATH_COLOR_IDLE = (150, 150, 150)
_PATH_COLOR_DETOUR = (255, 160, 40)
_PATH_COLOR_WARN = (255, 80, 80)


_CHALLENGE_LABELS = {
    "Delivery": Challenge.DELIVERY,
    "Speed": Challenge.SPEED,
}
_CHALLENGE_NAMES = {challenge: label for label, challenge in _CHALLENGE_LABELS.items()}


_TURN_SIGNAL_LABELS = {
    "Off": TurnSignal.OFF,
    "Left": TurnSignal.LEFT,
    "Right": TurnSignal.RIGHT,
    "Hazard": TurnSignal.HAZARD,
}

_STRIP_EFFECT_LABELS = {
    "Auto": None,
    "Off": StripEffect.OFF,
    "Red": StripEffect.RED,
    "Green": StripEffect.GREEN,
    "Blue": StripEffect.BLUE,
    "Turquoise": StripEffect.CYAN,
    "Red Pulse": StripEffect.RED_PULSE,
    "Strobe": StripEffect.STROBE,
    "Rainbow": StripEffect.RAINBOW,
    "Police": StripEffect.POLICE,
    "Blue Breathe": StripEffect.BLUE_BREATHE,
    "Turquoise Sweep": StripEffect.CYAN_SWEEP,
    "Yellow Pulse": StripEffect.YELLOW_PULSE,
}

_TURN_SIGNAL_NAMES = {signal: label for label, signal in _TURN_SIGNAL_LABELS.items()}
_STRIP_EFFECT_NAMES = {effect: label for label, effect in _STRIP_EFFECT_LABELS.items()}

_SONAR_STATES = {
    "clear": ("clear", (0, 255, 0)),
    "slow": ("slowing", (255, 200, 0)),
    "stop": ("STOP", (255, 80, 80)),
    "stale": ("STALE", (255, 80, 80)),
}
_SONAR_SIDES = ("L", "C", "R")

_NO_LETTER = "-"

_MISSION_COLOUR = {
    MissionState.IDLE: (150, 150, 150),
    MissionState.DRIVING: (0, 255, 0),
    MissionState.SLOWING: (255, 200, 0),
    MissionState.DOCKED: (120, 160, 255),
    MissionState.CHECKING: (255, 200, 0),
    MissionState.LEAVING: (255, 200, 0),
    MissionState.DONE: (130, 200, 255),
}

_POLICY_COLOUR = {
    PolicyState.CLEAR: (0, 255, 0),
    PolicyState.SLOWING: (255, 200, 0),
    PolicyState.HOLDING: (255, 80, 80),
    PolicyState.DWELLING: (255, 80, 80),
    PolicyState.WAITING_GREEN: (255, 80, 80),
}


class TunerGUI:
    def __init__(self, config: Config, frame_w: int, frame_h: int):
        self._estop_toggle = False
        self._mode_toggle = False
        self._stopped = True
        self._tags = {}
        self._signs_uploaded_at = -1.0
        self._lights: LightsController | None = None
        self._path_request: Path | None = None
        self._challenge_request: Challenge | None = None
        self._mission_request: tuple[str | None, str | None] | None = None
        self._shown_letters: tuple[str | None, str | None] = (None, None)
        self._dock_action: str | None = None
        self._dock_request: str | None = None
        self._open_request = False
        self._shown_lights: tuple | None = None
        self._tuning_request: dict[str, float] = {}
        self._shown_tuning: dict[str, float] | None = None
        self._build(config, frame_w, frame_h)

        self._refresh_ports()

    def _snap_odd(self, sender, value):
        dpg.set_value(sender, max(3, value | 1))

    def _toggle_mode(self, _sender):
        self._mode_toggle = True

    def _toggle_estop(self, _sender):
        self._estop_toggle = True

    def poll_estop_toggle(self) -> bool:
        """The e-stop button was clicked since the last call."""
        toggled, self._estop_toggle = self._estop_toggle, False
        return toggled

    def poll_mode_toggle(self) -> bool:
        """The mode button was clicked since the last call."""
        toggled, self._mode_toggle = self._mode_toggle, False
        return toggled

    def update_drive(self, estopped: bool, manual: bool):
        """Show the loop's e-stop and mode on the two buttons."""
        self._stopped = estopped
        dpg.set_item_label(self._tags["mode"], "Manual Mode" if manual else "Autonomous Mode")
        dpg.set_item_label(self._tags["estop"], "Resume" if estopped else "Emergency Stop")

    def _build(self, config: Config, frame_w: int, frame_h: int):
        dpg.create_context()

        logo_width, logo_height, _, logo_data = dpg.load_image("assets/logo.png")

        with dpg.font_registry():
            font = dpg.add_font("assets/Geist-Regular.ttf", 30, tag="ttf-font")
        dpg.bind_font(font)
        dpg.set_global_font_scale(0.5)

        with dpg.theme() as global_theme:
            with dpg.theme_component(dpg.mvAll):
                dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 4)
                dpg.add_theme_style(dpg.mvStyleVar_WindowPadding, 12, 8)
                dpg.add_theme_style(dpg.mvStyleVar_ItemSpacing, 8, 6)
                dpg.add_theme_style(dpg.mvStyleVar_FramePadding, 6, 4)
        dpg.bind_theme(global_theme)

        blank = np.ones(frame_w * frame_h * 4, dtype=np.float32)
        with dpg.texture_registry():
            self._tags["logo_tex"] = dpg.add_static_texture(logo_width, logo_height, logo_data)
            self._tags["tex_main"] = dpg.add_raw_texture(frame_w, frame_h, blank.copy(), format=dpg.mvFormat_Float_rgba)
            self._tags["tex_blur"] = dpg.add_raw_texture(frame_w, frame_h, blank.copy(), format=dpg.mvFormat_Float_rgba)
            blank_signs = np.ones(DEBUG_WIDTH * DEBUG_HEIGHT * 4, dtype=np.float32)
            self._tags["tex_signs"] = dpg.add_raw_texture(DEBUG_WIDTH, DEBUG_HEIGHT, blank_signs, format=dpg.mvFormat_Float_rgba)

        grid_w = frame_w // 2
        grid_h = frame_h // 2

        form_w = grid_w - 100

        vp_w = grid_w * 3 + 80
        vp_h = frame_h + 160
        dpg.create_viewport(title="Autonomous Car Tuner", width=vp_w, height=vp_h, resizable=True)

        with dpg.window(tag="primary", no_title_bar=True, no_move=True, no_resize=True):
            with dpg.table(header_row=False, borders_innerH=False,
                           borders_innerV=False, borders_outerH=False,
                           borders_outerV=False, pad_outerX=False):
                dpg.add_table_column(width_stretch=True)
                dpg.add_table_column(width_fixed=True)
                with dpg.table_row():
                    with dpg.group(horizontal=True):
                        dpg.add_image(self._tags["logo_tex"], width=24, height=24)
                        dpg.add_spacer(width=4)
                        self._tags["tel_fps"] = dpg.add_text("FPS: --", color=(0, 255, 0), wrap=100)
                        dpg.add_spacer(width=4)
                        self._tags["tel_error"] = dpg.add_text("Error: --", color=(0, 255, 0))
                        dpg.add_spacer(width=4)
                        self._tags["tel_steer"] = dpg.add_text("Steer: --", color=(0, 255, 0))
                        dpg.add_spacer(width=4)
                        self._tags["tel_throttle"] = dpg.add_text("Throttle: --", color=(0, 255, 0))
                        dpg.add_spacer(width=4)
                        self._tags["tel_sonar"] = dpg.add_text("Sonar: --", color=(150, 150, 150))
                        dpg.add_spacer(width=4)
                        self._tags["tel_cargo"] = dpg.add_text("Cargo: --", color=(150, 150, 150))
                        dpg.add_spacer(width=4)
                        self._tags["tel_lane"] = dpg.add_text("Lane: --", color=(150, 150, 150))
                        dpg.add_spacer(width=4)
                        self._tags["tel_stop"] = dpg.add_text("Stop: --", color=(150, 150, 150))
                        dpg.add_spacer(width=4)
                        self._tags["tel_signs"] = dpg.add_text("Signs: --", color=(150, 150, 150))
                    with dpg.group(horizontal=True, height=24):
                        dpg.add_text("Stage:", color=_PATH_COLOR_IDLE)
                        self._tags["challenge"] = dpg.add_radio_button(
                            tuple(_CHALLENGE_LABELS),
                            horizontal=True,
                            default_value=_CHALLENGE_NAMES[Challenge.DELIVERY],
                            callback=self._on_challenge,
                        )
                        dpg.add_spacer(width=8)
                        self._tags["path_label"] = dpg.add_text("Path:", color=_PATH_COLOR_IDLE)
                        self._tags["path"] = dpg.add_radio_button(
                            tuple(_path_item_label(label) for label in _PATH_LABELS),
                            horizontal=True,
                            default_value=_path_item_label(_PATH_NAMES[HOME_PATH]),
                            callback=self._on_path,
                        )
                        dpg.add_spacer(width=8)
                        self._tags["mode"] = dpg.add_button(label="Autonomous Mode", callback=self._toggle_mode, width=120)
                        self._tags["estop"] = dpg.add_button(label="Emergency Stop", callback=self._toggle_estop, width=110)


            with dpg.table(header_row=False, borders_innerH=False,
                           borders_innerV=False, borders_outerH=False,
                           borders_outerV=False, pad_outerX=False):

                dpg.add_table_column(width_fixed=True, init_width_or_weight=grid_w)
                dpg.add_table_column(width_fixed=True, init_width_or_weight=grid_w)
                dpg.add_table_column(width_fixed=True, init_width_or_weight=grid_w)

                with dpg.table_row():
                    with dpg.group():
                        dpg.add_text("Camera")
                        dpg.add_image(self._tags["tex_main"], width=grid_w, height=grid_h)
                    with dpg.group():
                        dpg.add_text("Threshold + ROI")
                        dpg.add_image(self._tags["tex_blur"], width=grid_w, height=grid_h)
                    with dpg.group():
                        dpg.add_text("Signs")
                        dpg.add_image(self._tags["tex_signs"], width=grid_w, height=grid_w * 9 // 16)

                with dpg.table_row():
                    with dpg.group():
                        dpg.add_spacer(height=4)
                        dpg.add_text("PID Tuning", color=(130, 200, 255))
                        self._tags["kp"] = dpg.add_slider_float(label="Kp", default_value=config.kp, min_value=0.0, max_value=0.1, format="%.4f", width=form_w, callback=self._on_tuning, user_data="kp")
                        self._tags["ki"] = dpg.add_slider_float(label="Ki", default_value=config.ki, min_value=0.0, max_value=0.05, format="%.4f", width=form_w, callback=self._on_tuning, user_data="ki")
                        self._tags["kd"] = dpg.add_slider_float(label="Kd", default_value=config.kd, min_value=0.0, max_value=0.1, format="%.4f", width=form_w, callback=self._on_tuning, user_data="kd")
                        dpg.add_spacer(height=4)
                        self._tags["pid_reset"] = dpg.add_button(label="Reset PID")

                        dpg.add_spacer(height=8)
                        dpg.add_text("Speed Tuning", color=(130, 200, 255))
                        self._tags["base_speed"] = dpg.add_slider_float(label="Base Speed", default_value=config.base_speed, min_value=0.0, max_value=1.0, format="%.2f", width=form_w, callback=self._on_tuning, user_data="base_speed")
                        self._tags["min_speed"] = dpg.add_slider_float(label="Min Speed", default_value=config.min_speed, min_value=0.0, max_value=1.0, format="%.2f", width=form_w, callback=self._on_tuning, user_data="min_speed")

                    with dpg.group():
                        dpg.add_spacer(height=4)
                        dpg.add_text("ROI", color=(130, 200, 255))
                        self._tags["alpha"] = dpg.add_slider_float(label="EMA Alpha", default_value=config.ema_alpha, min_value=0.01, max_value=1.0, format="%.2f", width=form_w)
                        self._tags["scan_lines"] = dpg.add_slider_int(label="Scan Lines", default_value=config.scan_lines, min_value=1, max_value=15, width=form_w)
                        self._tags["roi_x0"] = dpg.add_slider_float(label="ROI X Start", default_value=config.roi_x0, min_value=0.0, max_value=0.49, format="%.2f", width=form_w)
                        self._tags["roi_x1"] = dpg.add_slider_float(label="ROI X End", default_value=config.roi_x1, min_value=0.51, max_value=1.0, format="%.2f", width=form_w)
                        self._tags["roi_y0"] = dpg.add_slider_float(label="ROI Y Start", default_value=config.roi_y0, min_value=0.1, max_value=0.95, format="%.2f", width=form_w)
                        self._tags["roi_y1"] = dpg.add_slider_float(label="ROI Y End", default_value=config.roi_y1, min_value=0.5, max_value=0.99, format="%.2f", width=form_w)

                        dpg.add_spacer(height=8)
                        dpg.add_text("Threshold", color=(130, 200, 255))
                        self._tags["blur_k"] = dpg.add_slider_int(label="Blur Kernel", default_value=config.blur_ksize, min_value=3, max_value=15, callback=self._snap_odd, width=form_w)
                        self._tags["block_size"] = dpg.add_slider_int(label="Block Size", default_value=config.block_size, min_value=3, max_value=151, callback=self._snap_odd, width=form_w)
                        self._tags["threshold_c"] = dpg.add_slider_int(label="Constant", default_value=int(config.threshold_constant), min_value=-50, max_value=50, width=form_w)

                    with dpg.group():
                        dpg.add_spacer(height=8)
                        dpg.add_text("COM", color=(130, 200, 255))
                        self._tags["master_com"] = dpg.add_combo(items=[], label="Master COM", width=form_w)
                        self._tags["refresh_ports"] = dpg.add_button(label="Refresh", callback=self._refresh_ports)

                        dpg.add_spacer(height=8)
                        dpg.add_text("Lights", color=(130, 200, 255))
                        with dpg.group(horizontal=True):
                            dpg.add_text("Turn Signal")
                            self._tags["turn_signal"] = dpg.add_radio_button(
                                tuple(_TURN_SIGNAL_LABELS.keys()),
                                horizontal=True,
                                default_value="Off",
                                callback=self._on_turn_signal,
                            )
                        dpg.add_spacer(height=4)
                        with dpg.group(horizontal=True):
                            self._tags["drl"] = dpg.add_checkbox(label="DRL", callback=self._on_drl)
                            dpg.add_spacer(width=4)
                            self._tags["low_beam"] = dpg.add_checkbox(label="Low Beam", callback=self._on_low_beam)
                            dpg.add_spacer(width=4)
                            self._tags["reverse"] = dpg.add_checkbox(label="Reverse", callback=self._on_reverse)
                            dpg.add_spacer(width=4)
                            self._tags["logo"] = dpg.add_checkbox(label="Logo Projector", callback=self._on_logo)
                            dpg.add_spacer(width=4)
                            self._tags["projector"] = dpg.add_checkbox(label="Projector", callback=self._on_projector)
                        dpg.add_spacer(height=4)
                        self._tags["strip_effect"] = dpg.add_combo(
                            items=tuple(_STRIP_EFFECT_LABELS.keys()),
                            label="LED Strip",
                            default_value="Auto",
                            width=form_w,
                            callback=self._on_strip_effect,
                        )

                        dpg.add_spacer(height=8)
                        dpg.add_text("Signs", color=(130, 200, 255))
                        self._tags["signs_policy"] = dpg.add_text("Policy: --", color=(150, 150, 150))
                        self._tags["signs_detail"] = dpg.add_text("", color=(200, 200, 200))

                        dpg.add_spacer(height=8)
                        dpg.add_text("Mission", color=(130, 200, 255))
                        for key, label in (("pickup", "Pickup  "), ("delivery", "Delivery")):
                            with dpg.group(horizontal=True):
                                dpg.add_text(label)
                                self._tags[key] = dpg.add_radio_button(
                                    (_NO_LETTER,) + LETTERS,
                                    horizontal=True,
                                    default_value=_NO_LETTER,
                                    callback=self._on_mission_letters,
                                )
                        self._tags["mission_status"] = dpg.add_text("Mission: idle", color=(150, 150, 150))
                        self._tags["mission_route"] = dpg.add_text("", color=(200, 200, 200))
                        self._tags["dock"] = dpg.add_button(label="Release truck", callback=self._on_dock,
                                                            enabled=False, width=140)
                        self._tags["open_cargo"] = dpg.add_button(label="Open compartment", callback=self._on_open,
                                                                  enabled=False, width=140)

        dpg.set_primary_window("primary", True)
        dpg.setup_dearpygui()
        dpg.show_viewport()

    def _on_tuning(self, _sender, value, key):
        self._tuning_request[key] = value

    def poll_tuning_request(self) -> dict[str, float]:
        """The PID and speed sliders moved since the last call, keyed like Config."""
        request, self._tuning_request = self._tuning_request, {}
        return request

    def update_tuning(self, tuning: dict[str, float]):
        """Show the loop's PID gains and speeds, whichever console set them."""
        if tuning == self._shown_tuning:
            return
        self._shown_tuning = dict(tuning)
        for key, value in tuning.items():
            dpg.set_value(self._tags[key], value)

    def read_params(self) -> Config:
        """Every tuner slider but the PID and speeds, which the loop holds."""
        return Config(
            block_size=dpg.get_value(self._tags["block_size"]),
            threshold_constant=dpg.get_value(self._tags["threshold_c"]),
            blur_ksize=dpg.get_value(self._tags["blur_k"]),
            roi_x0=dpg.get_value(self._tags["roi_x0"]),
            roi_x1=dpg.get_value(self._tags["roi_x1"]),
            roi_y0=dpg.get_value(self._tags["roi_y0"]),
            roi_y1=dpg.get_value(self._tags["roi_y1"]),
            ema_alpha=dpg.get_value(self._tags["alpha"]),
            scan_lines=dpg.get_value(self._tags["scan_lines"]),


        )

    def set_pid_reset_callback(self, callback):
        dpg.set_item_callback(self._tags["pid_reset"], callback)

    def set_master_port_callback(self, callback):
        dpg.set_item_callback(self._tags["master_com"], lambda _s, port, _u: callback(port))

    def set_lights_controller(self, controller: LightsController):
        self._lights = controller

    def _refresh_ports(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        for tag_key in ("master_com",):
            tag = self._tags[tag_key]
            current = dpg.get_value(tag)
            dpg.configure_item(tag, items=ports)
            if current not in ports:
                dpg.set_value(tag, "")

    def _on_turn_signal(self, _sender, value):
        if self._lights is not None:
            self._lights.turn_signal = _TURN_SIGNAL_LABELS[value]

    def _on_drl(self, _sender, value):
        if self._lights is not None:
            self._lights.drl = value

    def _on_low_beam(self, _sender, value):
        if self._lights is not None:
            self._lights.low_beam = value

    def _on_logo(self, _sender, value):
        if self._lights is not None:
            self._lights.logo = value

    def _on_projector(self, _sender, value):
        if self._lights is not None:
            self._lights.projector = value

    def _on_reverse(self, _sender, value):
        if self._lights is not None:
            self._lights.reverse = value

    def _on_strip_effect(self, _sender, value):
        if self._lights is not None:
            self._lights.strip_effect = _STRIP_EFFECT_LABELS[value]

    def update_lights(self, controller: LightsController):
        """Show the light settings, whichever console set them. Only on a
        change, so a click still on its way through a callback is not undone."""
        shown = (controller.turn_signal, controller.drl, controller.low_beam,
                 controller.logo, controller.projector, controller.strip_effect)
        if shown == self._shown_lights:
            return
        self._shown_lights = shown
        dpg.set_value(self._tags["turn_signal"], _TURN_SIGNAL_NAMES[controller.turn_signal])
        for key in ("drl", "low_beam", "logo", "projector"):
            dpg.set_value(self._tags[key], getattr(controller, key))
        dpg.set_value(self._tags["strip_effect"], _STRIP_EFFECT_NAMES[controller.strip_effect])

    def update_textures(self, result: LineTrackingResult):
        dpg.set_value(self._tags["tex_main"], _frame_to_texture(result.display_frame))
        dpg.set_value(self._tags["tex_blur"], _frame_to_texture(result.debug_frame))

    def update_telemetry(self, fps: float, error: int, steering: float, throttle: float, lane_state: str = "--", stop_reason: str | None = None):
        dpg.set_value(self._tags["tel_fps"], f"FPS: {fps:.1f}")
        dpg.set_value(self._tags["tel_error"], f"Error: {error:+d} px")
        dpg.set_value(self._tags["tel_steer"], f"Steer: {steering:+.3f}")
        dpg.set_value(self._tags["tel_throttle"], f"Throttle: {throttle:.2f}")
        dpg.set_value(self._tags["tel_lane"], f"Lane: {lane_state}")
        dpg.set_value(self._tags["tel_stop"], f"Stop: {stop_reason or '--'}")
        dpg.configure_item(self._tags["tel_stop"], color=(255, 80, 80) if stop_reason else (150, 150, 150))

    # --- stage -----------------------------------------------------------

    def _on_challenge(self, _sender, value):
        self._challenge_request = _CHALLENGE_LABELS[value]

    def poll_challenge_request(self) -> Challenge | None:
        """The stage the operator clicked since the last call, if any."""
        request, self._challenge_request = self._challenge_request, None
        return request

    def update_challenge(self, challenge: Challenge):
        """Show the loop's stage (a change it refused snaps back). The speed
        test has no path choice and no mission."""
        dpg.set_value(self._tags["challenge"], _CHALLENGE_NAMES[challenge])
        enabled = challenge is Challenge.DELIVERY
        for key in ("path", "pickup", "delivery"):
            dpg.configure_item(self._tags[key], enabled=enabled)

    # --- path ------------------------------------------------------------

    def _on_path(self, _sender, value):
        self._path_request = _PATH_LABELS[_path_item_name(value)]

    def poll_path_request(self) -> Path | None:
        """The path the operator clicked since the last call, if any."""
        request, self._path_request = self._path_request, None
        return request

    def update_path_status(self, path: Path, profile_ready: bool, lane_state: str):
        """Show the loop's path on the radio. Colour the label: orange while
        the Detour is armed, red when an anchored path was selected before the
        tracker has a lane width."""
        dpg.set_value(self._tags["path"], _path_item_label(_PATH_NAMES[path]))
        tag = self._tags["path_label"]
        if path is not Path.NORMAL and not profile_ready:
            dpg.set_value(tag, f"Path ({lane_state}):")
            dpg.configure_item(tag, color=_PATH_COLOR_WARN)
        elif path is Path.DETOUR:
            dpg.set_value(tag, "Path:")
            dpg.configure_item(tag, color=_PATH_COLOR_DETOUR)
        else:
            dpg.set_value(tag, "Path:")
            dpg.configure_item(tag, color=_PATH_COLOR_IDLE)

    def update_sonar(self, reading, guard):
        """Show what the sonar guard decided and every sensor's echo."""
        tag = self._tags["tel_sonar"]
        if reading is None:
            dpg.set_value(tag, "Sonar: no data")
            dpg.configure_item(tag, color=(255, 80, 80))
            return

        label, color = _SONAR_STATES.get(guard.label, ("?", (150, 150, 150)))
        distance = "clear" if guard.nearest_cm is None else f"{guard.nearest_cm:.0f} cm"
        sensors = []
        for side, d in zip(_SONAR_SIDES, guard.distances):
            sensors.append(f"{side} {'--' if d is None else f'{d:.0f}'}")
        dpg.set_value(tag, f"Sonar: {distance} [{label}]  {'  '.join(sensors)}")
        dpg.configure_item(tag, color=color)

    def update_cargo(self, cargo: CargoReading | None):
        """The compartment as the cargo node reports it; "--" without one.
        "Open compartment" only works e-stopped, with a cargo node."""
        dpg.configure_item(self._tags["open_cargo"], enabled=cargo is not None and self._stopped)
        tag = self._tags["tel_cargo"]
        if cargo is None:
            dpg.set_value(tag, "Cargo: --")
            dpg.configure_item(tag, color=(150, 150, 150))
            return
        weight = "scale fault" if cargo.fault else f"{cargo.grams} g{'' if cargo.stable else ' ~'}"
        lid = "lid closed" if cargo.lid_closed else "LID OPEN"
        lock = "unlocked" if cargo.unlocked else "locked"
        dpg.set_value(tag, f"Cargo: {weight} · {lid} · {lock}")
        colour = (255, 80, 80) if cargo.fault else (255, 200, 0) if not cargo.lid_closed or cargo.unlocked else (0, 255, 0)
        dpg.configure_item(tag, color=colour)

    def update_signs(self, snapshot: SignSnapshot, constraint: SignConstraint, applied: bool, why: str = "manual"):
        # The worker publishes at ~10 fps and this loop runs at ~30: only
        # upload a frame once.
        if snapshot.frame is not None and snapshot.timestamp != self._signs_uploaded_at:
            dpg.set_value(self._tags["tex_signs"], _frame_to_texture(snapshot.frame))
            self._signs_uploaded_at = snapshot.timestamp

        chip = self._tags["tel_signs"]
        if snapshot.paused:
            dpg.set_value(chip, "Signs: paused")
            dpg.configure_item(chip, color=(150, 150, 150))
        elif not snapshot.camera_open:
            dpg.set_value(chip, f"Signs: {snapshot.status or 'NO CAMERA'}")
            dpg.configure_item(chip, color=(255, 80, 80))
        elif not snapshot.detector_available:
            dpg.set_value(chip, f"Signs: {snapshot.status or 'no model'}")
            dpg.configure_item(chip, color=(255, 200, 0))
        elif constraint.stale:
            dpg.set_value(chip, f"Signs: STALE {snapshot.age(_monotonic()):.1f}s")
            dpg.configure_item(chip, color=(255, 80, 80))
        else:
            seen = [_short(track) for track in snapshot.tracks if track.actionable
                    and track.label in ("stop", "traffic_light")]
            dpg.set_value(chip, "Signs: " + (", ".join(seen) if seen else "clear"))
            dpg.configure_item(chip, color=(0, 255, 0) if not seen else (255, 200, 0))

        cap = constraint.scale * dpg.get_value(self._tags["base_speed"])
        text = f"Policy: {constraint.state.value}"
        if constraint.reason:
            text += f" - {constraint.reason}"
        text += f" · cap {cap:.2f}"
        if not applied:
            text += f" (not applied - {why})"
        dpg.set_value(self._tags["signs_policy"], text)
        dpg.configure_item(self._tags["signs_policy"], color=_POLICY_COLOUR[constraint.state])

        lines = []
        for track in snapshot.tracks:
            lines.append(_detail(track))
        dpg.set_value(self._tags["signs_detail"], "\n".join(lines[:6]))

    # --- mission ---------------------------------------------------------

    def _on_mission_letters(self, sender, value):
        # The clicked value, not dpg.get_value: callbacks run on DearPyGui's
        # thread and `update_mission` may already have snapped the radio back.
        letter = None if value == _NO_LETTER else value
        pickup, delivery = self._shown_letters
        if sender == self._tags["pickup"]:
            pickup = letter
        else:
            delivery = letter
        self._mission_request = (pickup, delivery)

    def poll_mission_request(self) -> tuple[str | None, str | None] | None:
        """The (pickup, delivery) letters the operator picked since the last
        call, if any; None stands for "-"."""
        request, self._mission_request = self._mission_request, None
        return request

    def _on_dock(self, _sender):
        # The action the button showed when it was clicked.
        self._dock_request = self._dock_action

    def _on_open(self, _sender):
        self._open_request = True

    def poll_open_request(self) -> bool:
        """"Open compartment" clicked since the last call."""
        request, self._open_request = self._open_request, False
        return request

    def poll_dock_request(self) -> str | None:
        """"release" or "wait", clicked since the last call."""
        request, self._dock_request = self._dock_request, None
        return request

    def update_mission(self, mission: Mission, command: MissionCommand, applied: bool, why: str = "manual"):
        """Show the mission's letters (a change it refused snaps back) and state."""
        self._shown_letters = (mission.pickup, mission.delivery)
        dpg.set_value(self._tags["pickup"], mission.pickup or _NO_LETTER)
        dpg.set_value(self._tags["delivery"], mission.delivery or _NO_LETTER)

        text = f"Mission: {mission.stage.value}"
        if mission.target:
            text += f" {mission.target} · next exit {mission.next_exit}"
        if mission.dock is not None:
            waited = int(time.monotonic() - mission.dock.since)
            text += f" · docked at {mission.dock.letter} {waited // 60}:{waited % 60:02d}"
        if mission.manifest_g is not None:
            text += f" · cargo {mission.manifest_g} g"
        if command.reason:
            text += f" - {command.reason}"
        if not applied and command.state is not MissionState.IDLE:
            text += f" (not applied - {why})"
        dpg.set_value(self._tags["mission_status"], text)
        dpg.configure_item(self._tags["mission_status"], color=_MISSION_COLOUR[command.state])
        dpg.set_value(self._tags["mission_route"], " > ".join(mission.route))

        self._dock_action = {MissionState.DOCKED: "release", MissionState.CHECKING: "wait"}.get(command.state)
        dock = mission.dock
        failed = dock is not None and dock.check_failed is not None and not dock.override
        label = {"wait": "Wait", "release": "Release anyway" if failed else "Release truck"}.get(self._dock_action)
        dpg.set_item_label(self._tags["dock"], label or "Release truck")
        dpg.configure_item(self._tags["dock"], enabled=self._dock_action is not None)

    def is_running(self) -> bool:
        return dpg.is_dearpygui_running()

    def render_frame(self):
        dpg.render_dearpygui_frame()

    def shutdown(self):
        dpg.destroy_context()


def _path_item_label(name: str) -> str:
    """Radio item text; the home path is marked so the GUI and L1 always agree."""
    return f"{name} (home)" if _PATH_LABELS[name] is HOME_PATH else name


def _path_item_name(label: str) -> str:
    return label.replace(" (home)", "")


def _monotonic() -> float:
    import time
    return time.monotonic()


def _distance(track) -> str:
    return f"≈{track.distance_m:.1f} m" if track.distance_m is not None else "? m"


def _short(track) -> str:
    if track.label == "traffic_light":
        return f"light {track.light_state.value} {_distance(track)}"
    return f"{track.label} {_distance(track)}"


def _detail(track) -> str:
    text = f"#{track.id} {track.label}"
    if track.rejected:
        return f"{text} {track.light_reason}"
    if not track.confirmed:
        return f"{text} unconfirmed"
    if track.label == "traffic_light":
        text += f" {track.light_state.value}"
    elif track.label == "delivery_sign" and track.variant:
        text += f" {track.variant}"
    return f"{text} {_distance(track)}"


def _frame_to_texture(frame: np.ndarray) -> np.ndarray:
    rgba = cv.cvtColor(frame, cv.COLOR_BGR2RGBA)
    return (rgba.astype(np.float32) * (1.0 / 255.0)).ravel()
