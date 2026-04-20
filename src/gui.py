import dearpygui.dearpygui as dpg
import numpy as np
import cv2 as cv

from config import Config

from perception.line_tracking import LineTrackingResult

class TunerGUI:
    def __init__(self, frame_w: int, frame_h: int):
        self._autonomous = True
        self._stopped = False
        self._tags = {}
        self._build(frame_w, frame_h)

    def _snap_odd(self, sender, value):
        dpg.set_value(sender, max(3, value | 1))

    def _toggle_mode(self, sender):
        self._autonomous = not self._autonomous
        dpg.set_item_label(sender, "Autonomous Mode" if self._autonomous else "Manual Mode")

    def _toggle_estop(self, sender):
        self._stopped = not self._stopped
        dpg.set_item_label(sender, "Resume" if self._stopped else "Emergency Stop")

    def _build(self, frame_w: int, frame_h: int):
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
            self._tags["tex_canny"] = dpg.add_raw_texture(frame_w, frame_h, blank.copy(), format=dpg.mvFormat_Float_rgba)

        grid_w = frame_w // 2
        grid_h = frame_h // 2

        form_w = grid_w - 100

        vp_w = grid_w * 3 + 80
        vp_h = frame_h
        dpg.create_viewport(title="Autonomous Car Tuner", width=vp_w, height=vp_h, resizable=False)

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
                    with dpg.group(horizontal=True, height=24):
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
                        dpg.add_text("Blur + ROI")
                        dpg.add_image(self._tags["tex_blur"], width=grid_w, height=grid_h)
                    with dpg.group():
                        dpg.add_text("Canny")
                        dpg.add_image(self._tags["tex_canny"], width=grid_w, height=grid_h)

                with dpg.table_row():
                    with dpg.group():
                        dpg.add_spacer(height=4)
                        dpg.add_text("PID Tuning", color=(130, 200, 255))
                        self._tags["kp"] = dpg.add_slider_float(label="Kp", default_value=0.01, min_value=0.0, max_value=0.1, format="%.4f", width=form_w)
                        self._tags["ki"] = dpg.add_slider_float(label="Ki", default_value=0.0, min_value=0.0, max_value=0.05, format="%.4f", width=form_w)
                        self._tags["kd"] = dpg.add_slider_float(label="Kd", default_value=0.005, min_value=0.0, max_value=0.1, format="%.4f", width=form_w)
                        dpg.add_spacer(height=4)
                        self._tags["pid_reset"] = dpg.add_button(label="Reset PID")

                    with dpg.group():
                        dpg.add_spacer(height=4)
                        dpg.add_text("ROI", color=(130, 200, 255))
                        self._tags["alpha"] = dpg.add_slider_float(label="EMA Alpha", default_value=0.4, min_value=0.01, max_value=1.0, format="%.2f", width=form_w)
                        self._tags["scan_lines"] = dpg.add_slider_int(label="Scan Lines", default_value=5, min_value=1, max_value=15, width=form_w)
                        self._tags["roi_y0"] = dpg.add_slider_float(label="ROI Y Start", default_value=0.8, min_value=0.5, max_value=0.95, format="%.2f", width=form_w)
                        self._tags["roi_y1"] = dpg.add_slider_float(label="ROI Y End", default_value=0.92, min_value=0.5, max_value=0.99, format="%.2f", width=form_w)

                    with dpg.group():
                        dpg.add_spacer(height=4)
                        dpg.add_text("Canny", color=(130, 200, 255))
                        self._tags["blur_k"] = dpg.add_slider_int(label="Blur Kernel", default_value=3, min_value=3, max_value=15, callback=self._snap_odd, width=form_w)
                        self._tags["block_size"] = dpg.add_slider_int(label="Block Size", default_value=51, min_value=3, max_value=151, callback=self._snap_odd, width=form_w)
                        self._tags["threshold_c"] = dpg.add_slider_int(label="Constant", default_value=-25, min_value=-50, max_value=50, width=form_w)

        dpg.set_primary_window("primary", True)
        dpg.setup_dearpygui()
        dpg.show_viewport()

    def read_params(self) -> Config:
        return Config(
            block_size=dpg.get_value(self._tags["block_size"]),
            threshold_constant=dpg.get_value(self._tags["threshold_c"]),
            blur_ksize=dpg.get_value(self._tags["blur_k"]),
            roi_y0=dpg.get_value(self._tags["roi_y0"]),
            roi_y1=dpg.get_value(self._tags["roi_y1"]),
            ema_alpha=dpg.get_value(self._tags["alpha"]),
            scan_lines=dpg.get_value(self._tags["scan_lines"]),

            kp=dpg.get_value(self._tags["kp"]),
            ki=dpg.get_value(self._tags["ki"]),
            kd=dpg.get_value(self._tags["kd"]),
        )

    def set_pid_reset_callback(self, callback):
        dpg.set_item_callback(self._tags["pid_reset"], callback)

    def update_textures(self, result: LineTrackingResult):
        dpg.set_value(self._tags["tex_main"], _frame_to_texture(result.display_frame))
        dpg.set_value(self._tags["tex_blur"], _frame_to_texture(result.blur_frame))
        dpg.set_value(self._tags["tex_canny"], _frame_to_texture(result.binary_frame))

    def update_telemetry(self, fps: float, error: int, steering: float):
        dpg.set_value(self._tags["tel_fps"], f"FPS: {fps:.1f}")
        dpg.set_value(self._tags["tel_error"], f"Error: {error:+d} px")
        dpg.set_value(self._tags["tel_steer"], f"Steer: {steering:+.3f}")

    def is_running(self) -> bool:
        return dpg.is_dearpygui_running()

    def is_stopped(self) -> bool:
        return self._stopped

    def is_autonomous(self) -> bool:
        return self._autonomous

    def render_frame(self):
        dpg.render_dearpygui_frame()

    def shutdown(self):
        dpg.destroy_context()


def _frame_to_texture(frame: np.ndarray) -> np.ndarray:
    rgba = cv.cvtColor(frame, cv.COLOR_BGR2RGBA)
    return (rgba.astype(np.float32) * (1.0 / 255.0)).ravel()
