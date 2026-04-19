import dearpygui.dearpygui as dpg
import numpy as np
import cv2 as cv

from pipeline import PipelineParams, PipelineResult


_TAGS = {}
_autonomous = True
_stopped = False


def _snap_odd(sender, value):
    dpg.set_value(sender, max(3, value | 1))


def _toggle_mode(sender):
    global _autonomous
    _autonomous = not _autonomous
    dpg.set_item_label(sender, "Autonomous Mode" if _autonomous else "Manual Mode")


def _toggle_estop(sender):
    global _stopped
    _stopped = not _stopped
    dpg.set_item_label(sender, "Resume" if _stopped else "Emergency Stop")

def create_gui(frame_w: int, frame_h: int):
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
        _TAGS["logo_tex"] = dpg.add_static_texture(logo_width, logo_height, logo_data)
        _TAGS["tex_main"] = dpg.add_raw_texture(frame_w, frame_h, blank.copy(), format=dpg.mvFormat_Float_rgba)
        _TAGS["tex_blur"] = dpg.add_raw_texture(frame_w, frame_h, blank.copy(), format=dpg.mvFormat_Float_rgba)
        _TAGS["tex_canny"] = dpg.add_raw_texture(frame_w, frame_h, blank.copy(), format=dpg.mvFormat_Float_rgba)

    grid_w = frame_w // 2
    grid_h = frame_h // 2

    form_w = grid_w - 100

    vp_w = grid_w * 3 + 80
    vp_h = frame_h
    dpg.create_viewport(title="Autonomous Car Tuner", width=vp_w, height=vp_h, resizable=False)

    with dpg.window(tag="primary", no_title_bar=True, no_move=True, no_resize=True):
        # Header
        with dpg.table(header_row=False, borders_innerH=False,
                       borders_innerV=False, borders_outerH=False,
                       borders_outerV=False, pad_outerX=False):
            dpg.add_table_column(width_stretch=True)
            dpg.add_table_column(width_fixed=True)
            with dpg.table_row():
                with dpg.group(horizontal=True):
                    dpg.add_image(_TAGS["logo_tex"], width=24, height=24)
                    dpg.add_spacer(width=4)
                    _TAGS["tel_fps"] = dpg.add_text("FPS: --", color=(0, 255, 0), wrap=100)
                    dpg.add_spacer(width=4)
                    _TAGS["tel_error"] = dpg.add_text("Error: --", color=(0, 255, 0))
                    dpg.add_spacer(width=4)
                    _TAGS["tel_steer"] = dpg.add_text("Steer: --", color=(0, 255, 0))
                with dpg.group(horizontal=True, height=24):
                    _TAGS["mode"] = dpg.add_button(label="Autonomous Mode", callback=_toggle_mode, width=120)
                    _TAGS["estop"] = dpg.add_button(label="Emergency Stop", callback=_toggle_estop, width=110)


        with dpg.table(header_row=False, borders_innerH=False,
                       borders_innerV=False, borders_outerH=False,
                       borders_outerV=False, pad_outerX=False):
                       
            dpg.add_table_column(width_fixed=True, init_width_or_weight=grid_w)
            dpg.add_table_column(width_fixed=True, init_width_or_weight=grid_w)
            dpg.add_table_column(width_fixed=True, init_width_or_weight=grid_w)

            with dpg.table_row():
                with dpg.group():
                    dpg.add_text("Camera")
                    dpg.add_image(_TAGS["tex_main"], width=grid_w, height=grid_h)
                with dpg.group():
                    dpg.add_text("Blur + ROI")
                    dpg.add_image(_TAGS["tex_blur"], width=grid_w, height=grid_h)
                with dpg.group():
                    dpg.add_text("Canny")
                    dpg.add_image(_TAGS["tex_canny"], width=grid_w, height=grid_h)

            with dpg.table_row():
                with dpg.group():
                    dpg.add_spacer(height=4)
                    dpg.add_text("PID Tuning", color=(130, 200, 255))
                    _TAGS["kp"] = dpg.add_slider_float(label="Kp", default_value=0.01, min_value=0.0, max_value=0.1, format="%.4f", width=form_w)
                    _TAGS["ki"] = dpg.add_slider_float(label="Ki", default_value=0.0, min_value=0.0, max_value=0.05, format="%.4f", width=form_w)
                    _TAGS["kd"] = dpg.add_slider_float(label="Kd", default_value=0.005, min_value=0.0, max_value=0.1, format="%.4f", width=form_w)
                    dpg.add_spacer(height=4)
                    _TAGS["pid_reset"] = dpg.add_button(label="Reset PID")

                with dpg.group():
                    dpg.add_spacer(height=4)
                    dpg.add_text("ROI", color=(130, 200, 255))
                    _TAGS["alpha"] = dpg.add_slider_float(label="EMA Alpha", default_value=0.4, min_value=0.01, max_value=1.0, format="%.2f", width=form_w)
                    _TAGS["scan_lines"] = dpg.add_slider_int(label="Scan Lines", default_value=5, min_value=1, max_value=15, width=form_w)
                    _TAGS["roi_y0"] = dpg.add_slider_float(label="ROI Y Start", default_value=0.8, min_value=0.5, max_value=0.95, format="%.2f", width=form_w)
                    _TAGS["roi_y1"] = dpg.add_slider_float(label="ROI Y End", default_value=0.92, min_value=0.5, max_value=0.99, format="%.2f", width=form_w)

                with dpg.group():
                    dpg.add_spacer(height=4)
                    dpg.add_text("Canny", color=(130, 200, 255))
                    _TAGS["blur_k"] = dpg.add_slider_int(label="Blur Kernel", default_value=3, min_value=3, max_value=15, callback=_snap_odd, width=form_w)
                    _TAGS["block_size"] = dpg.add_slider_int(label="Block Size", default_value=51, min_value=3, max_value=151, callback=_snap_odd, width=form_w)
                    _TAGS["threshold_c"] = dpg.add_slider_int(label="Constant", default_value=-25, min_value=-50, max_value=50, width=form_w)
                
    dpg.set_primary_window("primary", True)
    dpg.setup_dearpygui()
    dpg.show_viewport()

    return _TAGS


def read_params() -> PipelineParams:
    return PipelineParams(
        kp=dpg.get_value(_TAGS["kp"]),
        ki=dpg.get_value(_TAGS["ki"]),
        kd=dpg.get_value(_TAGS["kd"]),
        block_size=dpg.get_value(_TAGS["block_size"]),
        threshold_constant=dpg.get_value(_TAGS["threshold_c"]),
        blur_ksize=dpg.get_value(_TAGS["blur_k"]),
        roi_y0=dpg.get_value(_TAGS["roi_y0"]),
        roi_y1=dpg.get_value(_TAGS["roi_y1"]),
        ema_alpha=dpg.get_value(_TAGS["alpha"]),
        scan_lines=dpg.get_value(_TAGS["scan_lines"]),
        autonomous=_autonomous,
        stopped=_stopped,
    )


def set_pid_reset_callback(callback):
    dpg.set_item_callback(_TAGS["pid_reset"], callback)


def _frame_to_texture(frame: np.ndarray) -> np.ndarray:
    rgba = cv.cvtColor(frame, cv.COLOR_BGR2RGBA)
    return (rgba.astype(np.float32) * (1.0 / 255.0)).ravel()


def update_textures(result: PipelineResult):
    dpg.set_value(_TAGS["tex_main"], _frame_to_texture(result.display_frame))
    dpg.set_value(_TAGS["tex_blur"], _frame_to_texture(result.blur_frame))
    dpg.set_value(_TAGS["tex_canny"], _frame_to_texture(result.binary_frame))


def update_telemetry(result: PipelineResult):
    dpg.set_value(_TAGS["tel_fps"], f"FPS: {result.fps:.1f}")
    dpg.set_value(_TAGS["tel_error"], f"Error: {result.error:+d} px")
    dpg.set_value(_TAGS["tel_steer"], f"Steer: {result.steering:+.3f}")
