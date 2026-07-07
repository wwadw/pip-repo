from __future__ import annotations

import argparse
import importlib.metadata
import os
import subprocess
import sys
import tempfile
import threading
from typing import Any, Dict, List, Optional

import tkinter as tk
import tkinter.font as tkfont
from tkinter import filedialog, ttk

from fov_filter.client import FovFilterRosClient
from fov_filter.config_io import write_filter_regions_yaml
from fov_filter.types import DEFAULT_MAX_DISTANCE_M, SUPPORTED_LIDAR_MODELS, parse_regions_config


def topic_join(prefix: str, leaf: str) -> str:
    normalized = (prefix or "/fov_filter").strip()
    if not normalized.startswith("/"):
        normalized = "/" + normalized
    return normalized.rstrip("/") + "/" + leaf.lstrip("/")


class FovFilterControlPanel:
    HORIZONTAL_MIN_DEG = 0.0
    HORIZONTAL_MAX_DEG = 360.0
    VERTICAL_MIN_DEG = -90.0
    VERTICAL_MAX_DEG = 90.0
    DISTANCE_MIN_M = 0.0
    DISTANCE_MAX_M = 2.0
    DISTANCE_STEP_M = 0.05
    APP_BG = "#f6efe6"
    PANEL_BG = "#fff5ed"
    PANEL_BG_ALT = "#fff0e3"
    PANEL_BORDER = "#e0c1a6"
    TEXT_PRIMARY = "#2f322b"
    TEXT_MUTED = "#73665d"
    PRIMARY = "#e7663d"
    PRIMARY_HOVER = "#d95731"
    PRIMARY_PRESSED = "#bf4929"
    PRIMARY_SOFT = "#f5b08d"
    SECONDARY = "#f2ddcf"
    SECONDARY_HOVER = "#ebcfbc"
    SECONDARY_PRESSED = "#dfbca6"
    CARD_BG = "#f6eadc"
    FIELD_BG = "#fff8f1"
    SCALE_BG = "#f3a37f"
    SCALE_TROUGH = "#efc4aa"
    LIST_BG = "#fff6ef"
    LIST_SELECT = "#e7663d"
    LIST_SELECT_TEXT = "#fff7f2"

    def __init__(
        self,
        client: FovFilterRosClient,
        poll_ms: int = 250,
        topic_prefix: str = "/fov_filter",
    ) -> None:
        self.client = client
        self.poll_ms = max(100, int(poll_ms))
        self.topic_prefix = topic_prefix
        self._owned_player_process: Optional[subprocess.Popen] = None
        self.app_version = self._package_version()
        self._player_log_path = os.path.join(
            tempfile.gettempdir(),
            f"fov_filter_ui_player_{os.getpid()}.log",
        )
        self.root = tk.Tk()
        self.root.title(f"FOV Filter Panel v{self.app_version}")
        self.root.geometry("1180x760")
        self.root.minsize(820, 540)
        self.root.resizable(True, True)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._configure_fonts()
        self._write_ui_log(f"UI started, version={self.app_version}, log={self._player_log_path}")

        self._updating_ui = False
        self._frame_dragging = False
        self._rate_editing = False
        self._region_slider_dragging = False
        self._region_editor_editing = False
        self._selected_region_name: Optional[str] = None
        self._rename_entry: Optional[tk.Entry] = None
        self._renaming_region_name: Optional[str] = None
        self._pending_frame_index: Optional[int] = None
        self._pending_option_values: Dict[str, Any] = {}
        self._pending_region_payload: Optional[Dict[str, Any]] = None
        self._state: Dict[str, Any] = {}
        self._selected_bag_path: Optional[str] = None

        self.status_text = tk.StringVar(value="等待 /fov_filter/state ...")
        self.topic_summary_text = tk.StringVar(value="Topics: -")
        self.bag_path_text = tk.StringVar(value="Bag: 未选择")
        self.bag_topic_value = tk.StringVar(value="")
        self.frame_summary_text = tk.StringVar(value="Frame - / -")
        self.kept_summary_text = tk.StringVar(value="Kept -")
        self.rejected_summary_text = tk.StringVar(value="Rejected -")
        self.total_summary_text = tk.StringVar(value="Total -")

        self.frame_value = tk.IntVar(value=0)
        self.rate_value = tk.DoubleVar(value=1.0)
        self.loop_value = tk.BooleanVar(value=False)
        self.paint_rejected_value = tk.BooleanVar(value=False)
        self.publish_rejected_value = tk.BooleanVar(value=True)
        self.export_lidar_model_value = tk.StringVar(value="pointcloud")

        self.region_name_value = tk.StringVar(value="region_1")
        self.region_enabled_value = tk.BooleanVar(value=True)
        self.region_hmin_value = tk.DoubleVar(value=315.0)
        self.region_hmax_value = tk.DoubleVar(value=45.0)
        self.region_vmin_value = tk.DoubleVar(value=-10.0)
        self.region_vmax_value = tk.DoubleVar(value=10.0)
        self.region_dmin_value = tk.DoubleVar(value=0.0)
        self.region_dmax_value = tk.DoubleVar(value=self.DISTANCE_MAX_M)

        self._configure_style()
        self._build_layout()
        self.root.after(self.poll_ms, self._refresh_loop)

    def _package_version(self) -> str:
        try:
            return importlib.metadata.version("fov-filter")
        except Exception:
            return "dev"

    def _write_ui_log(self, message: str) -> None:
        try:
            with open(self._player_log_path, "a", encoding="utf-8") as log_file:
                log_file.write(message.rstrip() + "\n")
        except Exception:
            pass

    def _select_ui_font_family(self) -> str:
        return "DejaVu Sans"

    def _configure_fonts(self) -> None:
        self.font_family = self._select_ui_font_family()
        self.font_normal = (self.font_family, 10)
        self.font_small = (self.font_family, 9)
        self.font_bold = (self.font_family, 10, "bold")
        self.font_panel_title = (self.font_family, 11, "bold")
        self.font_header = (self.font_family, 18, "bold")
        self.font_status = (self.font_family, 11, "bold")
        self.font_card_value = (self.font_family, 16, "bold")

        for font_name in [
            "TkDefaultFont",
            "TkTextFont",
            "TkMenuFont",
            "TkHeadingFont",
            "TkCaptionFont",
            "TkSmallCaptionFont",
            "TkIconFont",
            "TkTooltipFont",
        ]:
            try:
                tkfont.nametofont(font_name).configure(family=self.font_family, size=10)
            except Exception:
                pass

    def _configure_style(self) -> None:
        style = ttk.Style()
        style.theme_use("clam")

        self.root.configure(bg=self.APP_BG)
        style.configure(".", font=self.font_normal)
        style.configure(
            "App.TFrame",
            background=self.APP_BG,
        )
        style.configure(
            "Panel.TFrame",
            background=self.PANEL_BG,
            relief="flat",
        )
        style.configure(
            "PanelTitle.TLabel",
            background=self.PANEL_BG,
            foreground=self.TEXT_PRIMARY,
            font=self.font_panel_title,
        )
        style.configure(
            "Muted.TLabel",
            background=self.PANEL_BG,
            foreground=self.TEXT_MUTED,
        )
        style.configure(
            "HeaderTitle.TLabel",
            background=self.APP_BG,
            foreground=self.TEXT_PRIMARY,
            font=self.font_header,
        )
        style.configure(
            "HeaderSub.TLabel",
            background=self.APP_BG,
            foreground=self.TEXT_MUTED,
            font=self.font_normal,
        )
        style.configure(
            "Status.TLabel",
            background=self.APP_BG,
            foreground="#5d493d",
            font=self.font_status,
        )
        style.configure(
            "Primary.TButton",
            padding=(12, 8),
            background=self.PRIMARY,
            foreground="#fff7f2",
            borderwidth=0,
            font=self.font_bold,
        )
        style.map(
            "Primary.TButton",
            background=[
                ("active", self.PRIMARY_HOVER),
                ("pressed", self.PRIMARY_PRESSED),
            ],
            foreground=[("disabled", "#e7c9bc")],
        )
        style.configure(
            "Secondary.TButton",
            padding=(12, 8),
            background=self.SECONDARY,
            foreground=self.TEXT_PRIMARY,
            borderwidth=0,
            font=self.font_bold,
        )
        style.map(
            "Secondary.TButton",
            background=[
                ("active", self.SECONDARY_HOVER),
                ("pressed", self.SECONDARY_PRESSED),
            ],
        )
        style.configure(
            "Card.TLabel",
            background=self.CARD_BG,
            foreground="#534238",
            padding=(16, 10),
            font=self.font_bold,
        )
        style.configure(
            "CardValue.TLabel",
            background=self.CARD_BG,
            foreground=self.TEXT_PRIMARY,
            font=self.font_card_value,
        )
        style.configure(
            "EditorLabel.TLabel",
            background=self.PANEL_BG,
            foreground=self.TEXT_PRIMARY,
            font=self.font_bold,
        )
        style.configure(
            "Field.TEntry",
            fieldbackground=self.FIELD_BG,
            bordercolor=self.PANEL_BORDER,
            font=self.font_normal,
        )
        style.configure(
            "Field.TSpinbox",
            fieldbackground=self.FIELD_BG,
            arrowsize=14,
            font=self.font_normal,
        )
        style.configure(
            "Field.TCheckbutton",
            background=self.PANEL_BG,
            foreground=self.TEXT_PRIMARY,
            font=self.font_normal,
        )
        style.map(
            "Field.TCheckbutton",
            background=[("active", self.PANEL_BG)],
        )

    def _build_layout(self) -> None:
        viewport = ttk.Frame(self.root, style="App.TFrame")
        viewport.pack(fill=tk.BOTH, expand=True)
        viewport.rowconfigure(0, weight=1)
        viewport.columnconfigure(0, weight=1)

        self.canvas = tk.Canvas(
            viewport,
            bg=self.APP_BG,
            highlightthickness=0,
            bd=0,
        )
        vertical_scrollbar = ttk.Scrollbar(
            viewport,
            orient=tk.VERTICAL,
            command=self.canvas.yview,
        )
        horizontal_scrollbar = ttk.Scrollbar(
            viewport,
            orient=tk.HORIZONTAL,
            command=self.canvas.xview,
        )
        self.canvas.configure(
            yscrollcommand=vertical_scrollbar.set,
            xscrollcommand=horizontal_scrollbar.set,
        )
        self.canvas.grid(row=0, column=0, sticky="nsew")
        vertical_scrollbar.grid(row=0, column=1, sticky="ns")
        horizontal_scrollbar.grid(row=1, column=0, sticky="ew")

        root = ttk.Frame(self.canvas, style="App.TFrame", padding=18)
        self._content_frame = root
        self._canvas_window = self.canvas.create_window((0, 0), window=root, anchor="nw")
        root.bind("<Configure>", self._on_content_resize)
        self.canvas.bind("<Configure>", self._on_canvas_resize)
        self.canvas.bind_all("<MouseWheel>", self._on_mousewheel)
        self.canvas.bind_all("<Button-4>", self._on_mousewheel)
        self.canvas.bind_all("<Button-5>", self._on_mousewheel)

        header = ttk.Frame(root, style="App.TFrame")
        header.pack(fill=tk.X)

        title_block = ttk.Frame(header, style="App.TFrame")
        title_block.pack(side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Label(
            title_block,
            text=f"FOV Filter Control Panel v{self.app_version}",
            style="HeaderTitle.TLabel",
        ).pack(anchor="w")
        ttk.Label(
            title_block,
            text="拖滑块做粗调，直接输入数值做精调。帧跳转和区域修改都在当前帧立即刷新。",
            style="HeaderSub.TLabel",
        ).pack(anchor="w", pady=(4, 0))
        ttk.Label(
            title_block,
            textvariable=self.topic_summary_text,
            style="HeaderSub.TLabel",
        ).pack(anchor="w", pady=(3, 0))
        ttk.Label(header, textvariable=self.status_text, style="Status.TLabel").pack(
            side=tk.RIGHT, anchor="n"
        )

        stats_row = ttk.Frame(root, style="App.TFrame")
        stats_row.pack(fill=tk.X, pady=(16, 12))
        self._make_stat_card(stats_row, "Frame", self.frame_summary_text).pack(
            side=tk.LEFT, fill=tk.X, expand=True
        )
        self._make_stat_card(stats_row, "Kept", self.kept_summary_text).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=(12, 0)
        )
        self._make_stat_card(stats_row, "Rejected", self.rejected_summary_text).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=(12, 0)
        )
        self._make_stat_card(stats_row, "Total", self.total_summary_text).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=(12, 0)
        )

        top_panel = ttk.Frame(root, style="Panel.TFrame", padding=16)
        top_panel.pack(fill=tk.X)

        controls_row = ttk.Frame(top_panel, style="Panel.TFrame")
        controls_row.pack(fill=tk.X)
        self.play_button = ttk.Button(
            controls_row,
            text="播放",
            command=self._toggle_play,
            style="Primary.TButton",
        )
        self.play_button.pack(side=tk.LEFT)

        for text, command in [
            ("后退一帧", lambda: self._send({"op": "prev"})),
            ("前进一帧", lambda: self._send({"op": "next"})),
            ("重发当前帧", lambda: self._send({"op": "republish"})),
            ("清空区域", lambda: self._send({"op": "clear_regions"})),
            ("刷新状态", self._request_status),
            ("载入配置", self._load_config),
            ("导出配置", self._export_config),
        ]:
            ttk.Button(
                controls_row,
                text=text,
                command=command,
                style="Secondary.TButton",
            ).pack(side=tk.LEFT, padx=(8, 0))

        bag_row = ttk.Frame(top_panel, style="Panel.TFrame")
        bag_row.pack(fill=tk.X, pady=(14, 0))
        ttk.Button(
            bag_row,
            text="选择 bag",
            command=self._choose_bag,
            style="Secondary.TButton",
        ).pack(side=tk.LEFT)
        self.bag_topic_combobox = ttk.Combobox(
            bag_row,
            textvariable=self.bag_topic_value,
            values=[],
            width=42,
            state="readonly",
        )
        self.bag_topic_combobox.pack(side=tk.LEFT, padx=(10, 0))
        ttk.Button(
            bag_row,
            text="加载/启动 bag",
            command=self._load_selected_bag,
            style="Primary.TButton",
        ).pack(side=tk.LEFT, padx=(10, 0))
        self.start_player_button = ttk.Button(
            bag_row,
            text="仅启动 fov-filter",
            command=self._start_player_from_ui,
            style="Primary.TButton",
        )
        self.start_player_button.pack(side=tk.LEFT, padx=(10, 0))
        ttk.Button(
            bag_row,
            text="停止UI启动的节点",
            command=self._stop_owned_player,
            style="Secondary.TButton",
        ).pack(side=tk.LEFT, padx=(8, 0))
        ttk.Label(
            bag_row,
            textvariable=self.bag_path_text,
            style="Muted.TLabel",
        ).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(12, 0))

        frame_row = ttk.Frame(top_panel, style="Panel.TFrame")
        frame_row.pack(fill=tk.X, pady=(14, 0))
        ttk.Label(frame_row, text="帧位置", style="PanelTitle.TLabel").pack(anchor="w")
        self.frame_scale = tk.Scale(
            frame_row,
            from_=0,
            to=1,
            orient=tk.HORIZONTAL,
            resolution=1,
            variable=self.frame_value,
            font=self.font_normal,
            bg=self.SCALE_BG,
            activebackground=self.PRIMARY_HOVER,
            troughcolor=self.SCALE_TROUGH,
            highlightthickness=0,
            bd=0,
            sliderrelief=tk.FLAT,
            fg=self.TEXT_PRIMARY,
            width=18,
            sliderlength=26,
        )
        self.frame_scale.pack(fill=tk.X, pady=(6, 0))
        self.frame_scale.bind("<ButtonPress-1>", self._begin_frame_drag)
        self.frame_scale.bind("<ButtonRelease-1>", self._end_frame_drag)

        option_row = ttk.Frame(top_panel, style="Panel.TFrame")
        option_row.pack(fill=tk.X, pady=(14, 0))

        rate_box = ttk.Frame(option_row, style="Panel.TFrame")
        rate_box.pack(side=tk.LEFT)
        ttk.Label(rate_box, text="播放倍率", style="EditorLabel.TLabel").pack(anchor="w")
        self.rate_spinbox = ttk.Spinbox(
            rate_box,
            from_=0.1,
            to=4.0,
            increment=0.1,
            textvariable=self.rate_value,
            width=8,
            style="Field.TSpinbox",
            command=self._apply_options,
        )
        self.rate_spinbox.pack(anchor="w", pady=(6, 0))
        self.rate_spinbox.bind("<FocusIn>", lambda _e: self._set_rate_editing(True))
        self.rate_spinbox.bind("<FocusOut>", self._on_rate_commit)
        self.rate_spinbox.bind("<Return>", self._on_rate_commit)

        ttk.Checkbutton(
            option_row,
            text="循环播放",
            variable=self.loop_value,
            command=self._apply_options,
            style="Field.TCheckbutton",
        ).pack(side=tk.LEFT, padx=(28, 0), pady=(20, 0))
        ttk.Checkbutton(
            option_row,
            text="过滤点标红",
            variable=self.paint_rejected_value,
            command=self._apply_options,
            style="Field.TCheckbutton",
        ).pack(side=tk.LEFT, padx=(20, 0), pady=(20, 0))
        ttk.Checkbutton(
            option_row,
            text="发布 rejected",
            variable=self.publish_rejected_value,
            command=self._apply_options,
            style="Field.TCheckbutton",
        ).pack(side=tk.LEFT, padx=(20, 0), pady=(20, 0))

        export_model_box = ttk.Frame(option_row, style="Panel.TFrame")
        export_model_box.pack(side=tk.LEFT, padx=(28, 0))
        ttk.Label(export_model_box, text="导出雷达", style="EditorLabel.TLabel").pack(anchor="w")
        self.export_lidar_model_combobox = ttk.Combobox(
            export_model_box,
            textvariable=self.export_lidar_model_value,
            values=sorted(SUPPORTED_LIDAR_MODELS),
            width=12,
            state="readonly",
        )
        self.export_lidar_model_combobox.pack(anchor="w", pady=(6, 0))

        content = ttk.Frame(root, style="App.TFrame")
        content.pack(fill=tk.BOTH, expand=True, pady=(16, 0))

        left_panel = ttk.Frame(content, style="Panel.TFrame", padding=14)
        left_panel.pack(side=tk.LEFT, fill=tk.Y)
        right_panel = ttk.Frame(content, style="Panel.TFrame", padding=16)
        right_panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(16, 0))

        ttk.Label(left_panel, text="区域列表", style="PanelTitle.TLabel").pack(anchor="w")
        ttk.Label(
            left_panel,
            text="左侧选中区域，双击名称可重命名；右侧做角度和距离调整。",
            style="Muted.TLabel",
        ).pack(anchor="w", pady=(4, 12))

        list_container = ttk.Frame(left_panel, style="Panel.TFrame")
        list_container.pack(fill=tk.BOTH, expand=True)
        self.region_listbox = tk.Listbox(
            list_container,
            width=24,
            height=22,
            font=self.font_normal,
            exportselection=False,
            bg=self.LIST_BG,
            fg=self.TEXT_PRIMARY,
            relief=tk.FLAT,
            highlightthickness=0,
            activestyle="none",
            selectbackground=self.LIST_SELECT,
            selectforeground=self.LIST_SELECT_TEXT,
        )
        self.region_listbox.pack(fill=tk.BOTH, expand=True)
        self.region_listbox.bind("<<ListboxSelect>>", self._on_region_select)
        self.region_listbox.bind("<Double-Button-1>", self._begin_region_rename)

        sidebar_buttons = ttk.Frame(left_panel, style="Panel.TFrame")
        sidebar_buttons.pack(fill=tk.X, pady=(12, 0))
        ttk.Button(
            sidebar_buttons,
            text="新增区域",
            command=self._add_region,
            style="Primary.TButton",
        ).pack(fill=tk.X)
        ttk.Button(
            sidebar_buttons,
            text="删除选中区域",
            command=self._remove_region,
            style="Secondary.TButton",
        ).pack(fill=tk.X, pady=(8, 0))

        ttk.Label(right_panel, text="区域编辑器", style="PanelTitle.TLabel").pack(anchor="w")
        ttk.Label(
            right_panel,
            text="FOV 区域表示要删除的点云范围；拖动滑条粗调，输入数值精调，失焦或回车后自动应用。",
            style="Muted.TLabel",
        ).pack(anchor="w", pady=(4, 12))

        header_row = ttk.Frame(right_panel, style="Panel.TFrame")
        header_row.pack(fill=tk.X)
        ttk.Label(header_row, text="名称", style="EditorLabel.TLabel").pack(side=tk.LEFT)
        self.region_name_entry = ttk.Entry(
            header_row,
            textvariable=self.region_name_value,
            width=20,
            style="Field.TEntry",
        )
        self.region_name_entry.pack(side=tk.LEFT, padx=(10, 18))
        self.region_name_entry.bind("<FocusIn>", lambda _e: self._set_region_editor_editing(True))
        self.region_name_entry.bind("<FocusOut>", self._on_region_entry_commit)
        self.region_name_entry.bind("<Return>", self._on_region_entry_commit)
        ttk.Checkbutton(
            header_row,
            text="启用当前区域",
            variable=self.region_enabled_value,
            command=self._apply_region_editor,
            style="Field.TCheckbutton",
        ).pack(side=tk.LEFT)
        ttk.Button(
            header_row,
            text="应用当前编辑",
            command=self._apply_region_editor,
            style="Secondary.TButton",
        ).pack(side=tk.RIGHT)

        editor_body = ttk.Frame(right_panel, style="Panel.TFrame")
        editor_body.pack(fill=tk.BOTH, expand=True, pady=(12, 0))
        for column in range(3):
            editor_body.columnconfigure(column, weight=1, uniform="region_editor")

        self.hmin_scale = self._make_editor_card(
            editor_body,
            row=0,
            column=0,
            label="水平最小角",
            variable=self.region_hmin_value,
            min_value=self.HORIZONTAL_MIN_DEG,
            max_value=self.HORIZONTAL_MAX_DEG,
            step=1.0,
            integer_display=True,
            unit="deg",
        )
        self.hmax_scale = self._make_editor_card(
            editor_body,
            row=1,
            column=0,
            label="水平最大角",
            variable=self.region_hmax_value,
            min_value=self.HORIZONTAL_MIN_DEG,
            max_value=self.HORIZONTAL_MAX_DEG,
            step=1.0,
            integer_display=True,
            unit="deg",
        )
        self.vmin_scale = self._make_editor_card(
            editor_body,
            row=0,
            column=1,
            label="垂直最小角",
            variable=self.region_vmin_value,
            min_value=self.VERTICAL_MIN_DEG,
            max_value=self.VERTICAL_MAX_DEG,
            step=1.0,
            integer_display=True,
            unit="deg",
        )
        self.vmax_scale = self._make_editor_card(
            editor_body,
            row=1,
            column=1,
            label="垂直最大角",
            variable=self.region_vmax_value,
            min_value=self.VERTICAL_MIN_DEG,
            max_value=self.VERTICAL_MAX_DEG,
            step=1.0,
            integer_display=True,
            unit="deg",
        )
        self.dmin_scale = self._make_editor_card(
            editor_body,
            row=0,
            column=2,
            label="最小距离",
            variable=self.region_dmin_value,
            min_value=self.DISTANCE_MIN_M,
            max_value=self.DISTANCE_MAX_M,
            step=self.DISTANCE_STEP_M,
            integer_display=False,
            unit="m",
        )
        self.dmax_scale = self._make_editor_card(
            editor_body,
            row=1,
            column=2,
            label="最大距离",
            variable=self.region_dmax_value,
            min_value=self.DISTANCE_MIN_M,
            max_value=self.DISTANCE_MAX_M,
            step=self.DISTANCE_STEP_M,
            integer_display=False,
            unit="m",
        )

        helper = ttk.Label(
            right_panel,
            text=(
                "建议流程: 先拖动做大范围定位，再直接输入数字做精调。\n"
                "距离编辑范围固定为 0~2m，导出时会写成 filter_regions YAML。"
            ),
            style="Muted.TLabel",
            justify=tk.LEFT,
        )
        helper.pack(anchor="w", pady=(14, 0))

    def _make_stat_card(self, parent, title: str, value_var: tk.StringVar) -> ttk.Frame:
        card = ttk.Frame(parent, style="Panel.TFrame", padding=0)
        inner = ttk.Frame(card, style="Panel.TFrame")
        inner.pack(fill=tk.BOTH, expand=True)
        label = ttk.Label(inner, text=title, style="Card.TLabel")
        label.pack(fill=tk.X)
        value = ttk.Label(inner, textvariable=value_var, style="CardValue.TLabel")
        value.pack(fill=tk.X, padx=16, pady=(0, 10))
        return card

    def _make_editor_card(
        self,
        parent,
        row: int,
        column: int,
        label: str,
        variable: tk.DoubleVar,
        min_value: float,
        max_value: float,
        step: float,
        integer_display: bool,
        unit: str,
    ) -> tk.Scale:
        card = ttk.Frame(parent, style="Panel.TFrame", padding=(10, 8))
        card.grid(
            row=row,
            column=column,
            sticky="nsew",
            padx=(0 if column == 0 else 10, 0),
            pady=(0 if row == 0 else 10, 0),
        )
        card.columnconfigure(1, weight=1)

        ttk.Label(card, text=label, style="EditorLabel.TLabel").grid(
            row=0, column=0, sticky="w"
        )
        spinbox = ttk.Spinbox(
            card,
            from_=min_value,
            to=max_value,
            increment=step,
            textvariable=variable,
            width=10,
            style="Field.TSpinbox",
            format="%0.0f" if integer_display else "%.2f",
            command=self._apply_region_editor,
        )
        spinbox.grid(row=0, column=1, sticky="e", padx=(12, 0))
        spinbox.bind("<FocusIn>", lambda _e: self._set_region_editor_editing(True))
        spinbox.bind("<FocusOut>", self._on_region_entry_commit)
        spinbox.bind("<Return>", self._on_region_entry_commit)

        unit_label = ttk.Label(card, text=unit, style="Muted.TLabel")
        unit_label.grid(row=0, column=2, sticky="w", padx=(8, 0))

        scale = tk.Scale(
            card,
            from_=min_value,
            to=max_value,
            resolution=step,
            orient=tk.HORIZONTAL,
            variable=variable,
            font=self.font_normal,
            showvalue=False,
            bg=self.SCALE_BG,
            activebackground=self.PRIMARY_HOVER,
            troughcolor=self.SCALE_TROUGH,
            highlightthickness=0,
            bd=0,
            sliderrelief=tk.FLAT,
            fg=self.TEXT_PRIMARY,
            width=16,
            sliderlength=24,
        )
        scale.grid(row=1, column=0, columnspan=3, sticky="ew", pady=(8, 0))
        scale.bind("<ButtonPress-1>", self._begin_region_slider_drag)
        scale.bind("<ButtonRelease-1>", self._end_region_slider_drag)
        return scale

    def _on_content_resize(self, _event=None) -> None:
        self.canvas.configure(scrollregion=self.canvas.bbox("all"))

    def _on_canvas_resize(self, event) -> None:
        requested_width = self._content_frame.winfo_reqwidth()
        self.canvas.itemconfigure(self._canvas_window, width=max(event.width, requested_width))
        self.canvas.configure(scrollregion=self.canvas.bbox("all"))

    def _on_mousewheel(self, event) -> None:
        if event.num == 4:
            self.canvas.yview_scroll(-1, "units")
        elif event.num == 5:
            self.canvas.yview_scroll(1, "units")
        else:
            self.canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

    def _set_status(self, text: str) -> None:
        self.root.after(0, lambda: self.status_text.set(text))

    def _set_rate_editing(self, value: bool) -> None:
        self._rate_editing = value

    def _set_region_editor_editing(self, value: bool) -> None:
        self._region_editor_editing = value

    def _normalize_horizontal(self, value: float) -> float:
        normalized = float(value) % 360.0
        if abs(normalized - 360.0) < 1e-9:
            return 0.0
        return normalized

    def _send(
        self,
        payload: Dict[str, Any],
        wait: bool = True,
        timeout: float = 2.5,
        on_success=None,
        on_error=None,
    ) -> None:
        def worker() -> None:
            try:
                self._write_ui_log(f"send command: {payload}")
                state = self.client.send_command(payload, timeout=timeout, wait=wait)
                if state:
                    self._state = state
                    self._set_status(self._build_status_text(state))
                if on_success is not None:
                    self.root.after(0, lambda: on_success(state))
            except Exception as exc:
                error = exc
                message = str(exc)
                self._write_ui_log(f"command failed: {payload}, error={message}")
                if on_error is not None:
                    self.root.after(0, lambda: on_error(error))
                self._set_status(f"命令失败: {message}")

        threading.Thread(target=worker, daemon=True).start()

    def _has_command_subscriber(self) -> bool:
        try:
            return self.client.publisher.get_num_connections() > 0
        except Exception:
            return False

    def _short_path(self, path: str, max_chars: int = 72) -> str:
        if len(path) <= max_chars:
            return path
        return "..." + path[-max_chars:]

    def _scan_pointcloud_topics(self, bag_path: str) -> List[str]:
        import rosbag

        try:
            bag = rosbag.Bag(bag_path, "r", allow_unindexed=True)
        except TypeError:
            bag = rosbag.Bag(bag_path, "r")

        with bag:
            _types, topic_infos = bag.get_type_and_topic_info()
            return sorted(
                name
                for name, info in topic_infos.items()
                if getattr(info, "msg_type", "") == "sensor_msgs/PointCloud2"
            )

    def _apply_scanned_bag_topics(self, bag_path: str, topics: List[str]) -> None:
        self._selected_bag_path = bag_path
        self.bag_path_text.set(f"Bag: {self._short_path(bag_path)}")
        self.bag_topic_combobox.configure(values=topics)
        current_topic = str(self._state.get("topic", ""))
        if current_topic in topics:
            self.bag_topic_value.set(current_topic)
        elif topics:
            self.bag_topic_value.set(topics[0])
        else:
            self.bag_topic_value.set("")
        if topics:
            self._write_ui_log(f"scanned bag: {bag_path}, pointcloud_topics={topics}")
            self._set_status(f"已扫描到 {len(topics)} 个 PointCloud2 话题，请选择后加载")
        else:
            self._write_ui_log(f"scanned bag: {bag_path}, no PointCloud2 topics")
            self._set_status("这个 bag 中没有扫描到 PointCloud2 话题")

    def _choose_bag(self) -> None:
        path = filedialog.askopenfilename(
            parent=self.root,
            title="选择 ROS bag 文件",
            filetypes=[
                ("ROS bag", "*.bag"),
                ("All files", "*.*"),
            ],
        )
        if not path:
            return

        self._write_ui_log(f"choose bag: {path}")
        self._selected_bag_path = path
        self.bag_path_text.set(f"Bag: {self._short_path(path)}")
        self.bag_topic_combobox.configure(values=[])
        self.bag_topic_value.set("")
        self._set_status("正在扫描 bag 中的 PointCloud2 话题...")

        def worker() -> None:
            try:
                topics = self._scan_pointcloud_topics(path)
                self.root.after(0, lambda: self._apply_scanned_bag_topics(path, topics))
            except Exception as exc:
                message = str(exc)
                self._write_ui_log(f"bag scan failed: {path}, error={message}")
                self.root.after(0, lambda: self._set_status(f"bag 扫描失败: {message}"))

        threading.Thread(target=worker, daemon=True).start()

    def _load_selected_bag(self) -> None:
        bag_path = self._selected_bag_path
        topic = self.bag_topic_value.get().strip()
        if not bag_path:
            self._set_status("请先选择 bag 文件")
            return
        if not topic:
            self._set_status("请先选择 PointCloud2 话题")
            return

        self._write_ui_log(f"load selected bag requested: bag={bag_path}, topic={topic}")
        if not self._has_command_subscriber():
            self._write_ui_log("no fov-filter command subscriber; starting player from UI")
            self._set_status("未检测到运行中的 fov-filter，正在从 UI 启动...")
            self._start_player_from_ui()
            return

        self._set_status("正在加载 bag，请稍等...")
        self._send(
            {"op": "load_bag", "bag": bag_path, "topic": topic},
            timeout=120.0,
            on_success=lambda _state: self._set_status("bag 已加载并发布第 0 帧"),
        )

    def _owned_player_running(self) -> bool:
        return self._owned_player_process is not None and self._owned_player_process.poll() is None

    def _player_python(self) -> str:
        conda_prefix = os.environ.get("CONDA_PREFIX")
        if conda_prefix:
            candidate = os.path.join(conda_prefix, "bin", "python")
            if os.path.exists(candidate):
                return candidate
        return sys.executable

    def _clean_pythonpath(self, package_root: str) -> str:
        raw_paths = os.environ.get("PYTHONPATH", "").split(os.pathsep)
        paths = [package_root]
        for path in raw_paths:
            if not path:
                continue
            normalized = os.path.abspath(path)
            if normalized.startswith(os.path.expanduser("~/.local/lib/")):
                continue
            if normalized not in paths:
                paths.append(normalized)
        return os.pathsep.join(paths)

    def _player_command(self, bag_path: str, topic: str) -> List[str]:
        command = [
            self._player_python(),
            "-u",
            "-m",
            "fov_filter.cli",
            "--bag",
            bag_path,
            "--topic",
            topic,
            "--topic-prefix",
            self.topic_prefix,
            "--command-topic",
            self.client.command_topic,
            "--state-topic",
            self.client.state_topic,
            "--node-name",
            "fov_filter_player_ui",
            "--start-paused",
            "--rate",
            f"{float(self.rate_value.get()):.3f}",
        ]
        if bool(self.loop_value.get()):
            command.append("--loop")
        if bool(self.paint_rejected_value.get()):
            command.append("--paint-rejected")
        if not bool(self.publish_rejected_value.get()):
            command.append("--no-publish-rejected")
        return command

    def _start_player_from_ui(self) -> None:
        bag_path = self._selected_bag_path
        topic = self.bag_topic_value.get().strip()
        if not bag_path:
            self._set_status("请先选择 bag 文件")
            return
        if not topic:
            self._set_status("请先选择 PointCloud2 话题")
            return
        if self._owned_player_running():
            self._set_status("UI 启动的 fov-filter 已在运行")
            self._write_ui_log("start player ignored: owned player already running")
            return

        package_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        env = os.environ.copy()
        env["PYTHONPATH"] = self._clean_pythonpath(package_root)
        env["PYTHONNOUSERSITE"] = "1"
        env["PYTHONUNBUFFERED"] = "1"
        command = self._player_command(bag_path, topic)

        try:
            with open(self._player_log_path, "a", encoding="utf-8", buffering=1) as log_file:
                log_file.write("\n===== start fov-filter from UI =====\n")
                log_file.write("command: " + " ".join(command) + "\n")
                log_file.flush()
                self._owned_player_process = subprocess.Popen(
                    command,
                    cwd=package_root,
                    env=env,
                    stdout=log_file,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
        except Exception as exc:
            self._owned_player_process = None
            self._set_status(f"启动 fov-filter 失败: {exc}")
            return

        self._set_status(f"已从 UI 启动 fov-filter，正在读取首帧... 日志: {self._player_log_path}")
        self.root.after(1200, self._check_owned_player_startup)

    def _check_owned_player_startup(self) -> None:
        process = self._owned_player_process
        if process is not None and process.poll() is not None:
            self._set_status(
                f"fov-filter 已退出，exit={process.returncode}，请看日志: {self._player_log_path}"
            )
            return
        self._request_status(timeout=120.0)

    def _stop_owned_player(self) -> None:
        process = self._owned_player_process
        if process is None:
            self._set_status("没有由 UI 启动的 fov-filter 节点")
            return
        if process.poll() is not None:
            self._owned_player_process = None
            self._set_status("UI 启动的 fov-filter 已经退出")
            return

        def worker() -> None:
            try:
                process.terminate()
                try:
                    process.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=2.0)
                self.root.after(0, lambda: self._set_status("已停止 UI 启动的 fov-filter"))
            except Exception as exc:
                message = str(exc)
                self.root.after(0, lambda: self._set_status(f"停止 fov-filter 失败: {message}"))
            finally:
                self._owned_player_process = None

        threading.Thread(target=worker, daemon=True).start()

    def _on_close(self) -> None:
        process = self._owned_player_process
        if process is not None and process.poll() is None:
            try:
                process.terminate()
            except Exception:
                pass
        self.root.destroy()

    def _request_status(self, timeout: float = 2.5) -> None:
        def worker() -> None:
            try:
                state = self.client.request_status(timeout=timeout)
                self._state = state
                self._set_status(self._build_status_text(state))
            except Exception as exc:
                self._set_status(f"状态获取失败: {exc}")

        threading.Thread(target=worker, daemon=True).start()

    def _build_status_text(self, state: Dict[str, Any]) -> str:
        stats = state.get("last_stats", {})
        loading_suffix = " | loading frames..." if state.get("loading_frames") else ""
        return (
            f"frame {state.get('current_index', 0)}/{max(0, state.get('total_frames', 1) - 1)}"
            f" | kept {stats.get('kept_points', '-')}"
            f" | rejected {stats.get('rejected_points', '-')}"
            f" | rate {state.get('rate', 1.0):.1f}"
            f" | {'playing' if state.get('playing') else 'paused'}"
            f"{loading_suffix}"
        )

    def _update_summary_cards(self, state: Dict[str, Any]) -> None:
        stats = state.get("last_stats", {})
        topics = state.get("topics", {})
        current_bag = str(state.get("bag", "") or "")
        if current_bag and self._selected_bag_path is None:
            self._selected_bag_path = current_bag
            self.bag_path_text.set(f"Bag: {self._short_path(current_bag)}")
            current_topic = str(state.get("topic", "") or "")
            if current_topic and not self.bag_topic_value.get():
                self.bag_topic_value.set(current_topic)
        self.topic_summary_text.set(
            "Topics: "
            f"source={topics.get('source', state.get('topic', '-'))} | "
            f"kept={topics.get('filtered', '-')} | "
            f"removed={topics.get('rejected', '-')} | "
            f"fov={topics.get('fov_regions', '-')}"
        )
        self.frame_summary_text.set(
            f"{state.get('current_index', 0)} / {max(0, state.get('total_frames', 1) - 1)}"
            + ("+" if state.get("loading_frames") else "")
        )
        self.kept_summary_text.set(str(stats.get("kept_points", "-")))
        self.rejected_summary_text.set(str(stats.get("rejected_points", "-")))
        self.total_summary_text.set(str(stats.get("total_points", "-")))

    def _refresh_loop(self) -> None:
        state = self.client.latest_state()
        if state:
            self._state = state
            self._sync_ui_from_state(state)
        self.root.after(self.poll_ms, self._refresh_loop)

    def _sync_pending_markers(self, state: Dict[str, Any]) -> None:
        if self._pending_frame_index is not None:
            if int(state.get("current_index", -1)) == self._pending_frame_index:
                self._pending_frame_index = None

        for key, pending_value in list(self._pending_option_values.items()):
            current_value = state.get(key)
            matched = False
            if isinstance(pending_value, float):
                try:
                    matched = abs(float(current_value) - pending_value) < 1e-6
                except Exception:
                    matched = False
            else:
                matched = current_value == pending_value
            if matched:
                self._pending_option_values.pop(key, None)

        if self._pending_region_payload is not None:
            region_state = self._find_region_in_state(
                state.get("regions", []),
                self._pending_region_payload.get("name"),
            )
            if region_state is not None and self._region_matches_state(
                region_state,
                self._pending_region_payload,
            ):
                self._pending_region_payload = None

    def _sync_ui_from_state(self, state: Dict[str, Any]) -> None:
        self._updating_ui = True
        try:
            self._sync_pending_markers(state)
            self.status_text.set(self._build_status_text(state))
            self._update_summary_cards(state)

            if "loop" not in self._pending_option_values:
                self.loop_value.set(bool(state.get("loop", False)))
            if "paint_rejected" not in self._pending_option_values:
                self.paint_rejected_value.set(bool(state.get("paint_rejected", False)))
            if "publish_rejected" not in self._pending_option_values:
                self.publish_rejected_value.set(bool(state.get("publish_rejected", True)))
            if not self._rate_editing and "rate" not in self._pending_option_values:
                self.rate_value.set(float(state.get("rate", 1.0)))

            total_frames = max(1, int(state.get("total_frames", 1)))
            self.frame_scale.configure(to=total_frames - 1)
            if not self._frame_dragging and self._pending_frame_index is None:
                self.frame_value.set(int(state.get("current_index", 0)))

            self.play_button.configure(text="暂停" if state.get("playing") else "播放")
            self._sync_region_list(state.get("regions", []))
        finally:
            self._updating_ui = False

    def _sync_region_list(self, regions: List[Dict[str, Any]]) -> None:
        region_names = [region.get("name", "") for region in regions]
        current_items = list(self.region_listbox.get(0, tk.END))
        if current_items != region_names:
            self.region_listbox.delete(0, tk.END)
            for name in region_names:
                self.region_listbox.insert(tk.END, name)

        if self._selected_region_name not in region_names:
            self._selected_region_name = region_names[0] if region_names else None

        if self._selected_region_name in region_names:
            index = region_names.index(self._selected_region_name)
            self.region_listbox.selection_clear(0, tk.END)
            self.region_listbox.selection_set(index)
            self.region_listbox.activate(index)
            if (
                not self._region_slider_dragging
                and not self._region_editor_editing
                and self._pending_region_payload is None
            ):
                self._load_region_editor(regions[index])

    def _load_region_editor(self, region: Dict[str, Any]) -> None:
        horizontal = region.get("horizontal", [315.0, 45.0])
        vertical = region.get("vertical", [-10.0, 10.0])
        distance = region.get("distance", [0.0, DEFAULT_MAX_DISTANCE_M])

        self.region_name_value.set(region.get("name", ""))
        self.region_enabled_value.set(bool(region.get("enabled", True)))
        self.region_hmin_value.set(self._normalize_horizontal(horizontal[0]))
        self.region_hmax_value.set(self._normalize_horizontal(horizontal[1]))
        self.region_vmin_value.set(float(vertical[0]))
        self.region_vmax_value.set(float(vertical[1]))
        self.region_dmin_value.set(float(distance[0]))
        self.region_dmax_value.set(float(distance[1]))

    def _find_region_in_state(
        self,
        regions: List[Dict[str, Any]],
        region_name: Optional[str],
    ) -> Optional[Dict[str, Any]]:
        if not region_name:
            return None
        for region in regions:
            if region.get("name") == region_name:
                return region
        return None

    def _region_matches_state(self, region: Dict[str, Any], payload: Dict[str, Any]) -> bool:
        def approx_list(left: List[float], right: List[float]) -> bool:
            if len(left) != len(right):
                return False
            return all(abs(float(a) - float(b)) < 1e-6 for a, b in zip(left, right))

        normalized_region = {
            "name": region.get("name"),
            "enabled": bool(region.get("enabled", True)),
            "horizontal": [
                self._normalize_horizontal(value)
                for value in region.get("horizontal", [])
            ],
            "vertical": [float(value) for value in region.get("vertical", [])],
            "distance": [float(value) for value in region.get("distance", [])],
        }
        return (
            normalized_region["name"] == payload.get("name")
            and normalized_region["enabled"] == bool(payload.get("enabled", True))
            and approx_list(normalized_region["horizontal"], payload.get("horizontal", []))
            and approx_list(normalized_region["vertical"], payload.get("vertical", []))
            and approx_list(normalized_region["distance"], payload.get("distance", []))
        )

    def _on_region_select(self, _event=None) -> None:
        if self._rename_entry is not None:
            return
        selection = self.region_listbox.curselection()
        if not selection:
            return
        name = self.region_listbox.get(selection[0])
        self._selected_region_name = name
        region = self._find_region_in_state(self._state.get("regions", []), name)
        if region is not None:
            self._load_region_editor(region)

    def _existing_region_names(self) -> List[str]:
        names = [str(region.get("name", "")) for region in self._state.get("regions", [])]
        if not names:
            names = [str(name) for name in self.region_listbox.get(0, tk.END)]
        return [name for name in names if name]

    def _next_region_name(self) -> str:
        existing = set(self._existing_region_names())
        index = 1
        while f"region_{index}" in existing:
            index += 1
        return f"region_{index}"

    def _destroy_rename_entry(self) -> None:
        if self._rename_entry is not None:
            self._rename_entry.destroy()
        self._rename_entry = None
        self._renaming_region_name = None

    def _begin_region_rename(self, event=None) -> None:
        if event is None:
            return
        index = self.region_listbox.nearest(event.y)
        bbox = self.region_listbox.bbox(index)
        if bbox is None:
            return

        old_name = self.region_listbox.get(index)
        self.region_listbox.selection_clear(0, tk.END)
        self.region_listbox.selection_set(index)
        self.region_listbox.activate(index)
        self._selected_region_name = old_name

        self._destroy_rename_entry()
        x, y, width, height = bbox
        entry = tk.Entry(
            self.region_listbox,
            font=self.font_normal,
            bg=self.FIELD_BG,
            fg=self.TEXT_PRIMARY,
            relief=tk.FLAT,
            highlightthickness=1,
            highlightbackground=self.PANEL_BORDER,
            highlightcolor=self.PRIMARY,
            insertbackground=self.TEXT_PRIMARY,
        )
        entry.insert(0, old_name)
        entry.select_range(0, tk.END)
        entry.place(x=x, y=y, width=max(width, self.region_listbox.winfo_width() - 8), height=height)
        entry.focus_set()
        entry.bind("<Return>", lambda _e: self._finish_region_rename())
        entry.bind("<FocusOut>", lambda _e: self._finish_region_rename())
        entry.bind("<Escape>", lambda _e: self._cancel_region_rename())
        self._rename_entry = entry
        self._renaming_region_name = old_name

    def _finish_region_rename(self) -> None:
        if self._rename_entry is None or not self._renaming_region_name:
            return

        old_name = self._renaming_region_name
        new_name = self._rename_entry.get().strip()
        self._destroy_rename_entry()

        if not new_name or new_name == old_name:
            return

        existing = set(self._existing_region_names())
        existing.discard(old_name)
        if new_name in existing:
            self._set_status(f"重命名失败: 区域名称已存在 {new_name}")
            return

        self._selected_region_name = new_name
        self.region_name_value.set(new_name)
        self._send(
            {"op": "update_region", "name": old_name, "region": {"name": new_name}},
            on_error=lambda _exc: setattr(self, "_selected_region_name", old_name),
        )

    def _cancel_region_rename(self) -> None:
        self._destroy_rename_entry()

    def _current_region_payload(self) -> Dict[str, Any]:
        return {
            "name": self.region_name_value.get().strip() or "region",
            "horizontal": [
                self._normalize_horizontal(self.region_hmin_value.get()),
                self._normalize_horizontal(self.region_hmax_value.get()),
            ],
            "vertical": [
                float(self.region_vmin_value.get()),
                float(self.region_vmax_value.get()),
            ],
            "distance": [
                float(self.region_dmin_value.get()),
                float(self.region_dmax_value.get()),
            ],
            "enabled": bool(self.region_enabled_value.get()),
        }

    def _apply_region_editor(self, _event=None) -> None:
        if self._updating_ui or not self._selected_region_name:
            return
        payload = {
            "op": "update_region",
            "name": self._selected_region_name,
            "region": self._current_region_payload(),
        }
        self._selected_region_name = payload["region"]["name"]
        self._pending_region_payload = dict(payload["region"])
        self._send(
            payload,
            on_error=lambda _exc: setattr(self, "_pending_region_payload", None),
        )

    def _add_region(self) -> None:
        payload = self._current_region_payload()
        payload["name"] = self._next_region_name()
        self._selected_region_name = payload["name"]
        self.region_name_value.set(payload["name"])
        self._pending_region_payload = dict(payload)
        self._send(
            {"op": "add_region", "region": payload},
            on_error=lambda _exc: setattr(self, "_pending_region_payload", None),
        )

    def _remove_region(self) -> None:
        if not self._selected_region_name:
            return
        self._destroy_rename_entry()
        region_name = self._selected_region_name
        self._selected_region_name = None
        self._send({"op": "remove_region", "name": region_name})

    def _toggle_play(self) -> None:
        state = self._state or {}
        if bool(state.get("playing", False)):
            self._send({"op": "pause"}, wait=False)
        else:
            self._send({"op": "play"}, wait=False)

    def _apply_options(self, _event=None) -> None:
        if self._updating_ui:
            return
        pending_values = {
            "rate": round(float(self.rate_value.get()), 2),
            "loop": bool(self.loop_value.get()),
            "paint_rejected": bool(self.paint_rejected_value.get()),
            "publish_rejected": bool(self.publish_rejected_value.get()),
        }
        self._pending_option_values.update(pending_values)
        self._send(
            {"op": "set_option", **pending_values},
            on_error=lambda _exc: self._pending_option_values.clear(),
        )

    def _begin_frame_drag(self, _event=None) -> None:
        self._frame_dragging = True

    def _end_frame_drag(self, _event=None) -> None:
        self._frame_dragging = False
        self._pending_frame_index = int(self.frame_value.get())
        self._send(
            {"op": "seek", "index": self._pending_frame_index},
            on_error=lambda _exc: setattr(self, "_pending_frame_index", None),
        )

    def _set_region_editor_false(self) -> None:
        self._region_editor_editing = False

    def _on_region_entry_commit(self, _event=None) -> None:
        self._set_region_editor_false()
        self._apply_region_editor()

    def _begin_region_slider_drag(self, _event=None) -> None:
        self._region_slider_dragging = True
        self._region_editor_editing = True

    def _end_region_slider_drag(self, _event=None) -> None:
        self._region_slider_dragging = False
        self._region_editor_editing = False
        self._apply_region_editor()

    def _on_rate_commit(self, _event=None) -> None:
        self._set_rate_editing(False)
        self._apply_options()

    def _load_config(self) -> None:
        path = filedialog.askopenfilename(
            parent=self.root,
            title="选择 FOV 配置文件",
            filetypes=[
                ("Config", "*.toml *.yaml *.yml"),
                ("TOML", "*.toml"),
                ("YAML", "*.yaml *.yml"),
                ("All files", "*.*"),
            ],
        )
        if not path:
            return
        self._send({"op": "load_config", "config_path": path})

    def _export_config(self) -> None:
        if not self._state:
            self._set_status("当前没有可导出的状态")
            return

        path = filedialog.asksaveasfilename(
            parent=self.root,
            title="导出 filter_regions 配置",
            defaultextension=".yaml",
            filetypes=[("YAML", "*.yaml *.yml"), ("All files", "*.*")],
        )
        if not path:
            return

        try:
            regions = parse_regions_config(self._state.get("regions"))
            lidar_model = self.export_lidar_model_value.get() or "pointcloud"
            exported_count = write_filter_regions_yaml(
                path,
                regions,
                enabled_only=True,
                lidar_model=lidar_model,
            )
            self._set_status(f"已按 {lidar_model} 导出 {exported_count} 个启用区域到 {path}")
        except Exception as exc:
            self._set_status(f"导出失败: {exc}")

    def run(self) -> None:
        self._request_status()
        self.root.mainloop()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="fov-filter 桌面滑块控制面板")
    parser.add_argument("--topic-prefix", default="/fov_filter", help="控制/状态话题前缀")
    parser.add_argument("--command-topic", help="命令话题")
    parser.add_argument("--state-topic", help="状态话题")
    parser.add_argument("--poll-ms", type=int, default=250, help="UI 刷新周期")
    return parser


def main(argv: Optional[List[str]] = None) -> None:
    parser = build_parser()
    args = parser.parse_args(argv if argv is not None else None)

    try:
        client = FovFilterRosClient(
            command_topic=args.command_topic or topic_join(args.topic_prefix, "command"),
            state_topic=args.state_topic or topic_join(args.topic_prefix, "state"),
            node_name="fov_filter_ui",
            anonymous=True,
            init_node=True,
        )
    except Exception as exc:
        raise SystemExit(str(exc)) from exc

    panel = FovFilterControlPanel(
        client=client,
        poll_ms=args.poll_ms,
        topic_prefix=args.topic_prefix,
    )
    panel.run()


if __name__ == "__main__":
    main()
