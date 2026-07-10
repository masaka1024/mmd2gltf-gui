# -*- coding: utf-8 -*-
"""mmd2gltf の tkinter GUI (日本語/English)。

使い方 / Usage:
    python gui.py
依存 / deps: 標準ライブラリのみ(テクスチャ処理にPillow、
ドラッグ&ドロップにtkinterdnd2を推奨/optional)。
"""
import io
import locale
import os
import queue
import re
import sys
import threading
import traceback
import contextlib
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mmd2gltf.convert import convert, HAS_PIL  # noqa: E402

# ドラッグ&ドロップは標準のtkinterには無いため tkinterdnd2 を使う(任意)。
# 無ければ従来通りの「参照...」ボタンのみで動作する(機能を静かに無効化)。
try:
    from tkinterdnd2 import TkinterDnD, DND_FILES
    HAS_DND = True
except ImportError:
    HAS_DND = False

_BaseTk = TkinterDnD.Tk if HAS_DND else tk.Tk


def _parse_dropped_paths(data):
    """tkinterdnd2の event.data (例: '{C:/a b.pmx} C:/c.vmd') をパスのリストに変換。"""
    paths = []
    for m in re.finditer(r"\{([^{}]*)\}|(\S+)", data):
        p = m.group(1) if m.group(1) is not None else m.group(2)
        if p:
            paths.append(p)
    return paths


def _detect_default_lang():
    try:
        loc = locale.getdefaultlocale()[0] or ""
    except Exception:
        loc = ""
    return "ja" if loc.lower().startswith("ja") else "en"


# ---------------------------------------------------------------------
# 翻訳テーブル / translation table
# ---------------------------------------------------------------------
STRINGS = {
    "ja": {
        "window_title": "mmd2gltf - PMX/VMD → glTF (.glb) 変換",
        "frame_files": "ファイル",
        "label_pmx": "PMXモデル *",
        "label_vmd": "VMDモーション",
        "label_out": "出力 .glb",
        "browse": "参照...",
        "drop_hint": "(ドラッグ&ドロップ可)",
        "frame_options": "オプション",
        "opt_unlit": "unlit(MMDのトゥーン見た目に近づける)",
        "opt_doubleside": "全材質を両面描画(髪・スカートの裏面が消える場合に)",
        "opt_morph_mode": "モーフ格納方式",
        "opt_alpha_mode": "アルファモード",
        "opt_scale": "スケール(MMD単位→m。既定0.08。等倍にしたい場合は1.0)",
        "adv_toggle_closed": "詳細設定 ▸",
        "adv_toggle_open": "詳細設定 ▾",
        "frame_advanced": "詳細設定",
        "adv_noik": "IKを解かない(--no-ik)",
        "adv_ignore_vmd_ik": "VMD内のIK ON/OFFキーを無視(--ignore-vmd-ik)",
        "adv_no_extras": "extras.mmd を出力しない(--no-extras)",
        "adv_no_custom_attrs": "MMD固有の頂点属性を省略(--no-custom-attrs)\n"
                                "  ※Blenderで「メッシュを読み込めない」エラーが出る場合はこれをON",
        "adv_disable_ik_label": "無効化するIK名(カンマ区切り)",
        "adv_step_label": "サンプリング間隔(--step)",
        "adv_anim_name_label": "アニメーション名",
        "adv_bake_hair": "物理演算をベイクする(--bake-physics)",
        "adv_bake_target": "対象",
        "adv_target_hair": "髪のみ",
        "adv_target_all": "全部(髪・スカート・ネクタイ等)",
        "adv_margin_label": "衝突クリアランス",
        "run_button": "変換する",
        "frame_log": "ログ",
        "err_title": "入力エラー",
        "err_select_pmx": "PMXモデルを選択してください。",
        "err_pmx_not_found": "PMXファイルが見つかりません:\n",
        "err_vmd_not_found": "VMDファイルが見つかりません:\n",
        "err_scale_invalid": "スケールには数値を入力してください。",
        "err_drop_pmx": "PMXファイル(.pmx)をドロップしてください:\n",
        "err_drop_vmd": "VMDファイル(.vmd)をドロップしてください:\n",
        "info_done_title": "完了",
        "info_done_msg": "変換が完了しました。\n",
        "error_title": "エラー",
        "error_msg": "変換に失敗しました。\n",
        "log_converting": "変換を開始: %s\n",
        "log_done": "完了: %s (%.1f MB)\n",
        "warn_pillow_missing": (
            "\u26a0 Pillow(PIL)が見つかりません。BMP/TGA/sph/spa形式の"
            "テクスチャは変換できず、該当する材質は色が抜けます。\n"
            "  対処: このプロジェクトのvenvで `pip install Pillow` "
            "(uv環境なら `uv pip install Pillow`) を実行してから、"
            "このアプリを起動し直してください。\n\n"
        ),
        "info_dnd_missing": (
            "\u2139 ドラッグ&ドロップは無効です(tkinterdnd2が未導入)。\n"
            "  有効にするには: `uv pip install tkinterdnd2`"
            "(uv環境でない場合は `pip install tkinterdnd2`) を実行してから、"
            "このアプリを起動し直してください。\n\n"
        ),
    },
    "en": {
        "window_title": "mmd2gltf - PMX/VMD to glTF (.glb) Converter",
        "frame_files": "Files",
        "label_pmx": "PMX model *",
        "label_vmd": "VMD motion",
        "label_out": "Output .glb",
        "browse": "Browse...",
        "drop_hint": "(drag & drop supported)",
        "frame_options": "Options",
        "opt_unlit": "Unlit (closer to MMD's toon look)",
        "opt_doubleside": "Force all materials double-sided (fixes disappearing hair/skirt backfaces)",
        "opt_morph_mode": "Morph storage mode",
        "opt_alpha_mode": "Alpha mode",
        "opt_scale": "Scale (MMD units -> m. Default 0.08. Use 1.0 for no scaling)",
        "adv_toggle_closed": "Advanced \u25b8",
        "adv_toggle_open": "Advanced \u25be",
        "frame_advanced": "Advanced settings",
        "adv_noik": "Don't solve IK (--no-ik)",
        "adv_ignore_vmd_ik": "Ignore IK on/off keys in VMD (--ignore-vmd-ik)",
        "adv_no_extras": "Don't output extras.mmd (--no-extras)",
        "adv_no_custom_attrs": "Omit MMD-specific vertex attributes (--no-custom-attrs)\n"
                                "  \u2731 Turn this ON if Blender fails to load the mesh",
        "adv_disable_ik_label": "IK names to disable (comma-separated)",
        "adv_step_label": "Sampling step (--step)",
        "adv_anim_name_label": "Animation clip name",
        "adv_bake_hair": "Bake rigid-body physics (--bake-physics)",
        "adv_bake_target": "Target",
        "adv_target_hair": "Hair only",
        "adv_target_all": "All (hair, skirt, tie, etc.)",
        "adv_margin_label": "Collision clearance",
        "run_button": "Convert",
        "frame_log": "Log",
        "err_title": "Input error",
        "err_select_pmx": "Please select a PMX model.",
        "err_pmx_not_found": "PMX file not found:\n",
        "err_vmd_not_found": "VMD file not found:\n",
        "err_scale_invalid": "Scale must be a number.",
        "err_drop_pmx": "Please drop a PMX file (.pmx):\n",
        "err_drop_vmd": "Please drop a VMD file (.vmd):\n",
        "info_done_title": "Done",
        "info_done_msg": "Conversion complete.\n",
        "error_title": "Error",
        "error_msg": "Conversion failed.\n",
        "log_converting": "Starting conversion: %s\n",
        "log_done": "Done: %s (%.1f MB)\n",
        "warn_pillow_missing": (
            "\u26a0 Pillow (PIL) not found. Textures in .bmp/.tga/.sph/.spa "
            "format cannot be converted, and those materials will lose "
            "their color.\n"
            "  Fix: run `pip install Pillow` (or `uv pip install Pillow` "
            "in a uv-managed venv) in this project's folder, then restart "
            "this app.\n\n"
        ),
        "info_dnd_missing": (
            "\u2139 Drag & drop is disabled (tkinterdnd2 not installed).\n"
            "  To enable it: run `uv pip install tkinterdnd2` (or `pip "
            "install tkinterdnd2`) in this project's folder, then restart "
            "this app.\n\n"
        ),
    },
}


class QueueWriter(io.TextIOBase):
    """print 出力をGUIのログへ流すためのファイル風オブジェクト。"""

    def __init__(self, q):
        self.q = q

    def write(self, s):
        if s:
            self.q.put(("log", s))
        return len(s)


class App(_BaseTk):
    def __init__(self):
        super().__init__()
        self.lang = _detect_default_lang()
        self.minsize(560, 520)
        self.q = queue.Queue()
        self.worker = None
        self._out_edited = False
        self._i18n_widgets = []   # (widget, key) simple .config(text=...) targets
        self._build()
        self._retranslate()
        if not HAS_PIL:
            self._log(self.t("warn_pillow_missing"))
        if not HAS_DND:
            self._log(self.t("info_dnd_missing"))
        self.after(100, self._poll)

    # ---------- i18n ----------
    def t(self, key):
        return STRINGS[self.lang][key]

    def _on_lang_change(self, event=None):
        self.lang = "ja" if self.lang_var.get() == "日本語" else "en"
        self._retranslate()

    def _retranslate(self):
        self.title(self.t("window_title"))
        for widget, key in self._i18n_widgets:
            widget.configure(text=self.t(key))
        drop_hint = (" " + self.t("drop_hint")) if HAS_DND else ""
        self.pmx_label.configure(text=self.t("label_pmx") + drop_hint)
        self.vmd_label.configure(text=self.t("label_vmd") + drop_hint)
        self.adv_btn.configure(text=self.t("adv_toggle_open") if self.adv_shown.get()
                                else self.t("adv_toggle_closed"))
        self.files_frame.configure(text=self.t("frame_files"))
        self.opts_frame.configure(text=self.t("frame_options"))
        self.adv.configure(text=self.t("frame_advanced"))
        self.logf.configure(text=self.t("frame_log"))

    # ---------- UI ----------
    def _build(self):
        pad = dict(padx=8, pady=4)
        frm = ttk.Frame(self)
        frm.pack(fill="both", expand=True)

        top = ttk.Frame(frm)
        top.pack(fill="x", **pad)
        ttk.Label(top, text="Language / 言語").pack(side="left")
        self.lang_var = tk.StringVar(value="日本語" if self.lang == "ja" else "English")
        lang_box = ttk.Combobox(top, textvariable=self.lang_var, state="readonly",
                                 values=["日本語", "English"], width=10)
        lang_box.pack(side="left", padx=6)
        lang_box.bind("<<ComboboxSelected>>", self._on_lang_change)

        self.files_frame = ttk.LabelFrame(frm, text="")
        self.files_frame.pack(fill="x", **pad)
        self.files_frame.columnconfigure(1, weight=1)
        files = self.files_frame

        self.pmx_var = tk.StringVar()
        self.vmd_var = tk.StringVar()
        self.out_var = tk.StringVar()

        self.pmx_label = ttk.Label(files, text="")
        self.pmx_label.grid(row=0, column=0, sticky="w", **pad)
        pmx_entry = ttk.Entry(files, textvariable=self.pmx_var)
        pmx_entry.grid(row=0, column=1, sticky="ew", **pad)
        btn_pmx = ttk.Button(files, command=self._pick_pmx)
        btn_pmx.grid(row=0, column=2, **pad)
        self._i18n_widgets.append((btn_pmx, "browse"))

        self.vmd_label = ttk.Label(files, text="")
        self.vmd_label.grid(row=1, column=0, sticky="w", **pad)
        vmd_entry = ttk.Entry(files, textvariable=self.vmd_var)
        vmd_entry.grid(row=1, column=1, sticky="ew", **pad)
        btn_vmd = ttk.Button(files, command=self._pick_vmd)
        btn_vmd.grid(row=1, column=2, **pad)
        self._i18n_widgets.append((btn_vmd, "browse"))

        if HAS_DND:
            pmx_entry.drop_target_register(DND_FILES)
            pmx_entry.dnd_bind("<<Drop>>", self._on_drop_pmx)
            vmd_entry.drop_target_register(DND_FILES)
            vmd_entry.dnd_bind("<<Drop>>", self._on_drop_vmd)

        out_label = ttk.Label(files, text="")
        out_label.grid(row=2, column=0, sticky="w", **pad)
        self._i18n_widgets.append((out_label, "label_out"))
        out_entry = ttk.Entry(files, textvariable=self.out_var)
        out_entry.grid(row=2, column=1, sticky="ew", **pad)
        out_entry.bind("<KeyRelease>", lambda e: setattr(self, "_out_edited", True))
        btn_out = ttk.Button(files, command=self._pick_out)
        btn_out.grid(row=2, column=2, **pad)
        self._i18n_widgets.append((btn_out, "browse"))

        self.opts_frame = ttk.LabelFrame(frm, text="")
        self.opts_frame.pack(fill="x", **pad)
        self.opts_frame.columnconfigure(1, weight=1)
        opts = self.opts_frame

        self.unlit_var = tk.BooleanVar(value=False)
        self.dside_var = tk.BooleanVar(value=False)
        self.morph_var = tk.StringVar(value="sparse")
        self.alpha_var = tk.StringVar(value="auto")
        self.scale_var = tk.StringVar(value="0.08")

        cb1 = ttk.Checkbutton(opts, variable=self.unlit_var)
        cb1.grid(row=0, column=0, columnspan=2, sticky="w", **pad)
        self._i18n_widgets.append((cb1, "opt_unlit"))
        cb2 = ttk.Checkbutton(opts, variable=self.dside_var)
        cb2.grid(row=1, column=0, columnspan=2, sticky="w", **pad)
        self._i18n_widgets.append((cb2, "opt_doubleside"))

        lbl_morph = ttk.Label(opts, text="")
        lbl_morph.grid(row=2, column=0, sticky="w", **pad)
        self._i18n_widgets.append((lbl_morph, "opt_morph_mode"))
        ttk.Combobox(opts, textvariable=self.morph_var, state="readonly",
                     values=["sparse", "dense", "none"], width=10).grid(row=2, column=1, sticky="w", **pad)

        lbl_alpha = ttk.Label(opts, text="")
        lbl_alpha.grid(row=3, column=0, sticky="w", **pad)
        self._i18n_widgets.append((lbl_alpha, "opt_alpha_mode"))
        ttk.Combobox(opts, textvariable=self.alpha_var, state="readonly",
                     values=["auto", "opaque", "mask", "blend"], width=10).grid(row=3, column=1, sticky="w", **pad)

        lbl_scale = ttk.Label(opts, text="")
        lbl_scale.grid(row=4, column=0, sticky="w", **pad)
        self._i18n_widgets.append((lbl_scale, "opt_scale"))
        ttk.Entry(opts, textvariable=self.scale_var, width=10).grid(row=4, column=1, sticky="w", **pad)

        # 詳細設定(折りたたみ) / advanced (collapsible)
        self.adv_shown = tk.BooleanVar(value=False)
        self.adv_btn = ttk.Checkbutton(frm, text="", variable=self.adv_shown,
                                       command=self._toggle_adv, style="Toolbutton")
        self.adv_btn.pack(anchor="w", **pad)
        self.adv = ttk.LabelFrame(frm, text="")
        self.adv.columnconfigure(1, weight=1)

        self.noik_var = tk.BooleanVar(value=False)
        self.igvmdik_var = tk.BooleanVar(value=False)
        self.noextras_var = tk.BooleanVar(value=False)
        self.nocustom_var = tk.BooleanVar(value=False)
        self.disik_var = tk.StringVar()
        self.step_var = tk.IntVar(value=1)
        self.anim_var = tk.StringVar()
        self.bakehair_var = tk.BooleanVar(value=False)
        self.baketarget_var = tk.StringVar(value="hair")
        self.margin_var = tk.StringVar(value="0.01")

        cb3 = ttk.Checkbutton(self.adv, variable=self.noik_var)
        cb3.grid(row=0, column=0, columnspan=2, sticky="w", **pad)
        self._i18n_widgets.append((cb3, "adv_noik"))
        cb4 = ttk.Checkbutton(self.adv, variable=self.igvmdik_var)
        cb4.grid(row=1, column=0, columnspan=2, sticky="w", **pad)
        self._i18n_widgets.append((cb4, "adv_ignore_vmd_ik"))
        cb5 = ttk.Checkbutton(self.adv, variable=self.noextras_var)
        cb5.grid(row=2, column=0, columnspan=2, sticky="w", **pad)
        self._i18n_widgets.append((cb5, "adv_no_extras"))
        cb6 = ttk.Checkbutton(self.adv, variable=self.nocustom_var)
        cb6.grid(row=3, column=0, columnspan=2, sticky="w", **pad)
        self._i18n_widgets.append((cb6, "adv_no_custom_attrs"))

        lbl_disik = ttk.Label(self.adv, text="")
        lbl_disik.grid(row=4, column=0, sticky="w", **pad)
        self._i18n_widgets.append((lbl_disik, "adv_disable_ik_label"))
        ttk.Entry(self.adv, textvariable=self.disik_var).grid(row=4, column=1, sticky="ew", **pad)

        lbl_step = ttk.Label(self.adv, text="")
        lbl_step.grid(row=5, column=0, sticky="w", **pad)
        self._i18n_widgets.append((lbl_step, "adv_step_label"))
        ttk.Spinbox(self.adv, from_=1, to=30, textvariable=self.step_var, width=6).grid(row=5, column=1, sticky="w", **pad)

        lbl_anim = ttk.Label(self.adv, text="")
        lbl_anim.grid(row=6, column=0, sticky="w", **pad)
        self._i18n_widgets.append((lbl_anim, "adv_anim_name_label"))
        ttk.Entry(self.adv, textvariable=self.anim_var).grid(row=6, column=1, sticky="ew", **pad)

        cb7 = ttk.Checkbutton(self.adv, variable=self.bakehair_var)
        cb7.grid(row=7, column=0, columnspan=2, sticky="w", **pad)
        self._i18n_widgets.append((cb7, "adv_bake_hair"))

        lbl_tgt = ttk.Label(self.adv, text="")
        lbl_tgt.grid(row=8, column=0, sticky="w", **pad)
        self._i18n_widgets.append((lbl_tgt, "adv_bake_target"))
        tgt_frame = ttk.Frame(self.adv)
        tgt_frame.grid(row=8, column=1, sticky="w", **pad)
        rb_hair = ttk.Radiobutton(tgt_frame, variable=self.baketarget_var,
                                  value="hair")
        rb_hair.pack(side="left")
        self._i18n_widgets.append((rb_hair, "adv_target_hair"))
        rb_all = ttk.Radiobutton(tgt_frame, variable=self.baketarget_var,
                                 value="all")
        rb_all.pack(side="left", padx=8)
        self._i18n_widgets.append((rb_all, "adv_target_all"))

        lbl_mg = ttk.Label(self.adv, text="")
        lbl_mg.grid(row=9, column=0, sticky="w", **pad)
        self._i18n_widgets.append((lbl_mg, "adv_margin_label"))
        ttk.Entry(self.adv, textvariable=self.margin_var, width=8).grid(
            row=9, column=1, sticky="w", **pad)

        run = ttk.Frame(frm)
        run.pack(fill="x", **pad)
        self.run_btn = ttk.Button(run, command=self._run)
        self.run_btn.pack(side="left")
        self._i18n_widgets.append((self.run_btn, "run_button"))
        self.prog = ttk.Progressbar(run, mode="indeterminate")
        self.prog.pack(side="left", fill="x", expand=True, padx=8)

        self.logf = ttk.LabelFrame(frm, text="")
        self.logf.pack(fill="both", expand=True, **pad)
        self.log = tk.Text(self.logf, height=8, state="disabled", wrap="word")
        self.log.pack(side="left", fill="both", expand=True)
        sb = ttk.Scrollbar(self.logf, command=self.log.yview)
        sb.pack(side="right", fill="y")
        self.log.configure(yscrollcommand=sb.set)

    def _toggle_adv(self):
        if self.adv_shown.get():
            self.adv_btn.configure(text=self.t("adv_toggle_open"))
            self.adv.pack(fill="x", padx=8, pady=4, before=self.run_btn.master)
        else:
            self.adv_btn.configure(text=self.t("adv_toggle_closed"))
            self.adv.pack_forget()

    # ---------- ファイル選択 / file pickers ----------
    def _pick_pmx(self):
        p = filedialog.askopenfilename(
            title=self.t("label_pmx"),
            filetypes=[("PMX", "*.pmx"), ("*", "*.*")])
        if p:
            self.pmx_var.set(p)
            if not self._out_edited or not self.out_var.get():
                self.out_var.set(os.path.splitext(p)[0] + ".glb")

    def _pick_vmd(self):
        p = filedialog.askopenfilename(
            title=self.t("label_vmd"),
            filetypes=[("VMD", "*.vmd"), ("*", "*.*")])
        if p:
            self.vmd_var.set(p)

    def _pick_out(self):
        p = filedialog.asksaveasfilename(
            title=self.t("label_out"), defaultextension=".glb",
            filetypes=[("glTF binary", "*.glb")])
        if p:
            self.out_var.set(p)
            self._out_edited = True

    # ---------- ドラッグ&ドロップ / drag & drop ----------
    def _on_drop_pmx(self, event):
        paths = _parse_dropped_paths(event.data)
        if not paths:
            return
        p = paths[0]
        if os.path.splitext(p)[1].lower() != ".pmx":
            messagebox.showwarning(self.t("err_title"), self.t("err_drop_pmx") + p)
            return
        self.pmx_var.set(p)
        if not self._out_edited or not self.out_var.get():
            self.out_var.set(os.path.splitext(p)[0] + ".glb")

    def _on_drop_vmd(self, event):
        paths = _parse_dropped_paths(event.data)
        if not paths:
            return
        p = paths[0]
        if os.path.splitext(p)[1].lower() != ".vmd":
            messagebox.showwarning(self.t("err_title"), self.t("err_drop_vmd") + p)
            return
        self.vmd_var.set(p)

    # ---------- 実行 / run ----------
    def _run(self):
        pmx = self.pmx_var.get().strip()
        if not pmx:
            messagebox.showwarning(self.t("err_title"), self.t("err_select_pmx"))
            return
        if not os.path.isfile(pmx):
            messagebox.showerror(self.t("err_title"), self.t("err_pmx_not_found") + pmx)
            return
        vmd = self.vmd_var.get().strip() or None
        if vmd and not os.path.isfile(vmd):
            messagebox.showerror(self.t("err_title"), self.t("err_vmd_not_found") + vmd)
            return
        out = self.out_var.get().strip() or os.path.splitext(pmx)[0] + ".glb"
        disable_ik = [s.strip() for s in self.disik_var.get().split(",") if s.strip()] or None
        try:
            step = max(1, int(self.step_var.get()))
        except Exception:
            step = 1
        try:
            scale = float(self.scale_var.get())
        except Exception:
            messagebox.showwarning(self.t("err_title"), self.t("err_scale_invalid"))
            return
        try:
            margin = float(self.margin_var.get())
        except Exception:
            margin = 0.01
        kwargs = dict(
            vmd_path=vmd,
            unlit=self.unlit_var.get(),
            solve_ik=not self.noik_var.get(),
            step=step,
            extras=not self.noextras_var.get(),
            anim_name=self.anim_var.get().strip() or None,
            disable_ik=disable_ik,
            use_vmd_ik_frames=not self.igvmdik_var.get(),
            morph_mode=self.morph_var.get(),
            alpha_mode=self.alpha_var.get(),
            force_double_sided=self.dside_var.get(),
            custom_attrs=not self.nocustom_var.get(),
            scale=scale,
            bake_physics=self.bakehair_var.get(),
            bake_target=self.baketarget_var.get(),
            collision_margin=margin,
        )
        self.run_btn.configure(state="disabled")
        self.prog.start(12)
        self._log(self.t("log_converting") % os.path.basename(pmx))
        self.worker = threading.Thread(
            target=self._convert_thread, args=(pmx, out, kwargs), daemon=True)
        self.worker.start()

    def _convert_thread(self, pmx, out, kwargs):
        w = QueueWriter(self.q)
        try:
            with contextlib.redirect_stdout(w), contextlib.redirect_stderr(w):
                convert(pmx, out, **kwargs)
            self.q.put(("done", out))
        except Exception:
            self.q.put(("error", traceback.format_exc()))

    def _poll(self):
        try:
            while True:
                kind, payload = self.q.get_nowait()
                if kind == "log":
                    self._log(payload)
                elif kind == "done":
                    self._finish()
                    size = os.path.getsize(payload) / (1024.0 * 1024.0)
                    self._log(self.t("log_done") % (payload, size))
                    messagebox.showinfo(self.t("info_done_title"), self.t("info_done_msg") + payload)
                elif kind == "error":
                    self._finish()
                    self._log(payload)
                    last = payload.strip().splitlines()[-1]
                    messagebox.showerror(self.t("error_title"), self.t("error_msg") + last)
        except queue.Empty:
            pass
        self.after(100, self._poll)

    def _finish(self):
        self.prog.stop()
        self.run_btn.configure(state="normal")

    def _log(self, s):
        self.log.configure(state="normal")
        self.log.insert("end", s)
        self.log.see("end")
        self.log.configure(state="disabled")


if __name__ == "__main__":
    App().mainloop()
