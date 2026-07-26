# -*- coding: utf-8 -*-
"""mmd2gltf の tkinter GUI (日本語/English)。

使い方 / Usage:
    python gui.py
依存 / deps: 標準ライブラリのみ(テクスチャ処理にPillow、
ドラッグ&ドロップにtkinterdnd2を推奨/optional)。
"""
import io
import locale
import multiprocessing
import os
import queue
import re
import sys
import traceback
import contextlib
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mmd2gltf.convert import convert, HAS_PIL  # noqa: E402
from mmd2gltf import pmx as pmx_module  # noqa: E402

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
    # locale.getdefaultlocale() はPython 3.13でDeprecationWarning、3.15で削除
    # 予定。存在する間はこれまで通り使い(判定結果を変えないため)、無くなった
    # 環境では getlocale() → 環境変数 の順でフォールバックする。
    # 日本語Windowsでは getdefaultlocale()が'ja_JP'、getlocale()が
    # 'Japanese_Japan'を返し、どちらも "ja" 判定になるので結果は同じ。
    loc = ""
    try:
        get = getattr(locale, "getdefaultlocale", None)
        if get is not None:
            import warnings
            with warnings.catch_warnings():
                warnings.simplefilter("ignore", DeprecationWarning)
                loc = get()[0] or ""
        if not loc:
            loc = locale.getlocale()[0] or ""
        if not loc:
            loc = os.environ.get("LANG", "") or os.environ.get("LC_ALL", "")
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
        "opt_morph_hint": "\u2731 他ソフト(Unity/UniGLTF等)へ持ち込む・忠実な変換が目的なら"
                          "「dense」を選択してください(sparseは非対応のインポータで"
                          "表情が壊れることがあります)。容量を抑えたい場合のみ"
                          "「sparse」(既定)でOKです。",
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
        "adv_gravity_label": "重力の強さ",
        "adv_gravity_hint": "値を大きくするほど揺れ物が重く垂れ下がります。0で重力なし(元の形状を維持)。",
        "adv_stiffness_label": "バネの反発力(元の形へ戻る強さ)",
        "adv_stiffness_hint": "値を大きくするほど揺れが速く収まり、髪型が崩れにくくなります。",
        "adv_bounce_label": "衝突時の弾み",
        "adv_bounce_hint": "体や脚に当たった瞬間の弾みを追加します。0で現状の動き(変化なし)。"
                           "値を上げるほど跳ねるように見えます。",
        "adv_lateral_slack_label": "横リングの遊び",
        "adv_lateral_slack_hint": "スカートの横方向(隣接パネル間)のつながりに遊びを持たせ、\n"
                                   "折り紙のような硬さを和らげます。0で無効(変化なし)。モデルの\n"
                                   "ジョイントが実際に持つ移動範囲を読み取って使うので、まずは\n"
                                   "1.0前後から試し、硬さの残り具合に合わせて調整してください。",
        "adv_vertical_spread_label": "押し出しの縦方向への分配",
        "adv_vertical_spread_hint": "脚などに当たって1段だけ押し出された動きを、上下の隣の段にも\n"
                                     "分けて伝えます。0で無効(変化なし)。片足を上げるポーズなどで\n"
                                     "スカートが輪切りのように段になる場合、まず1.0前後から試して\n"
                                     "ください。",
        "adv_segment_aware_label": "衝突判定を「区間」単位で行う",
        "adv_segment_aware_hint": "従来は各段(パーティクル)を個別の点として衝突判定していました\n"
                                   "が、これを有効にすると親子の「区間」を線分として判定し、衝突\n"
                                   "位置に応じて押し出しを転写します。片足を上げるポーズでの段差\n"
                                   "(階段形状)の根本対策。既定オン(推奨)。",
        "adv_skirt_twist_label": "スカートボーンの傾き(ツイスト)を反映する",
        "adv_skirt_twist_hint": "従来はスカートボーンの向きを常に親と同じ(恒等)に固定していま\n"
                                 "したが、これを有効にすると実際の傾きをメッシュに反映します。\n"
                                 "区間の途中で横に押し出されたような段(電灯の笠のような形状)の\n"
                                 "対策。位置には影響しません。既定オン(推奨)。",
        "adv_margin_label": "衝突クリアランス",
        "adv_hem_margin_label": "裾だけ追加クリアランス",
        "adv_hem_margin_hint": "スカートの末端(裾)だけに、上のクリアランスを上乗せします。"
                                "腰側には影響しません。太もも等への見た目の食い込みが\n"
                                "気になる場合、0.01前後から試してください。",
        "adv_rb_size_margin_label": "剛体サイズ由来の追加クリアランス",
        "adv_rb_size_margin_hint": "スカートの剛体(箱)自身の厚みサイズを読み取り、その分だけ\n"
                                    "クリアランスに上乗せします。0で無効(変化なし)。モデルの\n"
                                    "剛体が実際に持っている厚みを利用するので、パラメータを\n"
                                    "手探りで太くするより自然な密着感が期待できます。まずは\n"
                                    "1.0前後から試し、見た目に合わせて上下してください。",
        "adv_collision_mode_label": "衝突モード",
        "adv_collision_mode_normal": "通常（リストの剛体だけ衝突させない）",
        "adv_collision_mode_allow": "限定（リストの剛体だけ衝突させる）",
        "adv_collision_names_label": "対象の剛体名(カンマ区切り)",
        "adv_collision_names_hint": "スカートや髪が体の一部に食い込む／開いたまま戻らない、といった場合に使います。\n"
                                     "下の「剛体名を確認」で、モデルの剛体名を見ながら入力できます。",
        "adv_show_rigidbodies_btn": "剛体名を確認...",
        "adv_vrm_compat_btn": "VRM互換モードにする",
        "adv_vrm_compat_hint": "脚まわりの剛体だけを自動で選び、「限定」モードに切り替えます"
                                "（VRMのSpringBone/VRChat PhysBonesと同じ考え方）。",
        "vrm_compat_no_match": "脚に該当しそうな剛体名が見つかりませんでした。"
                                "「剛体名を確認」で手動で選んでください。",
        "vrm_compat_applied": "%d 個の剛体を「限定」モードに設定しました。",
        "adv_tab_general": "基本",
        "adv_tab_physics": "物理ベイク",
        "adv_tab_clearance": "衝突・クリアランス",
        "adv_tab_substep": "サブステップ／中間パーティクル",
        "adv_substep_enable": "適応サブステップを有効にする",
        "adv_substep_hint": "動きが速いフレームだけ物理ステップを分割し、スカート等が"
                            "コライダーを瞬間的に素通りする「トンネリング」を緩和します。\n"
                            "目安: 閾値0.5〜0.75（実測でこのあたりが「明らかに怪しい"
                            "フレームだけ拾う」現実的な範囲でした）。",
        "adv_substep_threshold_label": "発動閾値(コライダー半径への比率)",
        "adv_substep_maxn_label": "最大分割数",
        "adv_substep_names_label": "対象コライダー名(カンマ区切り、空欄で全コライダー)",
        "adv_midpoint_enable": "中間パーティクル補正を有効にする",
        "adv_midpoint_hint": "隣接するチェーン同士を結ぶ横リング(同じ高さの輪)の"
                             "途中がコライダーに食い込んでいたら、実ボーン位置を"
                             "少し押し出します。チェーン内の縦方向(腰→裾)の"
                             "セグメントは対象にしません(縦方向を押すと1本の"
                             "ズレが裾側全体に伝わり、傘のように広がってしまう"
                             "ため)。押し出し後は縦方向のセグメント長さを"
                             "rest_lenへ戻す「念押し」も行うので、チェーンが"
                             "伸び縮みすることはありません。\n"
                             "ボーン自体の貫入とは別に、ボーンとボーンの間のメッシュ面が"
                             "脚を視覚的に横切ってしまう現象を緩和します。\n"
                             "実測: 貫入の深さ・面積を6〜7割程度緩和しますが、"
                             "「貫入が発生するフレーム数」自体はゼロにはなりません"
                             "（あくまで緩和策です）。処理コストの増加はわずかです。",
        "adv_midpoint_iters_label": "補正の反復回数",
        "adv_midpoint_margin_label": "補正のクリアランス",
        "adv_midpoint_samples_label": "セグメントごとのサンプル点数",
        "adv_midpoint_names_label": "対象コライダー名(カンマ区切り、空欄で全コライダー)",
        "rigidbody_picker_title": "剛体一覧（ダブルクリックで追加）",
        "rigidbody_picker_static": "■ 静的（脚・胴体など、コライダーになるもの）",
        "rigidbody_picker_dynamic": "■ 揺れ物（髪・スカートなど）",
        "rigidbody_picker_no_pmx": "先にPMXファイルを選択してください。",
        "rigidbody_picker_close": "閉じる",
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
        "opt_morph_hint": "\u2731 If you're importing into another tool (Unity/UniGLTF, etc.) "
                          "or need a faithful conversion, choose \"dense\" (some importers "
                          "don't support sparse accessors and facial morphs can break). "
                          "\"sparse\" (default) is fine if you just want a smaller file.",
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
        "adv_gravity_label": "Gravity strength",
        "adv_gravity_hint": "Higher values make dynamics hang heavier. 0 disables gravity "
                            "(keeps rest shape).",
        "adv_stiffness_label": "Spring stiffness (rest-shape pull)",
        "adv_stiffness_hint": "Higher values settle faster and hold the hairstyle's shape "
                              "more.",
        "adv_bounce_label": "Bounce on contact",
        "adv_bounce_hint": "Adds extra bounce the instant something hits the body or legs. "
                           "0 keeps current behavior unchanged; higher looks springier.",
        "adv_lateral_slack_label": "Lateral ring slack",
        "adv_lateral_slack_hint": "Allows slack in the skirt's lateral (ring) connections to "
                                  "soften an origami-like stiffness. 0 disables this (unchanged). "
                                  "Reads the actual joint travel from the model, so start around "
                                  "1.0 and adjust to how much stiffness remains.",
        "adv_vertical_spread_label": "Vertical push spread",
        "adv_vertical_spread_hint": "Leaks a pushed-out ring's collision displacement to the "
                                    "rings just above and below it. 0 disables this (unchanged). "
                                    "If a bent-leg pose makes the skirt look like a sliced tube, "
                                    "start around 1.0.",
        "adv_segment_aware_label": "Resolve collisions per segment (not per particle)",
        "adv_segment_aware_hint": "Previously each ring (particle) was tested against colliders "
                                   "as an independent point. Enabling this tests the parent-child "
                                   "segment as a line instead, and transfers the push-out based "
                                   "on where along the segment the collision actually falls. "
                                   "Targets the 'staircase' look (a lifted leg making the skirt "
                                   "look sliced into rings) at its root cause. On by default "
                                   "(recommended).",
        "adv_skirt_twist_label": "Keep skirt bone twist (tilt)",
        "adv_skirt_twist_hint": "Previously each skirt bone's rotation was always forced to "
                                 "identity (same orientation as its parent). Enabling this keeps "
                                 "the segment's real tilt in the mesh instead. Targets a bulge "
                                 "partway along a segment (a lampshade-like silhouette) rather "
                                 "than at the bone itself. Doesn't affect position. On by "
                                 "default (recommended).",
        "adv_margin_label": "Collision clearance",
        "adv_hem_margin_label": "Extra clearance (hem only)",
        "adv_hem_margin_hint": "Adds extra clearance on top of the setting above, but only "
                                "at the skirt's hem (tip). The waist side is unaffected. "
                                "Try around 0.01 if the legs visually poke through the hem.",
        "adv_rb_size_margin_label": "Extra clearance from rigid body size",
        "adv_rb_size_margin_hint": "Reads each skirt rigid body's own box thickness from the "
                                    "model and adds that much clearance on top. 0 disables "
                                    "this (unchanged). Since it uses the thickness the model "
                                    "already has, it tends to look more natural than hand-"
                                    "tuning a flat margin. Start around 1.0 and adjust to "
                                    "taste.",
        "adv_collision_mode_label": "Collision mode",
        "adv_collision_mode_normal": "Normal (exclude only the listed bodies)",
        "adv_collision_mode_allow": "Restricted (only the listed bodies collide)",
        "adv_collision_names_label": "Rigid body names (comma-separated)",
        "adv_collision_names_hint": "Use this when a skirt or hair pokes through part of the body, "
                                     "or stays flared open. \"Browse rigid bodies\" below lets you "
                                     "see the model's names while you type.",
        "adv_show_rigidbodies_btn": "Browse rigid bodies...",
        "adv_vrm_compat_btn": "Use VRM-compatible mode",
        "adv_vrm_compat_hint": "Auto-selects leg/hip rigid bodies and switches to "
                                "Restricted mode (same idea as VRM SpringBone / "
                                "VRChat PhysBones).",
        "vrm_compat_no_match": "Couldn't find any leg-like rigid body names. "
                                "Use \"Browse rigid bodies\" to pick manually.",
        "vrm_compat_applied": "Set %d rigid body name(s) to Restricted mode.",
        "adv_tab_general": "General",
        "adv_tab_physics": "Physics baking",
        "adv_tab_clearance": "Collision & clearance",
        "adv_tab_substep": "Substep / midpoint correction",
        "adv_substep_enable": "Enable adaptive substepping",
        "adv_substep_hint": "Splits the physics step only on fast-motion frames, "
                            "mitigating \"tunneling\" where a skirt etc. passes "
                            "instantly through a collider.\n"
                            "Rule of thumb: threshold 0.5-0.75 (measurement showed "
                            "this range catches only the clearly-risky frames).",
        "adv_substep_threshold_label": "Trigger threshold (x collider radius)",
        "adv_substep_maxn_label": "Max split count",
        "adv_substep_names_label": "Collider names (comma-separated, blank = all)",
        "adv_midpoint_enable": "Enable midpoint correction",
        "adv_midpoint_hint": "If a point along a lateral ring edge (connecting "
                             "neighboring chains at the same height) is found "
                             "inside a collider, nudges the real bone positions "
                             "apart. Vertical (waist-to-hem) segments within a "
                             "chain are never targeted directly \u2014 pushing those "
                             "lets one shift propagate down the whole chain and "
                             "flare it out like an umbrella. After each push, "
                             "vertical segment lengths are re-clamped back to "
                             "rest length, so chains never stretch or compress.\n"
                             "Separate from bone-level penetration, this mitigates "
                             "the mesh face between bones visibly crossing a leg.\n"
                             "Measured: cuts penetration depth/area by roughly "
                             "60-75%, but does not eliminate it entirely (a "
                             "mitigation, not a full fix). Cost overhead is small.",
        "adv_midpoint_iters_label": "Correction iterations",
        "adv_midpoint_margin_label": "Correction clearance",
        "adv_midpoint_samples_label": "Sample points per segment",
        "adv_midpoint_names_label": "Collider names (comma-separated, blank = all)",
        "rigidbody_picker_title": "Rigid bodies (double-click to add)",
        "rigidbody_picker_static": "\u25a0 Static (legs, torso, etc. \u2014 these act as colliders)",
        "rigidbody_picker_dynamic": "\u25a0 Dynamic (hair, skirt, etc. \u2014 these sway)",
        "rigidbody_picker_no_pmx": "Select a PMX file first.",
        "rigidbody_picker_close": "Close",
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


def _convert_worker(q, pmx, out, kwargs):
    """変換処理の本体。multiprocessing.Process の target として、GUIプロセスとは
    別プロセスで実行される(トップレベル関数である必要がある: spawn方式で子プロセス
    が再インポートして見つけられるように)。

    以前は threading.Thread で実行していたが、bake_hair.py の物理ベイクは純粋な
    Python(numpy不使用)による重いループのため、CPUを長時間占有し続けてGIL
    (Global Interpreter Lock)を手放す機会が乏しく、tkinterのメインループ
    (ウィンドウ操作・再描画)が長時間止まって見える(実際にはハングしていないが
    無応答に見える)問題があった。別プロセスに切り出すことでGILの奪い合い自体が
    無くなり、GUIは変換の重さに関係なく常に応答する。
    """
    w = QueueWriter(q)
    try:
        with contextlib.redirect_stdout(w), contextlib.redirect_stderr(w):
            convert(pmx, out, **kwargs)
        q.put(("done", out))
    except Exception:
        q.put(("error", traceback.format_exc()))


class App(_BaseTk):
    def __init__(self):
        super().__init__()
        self.lang = _detect_default_lang()
        self.minsize(560, 480)
        self.geometry("680x900")
        self.q = multiprocessing.Queue()
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

    def _make_scroll_tab(self, notebook, height=300):
        """Notebookのタブ1枚分を、縦にスクロール可能なフレームとして作る。
        戻り値は(outer, inner)で、outerをnotebook.add()に渡し、以降の
        .grid()呼び出しは全てinnerに対して行う(呼び出し側の既存コードは
        変更不要、tab_xxx変数がinnerを指すようにするだけでよい)。
        タブ内の項目が増えてウィンドウが縦に伸び続けボタンが画面外に
        出てしまう問題(実機フィードバックで発覚、縦1024ピクセル以内の
        画面で変換ボタンが押せなかった)への対策。
        """
        outer = ttk.Frame(notebook)
        canvas = tk.Canvas(outer, highlightthickness=0, height=height)
        vsb = ttk.Scrollbar(outer, orient="vertical", command=canvas.yview)
        inner = ttk.Frame(canvas)
        inner.bind("<Configure>",
                   lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        win_id = canvas.create_window((0, 0), window=inner, anchor="nw")
        canvas.configure(yscrollcommand=vsb.set)

        def _on_canvas_resize(event):
            canvas.itemconfig(win_id, width=event.width)
        canvas.bind("<Configure>", _on_canvas_resize)

        def _on_mousewheel(event):
            delta = event.delta
            if delta == 0:
                return
            canvas.yview_scroll(int(-1 * (delta / 120)), "units")
        # タブ切り替え時にだけホイールを有効化(他タブ表示中に誤って
        # スクロールしないよう、Enter/Leaveで bind/unbind する)
        canvas.bind("<Enter>", lambda e: canvas.bind_all("<MouseWheel>", _on_mousewheel))
        canvas.bind("<Leave>", lambda e: canvas.unbind_all("<MouseWheel>"))

        canvas.pack(side="left", fill="both", expand=True)
        vsb.pack(side="right", fill="y")
        return outer, inner

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
        for tab_frame, key in self._nb_tabs:
            self.nb.tab(tab_frame, text=self.t(key))

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

        lbl_morph_hint = ttk.Label(opts, text="", foreground="#666666",
                                    font=("TkDefaultFont", 8), justify="left",
                                    wraplength=460)
        lbl_morph_hint.grid(row=3, column=0, columnspan=2, sticky="w",
                             padx=8, pady=(0, 4))
        self._i18n_widgets.append((lbl_morph_hint, "opt_morph_hint"))

        lbl_alpha = ttk.Label(opts, text="")
        lbl_alpha.grid(row=4, column=0, sticky="w", **pad)
        self._i18n_widgets.append((lbl_alpha, "opt_alpha_mode"))
        ttk.Combobox(opts, textvariable=self.alpha_var, state="readonly",
                     values=["auto", "opaque", "mask", "blend"], width=10).grid(row=4, column=1, sticky="w", **pad)

        lbl_scale = ttk.Label(opts, text="")
        lbl_scale.grid(row=5, column=0, sticky="w", **pad)
        self._i18n_widgets.append((lbl_scale, "opt_scale"))
        ttk.Entry(opts, textvariable=self.scale_var, width=10).grid(row=5, column=1, sticky="w", **pad)

        # 詳細設定(折りたたみ) / advanced (collapsible)。タブ(Notebook)で
        # 「基本」「物理ベイク」「サブステップ／中間パーティクル」に分割し、
        # 1画面の縦の長さを抑える。
        self.adv_shown = tk.BooleanVar(value=False)
        self.adv_btn = ttk.Checkbutton(frm, text="", variable=self.adv_shown,
                                       command=self._toggle_adv, style="Toolbutton")
        self.adv_btn.pack(anchor="w", **pad)
        self.adv = ttk.LabelFrame(frm, text="")
        self.nb = ttk.Notebook(self.adv)
        self.nb.pack(fill="both", expand=True, padx=4, pady=4)

        tab_general_outer, tab_general = self._make_scroll_tab(self.nb)
        tab_physics_outer, tab_physics = self._make_scroll_tab(self.nb)
        tab_clearance_outer, tab_clearance = self._make_scroll_tab(self.nb)
        tab_substep_outer, tab_substep = self._make_scroll_tab(self.nb)
        tab_general.columnconfigure(1, weight=1)
        tab_physics.columnconfigure(1, weight=1)
        tab_clearance.columnconfigure(1, weight=1)
        tab_substep.columnconfigure(1, weight=1)
        self.nb.add(tab_general_outer, text="")
        self.nb.add(tab_physics_outer, text="")
        self.nb.add(tab_clearance_outer, text="")
        self.nb.add(tab_substep_outer, text="")
        self._nb_tabs = [(tab_general_outer, "adv_tab_general"),
                         (tab_physics_outer, "adv_tab_physics"),
                         (tab_clearance_outer, "adv_tab_clearance"),
                         (tab_substep_outer, "adv_tab_substep")]

        self.noik_var = tk.BooleanVar(value=False)
        self.igvmdik_var = tk.BooleanVar(value=False)
        self.noextras_var = tk.BooleanVar(value=False)
        self.nocustom_var = tk.BooleanVar(value=False)
        self.disik_var = tk.StringVar()
        self.step_var = tk.IntVar(value=1)
        self.anim_var = tk.StringVar()
        self.bakehair_var = tk.BooleanVar(value=False)
        self.baketarget_var = tk.StringVar(value="hair")
        self.gravity_var = tk.StringVar(value="0.02")
        self.stiffness_var = tk.StringVar(value="1.5")
        self.bounce_var = tk.StringVar(value="0.0")
        self.lateralslack_var = tk.StringVar(value="6.0")
        self.verticalspread_var = tk.StringVar(value="1.5")
        self.margin_var = tk.StringVar(value="0.01")
        self.hemmargin_var = tk.StringVar(value="0.0")
        self.rbsizemargin_var = tk.StringVar(value="1.0")
        self.segmentaware_var = tk.BooleanVar(value=True)
        self.skirttwist_var = tk.BooleanVar(value=True)
        self.collmode_var = tk.StringVar(value="normal")
        self.collnames_var = tk.StringVar()
        self.substep_enable_var = tk.BooleanVar(value=False)
        self.substep_threshold_var = tk.StringVar(value="0.5")
        self.substep_maxn_var = tk.IntVar(value=4)
        self.substep_names_var = tk.StringVar()
        self.midpoint_enable_var = tk.BooleanVar(value=False)
        self.midpoint_iters_var = tk.IntVar(value=2)
        self.midpoint_margin_var = tk.StringVar(value="0.0")
        self.midpoint_names_var = tk.StringVar()
        self.midpoint_samples_var = tk.IntVar(value=1)

        # ---- タブ1: 基本 ----
        g = tab_general
        cb3 = ttk.Checkbutton(g, variable=self.noik_var)
        cb3.grid(row=0, column=0, columnspan=2, sticky="w", **pad)
        self._i18n_widgets.append((cb3, "adv_noik"))
        cb4 = ttk.Checkbutton(g, variable=self.igvmdik_var)
        cb4.grid(row=1, column=0, columnspan=2, sticky="w", **pad)
        self._i18n_widgets.append((cb4, "adv_ignore_vmd_ik"))
        cb5 = ttk.Checkbutton(g, variable=self.noextras_var)
        cb5.grid(row=2, column=0, columnspan=2, sticky="w", **pad)
        self._i18n_widgets.append((cb5, "adv_no_extras"))
        cb6 = ttk.Checkbutton(g, variable=self.nocustom_var)
        cb6.grid(row=3, column=0, columnspan=2, sticky="w", **pad)
        self._i18n_widgets.append((cb6, "adv_no_custom_attrs"))

        lbl_disik = ttk.Label(g, text="")
        lbl_disik.grid(row=4, column=0, sticky="w", **pad)
        self._i18n_widgets.append((lbl_disik, "adv_disable_ik_label"))
        ttk.Entry(g, textvariable=self.disik_var).grid(row=4, column=1, sticky="ew", **pad)

        lbl_step = ttk.Label(g, text="")
        lbl_step.grid(row=5, column=0, sticky="w", **pad)
        self._i18n_widgets.append((lbl_step, "adv_step_label"))
        ttk.Spinbox(g, from_=1, to=30, textvariable=self.step_var, width=6).grid(row=5, column=1, sticky="w", **pad)

        lbl_anim = ttk.Label(g, text="")
        lbl_anim.grid(row=6, column=0, sticky="w", **pad)
        self._i18n_widgets.append((lbl_anim, "adv_anim_name_label"))
        ttk.Entry(g, textvariable=self.anim_var).grid(row=6, column=1, sticky="ew", **pad)

        # ---- タブ2: 物理ベイク ----
        p = tab_physics
        cb7 = ttk.Checkbutton(p, variable=self.bakehair_var)
        cb7.grid(row=0, column=0, columnspan=2, sticky="w", **pad)
        self._i18n_widgets.append((cb7, "adv_bake_hair"))

        lbl_tgt = ttk.Label(p, text="")
        lbl_tgt.grid(row=1, column=0, sticky="w", **pad)
        self._i18n_widgets.append((lbl_tgt, "adv_bake_target"))
        tgt_frame = ttk.Frame(p)
        tgt_frame.grid(row=1, column=1, sticky="w", **pad)
        rb_hair = ttk.Radiobutton(tgt_frame, variable=self.baketarget_var,
                                  value="hair")
        rb_hair.pack(side="left")
        self._i18n_widgets.append((rb_hair, "adv_target_hair"))
        rb_all = ttk.Radiobutton(tgt_frame, variable=self.baketarget_var,
                                 value="all")
        rb_all.pack(side="left", padx=8)
        self._i18n_widgets.append((rb_all, "adv_target_all"))

        lbl_grav = ttk.Label(p, text="")
        lbl_grav.grid(row=2, column=0, sticky="w", **pad)
        self._i18n_widgets.append((lbl_grav, "adv_gravity_label"))
        ttk.Entry(p, textvariable=self.gravity_var, width=8).grid(
            row=2, column=1, sticky="w", **pad)
        self.lbl_grav_hint = ttk.Label(p, text="", foreground="#666666",
                                       wraplength=380, justify="left")
        self.lbl_grav_hint.grid(row=3, column=1, sticky="w", **pad)
        self._i18n_widgets.append((self.lbl_grav_hint, "adv_gravity_hint"))

        lbl_stiff = ttk.Label(p, text="")
        lbl_stiff.grid(row=4, column=0, sticky="w", **pad)
        self._i18n_widgets.append((lbl_stiff, "adv_stiffness_label"))
        ttk.Entry(p, textvariable=self.stiffness_var, width=8).grid(
            row=4, column=1, sticky="w", **pad)
        self.lbl_stiff_hint = ttk.Label(p, text="", foreground="#666666",
                                        wraplength=380, justify="left")
        self.lbl_stiff_hint.grid(row=5, column=1, sticky="w", **pad)
        self._i18n_widgets.append((self.lbl_stiff_hint, "adv_stiffness_hint"))

        lbl_bounce = ttk.Label(p, text="")
        lbl_bounce.grid(row=6, column=0, sticky="w", **pad)
        self._i18n_widgets.append((lbl_bounce, "adv_bounce_label"))
        ttk.Entry(p, textvariable=self.bounce_var, width=8).grid(
            row=6, column=1, sticky="w", **pad)
        self.lbl_bounce_hint = ttk.Label(p, text="", foreground="#666666",
                                         wraplength=380, justify="left")
        self.lbl_bounce_hint.grid(row=7, column=1, sticky="w", **pad)
        self._i18n_widgets.append((self.lbl_bounce_hint, "adv_bounce_hint"))

        lbl_lslack = ttk.Label(p, text="")
        lbl_lslack.grid(row=8, column=0, sticky="w", **pad)
        self._i18n_widgets.append((lbl_lslack, "adv_lateral_slack_label"))
        ttk.Entry(p, textvariable=self.lateralslack_var, width=8).grid(
            row=8, column=1, sticky="w", **pad)
        self.lbl_lslack_hint = ttk.Label(p, text="", foreground="#666666",
                                         wraplength=380, justify="left")
        self.lbl_lslack_hint.grid(row=9, column=1, sticky="w", **pad)
        self._i18n_widgets.append((self.lbl_lslack_hint, "adv_lateral_slack_hint"))

        lbl_vspread = ttk.Label(p, text="")
        lbl_vspread.grid(row=10, column=0, sticky="w", **pad)
        self._i18n_widgets.append((lbl_vspread, "adv_vertical_spread_label"))
        ttk.Entry(p, textvariable=self.verticalspread_var, width=8).grid(
            row=10, column=1, sticky="w", **pad)
        self.lbl_vspread_hint = ttk.Label(p, text="", foreground="#666666",
                                          wraplength=380, justify="left")
        self.lbl_vspread_hint.grid(row=11, column=1, sticky="w", **pad)
        self._i18n_widgets.append((self.lbl_vspread_hint, "adv_vertical_spread_hint"))

        cb_segaware = ttk.Checkbutton(p, variable=self.segmentaware_var)
        cb_segaware.grid(row=12, column=0, columnspan=2, sticky="w", **pad)
        self._i18n_widgets.append((cb_segaware, "adv_segment_aware_label"))
        self.lbl_segaware_hint = ttk.Label(p, text="", foreground="#666666",
                                           wraplength=380, justify="left")
        self.lbl_segaware_hint.grid(row=13, column=1, sticky="w", **pad)
        self._i18n_widgets.append((self.lbl_segaware_hint, "adv_segment_aware_hint"))

        cb_skirttwist = ttk.Checkbutton(p, variable=self.skirttwist_var)
        cb_skirttwist.grid(row=14, column=0, columnspan=2, sticky="w", **pad)
        self._i18n_widgets.append((cb_skirttwist, "adv_skirt_twist_label"))
        self.lbl_skirttwist_hint = ttk.Label(p, text="", foreground="#666666",
                                             wraplength=380, justify="left")
        self.lbl_skirttwist_hint.grid(row=15, column=1, sticky="w", **pad)
        self._i18n_widgets.append((self.lbl_skirttwist_hint, "adv_skirt_twist_hint"))

        # ---- タブ3: 衝突・クリアランス ----
        c = tab_clearance
        lbl_mg = ttk.Label(c, text="")
        lbl_mg.grid(row=0, column=0, sticky="w", **pad)
        self._i18n_widgets.append((lbl_mg, "adv_margin_label"))
        ttk.Entry(c, textvariable=self.margin_var, width=8).grid(
            row=0, column=1, sticky="w", **pad)

        lbl_hmg = ttk.Label(c, text="")
        lbl_hmg.grid(row=1, column=0, sticky="w", **pad)
        self._i18n_widgets.append((lbl_hmg, "adv_hem_margin_label"))
        ttk.Entry(c, textvariable=self.hemmargin_var, width=8).grid(
            row=1, column=1, sticky="w", **pad)
        self.lbl_hmg_hint = ttk.Label(c, text="", foreground="#666666",
                                      wraplength=380, justify="left")
        self.lbl_hmg_hint.grid(row=2, column=1, sticky="w", **pad)
        self._i18n_widgets.append((self.lbl_hmg_hint, "adv_hem_margin_hint"))

        lbl_rsm = ttk.Label(c, text="")
        lbl_rsm.grid(row=3, column=0, sticky="w", **pad)
        self._i18n_widgets.append((lbl_rsm, "adv_rb_size_margin_label"))
        ttk.Entry(c, textvariable=self.rbsizemargin_var, width=8).grid(
            row=3, column=1, sticky="w", **pad)
        self.lbl_rsm_hint = ttk.Label(c, text="", foreground="#666666",
                                      wraplength=380, justify="left")
        self.lbl_rsm_hint.grid(row=4, column=1, sticky="w", **pad)
        self._i18n_widgets.append((self.lbl_rsm_hint, "adv_rb_size_margin_hint"))

        lbl_cm = ttk.Label(c, text="")
        lbl_cm.grid(row=5, column=0, sticky="w", **pad)
        self._i18n_widgets.append((lbl_cm, "adv_collision_mode_label"))
        cm_frame = ttk.Frame(c)
        cm_frame.grid(row=5, column=1, sticky="w", **pad)
        rb_normal = ttk.Radiobutton(cm_frame, variable=self.collmode_var,
                                    value="normal")
        rb_normal.pack(anchor="w")
        self._i18n_widgets.append((rb_normal, "adv_collision_mode_normal"))
        rb_allow = ttk.Radiobutton(cm_frame, variable=self.collmode_var,
                                   value="allow")
        rb_allow.pack(anchor="w")
        self._i18n_widgets.append((rb_allow, "adv_collision_mode_allow"))

        lbl_cn = ttk.Label(c, text="")
        lbl_cn.grid(row=6, column=0, sticky="w", **pad)
        self._i18n_widgets.append((lbl_cn, "adv_collision_names_label"))
        cn_frame = ttk.Frame(c)
        cn_frame.grid(row=6, column=1, sticky="ew", **pad)
        ttk.Entry(cn_frame, textvariable=self.collnames_var).pack(
            side="left", fill="x", expand=True)
        self.browse_rb_btn = ttk.Button(cn_frame, command=self._show_rigidbody_picker)
        self.browse_rb_btn.pack(side="left", padx=(6, 0))
        self._i18n_widgets.append((self.browse_rb_btn, "adv_show_rigidbodies_btn"))

        self.lbl_cn_hint = ttk.Label(c, text="", foreground="#666666",
                                     wraplength=380, justify="left")
        self.lbl_cn_hint.grid(row=7, column=1, sticky="w", **pad)
        self._i18n_widgets.append((self.lbl_cn_hint, "adv_collision_names_hint"))

        vrm_frame = ttk.Frame(c)
        vrm_frame.grid(row=8, column=1, sticky="w", **pad)
        self.vrm_compat_btn = ttk.Button(vrm_frame, command=self._apply_vrm_compat_mode)
        self.vrm_compat_btn.pack(side="left")
        self._i18n_widgets.append((self.vrm_compat_btn, "adv_vrm_compat_btn"))
        self.lbl_vrm_hint = ttk.Label(c, text="", foreground="#666666",
                                      wraplength=380, justify="left")
        self.lbl_vrm_hint.grid(row=9, column=1, sticky="w", **pad)
        self._i18n_widgets.append((self.lbl_vrm_hint, "adv_vrm_compat_hint"))

        # ---- タブ4: サブステップ／中間パーティクル ----
        s = tab_substep
        cb_sub = ttk.Checkbutton(s, variable=self.substep_enable_var)
        cb_sub.grid(row=0, column=0, columnspan=2, sticky="w", **pad)
        self._i18n_widgets.append((cb_sub, "adv_substep_enable"))
        self.lbl_substep_hint = ttk.Label(s, text="", foreground="#666666",
                                          wraplength=380, justify="left")
        self.lbl_substep_hint.grid(row=1, column=0, columnspan=2, sticky="w", **pad)
        self._i18n_widgets.append((self.lbl_substep_hint, "adv_substep_hint"))

        lbl_sth = ttk.Label(s, text="")
        lbl_sth.grid(row=2, column=0, sticky="w", **pad)
        self._i18n_widgets.append((lbl_sth, "adv_substep_threshold_label"))
        ttk.Entry(s, textvariable=self.substep_threshold_var, width=8).grid(
            row=2, column=1, sticky="w", **pad)

        lbl_smn = ttk.Label(s, text="")
        lbl_smn.grid(row=3, column=0, sticky="w", **pad)
        self._i18n_widgets.append((lbl_smn, "adv_substep_maxn_label"))
        ttk.Spinbox(s, from_=1, to=16, textvariable=self.substep_maxn_var, width=6).grid(
            row=3, column=1, sticky="w", **pad)

        lbl_sn = ttk.Label(s, text="")
        lbl_sn.grid(row=4, column=0, sticky="w", **pad)
        self._i18n_widgets.append((lbl_sn, "adv_substep_names_label"))
        ttk.Entry(s, textvariable=self.substep_names_var).grid(
            row=4, column=1, sticky="ew", **pad)

        ttk.Separator(s, orient="horizontal").grid(
            row=5, column=0, columnspan=2, sticky="ew", padx=8, pady=8)

        cb_mid = ttk.Checkbutton(s, variable=self.midpoint_enable_var)
        cb_mid.grid(row=6, column=0, columnspan=2, sticky="w", **pad)
        self._i18n_widgets.append((cb_mid, "adv_midpoint_enable"))
        self.lbl_midpoint_hint = ttk.Label(s, text="", foreground="#666666",
                                           wraplength=380, justify="left")
        self.lbl_midpoint_hint.grid(row=7, column=0, columnspan=2, sticky="w", **pad)
        self._i18n_widgets.append((self.lbl_midpoint_hint, "adv_midpoint_hint"))

        lbl_mit = ttk.Label(s, text="")
        lbl_mit.grid(row=8, column=0, sticky="w", **pad)
        self._i18n_widgets.append((lbl_mit, "adv_midpoint_iters_label"))
        ttk.Spinbox(s, from_=1, to=10, textvariable=self.midpoint_iters_var, width=6).grid(
            row=8, column=1, sticky="w", **pad)

        lbl_mmg = ttk.Label(s, text="")
        lbl_mmg.grid(row=9, column=0, sticky="w", **pad)
        self._i18n_widgets.append((lbl_mmg, "adv_midpoint_margin_label"))
        ttk.Entry(s, textvariable=self.midpoint_margin_var, width=8).grid(
            row=9, column=1, sticky="w", **pad)

        lbl_msp = ttk.Label(s, text="")
        lbl_msp.grid(row=10, column=0, sticky="w", **pad)
        self._i18n_widgets.append((lbl_msp, "adv_midpoint_samples_label"))
        ttk.Spinbox(s, from_=1, to=9, textvariable=self.midpoint_samples_var, width=6).grid(
            row=10, column=1, sticky="w", **pad)

        lbl_mn = ttk.Label(s, text="")
        lbl_mn.grid(row=11, column=0, sticky="w", **pad)
        self._i18n_widgets.append((lbl_mn, "adv_midpoint_names_label"))
        ttk.Entry(s, textvariable=self.midpoint_names_var).grid(
            row=11, column=1, sticky="ew", **pad)

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

    def _apply_vrm_compat_mode(self):
        pmx_path = self.pmx_var.get().strip()
        if not pmx_path or not os.path.isfile(pmx_path):
            messagebox.showinfo(self.t("adv_vrm_compat_btn"),
                                self.t("rigidbody_picker_no_pmx"))
            return
        try:
            self.config(cursor="watch")
            self.update_idletasks()
            model = pmx_module.parse_pmx(pmx_path)
            rigids = model.get("rigid_bodies", [])
        except Exception as e:
            messagebox.showerror(self.t("err_title"), str(e))
            return
        finally:
            self.config(cursor="")

        # VRM SpringBone/VRChat PhysBonesと同じ考え方：スカート/揺れ物は
        # 「脚まわりのコライダーだけ」を見れば十分、というヒューリスティック。
        # 足/ひざ/太もも系の名前を持つ静的剛体を拾い、下半身(胴体側)は除外する。
        leg_names = [r["name"] for r in rigids
                    if r.get("mode") == 0
                    and any(k in r["name"] for k in ("足", "ひざ", "太もも"))
                    and "下半身" not in r["name"]]
        if not leg_names:
            messagebox.showwarning(self.t("adv_vrm_compat_btn"),
                                   self.t("vrm_compat_no_match"))
            return
        self.collmode_var.set("allow")
        self.collnames_var.set(",".join(leg_names))
        messagebox.showinfo(self.t("adv_vrm_compat_btn"),
                            self.t("vrm_compat_applied") % len(leg_names))

    def _show_rigidbody_picker(self):
        pmx_path = self.pmx_var.get().strip()
        if not pmx_path or not os.path.isfile(pmx_path):
            messagebox.showinfo(self.t("adv_show_rigidbodies_btn"),
                                self.t("rigidbody_picker_no_pmx"))
            return
        try:
            self.config(cursor="watch")
            self.update_idletasks()
            model = pmx_module.parse_pmx(pmx_path)
            rigids = model.get("rigid_bodies", [])
        except Exception as e:
            messagebox.showerror(self.t("err_title"), str(e))
            return
        finally:
            self.config(cursor="")

        win = tk.Toplevel(self)
        win.title(self.t("rigidbody_picker_title"))
        win.geometry("380x440")
        win.transient(self)

        lb = tk.Listbox(win, selectmode="extended", exportselection=False)
        lb.pack(fill="both", expand=True, padx=8, pady=8)

        # リストの各行に対応する実剛体名(見出し行はNone、ダブルクリック対象外)
        entries = []
        static_names = [r["name"] for r in rigids if r.get("mode") == 0]
        dynamic_names = [r["name"] for r in rigids if r.get("mode") in (1, 2)]

        def add_section(title, names):
            lb.insert("end", title)
            entries.append(None)
            for nm in names:
                lb.insert("end", "    " + nm)
                entries.append(nm)

        add_section(self.t("rigidbody_picker_static"), static_names)
        lb.insert("end", "")
        entries.append(None)
        add_section(self.t("rigidbody_picker_dynamic"), dynamic_names)

        def on_double_click(event):
            for idx in lb.curselection():
                nm = entries[idx]
                if nm is None:
                    continue
                cur = self.collnames_var.get().strip()
                parts = [s.strip() for s in cur.split(",") if s.strip()]
                if nm not in parts:
                    parts.append(nm)
                self.collnames_var.set(",".join(parts))

        lb.bind("<Double-Button-1>", on_double_click)
        ttk.Button(win, text=self.t("rigidbody_picker_close"),
                  command=win.destroy).pack(pady=(0, 8))

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
        try:
            gravity = float(self.gravity_var.get())
        except Exception:
            gravity = 0.02
        try:
            stiffness = float(self.stiffness_var.get())
        except Exception:
            stiffness = 1.5
        try:
            bounce = float(self.bounce_var.get())
        except Exception:
            bounce = 0.0
        try:
            lateral_slack_scale = float(self.lateralslack_var.get())
        except Exception:
            lateral_slack_scale = 0.0
        try:
            vertical_spread_scale = float(self.verticalspread_var.get())
        except Exception:
            vertical_spread_scale = 0.0
        try:
            hem_margin = float(self.hemmargin_var.get())
        except Exception:
            hem_margin = 0.0
        try:
            rb_size_margin_scale = float(self.rbsizemargin_var.get())
        except Exception:
            rb_size_margin_scale = 0.0
        _coll_names = [s.strip() for s in self.collnames_var.get().split(",") if s.strip()] or None
        force_no_collision = _coll_names if self.collmode_var.get() == "normal" else None
        allowed_collider = _coll_names if self.collmode_var.get() == "allow" else None

        substep_threshold = None
        if self.substep_enable_var.get():
            try:
                substep_threshold = float(self.substep_threshold_var.get())
            except Exception:
                substep_threshold = 0.5
        try:
            substep_maxn = max(1, int(self.substep_maxn_var.get()))
        except Exception:
            substep_maxn = 4
        substep_names = ([s.strip() for s in self.substep_names_var.get().split(",")
                          if s.strip()] or None)

        midpoint_on = self.midpoint_enable_var.get()
        try:
            midpoint_iters = max(1, int(self.midpoint_iters_var.get()))
        except Exception:
            midpoint_iters = 2
        try:
            midpoint_margin = float(self.midpoint_margin_var.get())
        except Exception:
            midpoint_margin = 0.0
        midpoint_names = ([s.strip() for s in self.midpoint_names_var.get().split(",")
                           if s.strip()] or None)
        try:
            midpoint_samples = max(1, int(self.midpoint_samples_var.get()))
        except Exception:
            midpoint_samples = 1

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
            hair_gravity=gravity,
            hair_stiffness=stiffness,
            collision_bounce=bounce,
            collision_margin=margin,
            force_no_collision_names=force_no_collision,
            allowed_collider_names=allowed_collider,
            hem_extra_margin=hem_margin,
            adaptive_substep_threshold=substep_threshold,
            adaptive_substep_max_n=substep_maxn,
            adaptive_substep_collider_names=substep_names,
            midpoint_correction=midpoint_on,
            midpoint_correction_iters=midpoint_iters,
            midpoint_correction_margin=midpoint_margin,
            midpoint_correction_collider_names=midpoint_names,
            midpoint_correction_samples=midpoint_samples,
            rb_size_margin_scale=rb_size_margin_scale,
            lateral_slack_scale=lateral_slack_scale,
            vertical_spread_scale=vertical_spread_scale,
            segment_aware_collision=self.segmentaware_var.get(),
            skirt_bone_twist=self.skirttwist_var.get(),
        )
        self.run_btn.configure(state="disabled")
        self.prog.start(12)
        self._log(self.t("log_converting") % os.path.basename(pmx))
        self.worker = multiprocessing.Process(
            target=_convert_worker, args=(self.q, pmx, out, kwargs), daemon=True)
        self.worker.start()

    # 1回の_poll呼び出しで処理するキューの上限。適応サブステップ有効時など、
    # 変換処理が数千行のログを短時間に出す場面があり(例: 1回のベイクでサブ
    # ステップが数千フレーム発動)、上限を設けず全部を一度に捌こうとすると
    # そのぶんメインループが長時間ブロックされてGUIが固まって見える。
    _MAX_QUEUE_ITEMS_PER_POLL = 500

    def _poll(self):
        log_chunks = []
        n = 0
        try:
            while n < self._MAX_QUEUE_ITEMS_PER_POLL:
                kind, payload = self.q.get_nowait()
                n += 1
                if kind == "log":
                    log_chunks.append(payload)
                elif kind == "done":
                    if log_chunks:
                        self._log_batch(log_chunks)
                        log_chunks = []
                    self._finish()
                    size = os.path.getsize(payload) / (1024.0 * 1024.0)
                    self._log_batch([self.t("log_done") % (payload, size)])
                    messagebox.showinfo(self.t("info_done_title"), self.t("info_done_msg") + payload)
                elif kind == "error":
                    if log_chunks:
                        self._log_batch(log_chunks)
                        log_chunks = []
                    self._finish()
                    self._log_batch([payload])
                    last = payload.strip().splitlines()[-1]
                    messagebox.showerror(self.t("error_title"), self.t("error_msg") + last)
        except queue.Empty:
            pass
        if log_chunks:
            self._log_batch(log_chunks)
        # 上限に達してキューにまだ残りがある場合も、次の呼び出し(100ms後)で
        # 続きを処理する。この短い間隔を挟むことでメインループに制御を返し、
        # ウィンドウ操作・再描画への応答性を保つ。
        self.after(100, self._poll)

    def _finish(self):
        self.prog.stop()
        self.run_btn.configure(state="normal")
        if self.worker is not None:
            self.worker.join(timeout=1.0)
            self.worker = None

    def _log_batch(self, chunks):
        """複数行のログ文字列をまとめて1回のinsert/seeで挿入する。1行ずつ
        insert+seeを繰り返すと(特にsee呼び出しのたびに発生するスクロール・
        再レイアウトのコストが)行数に比例して重くなり、ログが大量に出る場面
        (適応サブステップのフレームごとのメッセージ等)でGUIが固まって見える
        原因になっていた。
        """
        self.log.configure(state="normal")
        self.log.insert("end", "".join(chunks))
        self.log.see("end")
        self.log.configure(state="disabled")

    def _log(self, s):
        self._log_batch([s])


if __name__ == "__main__":
    # PyInstallerで固めたWindows実行ファイルでmultiprocessing.Processを使うには
    # 必須。無いと、凍結exeの子プロセスがGUI全体を再起動してしまい、プロセスが
    # 際限なく増殖する。開発環境(python gui.py)では実質何もしないため、常に呼ぶ。
    multiprocessing.freeze_support()
    App().mainloop()