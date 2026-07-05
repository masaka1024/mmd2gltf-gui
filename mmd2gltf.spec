# -*- mode: python ; coding: utf-8 -*-
"""PyInstaller spec for mmd2gltf-gui

Usage:
    uv run pyinstaller mmd2gltf.spec
"""

import os
import sys
from PyInstaller.utils.hooks import collect_submodules

block_cipher = None

# ---------------------------------------------------------------------------
# 1. tkinterdnd2 の tkdnd 本体をプラットフォームごとに収集
# ---------------------------------------------------------------------------
def _collect_tkdnd_files():
    """tkinterdnd2 に同梱の tkdnd ファイル (.tcl / .dll) を回収する。"""
    import tkinterdnd2
    tkdnd_base = os.path.join(os.path.dirname(tkinterdnd2.__file__), "tkdnd")

    # プラットフォーム別のディレクトリ名を決定
    if sys.platform == "win32":
        # win-x64, win-x86, win-arm64 のいずれか
        import struct
        bits = struct.calcsize("P") * 8
        plat = "win-arm64" if bits == 64 and sys.maxsize > 2**32 and "arm" in sys.platform else "win-x64" if bits == 64 else "win-x86"
        subdir = plat
    elif sys.platform == "linux":
        import struct
        bits = struct.calcsize("P") * 8
        subdir = "linux-arm64-tcl9" if bits == 64 and "arm" in os.uname().machine else "linux-x64-tcl9" if bits == 64 else "linux-x64"
    elif sys.platform == "darwin":
        subdir = "osx-arm64-tcl9" if "arm" in os.uname().machine else "osx-x64-tcl9"
    else:
        subdir = None

    if subdir is None:
        return []

    src_dir = os.path.join(tkdnd_base, subdir)
    if not os.path.isdir(src_dir):
        return []

    result = []
    for fname in os.listdir(src_dir):
        fpath = os.path.join(src_dir, fname)
        if os.path.isfile(fpath) and (fname.endswith(".tcl") or fname.endswith(".dll")):
            # 出力先: tkdnd/<subdir>/...
            result.append((fpath, os.path.join("tkdnd", subdir, fname)))

    return result

tkdnd_datas = _collect_tkdnd_files()

# ---------------------------------------------------------------------------
# 2. メインファイル
# ---------------------------------------------------------------------------
a = Analysis(
    ["gui.py"],
    pathex=[],
    binaries=[],
    datas=[
        # mmd2gltf パッケージ (自作モジュール群)
        (os.path.join(".", "mmd2gltf"), "mmd2gltf"),
        *tkdnd_datas,
    ],
    hiddenimports=["mmd2gltf", "mmd2gltf.pmx", "mmd2gltf.vmd", "mmd2gltf.animation",
                   "mmd2gltf.gltf", "mmd2gltf.convert", "mmd2gltf.cli"],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    win_no_prefer_redirects=False,
    win_private_assemblies=False,
    cipher=block_cipher,
    noarchive=False,
)

# ---------------------------------------------------------------------------
# 3. PKG / EXE
# ---------------------------------------------------------------------------
pyz = PYZ(a.pure, a.zipped_data, cipher=block_cipher)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.zipfiles,
    a.datas,
    [],
    name="mmd2gltf",
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=False,  # GUIアプリなのでコンソールを隠す
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch="x64",
    codesign_identity=None,
    entitlements_file=None,
    icon="icon.ico",
)
