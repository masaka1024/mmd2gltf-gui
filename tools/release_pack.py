# -*- coding: utf-8 -*-
"""dist/mmd2gltf.exe を検査して配布 zip を組み立てる。

build_release.ps1 から呼ばれる。単体でも動く:

    python tools/release_pack.py --version v1.3.0 --out dist/release

過去に踏んだ失敗を全部ここで落とす。詳細は各 check_* の docstring を参照。
"""
import argparse
import os
import stat
import sys
import time
import zipfile

# zip に入れるもの。(リポジトリからの相対パス, zip 内の名前, 8進パーミッション)
PAYLOAD = [
    ("dist/mmd2gltf.exe",           "mmd2gltf.exe",            0o755),
    ("LICENSE",                     "LICENSE",                 0o644),
    ("dist_docs/README.txt",        "README.txt",              0o644),
    ("THIRD_PARTY_LICENSES.md",     "THIRD_PARTY_LICENSES.md", 0o644),
    ("dist_docs/使い方説明書.txt",   "使い方説明書.txt",         0o644),
]

EXE_NAME = "mmd2gltf.exe"        # spec の name= と一致させること
OLD_EXE_NAME = "mmd2gltf_gui.exe"   # v1.1.0〜v1.2.0 の同梱ドキュメントに残っていた誤記

_failures = []


def fail(msg):
    _failures.append(msg)
    print("  NG   " + msg)


def ok(msg):
    print("  ok   " + msg)


# ---------------------------------------------------------------------------
# 検査
# ---------------------------------------------------------------------------
def check_payload_present(root):
    """同梱物が5点そろっているか。"""
    for rel, _, _ in PAYLOAD:
        p = os.path.join(root, rel)
        if os.path.isfile(p):
            ok("%s (%d bytes)" % (rel, os.path.getsize(p)))
        else:
            fail("同梱物が無い: " + rel)


def check_exe_is_current(root):
    """exe に焼かれた mmd2gltf/*.py が作業ツリーと一致するか。

    spec は mmd2gltf ディレクトリを datas でまるごと同梱するので、
    ビルドし忘れ / 古い __pycache__ の混入がそのまま出荷事故になる。
    """
    try:
        from PyInstaller.archive.readers import CArchiveReader
    except ImportError:
        fail("PyInstaller が import できないため exe の中身を検査できない")
        return

    exe = os.path.join(root, "dist", EXE_NAME)
    if not os.path.isfile(exe):
        return                      # check_payload_present が既に報告済み

    arc = CArchiveReader(exe)
    bundled = {}
    for name in arc.toc:
        n = name.replace("\\", "/")
        if n.startswith("mmd2gltf/") and n.endswith(".py"):
            bundled[n] = name

    src_dir = os.path.join(root, "mmd2gltf")
    expected = sorted(f for f in os.listdir(src_dir) if f.endswith(".py"))
    if not expected:
        fail("mmd2gltf/*.py が見つからない")
        return

    mismatched, missing = [], []
    for fn in expected:
        key = "mmd2gltf/" + fn
        if key not in bundled:
            missing.append(fn)
            continue
        data = arc.extract(bundled[key])
        if isinstance(data, tuple):
            data = data[1]
        with open(os.path.join(src_dir, fn), "rb") as f:
            if f.read() != data:
                mismatched.append(fn)

    if missing:
        fail("exe に同梱されていない: " + ", ".join(missing))
    if mismatched:
        fail("exe の中身が作業ツリーと違う (ビルドし直すこと): "
             + ", ".join(mismatched))
    if not missing and not mismatched:
        ok("exe の mmd2gltf/*.py %d 件が作業ツリーと一致" % len(expected))

    if any("__pycache__" in n for n in arc.toc):
        fail("exe に __pycache__ が入っている (ビルド前に消すこと)")


def check_docs(root, version):
    """同梱ドキュメントの版数と実行ファイル名。

    v1.2.0 は版数が v1.1.0 のまま、実行ファイル名も誤ったまま出荷された。
    ここで落とせば同じことは起きない。
    """
    readme = os.path.join(root, "dist_docs", "README.txt")
    if not os.path.isfile(readme):
        return

    with open(readme, encoding="utf-8-sig") as f:
        text = f.read()

    stamped = [l for l in text.splitlines() if l.startswith("バージョン")]
    if not stamped:
        fail("README.txt に「バージョン :」の行が無い")
    elif version not in stamped[0]:
        fail("README.txt の版数が古い: %r (期待 %s)" % (stamped[0].strip(), version))
    else:
        ok("README.txt の版数 = " + version)

    if version not in text.split("■ 変更履歴")[-1][:400]:
        fail("README.txt の変更履歴に %s の項目が無い" % version)
    else:
        ok("README.txt に %s の変更履歴あり" % version)

    for rel in ("dist_docs/README.txt", "dist_docs/使い方説明書.txt"):
        p = os.path.join(root, rel)
        if not os.path.isfile(p):
            continue
        raw = open(p, "rb").read()
        if not raw.startswith(b"\xef\xbb\xbf"):
            fail(rel + ": UTF-8 BOM が無い (メモ帳で文字化けする)")
        if raw.count(b"\n") != raw.count(b"\r\n"):
            fail(rel + ": CRLF でない行がある")
        if OLD_EXE_NAME.encode() in raw:
            fail("%s に古い実行ファイル名 %s が残っている" % (rel, OLD_EXE_NAME))
    ok("同梱ドキュメントの改行/BOM/実行ファイル名を確認")


# ---------------------------------------------------------------------------
# zip
# ---------------------------------------------------------------------------
def build_zip(root, out_path):
    """配布 zip を作る。

    ★host OS バイトを Unix (create_system=3) にすること。
      MS-DOS のままだと、UTF-8 フラグ(0x800)を立てても Info-ZIP UnZip が
      OEM 変換をかけて「使い方説明書.txt」が壊れる。
      PowerShell の Compress-Archive はそもそも UTF-8 フラグを立てないので使わない。
    """
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    if os.path.exists(out_path):
        os.remove(out_path)

    with zipfile.ZipFile(out_path, "w", zipfile.ZIP_DEFLATED, compresslevel=9) as z:
        for rel, arcname, mode in PAYLOAD:
            src = os.path.join(root, rel)
            zi = zipfile.ZipInfo(arcname, time.localtime(os.stat(src).st_mtime)[:6])
            zi.create_system = 3                      # Unix
            zi.external_attr = (mode | stat.S_IFREG) << 16
            zi.compress_type = zipfile.ZIP_DEFLATED
            with open(src, "rb") as f:
                z.writestr(zi, f.read(), compresslevel=9)
    return out_path


def check_zip(out_path):
    """作った zip を開き直して、名前が壊れていないことまで確認する。"""
    expect = [arc for _, arc, _ in PAYLOAD]
    with zipfile.ZipFile(out_path) as z:
        infos = z.infolist()
        names = [i.filename for i in infos]
        if names != expect:
            fail("zip の中身が想定と違う: %r" % (names,))
        else:
            ok("zip の同梱物 %d 点" % len(names))

        for i in infos:
            if i.create_system != 3:
                fail("%s: host OS が Unix でない (=%d)。Info-ZIP で名前が壊れる"
                     % (i.filename, i.create_system))
            if not i.filename.isascii() and not (i.flag_bits & 0x800):
                fail("%s: 非ASCII名なのに UTF-8 フラグが立っていない" % i.filename)
        bad = z.testzip()
        if bad:
            fail("zip の CRC 不一致: " + bad)
        else:
            ok("zip の CRC 検査を通過 (host OS / UTF-8 フラグも確認)")


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--version", required=True, help="例: v1.3.0")
    ap.add_argument("--out", default="dist/release", help="zip の出力先ディレクトリ")
    ap.add_argument("--root", default=None, help="リポジトリのルート")
    a = ap.parse_args()

    root = a.root or os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    tag = a.version.upper() if a.version[:1].lower() == "v" else a.version
    tag = "V" + tag[1:] if tag[:1] in "vV" else tag
    zip_name = "mmd2gltf-gui-%s-win64.zip" % tag
    out_path = os.path.join(root, a.out, zip_name)

    print("リポジトリ : %s" % root)
    print("バージョン : %s  (タグ %s)" % (a.version, tag))
    print("\n[1/4] 同梱物")
    check_payload_present(root)
    print("\n[2/4] exe の中身")
    check_exe_is_current(root)
    print("\n[3/4] 同梱ドキュメント")
    check_docs(root, a.version)

    if _failures:
        print("\n==== %d 件の問題があるため zip を作りません ====" % len(_failures))
        for m in _failures:
            print("  - " + m)
        return 1

    print("\n[4/4] zip")
    build_zip(root, out_path)
    check_zip(out_path)

    if _failures:
        print("\n==== zip の検査に失敗 ====")
        for m in _failures:
            print("  - " + m)
        return 1

    print("\n==== OK ====")
    print("  %s (%.1f MB)" % (out_path, os.path.getsize(out_path) / 1e6))
    return 0


if __name__ == "__main__":
    sys.exit(main())
