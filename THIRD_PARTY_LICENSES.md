# サードパーティ ライセンス表記 (Third-Party Licenses)

本ソフトウェア **mmd2gltf** の Windows 実行ファイル版（`mmd2gltf_gui.exe`、PyInstaller でビルド）には、以下のオープンソースソフトウェアが含まれています。各コンポーネントはそれぞれのライセンスのもとで配布されています。

> ソースコード版（本体 `mmd2gltf`）自体は MIT ライセンスです。詳細はリポジトリ同梱の [LICENSE](LICENSE) を参照してください。
> 本ファイルは、実行ファイルに **バンドルされて再配布される** コンポーネントを対象としています。

## 同梱コンポーネント一覧

| コンポーネント | バージョン | ライセンス | 役割 |
| --- | --- | --- | --- |
| Python (CPython) | 3.11 | PSF License Agreement | 実行環境（インタプリタ） |
| PyInstaller（ブートローダ） | — | GPL-2.0 + bootloader exception | exe化のブートローダ |
| NumPy | 2.4.6 | BSD 3-Clause | 数値計算 |
| Pillow (PIL fork) | — | MIT-CMU（旧称 HPND / PIL Software License） | テクスチャ変換 |
| tkinterdnd2 | — | MIT | ドラッグ&ドロップ（Pythonラッパ） |
| tkDnD | 2.10.1 | BSD系（下記参照） | ドラッグ&ドロップ（Tcl/Tk拡張, `libtkdnd2.10.1.dll`） |
| Tcl / Tk | 8.6 | Tcl/Tk License（BSD系） | GUIツールキット |

全文が短いライセンス（BSD 3-Clause / MIT / MIT-CMU）は本ファイル末尾に全文を掲載しています。全文が長いもの・特別条項を持つもの（Python PSF / Tcl/Tk / PyInstaller）は、ライセンス名・著作権表記・公式URLを記載しています。各公式URLで完全な条文を確認できます。

---

## 各コンポーネントの詳細

### Python (CPython) 3.11
- ライセンス: **PSF License Agreement**（Python Software Foundation License）
- 著作権: Copyright © 2001–2025 Python Software Foundation. All Rights Reserved.
- 全文: https://docs.python.org/3/license.html
- 実行ファイルに Python インタプリタとして同梱されています。

### PyInstaller（ブートローダ）
- ライセンス: **GPL 2.0** に、生成された実行ファイルへのリンクを許可する **特別なブートローダ例外（bootloader exception）** が付随します（PyInstaller ライブラリ本体は Apache-2.0）。
- この例外により、PyInstaller でビルドされた実行ファイル（本ソフト）は **任意のライセンスで配布できます**。実行ファイルに埋め込まれるのはブートローダ部分のみです。
- 全文: https://github.com/pyinstaller/pyinstaller/blob/develop/COPYING.txt

### NumPy 2.4.6
- ライセンス: **BSD 3-Clause License**
- 著作権: Copyright (c) 2005–2024, NumPy Developers. All rights reserved.
- 全文: 本ファイル末尾「BSD 3-Clause License」を参照。
- 公式: https://github.com/numpy/numpy/blob/main/LICENSE.txt

### Pillow (PIL fork)
- ライセンス: **MIT-CMU License**（Pillow プロジェクトでは歴史的に HPND / PIL Software License と表記）
- 著作権:
  - The Python Imaging Library (PIL): Copyright © 1997–2011 by Secret Labs AB / Copyright © 1995–2011 by Fredrik Lundh and contributors
  - Pillow: Copyright © 2010 by Jeffrey A. Clark and contributors
- 全文: 本ファイル末尾「Pillow (PIL Software License / MIT-CMU)」を参照。
- 公式: https://github.com/python-pillow/Pillow/blob/main/LICENSE

### tkinterdnd2
- ライセンス: **MIT License**
- 概要: George Petasis 氏による tkDnD（Tcl/Tk 拡張）の Python ラッパ。作者: petasis / pmgagne / Eliav2 / Squiblydoo ほか。
- 全文: 本ファイル末尾「MIT License」を参照。
- 公式: https://github.com/Eliav2/tkinterdnd2

### tkDnD 2.10.1
- ライセンス: **BSD系ライセンス**（Tcl/Tk ライセンスに準じた寛容なオープンソースライセンス）
- 著作権: Copyright © George Petasis, and other parties.
- 実行ファイルには `libtkdnd2.10.1.dll` および付随する Tcl スクリプトとして同梱されています。
- 公式（正確な条文はこちら）: https://github.com/petasis/tkdnd

### Tcl / Tk 8.6
- ライセンス: **Tcl/Tk License**（BSD系の寛容なライセンス。使用・複製・改変・再配布を許諾）
- 著作権: This software is copyrighted by the Regents of the University of California, Sun Microsystems, Inc., Scriptics Corporation, and other parties.
- 全文: https://www.tcl-lang.org/software/tcltk/license.html
- 実行ファイルに `tcl86t.dll` / `tk86t.dll` および関連スクリプトとして同梱されています。

---

## ライセンス全文 (License Texts)

### BSD 3-Clause License
（適用: NumPy）

```
Copyright (c) 2005-2024, NumPy Developers.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its contributors
  may be used to endorse or promote products derived from this software
  without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```

### MIT License
（適用: tkinterdnd2）

```
MIT License

Copyright (c) tkinterdnd2 authors (petasis, pmgagne, Eliav2, Squiblydoo, and contributors)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

### Pillow (PIL Software License / MIT-CMU)
（適用: Pillow）

```
The Python Imaging Library (PIL) is

    Copyright © 1997-2011 by Secret Labs AB
    Copyright © 1995-2011 by Fredrik Lundh and contributors

Pillow is the friendly PIL fork. It is

    Copyright © 2010 by Jeffrey A. Clark and contributors

Like PIL, Pillow is licensed under the open source MIT-CMU License:

By obtaining, using, and/or copying this software and/or its associated
documentation, you agree that you have read, understood, and will comply
with the following terms and conditions:

Permission to use, copy, modify and distribute this software and its
documentation for any purpose and without fee is hereby granted, provided
that the above copyright notice appears in all copies, and that both that
copyright notice and this permission notice appear in supporting
documentation, and that the name of Secret Labs AB or the author not be
used in advertising or publicity pertaining to distribution of the software
without specific, written prior permission.

SECRET LABS AB AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH REGARD TO THIS
SOFTWARE, INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS.
IN NO EVENT SHALL SECRET LABS AB OR THE AUTHOR BE LIABLE FOR ANY SPECIAL,
INDIRECT OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM
LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE
OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
PERFORMANCE OF THIS SOFTWARE.
```

---

*このファイルは配布物に含まれるコンポーネントに基づいて作成されています。ビルド構成（同梱ライブラリ）を変更した場合は、本ファイルも合わせて更新してください。より厳密にするには、Python PSF・Tcl/Tk・PyInstaller・tkDnD の各条文全文を上記URLから取得して追記できます。*
