# mmd2gltf

[日本語](#mmd2gltf) | [English](#english)

MMDのモデル(.pmxファイル)とモーション(.vmdファイル)を、VRChatやBlender、その他多くの3Dソフトで使われている標準フォーマット「glTF(.glb)」に変換するツールです。

見た目や動きをできるだけ
そのまま保つことを目指しています。GUI(画面操作)とCLI(コマンド操作)の両方が用意されているので、コマンド操作に慣れていない方はGUIから使えます。

## まず試す(GUI)

1. Pillowをインストールします(テクスチャ画像がPNG/JPG以外の場合に必要です)

   ```bash
   pip install Pillow
   ```

2. GUIを起動します

   ```bash
   python gui.py
   ```

3. 画面で以下を設定します

   - 変換したい`.pmx`ファイル(モデル)
   - 必要なら`.vmd`ファイル(モーション)
   - 出力先の`.glb`ファイル名

   ファイルはドラッグ&ドロップでも設定できます(`pip install tkinterdnd2`が必要。未導入でも「参照...」ボタンからの選択は可能です)。

4. 「変換」を実行すると、バックグラウンドで処理が進み、画面にログが表示されます。

画面右上から日本語/Englishをいつでも切り替えられます。

### GUIの主なオプション

- **unlit**: MMD特有のトゥーン(アニメ塗り)の見た目に近づけます
- **両面描画**: 髪やスカートの裏側が透けて見えなくなる場合があります
- **モーフ格納方式**: 通常は「sparse(軽量)」のままでOK。表情が崩れる場合だけ「dense」に変更してください
- **アルファモード**: 通常は「自動」のままでOK。顔が透けて見える等の不具合がある場合のみ変更してください

「詳細設定」を開くと、IK(足の動きなど)やコマ数、アニメーション名なども調整できます。

## コマンドラインで使う(CLI)

GUIと同じ変換をコマンドで行うこともできます。

```bash
# モデルだけを変換
python -m mmd2gltf モデル.pmx -o モデル.glb

# モデル+モーションを変換
python -m mmd2gltf モデル.pmx --vmd モーション.vmd -o ダンス.glb
```

### よく使うオプション

| オプション | 説明 |
|---|---|
| `--vmd FILE` | モーションファイルを指定してアニメーションとして変換 |
| `--unlit` | トゥーン調の見た目に近づける |
| `--force-double-sided` | 髪やスカートの裏面を消えなくする |
| `--morph-mode dense` | 表情(モーフ)が崩れる場合にこちらへ変更 |
| `--scale F` | サイズ調整(既定値0.08のままで通常問題なし) |

<details>
<summary>その他の細かいオプション(通常は変更不要)</summary>

| オプション | 説明 |
|---|---|
| `--no-ik` | IKを解決せず、生のモーションカーブのまま変換 |
| `--disable-ik 名前` | 指定した名前を含むIKボーンのみ無効化(繰り返し指定可。例: `--disable-ik 足`) |
| `--ignore-vmd-ik` | VMD内のIK ON/OFF指定を無視する |
| `--step N` | Nフレームごとに間引いてファイルサイズを削減(既定は全フレーム) |
| `--no-extras` | MMD固有の追加情報(`extras.mmd`)を出力しない |
| `--anim-name NAME` | アニメーションの名前を指定 |
| `--alpha-mode MODE` | 透明処理を`opaque`/`mask`/`blend`のいずれかに固定(既定は自動判別) |
| `--no-custom-attrs` | MMD固有の頂点情報を出力しない(Blender標準インポータでエラーになる場合に指定) |

</details>

## うまく表示されないときは

- **macOSのQuick Look/プレビューで表情が動かない** → OS標準ビューアの制限です。ファイル自体は正常です。
- **モーフ適用時に顔が透ける・メッシュが崩れる** → `--morph-mode dense`を試してください。
- **動作確認済みの推奨ビューア**: three.js系(gltf-viewer.donmccurdy.com)、Babylon.js Sandbox(sandbox.babylonjs.com)、Blender 3.x以降

## できること・できないこと

**変換されるもの**: メッシュ、ボーン、スキニング、表情モーフ、材質、VMDモーション(IK込みでベイク)

**MMD特有の情報はそのまま保存**: 物理演算(剛体・ジョイント)、IK設定、表示枠などはglTFの標準機能にはないため、`extras.mmd`という形でファイル内に保持されます。対応するツール側でこれを読み取れば再現可能です。

**このツールが行わないこと**:

- 物理演算そのものの計算(髪・スカートはボーンに追従するのみ)
- トゥーンシェーディングやスフィアマップなど、MMD特有の質感の完全な再現(`--unlit`で近づけることは可能)
- PMD(旧形式)、VMDのカメラ・照明・セルフ影キーへの対応

## 動作確認・テスト

```bash
python tests/make_test_data.py                # テスト用データを生成
python -m mmd2gltf tests/test.pmx --vmd tests/test.vmd -o tests/test.glb
python tests/check_glb.py tests/test.glb      # 変換結果を検証
```

## ファイル構成(開発者向け)

```
mmd2gltf/
  pmx.py        PMXファイルの読み込み
  vmd.py        VMDファイルの読み込み・補間計算
  animation.py  MMD式のボーン変形・IK計算
  gltf.py       glbファイルの生成
  convert.py    変換処理の本体
  cli.py        コマンドライン処理
```

## ライセンス

このツール(mmd2gltf)のソースコードはMITライセンスです。詳細は[LICENSE](./LICENSE)を参照してください。

**変換後のモデルには、元モデルの利用規約が適用されます。** このツールはファイル形式を変換するだけで、変換元モデル・モーションの著作権には関与しません。変換したモデルを配布・利用する際は、必ず元モデルの規約(改変可否、二次配布可否、クレジット表記の要否など)を確認してください。

### 使用している外部ライブラリのライセンス

本体の変換処理は標準ライブラリのみで動作します。以下は、テクスチャ変換やGUIのドラッグ&ドロップなど、一部機能を使う場合にのみ必要な任意インストールのライブラリです。

| ライブラリ | 用途 | ライセンス |
|---|---|---|
| [Pillow](https://github.com/python-pillow/Pillow) | テクスチャ画像の変換(PNG/JPG以外の形式) | MIT-CMU(HPND系) |
| [tkinterdnd2](https://github.com/pmgagne/tkinterdnd2) | GUIでのファイルのドラッグ&ドロップ | MIT License |

いずれも寛容な(商用利用・再配布可能な)オープンソースライセンスですが、配布時は各ライブラリのライセンス表記義務(著作権表示など)に従ってください。

## 実行ファイル化

PyInstallerを使って実行ファイル(.exe)にパッケージ化できます。

```bash
# 1) 依存パッケージをインストール
pip install Pillow PyInstaller

# 2) spec ファイルを使ってビルド
uv run pyinstaller mmd2gltf.spec

# 3) dist/mmd2gltf.exe が生成される
```

specファイルでは以下の設定を行っています:
- `datas`: `mmd2gltf`パッケージとtkinterdnd2のtkdnd本体(tcl/dll)を同梱
- `hiddenimports`: `mmd2gltf`サブモジュールを正しく読み込ませる
- `console=False`: GUIアプリとして起動(コンソールウィンドウ非表示)
- `icon`: ウィンドウアイコンに `icon.png` を使用

※ `icon.png` → `icon.ico` (Windowsアイコン形式) への変換は
`icon.png.save('icon.ico')` 等で事前に実行してください。

---

## English

[日本語](#mmd2gltf) | [English](#english)

mmd2gltf converts MMD models (`.pmx`) and motions (`.vmd`) into glTF (`.glb`), the standard format used by VRChat, Blender, and most other 3D software.

The goal is to preserve the original look and motion as closely as possible. Both a GUI and a CLI are provided, so if you're not comfortable with the command line, you can use the GUI instead.

### Quick start (GUI)

1. Install Pillow (needed only if your texture images are not PNG/JPG)

   ```bash
   pip install Pillow
   ```

2. Launch the GUI

   ```bash
   python gui.py
   ```

3. In the window, set:

   - The `.pmx` model file to convert
   - Optionally, a `.vmd` motion file
   - The output `.glb` file name

   Files can also be set via drag and drop (requires `pip install tkinterdnd2`; without it, you can still use the "Browse..." buttons).

4. Click "Convert" to run the conversion in the background; progress is shown in the log area.

You can switch between Japanese and English at any time from the top-right of the window.

#### Main GUI options

- **Unlit**: gets closer to MMD's characteristic toon (flat-shaded) look
- **Double-sided**: prevents the back side of hair or skirts from appearing transparent
- **Morph storage mode**: leave on "sparse" (lighter) by default; switch to "dense" only if facial expressions look broken
- **Alpha mode**: leave on "auto" by default; change it only if you see issues such as the face appearing transparent

Opening "Advanced settings" lets you also adjust IK (e.g. leg motion), frame step, and the animation name.

### Command line (CLI)

The same conversion is also available from the command line.

```bash
# Convert model only
python -m mmd2gltf model.pmx -o model.glb

# Convert model + motion
python -m mmd2gltf model.pmx --vmd motion.vmd -o dance.glb
```

#### Common options

| Option | Description |
|---|---|
| `--vmd FILE` | Specify a motion file to convert as an animation |
| `--unlit` | Get closer to a toon-style look |
| `--force-double-sided` | Keep the back side of hair/skirts from disappearing |
| `--morph-mode dense` | Use this if morphs (facial expressions) look broken |
| `--scale F` | Adjust scale (default 0.08 is fine in most cases) |

<details>
<summary>Other fine-grained options (usually no need to change)</summary>

| Option | Description |
|---|---|
| `--no-ik` | Convert raw motion curves without resolving IK |
| `--disable-ik NAME` | Disable only IK bones whose name contains NAME (repeatable, e.g. `--disable-ik leg`) |
| `--ignore-vmd-ik` | Ignore the IK on/off flags stored in the VMD |
| `--step N` | Reduce file size by sampling every N frames (default: every frame) |
| `--no-extras` | Don't output MMD-specific extra data (`extras.mmd`) |
| `--anim-name NAME` | Set the animation's name |
| `--alpha-mode MODE` | Force alpha handling to `opaque`/`mask`/`blend` (default: auto-detect) |
| `--no-custom-attrs` | Don't output MMD-specific vertex attributes (use this if Blender's standard importer errors out) |

</details>

### If something doesn't display correctly

- **Facial expressions don't animate in macOS Quick Look/Preview** → This is a limitation of the OS's built-in viewer; the file itself is fine.
- **Face becomes transparent or the mesh breaks when morphs are applied** → Try `--morph-mode dense`.
- **Recommended viewers known to work well**: three.js-based viewers (gltf-viewer.donmccurdy.com), Babylon.js Sandbox (sandbox.babylonjs.com), Blender 3.x and later

### What is and isn't converted

**Converted**: meshes, bones, skinning, facial morphs, materials, VMD motion (baked, including IK)

**MMD-specific data is preserved as-is**: physics (rigid bodies/joints), IK settings, display frames, and similar data have no equivalent in standard glTF, so they are kept in the file under `extras.mmd`. Tools that understand this data can reproduce it.

**What this tool does not do**:

- Simulate physics itself (hair/skirts simply follow their bones)
- Fully reproduce MMD-specific shading such as toon shading or sphere maps (`--unlit` gets you closer)
- Support the legacy PMD format, or VMD camera/lighting/self-shadow keyframes

### Verification / tests

```bash
python tests/make_test_data.py                # Generate test data
python -m mmd2gltf tests/test.pmx --vmd tests/test.vmd -o tests/test.glb
python tests/check_glb.py tests/test.glb      # Validate the conversion result
```

### Project layout (for developers)

```
mmd2gltf/
  pmx.py        PMX file loading
  vmd.py        VMD file loading and interpolation
  animation.py  MMD-style bone deformation and IK
  gltf.py       glb file generation
  convert.py    Main conversion logic
  cli.py        Command-line interface
```

### License

The mmd2gltf source code is licensed under the MIT License. See [LICENSE](./LICENSE) for details.

**The terms of use of the original model apply to the converted model.** This tool only converts file formats; it has no bearing on the copyright of the source model or motion. Before distributing or using a converted model, always check the original model's terms (whether modification and redistribution are allowed, whether credit is required, etc.).

#### Licenses of external libraries used

The core conversion logic uses only the standard library. The following are optional libraries needed only for certain features, such as texture conversion or drag-and-drop in the GUI.

| Library | Purpose | License |
|---|---|---|
| [Pillow](https://github.com/python-pillow/Pillow) | Texture image conversion (formats other than PNG/JPG) | MIT-CMU (HPND-style) |
| [tkinterdnd2](https://github.com/pmgagne/tkinterdnd2) | Drag-and-drop file support in the GUI | MIT License |

Both are permissive open-source licenses (commercial use and redistribution allowed), but be sure to follow each library's attribution requirements when distributing your software.

### Building a standalone executable

You can package the app into a standalone executable (`.exe`) using PyInstaller.

```bash
# 1) Install dependencies
pip install Pillow PyInstaller

# 2) Build using the spec file
uv run pyinstaller mmd2gltf.spec

# 3) dist/mmd2gltf.exe is generated
```

The spec file configures the following:
- `datas`: bundles the `mmd2gltf` package and tkinterdnd2's tkdnd binaries (tcl/dll)
- `hiddenimports`: ensures the `mmd2gltf` submodules are loaded correctly
- `console=False`: runs as a GUI app (no console window)
- `icon`: uses `icon.png` as the window icon

Note: convert `icon.png` to `icon.ico` (Windows icon format) beforehand, e.g. with `icon.png.save('icon.ico')`.
