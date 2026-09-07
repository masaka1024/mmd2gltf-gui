<p align="center">
  <img src="icon.png" alt="mmd2gltf" width="160">
</p>

# mmd2gltf

![Release](https://img.shields.io/github/v/release/masaka1024/mmd2gltf-gui)
![Downloads](https://img.shields.io/github/downloads/masaka1024/mmd2gltf-gui/total)
![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)
![Python](https://img.shields.io/badge/Python-3-blue.svg)

**日本語 | [English](README.en.md)**

<!-- mmd2gltf-ecosystem:start -->
> **mmd2gltf エコシステム** — MMD モデル(PMX/VMD)を一度 `.glb` に変換すれば、**Unity・Unreal Engine 5・Blender** のどれにも、物理(揺れ物)と材質(トゥーン/スフィア)つきでそのまま持ち込めます。glTF で表現できない MMD 固有データは `extras.mmd` に元の値のまま残り、各インポーターがそれを読んでエンジン側で再構築します。
>
> ```
> PMX / VMD
>    │  変換 (mmd2gltf-gui または mmd2gltf-cs)
>    ▼
> .glb + extras.mmd   ← 変換はこの 1 回だけ
>    │
>    ├─▶ Unity   … mmd2gltf-unity-physics-importer
>    ├─▶ UE5     … mmd2gltf-ue5-physics-importer
>    └─▶ Blender … mmd2gltf-blender-physics-importer
> ```
>
> | 目的 | 使うもの |
> |---|---|
> | まず変換する(Windows EXE あり) | [mmd2gltf-gui](https://github.com/masaka1024/mmd2gltf-gui) — Python 版。GUI / CLI |
> | 変換する(C# 版、物理ベイク内蔵) | [mmd2gltf-cs](https://github.com/masaka1024/mmd2gltf-cs) — 出力形式は gui と同一。揺れ物を自作エンジンでベイク |
> | Unity で動かす | [mmd2gltf-unity-physics-importer](https://github.com/masaka1024/mmd2gltf-unity-physics-importer) — Editor 拡張。Bullet 互換エンジン同梱(PhysX 不使用) |
> | Unreal Engine 5 で動かす | [mmd2gltf-ue5-physics-importer](https://github.com/masaka1024/mmd2gltf-ue5-physics-importer) — C++ プラグイン。同じエンジンの C++ 移植(Chaos 不使用) |
> | Blender で動かす | `mmd2gltf-blender-physics-importer`(準備中) — アドオン。Blender 内蔵の Bullet に配線 |
> | (ライブラリ)物理エンジン本体 | [mmd2gltf-cs-physics](https://github.com/masaka1024/mmd2gltf-cs-physics) — cs / Unity / UE5 が使う Bullet 2.75 互換エンジン。直接は使いません |
> | (別ルート)Blender の mmd_tools から glTF/FBX を出す | [mmd-to-gltf-exporter](https://github.com/masaka1024/mmd-to-gltf-exporter) — `extras.mmd` は付きません(物理の再構築は対象外) |
>
> 揺れ物の挙動は、Unity と UE5 が同じエンジンを使うため一致します。Blender 版は Blender 内蔵の Bullet で動かすため、動きは近いものの完全には一致しません。
>
> **このリポジトリ**: 変換ツールの **Python 版**(Windows EXE 配布あり)。ここで `.glb` を作ります。まずはこちらから。
<!-- mmd2gltf-ecosystem:end -->

MMDのPMXモデル(+VMDモーション)を **glTF 2.0(.glb)** に変換するツールです。GUIとCLIの両方で使えます。標準ライブラリだけで動作し、テクスチャ変換(BMP/TGA/sph/spa→PNG)にPillow、アルファ判定の一部にNumPyを任意で使います。

glTFで表現できない情報(剛体・ジョイント・IK設定・材質のトゥーン/スフィア設定など)は `extras.mmd` に元の値のまま残します。受け取り側(ゲームエンジンや他のツール)はこの情報からMMDモデルの構造を再構築できます。

> **▶ Windowsで使う:** [Releases](https://github.com/masaka1024/mmd2gltf-gui/releases/latest) にビルド済みのEXEがあります(Pythonのインストール不要)。

<p align="center">
  <img src="docs/screenshot.png" alt="mmd2gltf GUI" width="640">
</p>

## 変換例(スクリーンショット)

Tda式初音ミク・アペンド / 初音ミクV4X (モデル: Tda様) を変換し、[Unityインポーター](https://github.com/masaka1024/mmd2gltf-unity-physics-importer)で取り込んだ表示例です。

<p align="center">
  <img src="docs/screenshot_tda_unity.png" alt="Tda式初音ミク・アペンドとV4XをUnityで表示した例" width="640">
</p>

> モデルデータ自体は本リポジトリに含まれません。[ピアプロ・キャラクター・ライセンス](https://piapro.jp/license/pcl/summary)および各モデルの利用規約に基づいて使用しています。
> © Crypton Future Media, INC. www.piapro.net

## 検証済みモデル

以下のモデルで変換・表示を確認しています(モデルデータは各配布元から入手してください)。

| モデル | 作者 | 主な検証内容 |
|---|---|---|
| Tda式初音ミク・アペンド Ver1.10 | Tda様 | マテリアル色・スフィア、額の髪影/チーク等の半透明オーバーレイ、Unityでのトゥーン再構築 |
| Tda式初音ミク V4X Ver1.00 | Tda様 | 共有テクスチャのα分類、メガネ・レンズ等の半透明主体マテリアルの描画順 |
| IA (PMXモデル) | お宮様 ([bowlroll](https://bowlroll.net/file/81272)) | 透け髪(高α域)のブレンド復元、眉・目の縁取りの半透明表現 |

## 目次

- [このツールの狙い](#このツールの狙い)
- [特徴](#特徴)
- [動作環境](#動作環境)
- [Windows EXE版(Pythonなしで使う)](#windows-exe版pythonなしで使う)
- [Python版のインストール](#python版のインストール)
- [使い方](#使い方)
- [IKの扱い](#ikの扱い)
- [物理ベイク(実験的機能)](#物理ベイク実験的機能)
- [剛体を目で確かめる(物理インスペクター)](#剛体を目で確かめる物理インスペクター)
- [オプション一覧](#オプション一覧)
- [ビュアー互換性の注意](#ビュアー互換性の注意)
- [変換内容](#変換内容)
- [制限](#制限)
- [テスト](#テスト)
- [構成](#構成)
- [ライセンス](#ライセンス)

## このツールの狙い

mmd2gltfの目的は、**MMDのデータを他の環境へ持ち出すこと**です。glTF(.glb)は、そのための「器(コンテナ)」として使っています。

変換結果は次の二層構造になっています。

1. **保存層(`extras.mmd`)** — 剛体・ジョイント・IK設定・付与親・材質のトゥーン/スフィア/エッジ設定など、glTFでは表現できない情報を元のPMX値のまま保存します。受け取り側はこの層から元のモデル構造を再構築できます。剛体・ジョイントについては、glTF座標系・スケール済みに変換したビュー(`physicsGltf`)も併せて書き出します。
2. **表示層(glTF標準のメッシュ・スキン・アニメーション)** — 一般のglTFビューアでそのまま見るための層です。VMDモーションはIK解決込みでキーフレームに焼き込みます。

逆変換(glTF→PMX)のツールはありません。データ仕様は `mmd2gltf/extrasMmd_schema.md` と `mmd2gltf/physicsGltf_schema.md` にまとめてあります。

> **利用規約について:** 変換してもモデル・モーションの利用規約はそのまま適用されます。`extras.mmd` には元データがそのまま含まれるため、**変換後のファイルの配布は、元データの配布と同じ扱いになります**。再配布が禁止されているモデルは、変換後も配布できません。

### Unityインポーター

`extras.mmd` を読み取ってUnity上でモデルを再構築するエディタ拡張を、別リポジトリで公開しています: **[mmd2gltf-unity-physics-importer](https://github.com/masaka1024/mmd2gltf-unity-physics-importer)**(要 UniGLTF / lilToon、URPプロジェクト向け)。剛体・ジョイント情報からの物理の再構築と、材質のトゥーン/スフィア/輪郭線設定からのMMD風表示の復元を行います。

## 特徴

- **追加ライブラリ不要** — 標準ライブラリのみで動作(テクスチャ変換時のみPillow、アルファ自動判定の精度向上にNumPyを任意で使用)。
- **PMX 2.0/2.1 を解析** — メッシュ・スキニング・材質・モーフ・物理設定を読み込み。
- **VMDモーションをベイク** — ベジェ補間を評価し、MMDと同じ変形順(変形階層→付与→CCD-IK+軸制限)で毎フレーム焼き込み。
- **IKの扱いを4通りで制御** — フルベイクから「一切解かない」まで([IKの扱い](#ikの扱い)を参照)。
- **glTFで表現できない情報を `extras.mmd` に保存** — 剛体・ジョイント・IK設定・付与親など。
- **GUI / CLI 両対応** — GUIは日本語/English切り替え、ドラッグ&ドロップ対応。
- **Windows EXE版あり** — Pythonを入れなくても使えます。

## 動作環境

- **Windows EXE版** — Pythonのインストール不要。
- **Python版** — Python 3(標準ライブラリのみで動作)
  - 任意: **Pillow**(テクスチャがPNG/JPG以外の場合に必要)
  - 任意: **NumPy**(アルファ自動判定で半透明の事前ブレンドとUV領域サンプリングを行う場合。無くても変換はできます)
  - 任意: **tkinterdnd2**(GUIでドラッグ&ドロップを使う場合)

## Windows EXE版(Pythonなしで使う)

1. [Releases](https://github.com/masaka1024/mmd2gltf-gui/releases) ページから最新のzipをダウンロードします。
2. 展開して `mmd2gltf.exe` をダブルクリックすると、GUIが起動します。
3. あとは[使い方](#使い方)のGUIと同じです。Pillow・NumPy・tkinterdnd2は同梱済みです。

> **初回起動時の警告について**
> 署名を付けていない個人開発ソフトのため、初回起動時にWindows SmartScreenが
> 「WindowsによってPCが保護されました」と表示することがあります。
> 「詳細情報」→「実行」で起動できます。ウイルス対策ソフトがPyInstaller製exeを
> 誤検知する場合もありますが、ソースは本リポジトリで公開しています。

## Python版のインストール

リポジトリを取得して、必要に応じて任意ライブラリを入れるだけです。

```bash
# 任意: テクスチャがPNG/JPG以外の場合
pip install Pillow

# 任意: アルファ自動判定の精度向上
pip install numpy

# 任意: GUIでドラッグ&ドロップを使う場合
pip install tkinterdnd2
```

## 使い方

### GUI

```bash
python gui.py
```

ファイル選択と主要オプション(unlit・両面描画・モーフ格納方式・アルファモード・スケール)を画面から指定できます。「詳細設定」を開くとIK関連・step・アニメーション名・物理ベイク関連も設定できます。変換はバックグラウンドで実行され、ログが画面に表示されます。

- PMX/VMD欄には、エクスプローラーからファイルをドラッグ&ドロップして設定できます(要 `tkinterdnd2`。未導入でも「参照...」ボタンでの選択は使えます)。
- 画面右上のドロップダウンで日本語/Englishを切り替えられます(初回起動時はOSのロケールに応じて自動選択)。

### CLI

```bash
python -m mmd2gltf モデル.pmx -o モデル.glb
python -m mmd2gltf モデル.pmx --vmd モーション.vmd -o ダンス.glb
```

### Pythonから

```python
from mmd2gltf import convert
```

## IKの扱い

VMDモーションを焼き込むとき、IK(足・腕などの逆運動学)をどう処理するかを4通りで選べます。通常は既定のフルベイクのままで問題なく、意図しない足の動きが出たときに切り替えます。

| モード | 指定 | 挙動 |
| --- | --- | --- |
| **フルベイク(既定)** | `--vmd FILE` のみ | 全IKを解決してアニメーションに焼き込む(30fps)。VMD内のIK ON/OFFキーは尊重する。 |
| **全IKを解かない** | `--no-ik` | ベイク時にIKを一切解かず、生のFKカーブのみを出力する。 |
| **一部だけ無効化** | `--disable-ik 名前` | 名前を含むIKボーンだけ無効化(繰り返し可)。例: `--disable-ik 足` で足/つま先IKだけ切る。 |
| **VMDのIKキーを無視** | `--ignore-vmd-ik` | VMD内のIK ON/OFFキーを無視して解く。フルキー版モーション等で足IKがOFF指定されていると既定では自動で解かないため、それを打ち消したいとき。 |

## 物理ベイク(実験的機能)

`--bake-physics` を付けると、髪・スカートなどの剛体物理を簡易シミュレーション(PBDばねモデル)で解き、結果をボーンのキーフレームとして焼き込みます。物理エンジンを持たないglTFビューア向けの近似機能です。

> **この機能は実験的なもので、現在は積極的にお勧めしていません。** 内部のソルバーは初期のまま更新しておらず、MMD本体(Bullet物理)の挙動とは大きく異なります。髪やスカートを本来の挙動で動かしたい場合は、`extras.mmd` に保存された剛体・ジョイント情報を使ってエンジン側(Unity等)で物理を再構築してください。ベイク結果を確認用の参考として使う程度に留めるのが無難です。

```bash
# 髪だけベイク(既定)
python -m mmd2gltf モデル.pmx --vmd モーション.vmd --bake-physics -o out.glb

# スカートも含めて全部ベイク
python -m mmd2gltf モデル.pmx --vmd モーション.vmd --bake-physics --bake-target all -o out.glb
```

物理ベイクを使っても `extras.mmd` の剛体・ジョイント情報は変更されません。調整用のパラメータは多数ありますが、上記の理由から本READMEでは個別に説明しません。必要な場合は `python -m mmd2gltf --help` を参照してください。GUIでは「詳細設定」から同じ項目を設定できます。

## 剛体を目で確かめる(物理インスペクター)

`tools/mmd_physics_inspector.html` は、変換後のGLBをブラウザで開いて `extras.mmd` の中身を確認するためのスタンドアロンツールです(ブラウザにドラッグ&ドロップするだけ。インストール不要)。

剛体・ジョイント・衝突グループを3D表示し、剛体をクリックすると形状・サイズ・モード・紐づくボーン・衝突するグループを表示します。エンジン側で物理を組むときに、「どの剛体がどこにあるのか」「本当に衝突する設定になっているのか」を確認するのに使えます。

> **16bitフィールドの解釈について:** PMXの剛体には「非衝突グループフラグ」と呼ばれる16bitのフィールドがありますが、**立っているビットは「そのグループと衝突する」を意味します**(Bulletのcollision filter maskそのもの)。PMXエディタのチェックボックス表示(チェック=非衝突)とは反転しているため、名前のとおりに読むと判定が逆になります。本ツールとインスペクターはこの解釈に統一しています(`extras.mmd` に書き出す値はPMXのまま。詳細は `mmd2gltf/physicsGltf_schema.md`)。

## オプション一覧

| オプション | 説明 |
| --- | --- |
| `--vmd FILE` | VMDモーションをglTFアニメーションとしてベイク(IK解決込み・30fps) |
| `--no-ik` | ベイク時に全IKを解かない(生のFKカーブのみ) |
| `--disable-ik 名前` | 名前を含むIKボーンだけ無効化(繰り返し可。例: `--disable-ik 足` で足/つま先IK) |
| `--ignore-vmd-ik` | VMD内のIK ON/OFFキーを無視する(既定では尊重) |
| `--step N` | Nフレームおきにサンプリングして容量削減(既定1=全フレーム) |
| `--unlit` | 材質に `KHR_materials_unlit` を付与(MMDのトゥーン見た目に近い) |
| `--no-extras` | `extras.mmd` を出力しない |
| `--anim-name NAME` | アニメーション名 |
| `--morph-mode MODE` | モーフの格納方式。`sparse`=軽量(既定)、`dense`=最大互換(sparse非対応ビュアーで顔が壊れる場合はこれ)、`none`=モーフ無し |
| `--alpha-mode MODE` | `auto`(既定)はテクスチャのα分布を解析し OPAQUE/MASK/BLEND を自動判別。`opaque`/`mask`/`blend`で全材質強制も可 |
| `--force-double-sided` | 全材質を両面描画に(髪やスカートの裏面が消える場合に) |
| `--scale F` | MMD単位→glTF単位(メートル)への一律スケール(既定0.08)。MMDモデルは慣習的に1ユニット≈8cmで作られているため、`1.0`(無変換)にするとglTFビューアでは約12.5倍の大きさで表示されます。頂点・ボーン位置・SDEFパラメータ・モーフ・ベイク済みアニメーションの平行移動に適用され、`extras.mmd`内の生データには影響しません(`extras.mmd.unitScale`に係数を記録) |
| `--no-custom-attrs` | MMD固有の頂点属性(`_SDEF_C`/`_SDEF_R0`/`_SDEF_R1`/`_ADDUV1..4`/`_EDGESCALE`/`_WEIGHTTYPE`)を出力しない。Blenderの標準glTFインポータがこれらの属性でエラーになる場合に指定(データは`extras.mmd`側に残ります) |
| `--bake-physics` ほか | 物理ベイク関連([実験的機能](#物理ベイク実験的機能))。一覧は `--help` を参照 |

## ビュアー互換性の注意

- macOSのQuick Look/プレビュー(RealityKit)はglTFのモーフターゲット自体に非対応です。モーフが「抜ける」のはビュアー側の制限で、ファイルは正常です。
- sparseアクセッサの実装が不完全なビュアー/ローダーではモーフ適用時にメッシュが壊れる(顔が透ける・口パクが別の場所の変形に見える等)ことがあります。その場合は `--morph-mode dense` で変換してください(容量は増加)。UniGLTF/UniVRM系のUnityインポータで使う場合も `dense` を推奨します。
- 動作確認済みのビュアー: three.js系 (gltf-viewer.donmccurdy.com)、Babylon.js Sandbox (sandbox.babylonjs.com)、Blender 3.x以降のglTFインポータ。

## 変換内容

glTFで直接表現するもの:

- メッシュ(頂点・法線・UV・材質ごとのプリミティブ分割)
- スキニング(BDEF1/2/4。SDEF/QDEFは線形ブレンド近似+元パラメータを保持)
- ボーン階層(PMXボーン順=skin.joints順なのでインデックス互換)
- 頂点モーフ→モーフターゲット(sparseアクセッサ)、UVモーフ→`TEXCOORD_0`ターゲット、グループモーフ→展開して合成ターゲット
- 材質(拡散色→baseColor、両面フラグ、αによるBLEND判定、テクスチャ埋め込み)
- VMDモーション: ベジェ補間を評価し、MMDと同じ変形順(変形階層→付与→CCD-IK+軸制限)で毎フレームベイク。モーフキーはweightsアニメーションに変換

`extras.mmd` に保存するもの(生のPMX値・MMD左手系座標):

- 剛体・ジョイント(物理)、IK設定、付与親、固定軸/ローカル軸、表示枠
- ボーンモーフ・材質モーフ・フリップ/インパルスモーフの内容
- 材質のスフィアマップ/トゥーン/エッジ/環境光/反射光設定、メモ
- アルファ分類(`alphaClass`)と、事前ブレンド前のテクスチャ(`origTexture`)
- 頂点単位データはカスタム属性として保持: `_ADDUV1..4`、`_EDGESCALE`、`_SDEF_C/_SDEF_R0/_SDEF_R1`、`_WEIGHTTYPE`

`extras.mmd.physicsGltf` には、剛体・ジョイントをglTF座標系・スケール済み・ボーンローカルに変換したビューを併載します(生データは変更しません)。エンジン側で物理を組む場合はこちらを使うと座標変換が不要です。

座標変換: 位置/法線 `(x,y,z)→(x,y,-z)`、クォータニオン `(x,y,z,w)→(-x,-y,z,w)`、三角形は巻き順反転。

## 制限

- PMD(旧形式)・PMX2.1のソフトボディは未対応(PMXへの変換はPMXEditor等で)
- 物理ベイクは簡易シミュレーションによる近似で、MMD本体(Bullet物理)の挙動を再現するものではありません([物理ベイク(実験的機能)](#物理ベイク実験的機能)を参照)
- MMDのトゥーンシェーディング/スフィアマップ/エッジ描画はglTFのPBRでは再現できないため、見た目はビューア依存です(`--unlit`で近づきます)
- 共有トゥーン(toon01〜10.bmp)はMMD本体同梱のため埋め込まれません(番号はextrasに保持)
- VMDのカメラ・照明・セルフ影キーは対象外

## テスト

```bash
python tests/make_test_data.py                # 合成PMX/VMDを生成
python -m mmd2gltf tests/test.pmx --vmd tests/test.vmd -o tests/test.glb
python tests/check_glb.py tests/test.glb      # 構造検証
```

## 構成

```
mmd2gltf/
  pmx.py                  PMX 2.0/2.1 パーサー
  vmd.py                  VMDパーサー+ベジェ補間
  animation.py            MMD式変形パイプライン(付与・CCD-IK)とベイク
  bake_hair.py            物理ベイク(実験的機能)
  drape.py                物理ベイク用: PMXメッシュからドレープ深さを実測
  physics.py              extras.mmd.physicsGltf(剛体・ジョイントのglTF座標系ビュー)の生成
  mathutil.py             クォータニオン/ベクトル演算
  gltf.py                 GLBビルダー(sparseアクセッサ対応)
  convert.py              変換本体
  cli.py                  CLI
  extrasMmd_schema.md     extras.mmd の仕様
  physicsGltf_schema.md   extras.mmd.physicsGltf の仕様
gui.py                    GUI
tools/
  mmd_physics_inspector.html  剛体・ジョイント・衝突グループの目視確認ツール
  build_release.ps1           Windows EXE版のビルドから配布zipまで
  release_pack.py             配布物の検査とzipの組み立て(上記から呼ばれる)
tests/                    合成テストデータの生成と構造検証
dist_docs/                EXE版に同梱する説明書
```

## ライセンス

本ツール(mmd2gltf)のソースコードはMITライセンスで公開しています。詳細は [LICENSE](LICENSE) を参照してください。

Windows EXE版には、Python・Pillow・NumPy・tkinterdnd2・tkDnD・Tcl/Tk などのオープンソースコンポーネントが同梱されています。各コンポーネントのライセンスは [THIRD_PARTY_LICENSES.md](THIRD_PARTY_LICENSES.md) にまとめています。
