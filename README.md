# mmd2gltf

MMDのPMXモデル(+VMDモーション)を、可能な限り忠実にglTF 2.0(.glb)へ変換するPython CLIです。依存は標準ライブラリのみで動作し、テクスチャ変換(BMP/TGA/sph/spa→PNG)にのみPillowを使います。

## 使い方

```bash
pip install Pillow   # テクスチャがPNG/JPG以外の場合に必要

python -m mmd2gltf モデル.pmx -o モデル.glb
python -m mmd2gltf モデル.pmx --vmd モーション.vmd -o ダンス.glb
```

オプション:

| オプション | 説明 |
|---|---|
| `--vmd FILE` | VMDモーションをglTFアニメーションとしてベイク(IK解決込み・30fps) |
| `--no-ik` | ベイク時に全IKを解かない(生のFKカーブのみ) |
| `--disable-ik 名前` | 名前を含むIKボーンだけ無効化(繰り返し可。例: `--disable-ik 足` で足/つま先IK) |
| `--ignore-vmd-ik` | VMD内のIK ON/OFFキーを無視する(既定では尊重。フルキー版モーション等で足IKがOFF指定されていれば自動で解かない) |
| `--step N` | Nフレームおきにサンプリングして容量削減(既定1=全フレーム) |
| `--unlit` | 材質に `KHR_materials_unlit` を付与(MMDのトゥーン見た目に近い) |
| `--no-extras` | `extras.mmd`(下記)を出力しない |
| `--anim-name NAME` | アニメーション名 |
| `--morph-mode MODE` | モーフの格納方式。`sparse`=軽量(既定)、`dense`=最大互換(sparse非対応ビュアーで顔が壊れる場合はこれ)、`none`=モーフ無し |
| `--alpha-mode MODE` | `auto`(既定)はテクスチャのα分布を解析し OPAQUE/MASK/BLEND を自動判別。肌テクスチャの未使用領域の透明を誤ってBLENDにしない(顔が透ける・裏返って見える問題の対策)。`opaque`/`mask`/`blend`で全材質強制も可 |
| `--force-double-sided` | 全材質を両面描画に(three.js MMDLoaderと同じ挙動。髪やスカートの裏面が消える場合に) |
| `--scale F` | MMD単位→glTF単位(メートル)への一律スケール(既定0.08)。MMDモデルは慣習的に1ユニット≈8cmで作られており(身長160cmなら約20ユニット)、`1.0`のまま(無変換)にするとglTFビューアでは約12.5倍の巨人として表示される。既定の`0.08`はそのための変換係数。頂点・ボーン位置・SDEFパラメータ・モーフの位置オフセット・ベイク済みアニメーションの平行移動に適用され、法線・UV・回転・`extras.mmd`内の生データ(剛体・ジョイント等)には影響しない(`extras.mmd.unitScale`に適用した係数を記録)。元モデルが別の単位系で作られている場合は`--scale 1.0`などに調整 |
| `--no-custom-attrs` | MMD固有の頂点属性(`_SDEF_C`/`_SDEF_R0`/`_SDEF_R1`/`_ADDUV1..4`/`_EDGESCALE`/`_WEIGHTTYPE`)を出力しない。**Blenderの標準glTFインポータがこれらの属性名でメッシュを読み込めずエラーになる場合はこれを指定**(データはそのまま`extras.mmd`側には残るので情報は失われない) |

### ビュアー互換性の注意

- macOSのQuick Look/プレビュー(RealityKit)はglTFのモーフターゲット自体に非対応です。モーフが「抜ける」のはビュアー側の制限で、ファイルは正常です。
- sparseアクセッサの実装が不完全なビュアーではモーフ適用時にメッシュが壊れる(顔が透ける等)ことがあります。その場合は `--morph-mode dense` で変換してください(容量は増加)。sparse時もゼロ基底bufferViewを持たせているため、sparse未対応ローダーでは「モーフ無効」に安全に落ちます。
- 動作確認済みの推奨ビュアー: three.js系 (gltf-viewer.donmccurdy.com)、Babylon.js Sandbox (sandbox.babylonjs.com)、Blender 3.x以降のglTFインポータ。


### GUI

```bash
python gui.py
```

ファイル選択と主要オプション(unlit・両面描画・モーフ格納方式・アルファモード)を画面から指定できます。「詳細設定」を開くとIK関連・step・アニメーション名なども設定可能です。変換はバックグラウンドで実行され、ログが画面に表示されます。依存は標準ライブラリのtkinterのみです。

PMX/VMD欄には、エクスプローラーからファイルをドラッグ&ドロップして設定することもできます(要`tkinterdnd2`。未導入でも「参照...」ボタンでの選択は引き続き使えます)。

画面右上のドロップダウンで日本語/English をいつでも切り替えられます(初回起動時はOSのロケールに応じて自動選択)。

```bash
pip install tkinterdnd2   # ドラッグ&ドロップを使う場合(uv環境なら uv pip install tkinterdnd2)
```

Pythonからも使えます: `from mmd2gltf import convert`

## 変換内容(忠実度)

glTFで直接表現できるもの:

- メッシュ(頂点・法線・UV・材質ごとのプリミティブ分割)
- スキニング(BDEF1/2/4。SDEF/QDEFは線形ブレンド近似+元パラメータを保持)
- ボーン階層(PMXボーン順=skin.joints順なのでインデックス互換)
- 頂点モーフ→モーフターゲット(sparseアクセッサで軽量)、UVモーフ→`TEXCOORD_0`ターゲット、グループモーフ→展開して合成ターゲット
- 材質(拡散色→baseColor、両面フラグ、αによるBLEND判定、テクスチャ埋め込み)
- VMDモーション: ベジェ補間を評価し、MMDと同じ変形順(変形階層→付与→CCD-IK+軸制限)で毎フレームベイク。モーフキーはweightsアニメーションに変換

glTFに存在しない概念は `extras.mmd`(生のPMX値・MMD左手系座標)として全て保存:

- 剛体・ジョイント(物理)、IK設定、付与親、固定軸/ローカル軸、表示枠
- ボーンモーフ・材質モーフ・フリップ/インパルスモーフの内容
- 材質のスフィアマップ/トゥーン/エッジ/環境光/反射光設定、メモ
- 頂点単位データはカスタム属性として保持: `_ADDUV1..4`、`_EDGESCALE`、`_SDEF_C/_SDEF_R0/_SDEF_R1`、`_WEIGHTTYPE`

座標変換: 位置/法線 `(x,y,z)→(x,y,-z)`、クォータニオン `(x,y,z,w)→(-x,-y,z,w)`、三角形は巻き順反転。

## 制限

- PMD(旧形式)・PMX2.1のソフトボディは未対応(PMXへの変換はPMXEditor等で)
- 物理演算はベイクしません(髪・スカートの剛体はボーン追従のまま)。剛体/ジョイント情報はextrasにあるので、エンジン側で再構築可能です
- MMDのトゥーンシェーディング/スフィアマップ/エッジ描画はglTFのPBRでは再現不可のため、見た目はビューア依存です(`--unlit`で近づきます)
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
  pmx.py        PMX 2.0/2.1 パーサー(全セクション)
  vmd.py        VMDパーサー+ベジェ補間
  animation.py  MMD式変形パイプライン(付与・CCD-IK)とベイク
  gltf.py       GLBビルダー(sparseアクセッサ対応)
  convert.py    変換本体
  cli.py        CLI
```

## ライセンス

本ツール(mmd2gltf)のソースコードはMITライセンスで公開しています。詳細は [LICENSE](./LICENSE) を参照してください。

**変換後モデルは元モデルの利用規約に従います。** 本ツールはPMX/VMDファイルの形式変換のみを行うものであり、変換元モデル・モーションの著作権や利用条件には一切関与しません。変換したモデルを配布・利用する際は、必ず元モデルの規約(改変可否、二次配布可否、クレジット表記義務など)を確認し、それに従ってください。
