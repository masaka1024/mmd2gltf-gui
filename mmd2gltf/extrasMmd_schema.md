# `extras.mmd` スキーマ

mmd2gltf は、glTF 標準では表現できない MMD（PMX）固有の情報と、
変換パイプラインが不可逆に加工した部分の復元用データを、
glTF JSON の `extras.mmd` に残します。

Unity / Unreal などのインポーターは、この情報を読むことで
MMD 本来の見た目・挙動を再構築できます。

> 剛体・ジョイント（物理）の変換済みビュー `physicsGltf` については
> 別ドキュメント **`physicsGltf_schema.md`** を参照してください。
> 本ドキュメントはそれ以外の全項目を扱います。

---

## extras.mmd に入る情報の2分類

| 分類 | 内容 | 例 |
|---|---|---|
| ① glTF に持って行けない MMD 固有情報 | glTF の PBR マテリアル・ノードに対応する概念が存在しないもの。extras に残さないと変換で失われる | スフィア、トゥーン、エッジ、ambient/specular、剛体、モーフ原本 |
| ② 不可逆加工の復元用データ | 表示互換性のためパイプラインがテクスチャや alphaMode を加工した際、「加工前の状態」に戻すための情報 | `origTexture`、`alphaClass` |

---

## 付与される場所（4か所）

```
glTF ルート  .extras.mmd   … モデル情報・モーフ原本・剛体/ジョイント原本・physicsGltf
materials[i].extras.mmd   … マテリアルの MMD 固有パラメータ ＋ α復元情報
nodes[i]    .extras.mmd   … ボーンの MMD 固有パラメータ（IK・付与親など）
（アニメーション焼き込み等は extras を持ちません）
```

`--no-extras` 指定時（`extras=False`）はいずれも出力されません。

---

## 1. ルート `extras.mmd`

```jsonc
"extras": { "mmd": {
  "coordinateNote": "...",      // 座標系の説明（下記参照）
  "unitScale": 0.08,            // 適用済みスケール
  "format": "pmx",
  "version": 2.0,               // PMX バージョン
  "name": "Tda式ミク・アペンド",  // モデル名（日本語）
  "nameEn": "...",              // モデル名（英語）
  "comment": "...",             // コメント欄（利用規約等が入ることが多い）
  "commentEn": "...",
  "morphs": [ ... ],            // モーフ原本（下記）
  "displayFrames": [ ... ],     // 表示枠（PMX raw）
  "rigidBodies": [ ... ],       // 剛体原本（PMX raw・未変換）
  "joints": [ ... ],            // ジョイント原本（PMX raw・未変換）
  "physicsGltf": { ... }        // 変換済み物理ビュー → physicsGltf_schema.md
}}
```

### 座標系の約束（coordinateNote）

`extras.mmd` 配下の数値は **PMX の生値**です。
MMD 左手系 (x, y, z)・**未スケール**のまま保持されます。
glTF 空間へは 位置 (x, y, -z)、クォータニオン (-x, -y, z, w) で変換し、
`unitScale` を乗じてください。

### morphs（モーフ原本）

```jsonc
{
  "name": "あ", "nameEn": "a",
  "panel": 3,                 // MMD の表示パネル (1=眉 2=目 3=口 4=その他)
  "type": 1,                  // PMX モーフ種別 (0=グループ 1=頂点 2=ボーン 3=UV 4-7=追加UV 8=材質)
  "target": 5,                // glTF morph target 番号（頂点/UVモーフのみ。無ければ null）
  "offsets": [ ... ]          // type が 1(頂点)/3(UV) 以外のとき、PMX raw オフセットを保持
}
```

頂点モーフ・UV モーフは glTF の morph target として本体に変換済みです。
グループ・ボーン・材質モーフは glTF に対応概念がないため、
raw オフセットを `offsets` に保持します（インポーター側で解釈してください）。

---

## 2. マテリアル `materials[i].extras.mmd`

```jsonc
"extras": { "mmd": {
  // ---- ① glTF に無い MMD 固有パラメータ ----
  "nameEn": "hairshadow",
  "ambient": [0.5, 0.5, 0.5],
  "specular": [0.0, 0.0, 0.0],
  "specularPower": 5.0,
  "flags": 0,                 // PMX 描画フラグ（両面/地面影/セルフ影/エッジ等のビット）
  "edgeColor": [0,0,0,1],
  "edgeSize": 1.0,
  "sphereMode": 0,            // 0=無効 1=乗算(sph) 2=加算(spa) 3=サブテクスチャ
  "sphereTexture": -1,        // glTF textures[] のインデックス（-1=なし）
  "toonTexture": -1,          // 個別トゥーンの glTF テクスチャインデックス（-1=なし）
  "toonShared": 3,            // 共有トゥーン番号 0始まり（0=toon01.bmp … 9=toon10.bmp、-1=個別）
  "memo": "",                 // PMX メモ欄

  // ---- ② α加工の復元用データ ----
  "alphaClass": "blend",      // このマテリアルの「本来の」α分類
  "origTexture": 14           // 無加工テクスチャの glTF インデックス（-1=標準テクスチャが無加工）
}}
```

### alphaClass / origTexture の詳細

パイプラインは、透過ソートに弱いビューアでも破綻しないよう、
半透明テクスチャを **prebake**（背景色と合成して不透明化）し
alphaMode=MASK で出力することがあります。この加工は不可逆なため、
正しくアルファブレンドできるインポーター向けに復元情報を残します。

| alphaClass | 意味 | インポーターの推奨挙動 |
|---|---|---|
| `"opaque"` | 実質不透明 | Opaque でよい |
| `"mask"`   | カットアウト（髪の房など二値的な抜き） | 基本は Cutout（cutoff は glTF の `alphaCutoff` を参照）。`origTexture` がある場合は無加工版を **Transparent（深度書き込みあり・TwoPass等）** で使うと柔らかい眉・透け髪まで再現できる（推奨） |
| `"blend"`  | 本来は半透明ブレンドが必要 | **`origTexture` を使い実アルファでブレンド**（Transparent） |

- `origTexture >= 0` のとき、glTF 内に prebake 前の**無加工テクスチャ**が
  `orig_<元ファイル名>` という名前で同梱されています。
  忠実な再現にはこちらを使ってください。
- `origTexture == -1` のときは `baseColorTexture` がそのまま無加工です。
- glTF 上の `alphaMode` はビューア互換性優先の値、
  `alphaClass` は「MMD 本来どう描くべきか」の値、と使い分けます。

### 半透明主体マテリアル（translucent）

メガネのレンズのように、**不透明ピクセルも持つが使用UV領域の過半（>50%）が
中間α**のマテリアルも、UV領域の実サンプリングで検出します
（Tda式ミクV4X 実測: lens=半透明率77%・平均α0.76、フレーム megane=46%）。

該当マテリアルは標準GLBでは **MASK＋prebakeのまま**（ビューア安全優先）ですが、
`alphaClass` が `"blend"` になります。対応インポーターは `origTexture` を使い
透明キューで実アルファブレンドしてください。

### 描画順（renderQueue）の推奨

MMD は**材質番号順**に描画・合成するため、モデル作者は重なりを材質順で
設計しています。インポーターは以下の2帯構成を推奨します
（Unity実装で実証済み）。

| alphaClass | 推奨キュー | 理由 |
|---|---|---|
| `"mask"`（origTextureで昇格） | AlphaTest帯 `2452＋材質番号` | 深度を書きつつ真の半透明より先に描画。透け髪・レンズ越しに正しく見える |
| `"blend"` | Transparent帯 `3000＋材質番号` | MMDの材質順合成を再現 |

全マテリアルを一律に透明キュー(3000)へ入れると、深度書き込みと
サブメッシュ順の組み合わせで「半透明の向こう側の後番号マテリアルが
消える」不具合が起きます（Tda式ミクV4Xで実測）。

### オーバーレイマテリアル（共有テクスチャの半透明領域）

額の髪影（hairshadow）やチークのように、**顔テクスチャの
半透明領域だけを使う専用マテリアル**は、テクスチャ単位の分類では
検出できないため、マテリアルが実際に使う UV 領域を実サンプリングして
判定します（使用領域の 95% 以上が α<0.5 かつ最大 α<0.6）。

該当マテリアルは glTF 上でも最初から
`alphaMode=BLEND` ＋ `baseColorTexture=origTexture`（無加工）で出力され、
`alphaClass` は `"blend"` になります。
インポーターは通常の `"blend"` 扱い（Transparent ＋ 実アルファ）で
そのまま正しく復元できます。

---

## 3. ボーンノード `nodes[i].extras.mmd`

```jsonc
"extras": { "mmd": {
  "nameEn": "left elbow",
  "flags": 27,                 // PMX ボーンフラグ（回転可/移動可/IK/付与/軸固定等のビット）
  "layer": 0,                  // 変形階層

  // ---- 以下はフラグに応じて存在する場合のみ ----
  "tail_bone": 12,             // 接続先ボーン番号（表示用）
  "tail_offset": [0, 1.2, 0],  // または接続先オフセット
  "inherit_parent": 8,         // 回転/移動付与の付与親ボーン番号
  "inherit_ratio": 0.5,        // 付与率
  "fixed_axis": [0,0,1],       // 軸固定の軸
  "local_x": [...], "local_z": [...],  // ローカル軸
  "external_key": 0,           // 外部親キー
  "ik": {                      // IK ボーンのみ
    "target": 34,              // IK ターゲットボーン
    "loop": 40,                // 演算回数
    "limit_angle": 1.0,        // 単位角（ラジアン）
    "links": [                 // IK リンク（min/max は角度制限があるリンクのみ）
      { "bone": 33, "min": [x,y,z], "max": [x,y,z] },
      { "bone": 32 }
    ]
  }
}}
```

座標値はルートの `coordinateNote` と同じく PMX raw です。
`--solve-ik`（既定 ON）の場合、IK はアニメーション焼き込みに使用済みですが、
実行時 IK を組み直したいエンジン向けに定義自体はここに残ります。

---

## バージョン互換について

- 既存キーの意味・座標系は変更しません（追加のみ）。
- インポーターは**未知のキーを無視**してください。
- `alphaClass` に `"blend"` 以外の新分類が将来増える可能性に備え、
  未知の値は `"mask"` 相当として扱うことを推奨します。
