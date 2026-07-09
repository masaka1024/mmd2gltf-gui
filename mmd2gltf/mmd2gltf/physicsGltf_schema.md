# `extras.mmd.physicsGltf` スキーマ

mmd2gltf は、MMD（PMX）の剛体・ジョイント情報を glTF 出力の `extras.mmd` に残します。
そのうち **`physicsGltf`** は、後工程（Unity / Unreal Engine など）で物理を組む人が
**そのまま使える形に変換済み**のビューです。座標系・単位・向きを自前で変換する必要はありません。

> glTF には実行時物理がありません。mmd2gltf は髪やスカートの「揺れ」を焼き込まず、
> 物理定義をこの `physicsGltf` として残します。エンジン側で揺れものを構築してください。
> （キーフレームに焼き込みたい場合は `--bake-hair` オプションを使用します。）

---

## 生データとの関係

`extras.mmd` には2種類が併存します。用途で使い分けてください。

| キー | 座標系 | 用途 |
|---|---|---|
| `rigidBodies` / `joints` | **MMD左手系・未変換・未スケール**（`coordinateNote`参照） | 原本の完全保持。忠実な再構築が必要な人向け |
| **`physicsGltf`** | **glTF系・スケール済み・ボーンローカル** | エンジンで物理を組む人向け（推奨） |

両者は同じ剛体・ジョイントを指します。`physicsGltf` は生データを一切上書きせず、追加されるだけです。

---

## トップレベル

```jsonc
"extras": { "mmd": {
  "physicsGltf": {
    "space": "boneLocal",       // 既定の基準空間
    "unitScale": 0.08,          // 適用済みスケール（参考値）
    "eulerOrderSource": "YXZ",  // 元PMX剛体回転のオイラー順序（監査用）
    "note": "...",              // 座標系・フォールバックの説明
    "rigidBodies": [ ... ],
    "joints": [ ... ]
  }
}}
```

- **`space`** … 全体の既定基準。個々のエントリも `space` を持ち、そちらが優先。
- **角度はすべてラジアン**、**距離は glTF シーン単位**（`unitScale` 適用済み）。

---

## `rigidBodies[]`

```jsonc
{
  "name": "左後髪1＿1",
  "bone": 170,              // 紐づくボーンの node index（-1 = ボーン無し）
  "space": "boneLocal",     // "boneLocal" | "world"
  "shape": 2,               // 0=球 / 1=箱 / 2=カプセル
  "mode": 2,                // 0=ボーン追従 / 1=物理 / 2=物理+ボーン
  "group": 4,               // 衝突グループ 0..15
  "noCollisionMask": 65507, // 非衝突グループのビットマスク（16bit）
  "size": [0.0061, 0.0305, 0.0],          // shape依存（下記）
  "position": [0.0131, 0.0078, -0.0008],  // space基準の位置
  "rotation": [-0.0218, 0.0123, -0.4922, 0.8701], // space基準の quat (x,y,z,w)
  "mass": 10.0,
  "linearDamping": 1.0,     // 並進減衰
  "angularDamping": 1.0,    // 回転減衰
  "restitution": 0.0,       // 反発
  "friction": 0.0
}
```

### `space`
- `"boneLocal"` … `position`/`rotation` は **`bone` のローカル空間**。
  エンジンでそのボーンの子オブジェクトを作り、`localPosition`/`localRotation` にそのまま代入できる。
- `"world"` … `bone == -1` の剛体のフォールバック。glTF シーンのワールド空間。

### `shape` と `size` の意味
| shape | 形状 | size の意味 |
|---|---|---|
| 0 | 球 | `size[0]` = 半径（`size[1]`,`[2]` 未使用） |
| 1 | 箱 | `[hx, hy, hz]` = **半寸法**（ローカル各軸方向） |
| 2 | カプセル | `size[0]` = 半径, `size[1]` = 高さ, `size[2]` 未使用。**長軸 = ローカル Y** |

### `mode`
| mode | 挙動 | エンジンでの扱い |
|---|---|---|
| 0 | ボーン追従（物理演算しない） | Kinematic。ボーンに固定 |
| 1 | 物理 | Dynamic。物理で動きボーンを駆動 |
| 2 | 物理＋ボーン | Dynamic だがボーン位置に補正あり |

---

## `joints[]`

すべて `type: 0`（6DOF スプリング拘束）です。2つの剛体 A・B を繋ぎます。

```jsonc
{
  "name": "左後髪1＿2",
  "type": 0,
  "rigidA": 16,             // physicsGltf.rigidBodies[] のインデックス
  "rigidB": 21,
  "space": "boneLocal",     // 剛体B側のボーンローカルで統一（子側基準）
  "refBone": 171,           // 基準ボーンの node index（= 剛体Bのbone、-1でworld）
  "position": [ ... ],      // space基準
  "rotation": [ x,y,z,w ],  // space基準
  "linearLimitMin": [0,0,0],      // 並進制限（glTF単位）
  "linearLimitMax": [0,0,0],
  "angularLimitMin": [-0.785, 0, -0.785],  // 回転制限（ラジアン）
  "angularLimitMax": [ 0.785, 0,  0.785],
  "springPosition": [0,0,0],      // 並進ばね定数（glTF単位）
  "springRotation": [0,0,0]       // 回転ばね定数
}
```

- **`rigidA`/`rigidB`** … `physicsGltf.rigidBodies` 配列のインデックス（`rigidBodies` と同順）。
- **`angularLimitMin/Max`** … ラジアン。glTF系への鏡映変換（z反転）に伴い、X/Y軸は符号反転、Z軸はそのまま（変換済み）。
- **制限が `min == max == 0` の軸** … その軸は**固定（自由度なし）**。
  例えばチェーン根元のジョイントは全軸0で、髪をアンカーする役割。

---

## Unity での組み方（例）

1. glTF/GLB をインポートし、スケルトンを取得。
2. 各剛体について、`bone`（node index）に対応する `Transform` を探す。
3. その子に GameObject を作り、`localPosition = position` / `localRotation = rotation`。
4. Collider を付ける：
   - 球 → `SphereCollider`（`radius = size[0]`）
   - 箱 → `BoxCollider`（`size = size * 2`、**半寸法→全寸法に注意**）
   - カプセル → `CapsuleCollider`（`radius = size[0]`, `height = size[1]`, `direction = Y-Axis`）
5. `mode >= 1` なら `Rigidbody` を追加：`mass`, `drag = linearDamping`, `angularDrag = angularDamping`。
   `mode == 0` は `isKinematic = true`。
6. Joint は `ConfigurableJoint` を `rigidA`↔`rigidB` 間に設定。
   角度制限は **ラジアン→度**（`* 180/π`）に変換して `angularXLimit` 等へ。

> 注意：Unity の `BoxCollider.size` は全寸法、`physicsGltf` の箱 `size` は半寸法です。2倍してください。

## Unreal Engine での組み方（概要）

Skeletal Mesh の Physics Asset で、各 `bone` に対応するボーンへ Body（Collider）を追加し、
`shape`/`size`/`position`/`rotation` を設定。Joint は Physics Constraint で `rigidA`↔`rigidB` を接続し、
角度制限を度に変換して適用します。

---

## 参考：座標変換の定義（監査用）

`physicsGltf` は生データ（`rigidBodies`/`joints`）を次の規則で変換した結果です。

- 位置：`(x, y, -z) × unitScale`
- 回転：`euler(x, y, z ラジアン, order=YXZ) → quat → (-x, -y, z, w)`
- サイズ：`× unitScale`（スカラー寸法、符号反転なし）
- 角度制限：z反転の鏡映により X/Y 軸は符号反転＋min/max入替、Z 軸は不変
- ボーンローカル化：`offset = inverse(boneWorldMatrix) × rigidWorldMatrix`

`eulerOrderSource: "YXZ"` は本ツールが確定した順序です。生データから自前で再変換する場合はこの順序を使用してください。
