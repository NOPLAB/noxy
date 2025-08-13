# GPU物理演算システム 技術選定書

## 概要

このドキュメントはGPU物理演算シミュレータ（noxy）の技術スタック選定理由と代替案検討結果をまとめています。

## 数学ライブラリ

### 主力ライブラリ: glam 0.30 (継続使用)

**選定理由**
- **既存コード継続性**: 現在`src/app/renderer/camera.rs`で使用中、移行コスト最小
- **GPU最適化**: SIMD命令（SSE2/AVX）による高速化、wgpu生態系との親和性
- **メモリ効率**: `bytemuck`対応でGPU↔CPUデータ転送がゼロコピー
- **型安全性**: コンパイル時の次元チェック、実行時エラー回避

```rust
glam = { version = "0.30", features = ["bytemuck", "serde"] }
```

**適用範囲**
- GPU演算（コンピュートシェーダー）
- リアルタイム描画パイプライン
- 基本物理演算（位置、速度、力）

### 補完ライブラリ: nalgebra 0.33

**選定理由**
- **高度数学機能**: SVD、QR分解、固有値解析
- **ロボティクス対応**: 高次元ヤコビアン、制約解決
- **精度選択**: f32/f64両対応、長時間シミュレーション対応

```rust
nalgebra = { version = "0.33", features = ["serde-serialize", "convert-glam"] }
```

**適用範囲**
- ロボット運動学/逆運動学
- 制約ソルバー（ジョイント、接触）
- 数値解析（安定性、収束判定）

**型変換戦略**
```rust
// glam ↔ nalgebra変換ユーティリティ
impl From<glam::Vec3> for nalgebra::Vector3<f32> {
    fn from(v: glam::Vec3) -> Self {
        nalgebra::Vector3::new(v.x, v.y, v.z)
    }
}
```

### 検討した代替案

| ライブラリ | 長所 | 短所 | 判定 |
|-----------|------|------|------|
| **cgmath** | 軽量、GPU特化 | 開発停滞、機能不足 | ❌ 却下 |
| **ultraviolet** | SIMD最適化 | 生態系小、安定性不明 | ❌ 却下 |
| **faer** | 大規模線形代数 | GPU統合未成熟 | ⏳ 将来検討 |

## GPU演算アーキテクチャ

### GPU API: WGPU 25.0 (継続使用)

**選定理由**
- **既存基盤活用**: 現在の描画システムと統合
- **クロスプラットフォーム**: Vulkan/DirectX/Metal統一API
- **Rust統合**: 型安全、ゼロコスト抽象化
- **コンピュートシェーダー**: 物理演算に必要な並列計算対応

**検討した代替案**
- **wgpu-hal**: 低レベル過ぎ、開発効率低下
- **ash (Vulkan直接)**: プラットフォーム依存、複雑性増加
- **CUDA**: NVIDIA専用、クロスプラットフォーム性失失

### シェーダー言語: WGSL

**選定理由**
- **WGPU標準**: APIとの一体性、最適化保証
- **可読性**: Rust類似文法、保守性向上
- **将来性**: WebGPU標準、ブラウザ対応

## 並列処理

### CPU並列: Rayon (継続使用)

**選定理由**
- **work-stealing**: 動的負荷分散、効率的
- **Rust統合**: データレース安全性、型チェック
- **シンプルAPI**: 学習コスト最小

```rust
rayon = "1.10"
```

## ロボティクス統合

### URDF パーサー: urdf-rs

**選定理由**
- **Pure Rust**: 依存関係最小、ビルド簡素
- **serde統合**: JSON/YAML変換容易

```rust
urdf-rs = "0.8"
```

### 制御システム: 自前実装

**選定理由**
- **特化最適化**: GPU物理演算と密結合
- **リアルタイム性**: 割り込み制御、決定的レイテンシ

## Python統合

### バインディング: PyO3 0.22

**選定理由**
- **Rust標準**: 型安全、メモリ安全
- **NumPy統合**: ゼロコピーデータ交換
- **非同期対応**: asyncio連携

```rust
pyo3 = { version = "0.22", features = ["extension-module", "abi3"] }
numpy = "0.22"
```

### 機械学習統合: Gymnasium

**選定理由**
- **標準準拠**: OpenAI Gym後継、エコシステム広範
- **型ヒント**: Python開発体験向上

## データ構造

### シリアライゼーション: serde

**選定理由**
- **Rust標準**: ゼロコスト、型安全
- **フォーマット豊富**: JSON/YAML/MessagePack対応

```rust
serde = { version = "1.0", features = ["derive"] }
serde_json = "1.0"
```

### 設定管理: config + clap

**選定理由**
```rust
config = "0.14"  # 階層設定ファイル
clap = { version = "4.5", features = ["derive"] }  # CLI引数
```

## メモリ管理

### GPU転送: bytemuck

**選定理由**
- **ゼロコスト変換**: CPU↔GPU間効率的転送
- **glam統合**: 数学型の直接GPU転送

```rust
bytemuck = { version = "1.16", features = ["derive"] }
```

## 開発ツール

### ログ: tracing

**選定理由**
- **構造化ログ**: デバッグ効率向上
- **非同期対応**: 性能影響最小

```rust
tracing = "0.1"
tracing-subscriber = "0.3"
```

### エラーハンドリング: anyhow + thiserror

**選定理由**
```rust
anyhow = "1.0"      # アプリケーションエラー
thiserror = "1.0"   # ライブラリエラー定義
```

## 性能測定

### プロファイリング: criterion

**選定理由**
- **統計的ベンチマーク**: 信頼性高い性能測定
- **回帰検出**: CI/CD統合、性能劣化防止

```rust
[dev-dependencies]
criterion = { version = "0.5", features = ["html_reports"] }
```

## 総合技術スタック

```toml
[dependencies]
# コア数学・GPU
glam = { version = "0.30", features = ["bytemuck", "serde"] }
nalgebra = { version = "0.33", features = ["serde-serialize", "convert-glam"] }
wgpu = "25.0"
bytemuck = { version = "1.16", features = ["derive"] }

# 並列・システム
rayon = "1.10"
anyhow = "1.0"
thiserror = "1.0"

# データ・設定
serde = { version = "1.0", features = ["derive"] }
serde_json = "1.0"
config = "0.14"
clap = { version = "4.5", features = ["derive"] }

# ロボティクス
urdf-rs = "0.8"

# ログ・測定
tracing = "0.1"
tracing-subscriber = "0.3"

# Python統合
pyo3 = { version = "0.22", features = ["extension-module", "abi3"] }
numpy = "0.22"

[dev-dependencies]
criterion = { version = "0.5", features = ["html_reports"] }
proptest = "1.4"
```

## 設計原則

1. **既存基盤活用**: 現在のwgpu/glam構成を最大限活用
2. **段階的移行**: 破壊的変更最小、漸進的機能追加
3. **性能ファースト**: GPU並列性を最優先、抽象化コスト最小
4. **型安全性**: Rust型システム活用、実行時エラー撲滅
5. **エコシステム統合**: 科学計算・ML分野との自然な連携

この技術選定により、高性能・高品質・保守性を兼ね備えたGPU物理演算システムを構築します。