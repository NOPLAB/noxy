# GPU物理演算シミュレータ アーキテクチャ設計

## 1. プロジェクト概要

### 目的
- GPU上での高性能剛体動力学シミュレーション
- リアルタイム3D可視化とヘッドレス計算の両対応
- ロボティクス・制御システム研究への応用
- Pythonバインディングによる科学計算エコシステム統合

### 技術スタック
- **コア**: Rust + WGPU 25.0 + winit 0.30
- **演算バックエンド**: CPU (Rayon並列) + GPU (WGPU Compute)
- **数学**: glam 0.30 + bytemuck
- **Python統合**: PyO3 + NumPy
- **CLI**: clap + serde

### システム要件
- **物理演算**: 剛体動力学、ソフトボディ（将来拡張）
- **ロボティクス対応**: ジョイント機構、摩擦モデル、アクチュエータ
- **現実的シミュレーション**: 接触力学、材料特性、環境相互作用
- **演算バックエンド**: CPU/GPU自動選択、フォールバック機能
- **可視化**: リアルタイム3D描画、カメラ制御
- **性能**: 1,000+ 剛体 or 複雑ロボット機構の60FPS動作

## 2. アーキテクチャ設計

### プロジェクト構成

```
noxy/
├── Cargo.toml                     # Rustプロジェクト設定
├── pyproject.toml                 # Python統合設定
├── CLAUDE.md                      # プロジェクト開発指針
├── GPU_PHYSICS_ARCHITECTURE.md   # アーキテクチャ設計書
│
├── src/                           # Rustコア実装
│   ├── main.rs                    # エントリーポイント・CLI
│   ├── lib.rs                     # ライブラリルート
│   ├── app.rs                     # アプリケーション管理
│   ├── app/                       # アプリケーション層
│   │   ├── modes.rs               # 実行モード切り替え
│   │   ├── interactive.rs         # 対話モード実装
│   │   └── headless.rs            # ヘッドレスモード実装
│   │
│   ├── physics.rs                 # 物理システム統合
│   ├── physics/                   # 物理演算システム
│   │   ├── core.rs                # コア物理実装統合
│   │   ├── core/                  # コア物理実装
│   │   │   ├── rigidbody.rs       # 剛体定義・動力学
│   │   │   ├── shapes.rs          # 形状定義（Box、Sphere、Mesh等）
│   │   │   ├── forces.rs          # 力計算
│   │   │   ├── integration.rs     # 数値積分
│   │   │   ├── collision.rs       # 衝突検出・応答
│   │   │   ├── constraints.rs     # 制約システム
│   │   │   ├── joints.rs          # ジョイント機構
│   │   │   ├── friction.rs        # 摩擦モデル
│   │   │   └── materials.rs       # 材料特性
│   │   ├── backends.rs            # バックエンド抽象化
│   │   ├── backends/              # 演算バックエンド
│   │   │   ├── traits.rs          # 統一インターフェース
│   │   │   ├── cpu.rs             # CPU実装統合 (Phase 1)
│   │   │   ├── cpu/               # CPU実装詳細
│   │   │   │   ├── parallel.rs    # Rayon並列処理
│   │   │   │   ├── simd.rs        # SIMD最適化
│   │   │   │   └── spatial.rs     # 空間分割
│   │   │   ├── gpu.rs             # GPU実装統合 (Phase 2)
│   │   │   └── gpu/               # GPU実装詳細
│   │   │       ├── compute.rs     # コンピュートシェーダー
│   │   │       ├── buffers.rs     # GPU バッファ管理
│   │   │       └── pipeline.rs    # GPU パイプライン
│   │   ├── scene.rs               # シーン・設定管理統合
│   │   ├── robotics.rs            # ロボティクス統合
│   │   ├── robotics/              # ロボティクス特化機能
│   │   │   ├── robot.rs           # ロボット定義
│   │   │   ├── actuators.rs       # アクチュエータ
│   │   │   ├── sensors.rs         # センサー
│   │   │   ├── kinematics.rs      # 運動学
│   │   │   └── dynamics.rs        # 動力学
│   │   ├── scene.rs               # シーン・設定管理統合
│   │   └── scene/                 # シーン・設定管理
│   │       ├── config.rs          # シーン設定
│   │       ├── loader.rs          # 設定読み込み
│   │       └── urdf.rs            # URDF読み込み
│   │
│   ├── render.rs                  # 描画統合
│   ├── render/                    # 描画システム
│   │   ├── renderer.rs            # メイン描画
│   │   ├── camera.rs              # カメラシステム
│   │   ├── particles.rs           # パーティクル描画
│   │   └── ui.rs                  # UI描画
│   │
│   ├── api.rs                     # API統合
│   ├── api/                       # API層設計
│   │   ├── low_level.rs           # 直接制御API
│   │   ├── high_level.rs          # 抽象化API
│   │   └── rl_interface.rs        # RL環境インターフェース
│   │
│   ├── bindings.rs                # バインディング統合
│   ├── bindings/                  # 言語バインディング
│   │   ├── python.rs              # Python統合統合 (Phase 3)
│   │   ├── python/                # Python統合詳細
│   │   │   ├── core.rs            # 低レベルPyO3
│   │   │   ├── rl_env.rs          # RL環境ラッパー
│   │   │   └── analysis.rs        # 解析機能
│   │   └── c_api.rs               # C API (将来拡張)
│   │
│   ├── io.rs                      # 入出力統合
│   ├── io/                        # 入出力システム
│   │   ├── input.rs               # 入力処理
│   │   ├── output.rs              # ファイル出力
│   │   └── formats.rs             # データフォーマット
│   │
│   ├── utils.rs                   # 共通ユーティリティ統合
│   └── utils/                     # 共通ユーティリティ
│       ├── math.rs                # 数学関数
│       ├── profiler.rs            # パフォーマンス測定
│       └── logger.rs              # ログ機能
│
├── python/                        # Pythonパッケージ群
│   ├── noxy_physics/              # 低レベル物理API
│   │   ├── __init__.py
│   │   ├── simulation.py          # メインシミュレーション
│   │   ├── particles.py           # パーティクル操作
│   │   └── backends.py            # バックエンド選択
│   ├── noxy_rl_env/              # Gymnasium互換RL環境
│   │   ├── __init__.py
│   │   ├── environments/          # 各種環境定義
│   │   │   ├── __init__.py
│   │   │   ├── manipulation.py    # 操作タスク
│   │   │   ├── locomotion.py      # 移動タスク
│   │   │   └── fluid_control.py   # 流体制御
│   │   ├── spaces.py              # 観測・行動空間
│   │   ├── rewards.py             # 報酬関数
│   │   └── multi_agent.py         # マルチエージェント
│   └── noxy_analysis/            # 解析・可視化ツール
│       ├── __init__.py
│       ├── physics_analyzer.py    # 物理量解析
│       ├── visualization.py       # 可視化機能
│       └── conservation.py        # 保存則チェック
│
├── tests/                         # テストスイート
│   ├── unit/                      # 単体テスト
│   │   ├── physics/               # 物理演算テスト
│   │   ├── backends/              # バックエンドテスト
│   │   └── api/                   # API テスト
│   ├── integration/               # 統合テスト
│   │   ├── cpu_gpu_consistency/   # CPU/GPU一致性
│   │   ├── python_bindings/       # Python統合
│   │   └── rl_environments/       # RL環境テスト
│   ├── physics/                   # 物理法則検証
│   │   ├── conservation/          # 保存則テスト
│   │   ├── stability/             # 数値安定性
│   │   └── accuracy/              # 精度検証
│   └── benchmarks/                # 性能テスト
│       ├── cpu_performance/       # CPU性能測定
│       ├── gpu_performance/       # GPU性能測定
│       └── scaling/               # スケーラビリティ
│
├── examples/                      # 使用例・サンプル
│   ├── basic/                     # 基本例
│   │   ├── simple_simulation.rs   # 基本シミュレーション
│   │   └── backend_comparison.rs  # バックエンド比較
│   ├── python/                    # Python使用例
│   │   ├── low_level_control.py   # 低レベルAPI例
│   │   ├── rl_training_basic.py   # 基本RL訓練
│   │   └── physics_analysis.py    # 物理解析例
│   ├── rl_training/               # 強化学習訓練例
│   │   ├── manipulation_task.py   # 操作タスク訓練
│   │   ├── multi_agent_env.py     # マルチエージェント
│   │   └── custom_reward.py       # カスタム報酬
│   └── research/                  # 研究用途例
│       ├── phase_space_analysis.py # 位相空間解析
│       ├── conservation_study.py   # 保存則研究
│       └── scaling_benchmark.py    # スケーリング研究
│
├── assets/                        # リソースファイル
│   ├── shaders/                   # WGSLシェーダー
│   │   ├── rigidbody_render.wgsl  # 剛体描画
│   │   ├── compute_dynamics.wgsl  # 剛体動力学計算
│   │   ├── integrate_motion.wgsl  # 運動積分
│   │   ├── collision_detect.wgsl  # 衝突検出
│   │   ├── contact_resolve.wgsl   # 接触解決
│   │   └── joint_solver.wgsl      # ジョイント解決
│   ├── robots/                    # ロボットモデル
│   │   ├── ur5.urdf               # UR5ロボットアーム
│   │   ├── quadruped.urdf         # 四足歩行ロボット
│   │   ├── humanoid.urdf          # ヒューマノイド
│   │   └── custom_robot.urdf      # カスタムロボット例
│   ├── scenes/                    # シーン設定
│   │   ├── basic_rigidbodies.json # 基本剛体シミュレーション
│   │   ├── collision_test.json    # 衝突テスト
│   │   ├── stacking_demo.json     # 積み上げデモ
│   │   ├── robot_workspace.json   # ロボット作業空間
│   │   └── multi_robot.json       # マルチロボット環境
│   └── rl_tasks/                  # 強化学習タスク定義
│       ├── manipulation.json      # 操作タスク設定
│       ├── locomotion.json        # 移動タスク設定
│       ├── robot_control.json     # ロボット制御タスク
│       ├── rigidbody_stacking.json # 剛体積み上げタスク
│       └── custom_env.json        # カスタム環境例
│
├── docs/                          # ドキュメント
│   ├── api/                       # API ドキュメント
│   ├── tutorials/                 # チュートリアル
│   ├── physics/                   # 物理理論解説
│   └── development/               # 開発者向け
│
├── scripts/                       # ビルド・開発スクリプト
│   ├── build_python.sh            # Python バインディングビルド
│   ├── run_benchmarks.sh          # ベンチマーク実行
│   ├── test_all.sh                # 全テスト実行
│   └── setup_dev_env.sh           # 開発環境セットアップ
│
└── configs/                       # 設定ファイル
    ├── simulation_defaults.json   # シミュレーション既定値
    ├── gpu_settings.json          # GPU設定
    ├── cpu_optimization.json      # CPU最適化設定
    └── rl_environment_configs.json # RL環境設定
```

### 実行モード

#### 3D描画モード (Interactive)
- リアルタイム可視化、ユーザー入力対応
- 60FPS描画ターゲット

#### ヘッドレスモード (Compute-Only)
- 高速計算、バッチ処理、サーバー環境
- GPU計算リソース100%活用、ファイル出力

## 3. 主要コンポーネント

### 物理演算システム
- **剛体動力学**: 回転・並進運動、慣性テンソル、角運動量保存
- **形状サポート**: Box、Sphere、Cylinder、Convex Hull、Triangle Mesh
- **ロボティクス機能**: ジョイント機構、摩擦モデル、アクチュエータ制御
- **現実的物理**: 接触力学、材料特性、環境相互作用
- **演算バックエンド抽象化**: 統一インターフェースによるCPU/GPU切り替え
- **CPU演算**: Rayon並列処理による高効率CPU計算
- **GPU演算**: WGPU コンピュートシェーダーによる大規模並列処理
- **自動選択**: 複雑度、GPU可用性に基づく最適バックエンド選択

### 描画システム  
- **剛体描画**: Box、Sphere、複雑形状の効率的描画
- **ロボット機構描画**: ジョイント、リンク、アクチュエータの可視化
- **リアリスティック描画**: 材料質感、照明、影の表現
- **カメラシステム**: 3D透視投影、WASD制御

### 実行モードシステム
- **Interactive**: ウィンドウ + 描画 + 入力
- **Headless**: 純粋計算 + ファイル出力（CPU/GPU両対応）

### Pythonバインディング
- **PyO3統合**: Rustコア + Pythonインターフェース  
- **NumPy配列**: ゼロコピーデータ取得
- **二層API設計**: 低レベル直接制御 + 高レベル強化学習対応
- **Gym互換**: OpenAI Gym/Gymnasium環境インターフェース

## 4. 演算バックエンド設計

### バックエンド抽象化
- **統一インターフェース**: CPU/GPU実装の完全な互換性
- **自動フォールバック**: GPU利用不可時のCPU自動切り替え
- **性能プロファイリング**: 実行時最適バックエンド選択

### CPU演算 (Phase 1実装)
- **Rayon並列処理**: ワークスチール型並列実行
- **SIMD最適化**: 数値計算の高速化
- **キャッシュ効率**: メモリアクセスパターン最適化

### GPU演算 (Phase 2実装)
- **コンピュートシェーダー**: WGPU大規模並列処理
- **メモリ効率**: GPU最適化データレイアウト
- **ワークグループ調整**: ハードウェア特性適応

## 5. 使用方法

### コマンドライン
```bash
# 3D描画モード
cargo run

# ヘッドレスモード (バックエンド自動選択)
cargo run -- --headless --output results/ --rigidbodies 500

# バックエンド指定
cargo run -- --backend cpu --rigidbodies 100
cargo run -- --backend gpu --rigidbodies 1000

# ロボットシミュレーション
cargo run -- --robot assets/robots/ur5.urdf --task manipulation
cargo run -- --headless --robot quadruped.urdf --physics-fidelity high
```

### Python API

#### 低レベルAPI (Direct Simulation Control)
```python
import noxy_physics

# 直接シミュレーション制御 (バックエンド選択可能)
sim = noxy_physics.PySimulation(
    rigidbody_count=100,
    backend="auto"  # "cpu", "gpu", "auto"
)
sim.set_gravity([0.0, -9.81, 0.0])

# 剛体作成・制御
box_id = sim.create_rigidbody(
    shape="box", 
    size=[1.0, 2.0, 0.5],
    position=[0.0, 5.0, 0.0],
    mass=10.0
)

# 詳細な物理パラメータ制御
sim.set_rigidbody_properties(
    indices=[0, 1, 2],
    masses=[1.0, 2.0, 0.5],
    materials=["steel", "wood", "rubber"],
    friction_coefficients=[0.7, 0.4, 0.9]
)

# 低レベルバッファアクセス (バックエンド透過的)
positions = sim.get_positions_buffer()      # 位置データ
orientations = sim.get_orientations_buffer() # 回転データ（クォータニオン）
velocities = sim.get_velocities_buffer()     # 線速度
angular_velocities = sim.get_angular_velocities_buffer()  # 角速度
forces = sim.get_forces_buffer()             # 力・トルク

# バックエンド情報とパフォーマンス
print(f"Active backend: {sim.get_active_backend()}")
print(f"Performance: {sim.get_performance_stats()}")

# 単一ステップ実行と詳細制御
sim.compute_forces()
sim.integrate_motion(dt=0.016)
sim.resolve_collisions()
```

#### 高レベルAPI (RL Environment Interface)
```python
import noxy_rl_env
import gymnasium as gym

# Gymnasium互換環境 (ロボティクス対応)
env = noxy_rl_env.RobotEnv(
    robot_urdf="assets/robots/ur5.urdf",
    task="manipulation",           # manipulation, locomotion, navigation
    observation_type="joint_states", # joint_states, images, forces, combined
    action_space="continuous",     # continuous, discrete, hybrid
    physics_fidelity="medium",     # low, medium, high, ultra
    contact_model="realistic",     # simple, realistic, advanced
    backend="auto"                 # CPU小規模, GPU大規模自動選択
)

# 剛体シミュレーション環境
rigidbody_env = noxy_rl_env.RigidbodyEnv(
    rigidbody_count=50,
    task="stacking",           # stacking, manipulation, physics_puzzle
    shapes=["box", "sphere", "cylinder"]
)

# 標準的なRL訓練ループ
obs, info = env.reset(seed=42)
for step in range(1000):
    action = agent.get_action(obs)
    obs, reward, terminated, truncated, info = env.step(action)
    
    if terminated or truncated:
        obs, info = env.reset()

# マルチエージェント対応
multi_env = noxy_rl_env.MultiAgentParticleEnv(
    agents=["agent_0", "agent_1", "agent_2"],
    action_spaces={
        "agent_0": "continuous_3d",  # 3D力制御
        "agent_1": "discrete_push",  # 離散的押し動作
        "agent_2": "constraint_manipulation"  # 制約操作
    }
)

# カスタム報酬関数
@noxy_rl_env.reward_function
def custom_reward(state, action, next_state):
    # エネルギー効率 + タスク達成度
    energy_penalty = np.sum(action**2) * 0.01
    task_reward = compute_task_progress(next_state)
    return task_reward - energy_penalty

env.set_reward_function(custom_reward)
```

#### ハイブリッドAPI (Research & Analysis)
```python
import noxy_physics
import noxy_analysis

# 研究用統合環境
research_env = noxy_physics.ResearchSimulation(
    config_file="experiments/complex_fluid.json"
)

# 詳細な解析機能
analyzer = noxy_analysis.PhysicsAnalyzer(research_env)

# 物理量の時系列解析
analyzer.track_conservation_laws(steps=10000)
energy_data = analyzer.get_energy_timeline()
momentum_data = analyzer.get_momentum_timeline()

# 位相空間解析
phase_portrait = analyzer.generate_phase_portrait(
    particles=[0, 1, 2],
    dimensions=["position_x", "velocity_x"]
)

# カスタム観測量定義
@noxy_analysis.observable
def cluster_coefficient(particles):
    return compute_clustering_metric(particles.positions)

analyzer.add_observable("clustering", cluster_coefficient)
time_series = analyzer.compute_observables(steps=5000)
```

## 6. テスト駆動開発戦略

### テスト設計思想
- **TDD (Test Driven Development)**: 実装前にテスト作成、Red-Green-Refactorサイクル
- **物理法則検証**: エネルギー保存、運動量保存、数値安定性の自動検証
- **性能回帰防止**: ベンチマークテストによる性能劣化の早期発見
- **クロスプラットフォーム**: GPU環境差異の自動検出とフォールバック

### テスト階層

#### 1. 単体テスト (Unit Tests)
```rust
#[cfg(test)]
mod tests {
    // 数学関数テスト
    #[test] fn test_verlet_integration() { ... }
    #[test] fn test_collision_detection() { ... }
    #[test] fn test_spatial_partitioning() { ... }
}
```

#### 2. 統合テスト (Integration Tests)
```rust
// GPU計算パイプライン全体テスト
#[test] fn test_headless_simulation_pipeline() { ... }
#[test] fn test_interactive_rendering_pipeline() { ... }
#[test] fn test_python_binding_data_consistency() { ... }
#[test] fn test_rl_environment_gym_compliance() { ... }
#[test] fn test_low_level_high_level_api_consistency() { ... }
```

#### 3. 物理法則検証テスト
```rust
#[test] fn test_energy_conservation() {
    // 孤立系での全エネルギー保存確認
}
#[test] fn test_momentum_conservation() {
    // 衝突前後の運動量保存確認
}
#[test] fn test_numerical_stability() {
    // 長時間シミュレーションでの数値安定性
}
```

#### 4. 性能テスト (Benchmark Tests)
```rust
#[bench] fn bench_particle_update_1k() { ... }
#[bench] fn bench_collision_detection_10k() { ... }
#[bench] fn bench_gpu_cpu_transfer() { ... }
```

#### 5. プロパティベーステスト
```rust
use proptest::prelude::*;

proptest! {
    #[test]
    fn particle_position_bounded(
        initial_pos in prop::array::uniform3(-100.0f32..100.0),
        steps in 1u32..1000
    ) {
        // ランダム初期条件での境界条件テスト
    }
}
```

### テスト環境とCI/CD

#### GitHub Actions設定
```yaml
# .github/workflows/tests.yml
name: Physics Tests
on: [push, pull_request]
jobs:
  test:
    strategy:
      matrix:
        os: [ubuntu-latest, windows-latest, macos-latest]
        gpu: [software, hardware]
    steps:
      - name: GPU Physics Tests
        run: cargo test --features gpu-tests
      - name: Benchmark Regression
        run: cargo bench --baseline main
```

#### テストデータ管理
```
tests/
├── unit/           # 単体テスト
├── integration/    # 統合テスト  
├── physics/        # 物理法則検証
├── benchmarks/     # 性能テスト
├── fixtures/       # テストデータ
│   ├── scenarios/  # 物理シナリオ
│   └── expected/   # 期待値データ
└── gpu_compat/     # GPU互換性テスト
```

### 開発ワークフロー

#### TDDサイクル実装例
```rust
// 1. Red: 失敗するテストを書く
#[test]
fn test_particle_collision_response() {
    let mut sim = Simulation::new();
    let p1 = Particle::new([0.0, 0.0, 0.0], [1.0, 0.0, 0.0]);
    let p2 = Particle::new([2.0, 0.0, 0.0], [-1.0, 0.0, 0.0]);
    
    sim.add_particle(p1);
    sim.add_particle(p2);
    sim.step(1.0);
    
    // 衝突後の速度が物理法則に従うことを検証
    assert_collision_physics_valid(&sim.particles);
}

// 2. Green: 最小限の実装でテストを通す
impl Simulation {
    pub fn detect_collisions(&mut self) {
        // 最小限の衝突検出実装
    }
}

// 3. Refactor: コードを改善
impl Simulation {
    pub fn detect_collisions(&mut self) {
        // 効率的な空間分割アルゴリズム実装
    }
}
```

## 7. 開発フェーズ (TDD対応)

### Phase 1: CPU基盤構築 + 基本テスト
- **TDD**: コア物理数学関数の単体テスト作成
- **実装**: CPU演算、基本パーティクル、モード切り替え
- **検証**: 単体テスト全通過、CPU基本シミュレーション動作

### Phase 2: GPU実装 + バックエンド抽象化
- **TDD**: CPU/GPU結果一致検証、バックエンド切り替えテスト
- **実装**: GPU計算パイプライン、統一インターフェース、自動選択
- **検証**: 両バックエンド同一結果、性能比較、フォールバック機能

### Phase 3: Pythonバインディング + 二層API設計
- **TDD**: Python-Rust間データ整合性、API一貫性テスト
- **実装**: 低レベル直接制御API、Gymnasium互換RL環境API
- **検証**: クロスランゲージ動作、ゼロコピー性能、RL標準準拠確認

### Phase 4: 最適化 + 回帰防止
- **TDD**: 性能回帰テスト、GPU互換性テスト拡充
- **実装**: パフォーマンス向上、高度物理機能、UI
- **検証**: ベンチマーク達成、全環境動作保証

### Phase 4: ロボティクス機能 + 高精度物理
- **TDD**: ジョイント機構、摩擦モデルの物理正確性テスト
- **実装**: ロボット機構、URDF対応、現実的接触力学
- **検証**: 実ロボットとの動作比較、精度検証

### Phase 5: 機能拡張 + 継続検証
- **TDD**: 新機能の物理正確性テスト
- **実装**: 柔体・流体シミュレーション、高度制御システム
- **検証**: 複雑系での物理法則維持、スケーラビリティ確認

## 8. 技術的利点

### アーキテクチャ
- **モード分離**: 描画と計算の独立開発、最適なリソース配分
- **バックエンド抽象化**: CPU/GPU統一インターフェース、自動最適化
- **段階的開発**: CPU先行実装によるリスク軽減、GPU後続追加
- **スケーラビリティ**: 小規模CPU～大規模GPU対応

### Python統合  
- **科学計算エコシステム**: NumPy、SciPy、Matplotlib、Jupyter
- **機械学習統合**: PyTorch、TensorFlow連携、Gymnasium標準準拠
- **二層API設計**: 研究用精密制御 + RL用高レベル抽象化
- **開発効率**: Pythonプロトタイピング + Rust性能

### テスト駆動開発
- **品質保証**: 物理法則の数学的正確性を自動検証
- **回帰防止**: 性能劣化やバグ混入の早期発見
- **継続的改善**: リファクタリング時の安全性確保
- **クロスプラットフォーム**: 異なるGPU環境での動作保証

この設計により研究用途から商用まで対応可能な高品質・高性能GPU物理演算システムが構築できます。