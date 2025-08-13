# Noxy プロジェクトファイル構成設計

## 全体構成

```
noxy/
├── src/                           # Rustコア実装
├── python/                        # Pythonパッケージ群
├── tests/                         # テストスイート
├── examples/                      # 使用例・サンプル
├── assets/                        # リソースファイル
├── docs/                          # ドキュメント
├── benchmarks/                    # 性能測定
├── scripts/                       # ビルド・開発スクリプト
└── configs/                       # 設定ファイル
```

## 設計の主要原則

1. **モジュラー設計**: 各コンポーネントが独立してテスト・開発可能
2. **バックエンド抽象化**: CPU/GPU実装の完全分離
3. **API層分離**: 低レベル直接制御と高レベル抽象化の明確な境界
4. **段階的開発**: CPU実装→GPU実装→Python統合の段階的構築
5. **テスト駆動**: 各レイヤーでの包括的テストカバレッジ

## 開発フェーズ対応

- Phase 1: CPU実装 (`physics/backends/cpu/`)
- Phase 2: GPU実装とバックエンド統合 (`physics/backends/gpu/`, `physics/backends/abstraction.rs`)
- Phase 3: Python統合 (`python/` 全体)
- Phase 4-5: 最適化と機能拡張

この構成により、各フェーズで必要な部分のみを実装しながら、全体の一貫性を保つことができます。