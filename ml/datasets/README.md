# LunaBot dataset storage

Do not commit generated image datasets to ordinary Git. The default local
location is repository-root `datasets/`, which is ignored. Use DVC, Git LFS, or
versioned object storage for shared datasets and commit only a manifest,
checksums, split file, and dataset report when appropriate.

Required sample channels are synchronized RGB PNG, depth PNG, 8-bit grayscale
semantic mask PNG, and JSON metadata. See `docs/dataset-tooling.md`.
