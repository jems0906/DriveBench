# Determinism

The repeatability contract is for the same scenario, driver version, seed, binary, and runtime environment, with the default IEEE-754 rounding environment. It does not promise identical bytes between compilers, operating systems, CPU architectures, or math libraries.

- Ticks are integers at 0.1 s, not elapsed wall time. The simulation has no timing, thread scheduling, network, or filesystem inputs after loading.
- Traffic and route order follow JSON arrays, and the runner draws exactly two noise values per active actor per recorded tick. A per-run `std::mt19937_64` engine is initialized directly from the seed. The top 53 bits of each result map to a unit interval without `std::uniform_real_distribution`, then scale to the configured symmetric position offset.
- The core builds without fast math and implicit floating-point contraction on GCC/Clang, or with `/fp:strict` on MSVC. Standard-library trigonometry and other math are why the contract is scoped to an environment.
- `trajectory_bytes()` writes a `DBR1` prefix followed by frame count and ordered fields. Numeric fields use little-endian 64-bit encodings; doubles preserve their IEEE-754 bit patterns. Actor IDs are length-prefixed UTF-8 bytes from JSON. No padding or object identities enter the encoding. Report SHA-256 values refer to these bytes, not the human-readable replay JSON.

The `determinism` test compares complete nontrivial replays byte-for-byte. The `seed_sweep` test varies only the simulator seed on a single crossing, requiring 100 distinct replays and multiple ground-truth pedestrian clearances. The JSON variation tool instead changes scenario inputs with its own separate seed. A scene with no active noisy observations need not change with the simulator seed.