# kapi — Kernel API Specification Extractor

Userspace utility that extracts and displays kernel API specifications from
three sources:

- `--source PATH` — parse kerneldoc blocks in a C source file or tree
- `--vmlinux PATH` — decode the `.kapi_specs` ELF section of a compiled vmlinux
- `--debugfs PATH` — read the live specs from `/sys/kernel/debug/kapi/` on a
  running kernel (this is also the mode used when no input option is given, in
  which case PATH is `/sys/kernel/debug`)

Output formats: `plain` (default), `json`, `rst`.

See `Documentation/dev-tools/kernel-api-spec.rst` for the full user guide,
including the kerneldoc DSL reference and the surrounding framework design.

## Build

```
make -C tools/kapi
```

(wraps `cargo build --release`; the binary is produced at
`tools/kapi/target/release/kapi`).

## Usage

```
tools/kapi/target/release/kapi --help
tools/kapi/target/release/kapi --source fs/open.c sys_open
tools/kapi/target/release/kapi --vmlinux vmlinux -f json
tools/kapi/target/release/kapi --debugfs /sys/kernel/debug
```
