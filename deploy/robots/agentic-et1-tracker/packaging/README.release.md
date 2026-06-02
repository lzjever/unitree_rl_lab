# agentic-et1-tracker release packaging

This packaging tree builds an isolated tar.gz release without changing the
developer build and test flow used by the source tree.

The installed layout is:

```text
PREFIX/
  current -> releases/VERSION
  releases/VERSION/
    bin/agentic-et1-tracker
    lib/
    config/config.robot.yaml.template
    share/agentic-et1-tracker/config/
    skills/et1-trk2motion/
    scripts/{install,start,stop,status,selftest}.sh
  shared/config/config.robot.yaml
  shared/logs/
  shared/run/
  shared/motions/
```

Runtime config points at `PREFIX/current/share/...`, so an OTA can unpack a new
release under `PREFIX/releases/<version>` and move `PREFIX/current` to switch
the app-local policy/control assets. The installer refuses common global
prefixes such as `/usr`, `/etc`, `/lib`, `/bin`, and `$HOME/.agents`, and only
writes below the requested prefix.

## Build

From the repo root:

```sh
deploy/robots/agentic-et1-tracker/packaging/build_release.sh \
  --version 2026.06.01-et1 \
  --out-dir /tmp/agentic-et1-releases
```

The default build is the real integration binary:

- `AGENTIC_ET1_BUILD_ONNX=ON`
- `AGENTIC_ET1_BUILD_ROBOT=ON`
- `AGENTIC_ET1_BUILD_TESTS=OFF`

The script uses repo-local ONNX Runtime packages under `deploy/thirdparty` when
available. Unitree SDK2 is discovered by the existing CMake logic; pass an
explicit root when building outside this workspace.

Cross builds are parameterized. For example:

```sh
deploy/robots/agentic-et1-tracker/packaging/build_release.sh \
  --target-arch aarch64 \
  --cmake-toolchain /path/to/aarch64-toolchain.cmake \
  --onnxruntime-root /path/to/onnxruntime-linux-aarch64 \
  --unitree-sdk2-root /path/to/unitree_sdk2_install \
  --version 2026.06.01-aarch64
```

Use `--cmake-arg -DNAME=VALUE` for SDK-specific settings that belong to the
external toolchain or sysroot.

## Install

Extract the tarball on the target and run:

```sh
tar -xzf agentic-et1-tracker-<version>-<arch>.tar.gz
agentic-et1-tracker-<version>-<arch>/scripts/install.sh --prefix /path/to/prefix
```

Review and edit:

```sh
/path/to/prefix/shared/config/config.robot.yaml
```

At minimum, set the robot network interface and motion directory policy for the
deployment. The default HTTP bind remains `127.0.0.1:8083`.

## Operations

```sh
/path/to/prefix/current/scripts/selftest.sh
/path/to/prefix/current/scripts/start.sh
/path/to/prefix/current/scripts/status.sh
/path/to/prefix/current/scripts/stop.sh
```

`status.sh` uses the bundled `skills/et1-trk2motion/scripts/et1-trk2motion`
client. Override the URL with `ET1_TRACKER_URL` or `status.sh --url URL`.

## OTA switch

To stage a new release without changing the running `current` symlink, install
with `--no-current`:

```sh
new-release/scripts/install.sh --prefix /path/to/prefix --no-current
```

After external validation, switch:

```sh
ln -s releases/<version> /path/to/prefix/current.next
mv -Tf /path/to/prefix/current.next /path/to/prefix/current
```

Restart the service with the scripts from `current`.
