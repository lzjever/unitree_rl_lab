#!/usr/bin/env bash
set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)/lib.sh"

usage() {
  cat <<'USAGE'
Usage: install.sh --prefix PREFIX [--version VERSION] [--no-current]

Installs this extracted release under:
  PREFIX/releases/VERSION
and, unless --no-current is used, atomically updates:
  PREFIX/current -> releases/VERSION

The installer only writes below PREFIX and refuses common global prefixes.
USAGE
}

prefix=""
version=""
update_current=1
while [[ $# -gt 0 ]]; do
  case "$1" in
    --prefix)
      [[ $# -ge 2 ]] || die "--prefix requires a path"
      prefix="$2"
      shift 2
      ;;
    --version)
      [[ $# -ge 2 ]] || die "--version requires a value"
      version="$2"
      shift 2
      ;;
    --no-current)
      update_current=0
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die "unknown argument: $1"
      ;;
  esac
done

[[ -n "$prefix" ]] || die "--prefix is required"
if [[ "$prefix" != /* ]]; then
  prefix="$(pwd -P)/$prefix"
fi
prefix="$(realpath -m "$prefix")"

home_agents="$(realpath -m "$HOME/.agents")"
case "$prefix" in
  /|/usr|/usr/*|/etc|/etc/*|/lib|/lib/*|/lib64|/lib64/*|/bin|/bin/*|/sbin|/sbin/*|"$home_agents"|"$home_agents"/*)
    die "refusing global prefix: $prefix"
    ;;
esac

if [[ -z "$version" ]]; then
  [[ -f "$ET1_RELEASE_DIR/VERSION" ]] || die "missing VERSION in release"
  version="$(tr -d '[:space:]' < "$ET1_RELEASE_DIR/VERSION")"
fi
[[ -n "$version" ]] || die "empty version"

target="$prefix/releases/$version"
tmp_target="$target.tmp.$$"
[[ ! -e "$target" ]] || die "release already exists: $target"

mkdir -p \
  "$prefix/releases" \
  "$prefix/shared/config" \
  "$prefix/shared/run" \
  "$prefix/shared/logs" \
  "$prefix/shared/motions"
mkdir -p "$tmp_target"

(cd "$ET1_RELEASE_DIR" && tar -cf - .) | (cd "$tmp_target" && tar -xf -)
mv "$tmp_target" "$target"

config_target="$prefix/shared/config/config.robot.yaml"
if [[ ! -e "$config_target" ]]; then
  tmp_config="$config_target.tmp.$$"
  sed "s|@PREFIX@|$prefix|g" "$target/config/config.robot.yaml.template" > "$tmp_config"
  mv "$tmp_config" "$config_target"
  printf 'created config %s\n' "$config_target"
else
  printf 'preserved existing config %s\n' "$config_target"
fi

if [[ "$update_current" -eq 1 ]]; then
  tmp_link="$prefix/current.tmp.$$"
  ln -s "releases/$version" "$tmp_link"
  mv -Tf "$tmp_link" "$prefix/current"
  printf 'updated current -> releases/%s\n' "$version"
fi

printf 'installed agentic-et1-tracker release=%s prefix=%s\n' "$version" "$prefix"
