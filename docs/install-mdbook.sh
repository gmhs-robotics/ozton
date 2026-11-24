#!/bin/sh
# Install the same verified mdBook binary in Vercel and Linux CI.
set -eu
version=0.5.3
sha=dfb86d8e20a3fed91bf549aa450b9d7f46a275b700292e050cdfd3171732e7fd
root=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
case "$(uname -s):$(uname -m)" in
    Linux:x86_64) ;;
    *) echo "Install mdBook $version for your platform and run mdbook build docs." >&2; exit 1 ;;
esac
mkdir -p "$root/.bin"
archive=$(mktemp)
trap 'rm -f "$archive"' EXIT HUP INT TERM
curl --fail --location --silent --show-error --retry 3 \
    "https://github.com/rust-lang/mdBook/releases/download/v$version/mdbook-v$version-x86_64-unknown-linux-musl.tar.gz" \
    -o "$archive"
printf '%s  %s\n' "$sha" "$archive" | sha256sum -c -
tar -xzf "$archive" -C "$root/.bin" mdbook
"$root/.bin/mdbook" --version
