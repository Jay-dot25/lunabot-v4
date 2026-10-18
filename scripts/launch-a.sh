#!/usr/bin/env bash
# Compatibility entry point for the repository's original Launch A name.
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec "$SCRIPT_DIR/launch-1.sh" "$@"
