#!/usr/bin/env bash
# Run Blender: $BLENDER if set, else `blender` on PATH, else the macOS app.
set -euo pipefail
if [[ -n "${BLENDER:-}" ]]; then exec "$BLENDER" "$@"; fi
if command -v blender >/dev/null 2>&1; then exec blender "$@"; fi
exec /Applications/Blender.app/Contents/MacOS/Blender "$@"
