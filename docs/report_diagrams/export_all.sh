#!/usr/bin/env bash
# Export all report Mermaid diagrams to PNG + SVG (uses system Chrome on macOS)
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SRC="$SCRIPT_DIR/mermaid"
OUT_PNG="$SCRIPT_DIR/exports"
OUT_SVG="$SCRIPT_DIR/exports/svg"
NPX="${NPX:-/opt/homebrew/bin/npx}"

# macOS: use installed Google Chrome (avoids puppeteer download)
if [[ -z "${PUPPETEER_EXECUTABLE_PATH:-}" ]] && [[ -x "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome" ]]; then
  export PUPPETEER_EXECUTABLE_PATH="/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"
fi

mkdir -p "$OUT_PNG" "$OUT_SVG"
for f in "$SRC"/*.mmd; do
  base="$(basename "$f" .mmd)"
  echo "Exporting $base ..."
  "$NPX" --yes @mermaid-js/mermaid-cli@11.4.0 -i "$f" -o "$OUT_PNG/${base}.png" -w 1600 -b white -q
  "$NPX" --yes @mermaid-js/mermaid-cli@11.4.0 -i "$f" -o "$OUT_SVG/${base}.svg" -b white -q
done
echo "Done:"
echo "  PNG → $OUT_PNG/"
echo "  SVG → $OUT_SVG/"
