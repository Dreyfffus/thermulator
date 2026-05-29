#!/usr/bin/env bash

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    echo "Error: source this script, don't call it:"
    echo "  source ${BASH_SOURCE[0]}"
    exit 1
fi

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
SCRIPTS_DIR=$(dirname "$SCRIPT_DIR")
BIN_DIR="$SCRIPTS_DIR/bin"
COMPLETION_SCRIPT="$SCRIPT_DIR/completion.bash"

chmod +x "$BIN_DIR"/*

export PATH="$BIN_DIR:$PATH"

if ! grep -q "$BIN_DIR" "$HOME/.bashrc"; then
    echo "" >> "$HOME/.bashrc"
    echo "# thermocator scripts -- added by setup.bash" >> "$HOME/.bashrc"
    echo "export PATH=\"$BIN_DIR:\$PATH\"" >> "$HOME/.bashrc"
    echo "Added $BIN_DIR to ~/.bashrc"
else
    echo "$BIN_DIR already in ~/.bashrc -- skipping PATH entry"
fi

if [[ -f "$COMPLETION_SCRIPT" ]]; then
    source "$COMPLETION_SCRIPT"
else
    echo "Warning: completion script not found at $COMPLETION_SCRIPT"
fi

if ! grep -q "$COMPLETION_SCRIPT" "$HOME/.bashrc"; then
    echo "" >> "$HOME/.bashrc"
    echo "# thermocator tab completion -- added by setup.bash" >> "$HOME/.bashrc"
    echo "[[ -f \"$COMPLETION_SCRIPT\" ]] && source \"$COMPLETION_SCRIPT\"" >> "$HOME/.bashrc"
    echo "Added completion to ~/.bashrc"
else
    echo "Completion already in ~/.bashrc -- skipping"
fi

echo "Done. Both robot and dock have tab completion in this session and all future sessions."
