#!/usr/bin/env bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
docker build --no-cache -f "$SCRIPT_DIR/central.dockerfile" --target base -t rises:base "$SCRIPT_DIR"
docker build -f "$SCRIPT_DIR/central.dockerfile" --target unity -t rises:unity "$SCRIPT_DIR"
