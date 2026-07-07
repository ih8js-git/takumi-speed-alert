#!/usr/bin/env bash
set -e

TARGET="pizero-dev"

if [ "$1" == "prod" ]; then
    TARGET="pizero-prod"
elif [ "$1" == "dev" ]; then
    TARGET="pizero-dev"
elif [ -n "$1" ]; then
    echo "Usage: ./build.sh [dev|prod]"
    echo "Defaults to 'dev'"
    exit 1
fi

echo "Building target: $TARGET"

time nix build .#nixosConfigurations."$TARGET".config.system.build.sdImage \
  --extra-experimental-features 'nix-command flakes' \
  --builders 'ssh://your-username@your-ip-or-hostname aarch64-linux' \
  --max-jobs 0
