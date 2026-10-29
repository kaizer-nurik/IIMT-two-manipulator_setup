#!/usr/bin/env bash
set -eu
# Empty/unset token env deliberately means Jupyter generates a random token.
# Do not pass token='' or disable authentication.
if [[ -z "${JUPYTER_TOKEN:-}" ]]; then unset JUPYTER_TOKEN; fi
exec jupyter lab --ip=127.0.0.1 --port="${JUPYTER_PORT:-8888}" \
    --ServerApp.port_retries=0 --no-browser --allow-root \
    --ServerApp.root_dir=/opt/iimt
