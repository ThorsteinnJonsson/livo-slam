#!/usr/bin/env bash
# Points config/active.yaml and datasets/active at the chosen dataset/sequence,
# so the launch config and the bag player both follow one selection.
set -euo pipefail

SELECTION="${1:?usage: select_dataset.sh <dataset>/<sequence>}"
WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

DATASET="${SELECTION%%/*}"
CONFIG="${WORKSPACE}/config/${DATASET}.yaml"
BAG="${WORKSPACE}/datasets/${SELECTION}"

if [[ "${SELECTION}" != */* ]]; then
  echo "expected <dataset>/<sequence>, got '${SELECTION}'" >&2
  exit 1
fi
if [[ ! -f "${CONFIG}" ]]; then
  echo "no config for dataset '${DATASET}' at ${CONFIG}" >&2
  exit 1
fi
if [[ ! -d "${BAG}" ]]; then
  echo "no bag at ${BAG}" >&2
  echo "available under datasets/${DATASET}:" >&2
  ls -1 "${WORKSPACE}/datasets/${DATASET}" >&2 2>/dev/null || echo "  (none)" >&2
  exit 1
fi

ln -sfn "${DATASET}.yaml" "${WORKSPACE}/config/active.yaml"
ln -sfn "${SELECTION}" "${WORKSPACE}/datasets/active"

echo "config ${DATASET}.yaml, bag datasets/${SELECTION}"
