#!/bin/bash
current_dir=$(git rev-parse --show-toplevel)

if [[ ! "$(pwd -P)" -ef "$current_dir" ]]; then
    echo -e "${red}This script must be executed in the repository root directory.${textreset}"
    exit -1
fi

# Hardcode the intended refs file (this script was picking the wrong one before)
pro_refs_file=./docs/03-exports/pro-only-refs.include

if [[ ! -f "$pro_refs_file" ]]; then
    echo "Error: file not found: $pro_refs_file" >&2
    exit 1
fi

result=$(sed -E 's/:ref:`([^<`]+) *<[^>]+>`/\1/g' "$pro_refs_file")

if echo "$result" | grep -q ':ref:'; then
    echo "Error: unprocessed :ref: entries remain" >&2
    echo "Please check the input file for correct :ref: syntax: :ref:`text <link>`" >&2
    echo "All the references should have a text to render that will be used in basic when it fails to resolve the link." >&2
    exit 1
fi

echo "$result" > "$pro_refs_file"
