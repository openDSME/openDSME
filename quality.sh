#!/bin/bash

./code_quality/clang_style.sh
echo "Auto-style applied."

./code_quality/check_guards.py
./code_quality/check_namespaces.py
