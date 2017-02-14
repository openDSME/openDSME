#!/bin/bash

echo "Applying auto-style."
./code_quality/clang_style.sh

./code_quality/check_atomic.py
./code_quality/check_guards.py
./code_quality/check_namespaces.py
./code_quality/check_includes.py
