#!/bin/bash

echo "Applying auto-style."
./utils/code_quality/clang_style.sh

./utils/code_quality/check_atomic.py
./utils/code_quality/check_guards.py
./utils/code_quality/check_namespaces.py
./utils/code_quality/check_includes.py
