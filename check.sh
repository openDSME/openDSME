#!/bin/bash

cppcheck -j 4 --xml-version=2 --enable=all --inconclusive . 2> err.xml

cppcheck-htmlreport --file=err.xml --report-dir=cppcheck --source-dir=.
