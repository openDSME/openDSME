#!/bin/bash

astyle \
    --suffix=none \
    --style=attach \
    --indent=spaces=4 \
    --indent-switches \
    --indent-col1-comments \
    --pad-oper \
    --unpad-paren \
    --align-pointer=type \
    --align-reference=type \
    --add-brackets \
    --convert-tabs \
    --close-templates \
    --max-code-length=180 \
    --mode=c \
    --recursive *.cc *.h
