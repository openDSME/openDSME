#!/usr/bin/env python3

from subprocess import Popen, PIPE
import ntpath
import re

def main():
    violated = False

    output, errors = Popen(['grep -rn --include \*.h --include \*.cc "dsme_atomicBegin();"'], shell=True, stdin=PIPE, stdout=PIPE, stderr=PIPE).communicate()
    error_message = errors.decode()
    if error_message != '':
        print(error_message)

    if len(output.decode().splitlines()) != 1:
        violated = True
        print('Incorrect use of dsme_atomicBegin(): ' + str(output.decode().splitlines()))

    output, errors = Popen(['grep -rn --include \*.h --include \*.cc "dsme_atomicEnd();"'], shell=True, stdin=PIPE, stdout=PIPE, stderr=PIPE).communicate()
    error_message = errors.decode()
    if error_message != '':
        print(error_message)

    if len(output.decode().splitlines()) != 1:
        violated = True
        print('Incorrect use of dsme_atomicEnd(): ' + str(output.decode().splitlines()))

    if violated == False:
        print('0 violations among atomic blocks found.')

if __name__ == "__main__":
    main()
