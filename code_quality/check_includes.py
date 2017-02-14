#!/usr/bin/env python3

from subprocess import Popen, PIPE
import os.path

def main():
    output, errors = Popen(['egrep -rn --include \*.h --include \*.cc "^#include \\".*\\""'], shell=True, stdin=PIPE, stdout=PIPE, stderr=PIPE).communicate()
    error_message = errors.decode()
    if error_message != '':
        print(error_message)

    violations = []
    include_dict = {}

    lines = output.decode().splitlines()
    lines.sort()
    for line in lines:

        parts = line.split(':')

        filename = os.path.basename(parts[0])
        pathname = os.path.dirname(parts[0])
        include = parts[2].split('"')[1]

        combined = os.path.normpath(os.path.join(pathname, include))

        if not os.path.isfile(combined):
            violations.append(parts[0] + ':' + parts[1] + ': File not found: ' + parts[2])
            continue

        relative = os.path.relpath(combined, pathname)
        if relative[0] != '.':
            relative = './' + relative

        if not include == relative:
            violations.append(parts[0] + ':' + parts[1] + ': Non-minimal path: ' + include + ' vs. ' + relative)
            continue

        if not filename in include_dict:
            include_dict[filename] = []

        if combined in include_dict[filename]:
            violations.append(parts[0] + ':' + parts[1] + ': Duplicate include: ' + parts[2])
            continue

        include_dict[filename].append(combined)

    if len(violations) > 0:
        print(str('\n').join(violations))

    print(str(len(violations)) + ' violations among includes found in ' + str(len(lines)) + ' instances.')

if __name__ == "__main__":
    main()
