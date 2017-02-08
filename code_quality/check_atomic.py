#!/usr/bin/env python3

from subprocess import Popen, PIPE
import ntpath
import re

def main():
    output, errors = Popen(['egrep -rn --include \*.h --include \*.cc "^\s*dsme_atomicBegin\(\);$"'], shell=True, stdin=PIPE, stdout=PIPE, stderr=PIPE).communicate()
    error_message = errors.decode()
    if error_message != '':
        print(error_message)

    violations = []

    lines = output.decode().splitlines()
    lines.sort()
    for line in lines:
        parts = line.split(':')

        f = open(parts[0], 'r')
        contents = f.read().splitlines()
        f.close()

        depth = 0;
        i = int(parts[1])
        closed = 0.0

        pattern = re.compile('^\s*dsme_atomicEnd\(\);$')
        while depth >= 0 and i < len(contents):
            l = contents[i]

            match = pattern.match(l)
            if match:
                closed += 1/pow(2, depth)
                if closed == 1:
                    break

            depth = depth + l.count('{')
            depth = depth - l.count('}')
            i = i + 1

        if closed != 1.0:
            violations.append(parts[0] + ':' + parts[1] + ' Incorrect number of closing blocks: ' + str(closed))
            continue

    if len(violations) > 0:
        print(str('\n').join(violations))

    print(str(len(violations)) + ' violations among atomic blocks found in ' + str(len(lines)) + ' instances.')

if __name__ == "__main__":
    main()
