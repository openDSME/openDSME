#!/usr/bin/env python3

from subprocess import Popen, PIPE
import ntpath

def main():
    output, errors = Popen(['egrep -rn --include \*.h --include \*.cc "^namespace"'], shell=True, stdin=PIPE, stdout=PIPE, stderr=PIPE).communicate()
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

        count = 1;
        i = int(parts[1])
        name = parts[2].split(' ')[1]

        while count > 0 and i < len(contents):
            l = contents[i]
            count = count + l.count('{')
            count = count - l.count('}')
            i = i + 1

        closeline = contents[i - 1]

        if closeline.split(' ')[0] != '}':
            violations.append(parts[0] + ' Error processing file: ' + closeline)
            continue

        if len(closeline.split(' ')) <= 2:
            violations.append(parts[0] + ' No label for closed namespace: ' + closeline + ' in line ' + str(i))
            continue

        if len(closeline.split(' ')) != 5 or closeline.split(' ')[2] != 'namespace':
            violations.append(parts[0] + ' Label should be "namespace ...": ' + closeline)
            continue

        closed_name = closeline.split(' ')[3]

        if closed_name != name:
            violations.append(parts[0] + ' Namespace label does not match: ' + closed_name + ' vs. ' + name)
            continue
    
    if len(violations) > 0:
        print(str('\n').join(violations))

    print(str(len(violations)) + " violations among namespaces found.")

if __name__ == "__main__":
    main()
