#!/usr/bin/env python3

from subprocess import Popen, PIPE
import ntpath

def main():
    output, errors = Popen(['grep -rn --include \*.h "#ifndef"'], shell=True, stdin=PIPE, stdout=PIPE, stderr=PIPE).communicate()
    print(errors.decode())

    violations = []

    for line in output.decode().splitlines():

        parts = line.split(':')

        if int(parts[1]) != 43 and int(parts[1]) != 30:
            violations.append('Invalid file offset: ' + parts[0] + ", " + parts[1])
            continue

        filename = ntpath.basename(parts[0])
        filedefine = filename.replace('.h', '_H_').upper()
        checked_define = parts[2].split(' ')[1]

        if checked_define != filedefine:
            violations.append('Checked define does not match filename: '
                    + parts[0] + ', ' + checked_define + ' vs. ' + filedefine)
            continue

        f = open(parts[0], 'r')
        contents = f.read().splitlines()
        f.close()
        defineline = contents[int(parts[1])]

        if defineline.split(' ')[0] != '#define':
            violations.append(parts[0] + ", " + defineline)
            continue

        define = defineline.split(' ')[1]

        if define != checked_define:
            violations.append('#define does not match: ' + define + ' vs. ' + checked_define)
            continue

        closeline = contents[len(contents) - 1]

        if closeline.split(' ')[0] != '#endif':
            violations.append('Guard not closed in correct line: ' + parts[0] + ", " + closeline)
            continue

        if len(closeline.split(' ')) != 4:
            violations.append('No label for closed guard: ' + parts[0] + ", " + closeline)
            continue

        closed_define = closeline.split(' ')[2]

        if closed_define != checked_define:
            violations.append('#endif label does not match: '
                    + parts[0] + ', ' + closed_define + ' vs. ' + checked_define)
            continue

    print(str('\n').join(violations))

    print(str(len(violations)) + " violations found.")

if __name__ == "__main__":
    main()
