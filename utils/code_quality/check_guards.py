#!/usr/bin/env python3

from subprocess import Popen, PIPE
import ntpath

def main():
    output, errors = Popen(['grep -rn --include \*.h "#ifndef"'], shell=True, stdin=PIPE, stdout=PIPE, stderr=PIPE).communicate()
    error_message = errors.decode()
    if error_message != '':
        print(error_message)

    violations = []

    lines = output.decode().splitlines()
    lines.sort()
    for line in lines:

        parts = line.split(':')

        if int(parts[1]) != 43 and int(parts[1]) != 30:
            violations.append(parts[0] + ' Invalid file offset: ' + parts[1])
            continue

        filename = ntpath.basename(parts[0])
        filedefine = filename.replace('.h', '_H_').upper()
        checked_define = parts[2].split(' ')[1]

        checked_define_file = checked_define.split('__')[-1]
        if checked_define_file != filedefine:
            violations.append(parts[0] + ' Checked define does not match filename: ' + checked_define_file + ' vs. ' + filedefine)
            continue

        f = open(parts[0], 'r')
        contents = f.read().splitlines()
        f.close()
        defineline = contents[int(parts[1])]

        if defineline.split(' ')[0] != '#define':
            violations.append(parts[0] + ' Next line does not #define: ' + defineline)
            continue

        define = defineline.split(' ')[1]

        if define != checked_define:
            violations.append(parts[0] + ' #define does not match: ' + define + ' vs. ' + checked_define)
            continue

        closeline = contents[len(contents) - 1]

        if closeline.split(' ')[0] != '#endif':
            violations.append(parts[0] + ' Guard not closed in correct line: ' + closeline)
            continue

        if len(closeline.split(' ')) != 4:
            violations.append(parts[0] + ' No label for closed guard: ' + closeline)
            continue

        closed_define = closeline.split(' ')[2]

        if closed_define != checked_define:
            violations.append(parts[0] + ' #endif label does not match: ' + closed_define + ' vs. ' + checked_define)
            continue

        pre_closeline = contents[len(contents) - 2]
        if pre_closeline != "":
            violations.append(parts[0] + ' Line before #endif should be empty: ' + pre_closeline)
            continue

    if len(violations) > 0:
        print(str('\n').join(violations))

    print(str(len(violations)) + ' violations among include guards found in ' + str(len(lines)) + ' instances.')

if __name__ == "__main__":
    main()
