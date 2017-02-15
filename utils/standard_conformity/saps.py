#!/usr/bin/env python3

import os
import os.path
import re

class Class:
    def __init__(self, name):
        self.name = name

        self.has_primitive = {}
        self.has_primitive['request']    = False
        self.has_primitive['indication'] = False
        self.has_primitive['response']   = False
        self.has_primitive['confirm']    = False

    def addPrimitive(self, primitive):
        self.has_primitive[primitive] = True

    def __str__(self):
        return str(sorted(self.has_primitive.items()))

    def __repr__(self):
        return self.__str__()

def main():
    file_name = 'utils/standard_conformity/SAPs.txt'

    saps_standard = {}
    saps_implemented = {}

    pattern = re.compile('^(MLME|MCPS)\-([A-Z\-]+)\.([a-z]+)$')
    for line in open(file_name):
        match = pattern.match(line)

        if match:
            sap = match.group(1).lower() + '_sap'
            group = match.group(2).replace('-', '_')
            primitive = match.group(3)

            if not sap in saps_standard:
                saps_standard[sap] = {}

            if not group in saps_standard[sap]:
                saps_standard[sap][group] = Class(group)

            saps_standard[sap][group].addPrimitive(primitive)

        else:
            print('ERROR')
            print(line)
            return

    for sap in ['mlme_sap', 'mcps_sap']:
        saps_implemented[sap] = {}

        for filename in os.listdir('mac_services/' + sap):
            if len(filename.split('.')) == 2 and filename.split('.')[1] == 'h':
                group = filename.split('.')[0]
                if group.lower() != sap:
                    saps_implemented[sap][group] = Class(group)

                    content = open('mac_services/' + sap + '/' + filename).read()
                    if re.search('ConfirmBase<[A-Za-z_]+>', content):
                        saps_implemented[sap][group].addPrimitive('confirm')
                    if re.search('IndicationBase<[A-Za-z_]+>', content):
                        saps_implemented[sap][group].addPrimitive('indication')
                    if re.search('void request\(request_parameters&\);', content):
                        saps_implemented[sap][group].addPrimitive('request')
                    if re.search('void response\(response_parameters&\);', content):
                        saps_implemented[sap][group].addPrimitive('response')

    empty        = '  ┃  '
    spacing      = '  ┃'
    indent       = '  ┣━━'
    indent_end   = '  ┗━━'
    indent_opt   = '  ┣┅┅'

    indent_t     = '  ├──'
    indent_t_end = '  └──'

    for sap, groups in sorted(saps_standard.items()):
        if sap in saps_implemented:
            print('\x1B[0;32m' + sap + '\033[0m')
            print(spacing)
            for group, primitives in sorted(groups.items()):
                if group in saps_standard[sap] and group in saps_implemented[sap]:
                    print(indent + '\x1B[0;32m' + group + '\033[0m')
                    for primitive, present in sorted(saps_standard[sap][group].has_primitive.items()):
                        current_indent = indent_t
                        last_present = ''

                        for k, v in reversed(sorted(saps_standard[sap][group].has_primitive.items())):
                            if v:
                                last_present = k
                                break

                        if primitive == last_present:
                            current_indent = indent_t_end
                        if present and saps_implemented[sap][group].has_primitive[primitive]:
                            print(empty + current_indent + '\x1B[0;32m' + primitive + '\033[0m')
                        elif present:
                            print(empty + current_indent + primitive)
                        elif saps_implemented[sap][group].has_primitive[primitive]:
                            print(empty + current_indent + '\x1B[0;31m' + primitive + '\033[0m')
                else:
                    print(indent + group)
            if sap in saps_implemented:
                for group, primitives in sorted(saps_implemented[sap].items()):
                    if group not in saps_standard[sap]:
                        print(indent + '\x1B[0;31m' + group + '\033[0m')
        else:
            print(sap)
            print(spacing)
            for group, primitives in sorted(groups.items()):
                print(indent + group)

        print('  ┻\n')

    for sap, groups in sorted(saps_implemented.items()):
        if sap not in saps_standard:
            print('\n\x1B[0;31m' + sap + '\033[0m')
            print(spacing)
            for group, primitives in groups.items():
                print(indent + '\x1B[0;31m' + group + '\033[0m')
            print('  ┻\n')

if __name__ == "__main__":
    main()
