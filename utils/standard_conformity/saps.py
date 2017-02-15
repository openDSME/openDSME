#!/usr/bin/env python3

import os
import os.path
import re

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
                saps_standard[sap][group] = []

            saps_standard[sap][group].append(primitive)

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
                    saps_implemented[sap][group] = []


    spacing    = '  ┃'
    indent     = '  ┣━━'
    indent_opt = '  ┣┅┅'

    for sap, groups in sorted(saps_standard.items()):
        if sap in saps_implemented:
            print('\n\x1B[0;32m' + sap + '\033[0m')
            print(spacing)
            for group, primitives in sorted(groups.items()):
                if group in saps_standard[sap] and group in saps_implemented[sap]:
                    print(indent + '\x1B[0;32m' + group + '\033[0m')
                else:
                    print(indent + group)
            if sap in saps_implemented:
                for group, primitives in sorted(saps_implemented[sap].items()):
                    if group not in saps_standard[sap]:
                        print(indent + '\x1B[0;31m' + group + '\033[0m')
        else:
            print('\n' + sap)
            print(spacing)
            for group, primitives in sorted(groups.items()):
                print(indent + group)

    for sap, groups in sorted(saps_implemented.items()):
        if sap not in saps_standard:
            print('\n\x1B[0;31m' + sap + '\033[0m')
            print(spacing)
            for group, primitives in groups.items():
                print(indent + '\x1B[0;31m' + group + '\033[0m')

if __name__ == "__main__":
    main()
