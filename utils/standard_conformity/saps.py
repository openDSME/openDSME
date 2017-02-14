#!/usr/bin/env python3

import os.path
import re

def main():
    file_name = 'utils/standard_conformity/SAPs.txt'

    saps = {}

    pattern = re.compile('^(MLME|MCPS)\-([A-Z\-]+)\.([a-z]+)$')
    for line in open(file_name):
        match = pattern.match(line)

        if match:
            sap = match.group(1).lower() + '_sap'
            group = match.group(2).replace('-', '_')
            primitive = match.group(3)

            if not sap in saps:
                saps[sap] = {}

            if not group in saps[sap]:
                saps[sap][group] = []

            saps[sap][group].append(primitive)

        else:
            print('ERROR')
            print(line)
            return

    for sap, groups in saps.items():
        for group, primitives in groups.items():
            path = 'mac_services/' + sap + '/' + group + '.h'
            if os.path.isfile(path):
                print('\x1B[0;32m' + sap + ' ' + group + '\033[0m')
            else :
                print('\x1B[0;31m' + sap + ' ' + group + '\033[0m')

if __name__ == "__main__":
    main()
