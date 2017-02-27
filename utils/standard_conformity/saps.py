#!/usr/bin/env python3

import os
import os.path
import re
import sys

declaration = '\s+[a-zA-Z0-9_\*:]+ ([a-zA-Z0-9_]+);.*\n'
available_primitives = ['confirm', 'indication', 'request', 'response']

class Class:
    def __init__(self, name):
        self.name = name
        self.has_primitive = {}
        self.parameters = {}
        for primitive in available_primitives:
            self.has_primitive[primitive] = False
            self.parameters[primitive]    = []

    def addPrimitive(self, primitive):
        self.has_primitive[primitive] = True

    def __str__(self):
        return str(sorted(self.has_primitive.items())) + ' ' +  str(sorted(self.parameters.items()))

    def __repr__(self):
        return self.__str__()

def process_parameters(declarations):
    pattern = re.compile(declaration)
    return pattern.findall(declarations)

def main():
    if sys.stdout.isatty():
        def red(string):
            return '\x1B[0;31m' + string + '\033[0m'
        def green(string):
            return '\x1B[0;32m' + string + '\033[0m'
    else:
        def red(string):
            return string
        def green(string):
            return string

    saps_standard = {}
    saps_implemented = {}

    pattern = re.compile('^(MLME|MCPS)\-([A-Z\-]+)\.([a-z]+)$')
    for line in open('utils/standard_conformity/SAPs.txt'):
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
            print(line)
            assert(False)

    pattern = re.compile('^(MLME|MCPS)\-([A-Z\-]+)\.([a-z]+)\((.*)\)$')
    for line in open('utils/standard_conformity/primitives.txt'):
        match = pattern.match(line)
        if match:
            sap = match.group(1).lower() + '_sap'
            group = match.group(2).replace('-', '_')
            primitive = match.group(3)
            parameters = match.group(4)
            lowercase = lambda s: s[:1].lower() + s[1:] if s else ''
            saps_standard[sap][group].parameters[primitive] = [lowercase(c) for c in parameters.split(', ')]
        else:
            print(line)
            assert(False)


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
                    for primitive in available_primitives:
                        match = re.search(primitive + '_parameters {\n((' + declaration + ')*)\s*};', content)
                        if match:
                            saps_implemented[sap][group].parameters[primitive] = process_parameters(match.group(1))

    skip         = '     '

    empty        = '  ┃  '
    spacing      = '  ┃'
    indent       = '  ┣━━'
    indent_end   = '  ┗━━'

    empty_t      = '  │  '
    indent_t     = '  ├──'
    indent_t_end = '  └──'

    empty_d      = '  ┆  '
    indent_d     = '  ├┄┄'
    indent_d_end = '  └┄┄'

    for sap, groups in sorted(saps_standard.items()):
        if sap in saps_implemented:
            print(green(sap))
            print(spacing)
            for group, primitives in sorted(groups.items()):
                if group in saps_standard[sap] and group in saps_implemented[sap]:
                    print(indent + green(group))
                    for primitive, present in sorted(saps_standard[sap][group].has_primitive.items()):
                        current_indent = indent_t
                        parameter_skip = empty_t
                        last_present = ''
                        for k, v in reversed(sorted(saps_standard[sap][group].has_primitive.items())):
                            if v:
                                last_present = k
                                break
                        if primitive == last_present:
                            current_indent = indent_t_end
                            parameter_skip = skip
                        if present and saps_implemented[sap][group].has_primitive[primitive]:
                            print(empty + current_indent + green(primitive))
                            parameters_standard = saps_standard[sap][group].parameters[primitive]
                            parameters_implemented = saps_implemented[sap][group].parameters[primitive]
                            for i, parameter in enumerate(parameters_standard):
                                parameter_indent = indent_d
                                if i == len(parameters_standard) - 1:
                                    parameter_indent = indent_d_end
                                if parameter in parameters_implemented:
                                    print(empty + parameter_skip + parameter_indent + green(parameter))
                                else:
                                    print(empty + parameter_skip + parameter_indent + parameter)
                            for i, parameter in enumerate(parameters_implemented):
                                if parameter not in parameters_standard:
                                    print(empty + parameter_skip + indent_d_end + red(parameter))
                        elif present:
                            print(empty + current_indent + primitive)
                        elif saps_implemented[sap][group].has_primitive[primitive]:
                            print(empty + current_indent + red(primitive))
                else:
                    print(indent + group)
            if sap in saps_implemented:
                for group, primitives in sorted(saps_implemented[sap].items()):
                    if group not in saps_standard[sap]:
                        print(indent + red(group))
        else:
            print(sap)
            print(spacing)
            for group, primitives in sorted(groups.items()):
                print(indent + group)
        print('  ┻\n')
    for sap, groups in sorted(saps_implemented.items()):
        if sap not in saps_standard:
            print('\n' + red(sap))
            print(spacing)
            for group, primitives in groups.items():
                print(indent + red(group))
            print('  ┻\n')

if __name__ == "__main__":
    main()
