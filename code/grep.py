# Copyright 2021 Proyectos y Sistemas de Mantenimiento SL (eProsima).
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
Scan a directory recursively looking for a regex in files of a given format.

Scan a directory recursively, checking all the files of a given format that
contain a given regular expression. The script's exit code is the total number
of lines that contain such expression. The script prints the files lines that
matched the regex.
"""
import os
import re
import sys

import argparse

from pathlib import Path

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        '-d',
        '--directory',
        help='The directory to scan',
        required=True
    )
    parser.add_argument(
        '-r',
        '--regex',
        help='The regular expresion to look for',
        required=True
    )
    parser.add_argument(
        '-f',
        '--file_format',
        help='The format of the files to scan',
        default='rst',
        required=False
    )
    args = parser.parse_args()

    directory = os.path.abspath(args.directory)

    number_occurrences = 0
    occurrences = []
    rst_files = list(Path(directory).rglob('*.{}'.format(args.file_format)))
    for rst_file in rst_files:
        with open(rst_file, encoding='utf-8') as f:
            line_number = 1
            for line in f:
                if re.search(args.regex, line):
                    occurrences.append('{}:{}'.format(rst_file, line_number))
                    number_occurrences += 1
                line_number += 1

    summary = ('{} files with format "{}" scanned looking for regex "{}".' +
               ' Number of occurrences: {}')
    print(
        summary.format(
            len(rst_files),
            args.file_format,
            args.regex,
            number_occurrences
        )
    )
    for occurrence in occurrences:
        print(occurrence)

    exit(number_occurrences)
