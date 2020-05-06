# Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
Run the DDSHelloWorld example test.

Runs the DDSHelloWorldPublsher and DDSHelloWorldSubscriber applications in
parallel for 15 seconds. These return an exit code equal to 0 in case of
successful completion, that is, if the publisher sends all the samples and the
subscriber receives all of them. If the timeout occurs, both applications are
forced to end with a return code other than 0.
"""
import subprocess


def run_test():
    """
    Run the test for the DDSHelloWorld example.

    :return: The command return code.
    """
    dds_example_path = './Examples/C++/DDSHelloWorld'
    command = 'timeout 15 {}/DDSHelloWorldPublisher & \
        timeout 15 {}/DDSHelloWorldSubscriber'.format(
            dds_example_path, dds_example_path)
    returncode = subprocess.run(command, shell=True).returncode
    return returncode


if __name__ == '__main__':
    exit(run_test())
