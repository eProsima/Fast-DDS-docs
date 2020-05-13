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
import os
import subprocess


def run_test():
    """
    Run the test for the DDSHelloWorld example.

    :return: The command return code.
    """
    helloworld_pub = os.environ.get('HELLOWORLD_PUB_TEST_BIN')
    helloworld_sub = os.environ.get('HELLOWORLD_SUB_TEST_BIN')
    try:
        subprocess_pub = subprocess.Popen(helloworld_pub)
        subprocess.run(helloworld_sub, timeout=15)
    except subprocess.TimeoutExpired:
        subprocess_pub.kill()
        return 1
    return 0


if __name__ == '__main__':
    exit(run_test())
