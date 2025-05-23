# Copyright 2025 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
Run the RpcClientServerBasic example test.

Runs the client and server applications in
parallel for each implemented operation. These return an exit code equal to 0 in case of
successful completion, that is, in case of client receiving the request results.
If the timeout occurs, both applications are
forced to end with a return code other than 0.
"""

import os
import subprocess

def run_test():
    """
    Run the test for the RpcClientServerBasic example.

    :return: The command return code.
    """

    operations = [
        "add",
        "sub",
        "rep"
    ]
    client = os.environ.get('CLIENT_TEST_BIN')
    server = os.environ.get('SERVER_TEST_BIN')

    for operation in operations:
        try:
            subprocess_server = subprocess.Popen(server)
            subprocess_client = subprocess.Popen([client, operation])
            client_return_code = subprocess_client.wait(timeout=30)

            # Stop the server process gracefully
            subprocess_server.terminate()
            server_return_code = subprocess_server.wait(timeout=5)

            if client_return_code != 0:
                print(f"Client failed with return code: {client_return_code}")
                return client_return_code
            if server_return_code != 0:
                print(f"Server failed with return code: {server_return_code}")
                return server_return_code

        except subprocess.TimeoutExpired:
            subprocess_server.kill()
            subprocess_client.kill()
            return 1

    return 0


if __name__ == '__main__':
    exit(run_test())
