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
Run the RPCClientServerFeed example test.

Runs the client and server applications in
parallel for each implemented operation. These return an exit code equal to 0 in case of
successful completion, that is, in case of client receiving the request results.
If the timeout occurs, both applications are
forced to end with a return code other than 0.
The input feed operations are tested with a sequence of inputs
simulating user interaction.
"""

import os
import subprocess
import time
import threading
from typing import List, IO

def print_output(stream: IO[str]) -> None:
    """
    Read a stream and print in terminal.

    :param stream: The stream to read.
    """

    while True:
        line = stream.readline()
        if not line:
            break
        print(line.strip())

def run_input_feed_operation(operation: str, inputs: List[str], delay: float = 1) -> int:
    """
    Run an input feed operation, with a given user sequence input

    :param operation: The name of the operation to be performed.
    :param inputs: A sequence of input commands to simulate the user interaction.
    :param delay: The delay between each input command (in seconds).

    :return: The command return code.
    """

    client = os.environ.get('CLIENT_TEST_BIN')
    server = os.environ.get('SERVER_TEST_BIN')

    if not client or not server:
        raise ValueError("Environment variables 'CLIENT_TEST_BIN' and 'SERVER_TEST_BIN' must be set and non-empty.")

    try:
        subprocess_server = subprocess.Popen(server)
        subprocess_client = subprocess.Popen(
            [client, operation],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )


        if subprocess_client.stdin is None:
            raise RuntimeError("Failed to open stdin for the client process.")
        if subprocess_client.stdout is None:
            raise RuntimeError("Failed to open stdout for the client process.")
        if subprocess_client.stderr is None:
            raise RuntimeError("Failed to open stderr for the client process.")

        # Start a daemon to read the feedback messages from the input feed CLI
        threading.Thread(target=print_output, args=(subprocess_client.stdout,), daemon=True).start()

        time.sleep(3)

        # Input feed CLI should be opened. Simulate the user input
        for input_command in inputs:
            print(f"Sending input: {input_command}")
            subprocess_client.stdin.write(input_command + "\n")
            subprocess_client.stdin.flush()
            time.sleep(delay)  # Wait for the command to be processed

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


def run_non_input_feed_operation(operation: str) -> int:
    """
    Run a non-input feed operation, i.e: an operation that does not require user input.

    :param operation: The name of the operation to be performed.

    :return: The command return code.
    """

    client = os.environ.get('CLIENT_TEST_BIN')
    server = os.environ.get('SERVER_TEST_BIN')

    if not client or not server:
        raise ValueError("Environment variables 'CLIENT_TEST_BIN' and 'SERVER_TEST_BIN' must be set and non-empty.")

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

def run_test():
    """
    Run the test for the RPCClientServerBasic example.

    :return: The command return code.
    """

    operations = [
        "add",
        "sub",
        "rep",
        "fib",
        "sumall",
        "acc",
        "filter"
    ]

    input_feed_operations = [
        "sumall",
        "acc",
        "filter"
    ]

    input_feed_data = [
        "1",
        "2",
        "3",
        "4",
        "5",
        "" # Empty line to indicate end of input
    ]

    for operation in operations:
        if operation in input_feed_operations:
            return_code = run_input_feed_operation(operation, input_feed_data)
        else:
            return_code = run_non_input_feed_operation(operation)

        if return_code != 0:
            return return_code

        time.sleep(2) # Wait a bit before running the next operation

    return 0

if __name__ == '__main__':
    exit(run_test())
