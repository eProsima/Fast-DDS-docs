.. include:: ../../03-exports/roles.include

.. _ffastddscli_cli:

CLI
===

The *Fast DDS* command line interface provides a set commands and sub-commands to perform, Fast-DDS
related, maintenance / configuration tasks.

An executable file for Linux and Windows that runs the Java *Fast DDS-Gen* application is
available in the ``tools`` folder.
If the ``tools/fastdds`` folder path is added to the ``PATH`` environment variable, *Fast DDS-Gen* can be executed
running the following commands:

- Linux:

    .. code-block:: bash

        $ fastdds <command> [<command-args>]

-  Windows:

    .. code-block:: bash

        > fastdds.bat <command> [<command-args>]



discovery
---------

Launches a server for :ref:`Server-Client Discovery<discovery_server>`.

.. code-block:: bash

    fastdds discovery -i {0-255} [optional parameters]

Where the optional parameters are:

+--------------------------+-------------------------------------------------------------------------------------------+
| Option                   | Description                                                                               |
+==========================+===========================================================================================+
| ``-h  -help``            | Produce help message.                                                                     |
+--------------------------+-------------------------------------------------------------------------------------------+
| ``-i  --server-id``      | Mandatory unique server identifier. Specifies zero based server position in |br|          |
|                          | `ROS_DISCOVERY_SERVER` environment variable.                                              |
+--------------------------+-------------------------------------------------------------------------------------------+
| ``-l  --ip-address``     | Server interface chosen to listen the clients. Defaults to any (0.0.0.0)                  |
+--------------------------+-------------------------------------------------------------------------------------------+
| ``-p  --port``           | Creates a server with a backup file associated.                                           |
+--------------------------+-------------------------------------------------------------------------------------------+
| ``-b  --backup``         | Creates a server with a backup file associated.                                           |
+--------------------------+-------------------------------------------------------------------------------------------+

Examples
^^^^^^^^

Launch a default server with id 0 (first on ROS_DISCOVERY_SERVER)
listening on all available interfaces on UDP port 11811. Only one
server can use default values per machine.

.. code-block:: bash

    fastdds discovery -i 0

Launch a default server with id 1 (second on ROS_DISCOVERY_SERVER)
listening on localhost with UDP port 14520. Only localhost clients
can reach the server using as ROS_DISCOVERY_SERVER=;127.0.0.1:14520

.. code-block:: bash

    fastdds discovery -i 1 -l 127.0.0.1 -p 14520

Launch a default server with id 3 (third on ROS_DISCOVERY_SERVER)
listening on Wi-Fi (192.168.36.34) and Ethernet (172.20.96.1) local
interfaces with UDP ports 8783 and 51083 respectively
(addresses and ports are made up for the example).

.. code-block:: bash

    fastdds discovery -i 1 -l 192.168.36.34 -p 14520 -l 172.20.96.1 -p 51083

Launch a default server with id 4 (fourth on ROS_DISCOVERY_SERVER)
listening on 172.30.144.1 with UDP port 12345 and provided with a
backup file. If the server crashes it will automatically restore its
previous state when reenacted.

.. code-block:: bash

    fastdds discovery -i 1 -l 172.30.144.1 -p 12345 -b



shm
---

Provides maintenance tasks related with :ref:`transport_sharedMemory_sharedMemory`.

.. code-block:: bash

    fastdds shm [<shm-command>]

+--------------------------+-------------------------------------------------------------------------------------------+
| Command                  | Description                                                                               |
+==========================+===========================================================================================+
| ``clean``                | Cleans SHM zombie files.                                                                  |
+--------------------------+-------------------------------------------------------------------------------------------+

+--------------------------+-------------------------------------------------------------------------------------------+
| Option                   | Description                                                                               |
+==========================+===========================================================================================+
| ``-h  -help``            | Produce help message.                                                                     |
+--------------------------+-------------------------------------------------------------------------------------------+
