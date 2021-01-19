.. _ros2-discovery-server:

Use ROS 2 with Fast-DDS Discovery Server
=========================================

This section explains how to run some ROS 2 examples using the Discovery Servers
as discovery communication.
In order to get more information about the specific use of this configuration,
please check the :ref:`Discovery Server Documentation <discovery_server>`
or read the :ref:`common use cases <discovery-server-use-case>` for this configuration.

The following tutorial gathers the steps to check this functionality and learn how to use it with ROS 2.

.. contents::
    :local:
    :backlinks: none
    :depth: 2

The :ref:`Simple Discovery Protocol <simple_disc_settings>` is the
standard protocol defined in the `DDS standard <https://www.omg.org/omg-dds-portal/>`__.
However, it has certain known disadvantages in some scenarios, mainly:

* It does not **Scale** efficiently, as the number of exchanged packets highly increases as new nodes are added.
* It requires **Multicasting** capabilities that may not work reliably in some scenarios, e.g. WiFi.

The **Discovery Server** provides a Client-Server Architecture that allows
the nodes to connect with each other using an intermediate server.
Each node will work as a *Client*, sharing its info with the *Discovery Server* and receiving
the discovery information from it.
This means that the network traffic is highly reduced in big systems, and it does not require *Multicasting*.

.. image:: /01-figures/fast_dds/discovery/ds_explanation.svg
    :align: center

These **Discovery Servers** can be independent, duplicated or connected with each other in order to create
redundancy over the network and avoid having a *Single-Point-Of-Failure*.

.. _ros2_discovery_server_v2:

Discovery Server v2
-------------------

The new version **v2** of Discovery Server, available from *Fast DDS* v2.0.2, implements a new filter feature
that allows to further reduce the number of discovery messages sent.
This version uses the *topic* of the different nodes to decide if two nodes must be connected, or they
could be left unmatched.
The following schema represents the decrease of the discovery packages:

.. image:: /01-figures/fast_dds/discovery/ds1vs2.svg
    :align: center

This architecture reduces the number of packages sent between the server and the different clients dramatically.
In the following graph, the reduction in traffic network over the discovery phase for a
RMF Clinic demo use case, is shown:

.. image:: /01-figures/fast_dds/discovery/discovery_server_v2_performance.svg
    :align: center


In order to use this functionality, **Fast-DDS Discovery Server** can be set using
the :ref:`XML configuration for Participants <DS_setup_concepts>`.
Furthermore, Fast DDS provides an easier way to set a **Discovery Server** communication using
the ``fastdds`` :ref:`CLI tool <ffastddscli_cli>` and an :ref:`environment variable <env_vars>`,
which are going to be used along this tutorial.
For a more detailed explanation about the configuration of the Discovery Server,
visit :ref:`discovery_server`.


Prerequisites
-------------

This tutorial assumes you have a `working Foxy ROS 2 installation <https://index.ros.org/doc/ros2/Installation/>`__.
In case your installation is using a Fast DDS version lower than v2.0.2 you could not use the ``fastdds`` tool.
You could update your repository to use a different Fast DDS version,
or :ref:`set the discovery server by Fast-DDS XML QoS configuration <discovery_server>`.


Run the demo
------------

The ``talker-listener`` ROS 2 demo allows to create a *talker* node that publishes a *Hello World* message every second,
and a *listener* node that listens to these messages.

By `Sourcing ROS 2 <https://index.ros.org/doc/ros2/Tutorials/Configuring-ROS2-Environment/>`__
you will get access to the CLI of *Fast DDS*: ``fastdds``.
This CLI gives access to the :ref:`discovery tool <cli_discovery>`,
which allows to launch a server. This server will manage the discovery process for the nodes that connect to it.

.. important::

    Do not forget to `source ROS 2 <https://index.ros.org/doc/ros2/Tutorials/Configuring-ROS2-Environment/>`__
    in every new terminal opened.


Setup Discovery Server
^^^^^^^^^^^^^^^^^^^^^^^

Start by launching a server with id 0, with port 11811 and listening on all available interfaces.

Open a new terminal and run:

.. code-block:: console

    fastdds discovery -i 0


Launch node listener
^^^^^^^^^^^^^^^^^^^^

Execute the listener demo, that will listen in ``/chatter`` topic.

In a new terminal, set the environment variable ``ROS_DISCOVERY_SERVER`` to use *Discovery Server*.
(Do not forget to source ROS 2 in every new terminal)

.. code-block:: console

    export ROS_DISCOVERY_SERVER=127.0.0.1:11811

Afterwards, launch the listener node. Use the argument ``--remap __node:=listener_discovery_server``
to change the node's name for future purpose.

.. code-block:: console

    ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener_discovery_server

This process will create a ROS 2 node, that will automatically create a client for the *Discovery Server*
and use the server created previously to run the discovery protocol.


Launch node talker
^^^^^^^^^^^^^^^^^^

Open a new terminal and set the environment variable as before, so the node raises a client for the discovery protocol.

.. code-block:: console

    export ROS_DISCOVERY_SERVER=127.0.0.1:11811
    ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker_discovery_server

Now, we should see the talker publishing *Hello World* messages, and the listener receiving these messages.


Demonstrate Discovery Server execution
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

So far, there is not proof that this example and the standard talker-listener example run differently.
For this purpose, run another node that is not connected to our Discovery Server.
Just run a new listener (listening in ``/chatter`` topic by default) in a new terminal and check that it is
not connected to the talker already running.

.. code-block:: console

    ros2 run demo_nodes_cpp listener --ros-args --remap __node:=simple_listener

In this case, we should not see the listener receiving the messages.

To finally verify that everything is running correctly, a new talker can be created using the
*simple discovery protocol*.

.. code-block:: console

    ros2 run demo_nodes_cpp talker --ros-args --remap __node:=simple_talker

Now we should see the listener *simple_listener* receiving the messages from *simple_talker* but not the other
messages from *talker_discovery_server*.


Advance user cases
------------------

The following paragraphs are going to show different features of the Discovery Server
that allows to hold a robust structure over the node's network.


Server Redundancy
^^^^^^^^^^^^^^^^^

By using the Fast DDS tool, several servers can be created, and the nodes can be connected to as many
servers as desired. This allows to have a safe redundancy network that will work even if some servers or
nodes shut down unexpectedly.
Next schema shows a simple architecture that will work with server redundancy:

.. image:: /01-figures/fast_dds/discovery/ds_redundancy_example.svg
    :align: center

In different terminals, run the next code to establish a communication over redundant servers.

.. code-block:: console

    fastdds discovery -i 0 -l 127.0.0.1 -p 11811

.. code-block:: console

    fastdds discovery -i 1 -l 127.0.0.1 -p 11888

``-i N`` means server with id N. When referencing the servers with ``ROS_DISCOVERY_SERVER``,
server ``0`` must be in first place and server ``1`` in second place.

.. code-block:: console

    export ROS_DISCOVERY_SERVER="127.0.0.1:11811;127.0.0.1:11888"
    ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker

.. code-block:: console

    export ROS_DISCOVERY_SERVER="127.0.0.1:11811;127.0.0.1:11888"
    ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener

Now, if one of these servers fails, there would still be discovery communication between nodes.


Backup Server
^^^^^^^^^^^^^

*Fast DDS* Discovery Server allows to easily build a server with a **backup** functionality.
This allows the server to retake the last state it saved in case of a shutdown.

.. image:: /01-figures/fast_dds/discovery/ds_backup_example.svg
    :align: center

In different terminals, run the next code to establish a communication over a backup server.

.. code-block:: console

    fastdds discovery -i 0 -l 127.0.0.1 -p 11811 -b

.. code-block:: console

    export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
    ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker

.. code-block:: console

    export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
    ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener

Several backup files are created in the path the server has run.
Two ``SQLite`` files and two ``json`` files that contains the information required to
raise a new server in case of failure, avoiding the whole discovery process to happen again and
without losing information.


Discovery partitions
^^^^^^^^^^^^^^^^^^^^

The **Discovery Server** communication could be used with different servers to split in virtual
partitions the discovery info.
This means that two endpoints only would know each other if there is a server or a server network
between them.
We are going to execute an example with two different independent servers.
The following image shows a schema of the architecture desired:

.. image:: /01-figures/fast_dds/discovery/ds_partition_example.svg
    :align: center

With this schema *Listener 1* will be connected to *Talker 1* and *Talker 2*, as they
share *Server 1*.
*Listener 2* will connect with *Talker 1* as they share *Server 2*.
But *Listener 2* will not hear the messages from *Talker 2* because they do not
share any server or servers' network that connect them.

Run the first server listening in localhost in default port 11811.

.. code-block:: console

    fastdds discovery -i 0 -l 127.0.0.1 -p 11811

In another terminal run the second server listening in localhost in port another port, in this case 11888.

.. code-block:: console

    fastdds discovery -i 1 -l 127.0.0.1 -p 11888

Now, run each node in a different terminal. Use the *environment variable* ``ROS_DISCOVERY_SERVER`` to decide which
server they are connected to. Be aware that the ids must match (:ref:`env_vars`).

.. code-block:: console

    export ROS_DISCOVERY_SERVER="127.0.0.1:11811;127.0.0.1:11888"
    ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker_1

.. code-block:: console

    export ROS_DISCOVERY_SERVER="127.0.0.1:11811;127.0.0.1:11888"
    ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener_1

.. code-block:: console

    export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
    ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker_2

.. code-block:: console

    export ROS_DISCOVERY_SERVER=";127.0.0.1:11888"
    ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener_2

We should see how *Listener 1* is receiving double messages, while *Listener 2* is in a different
partition from *Talker 2* and so it does not listen to it.

.. note::

    Once two endpoints know each other, they do not need the server network between them to
    listen to each other messages.


ROS 2 Introspection
-------------------

ROS 2 Command Line Interface (CLI) implements several introspection features to analyze the behaviour of a ROS2
execution.
These features (i.e. `rosbag`, `topic list`, etc.) are very helpful to understand a ROS 2 working network.

Most of these features use the DDS capability to share any topic information with every exiting participant.
However, the new :ref:`ros2_discovery_server_v2` implements a traffic network reduction
that limits the discovery data between nodes that do not share a topic.
This means that not every node will receive every topic data unless it has a reader in that topic.
As most of ROS 2 CLI Introspection is executed by adding a node into the network (some of them use ROS 2 Daemon,
and some create their own nodes), using Discovery Server v2 we will find that most of these functionalities are
limited and do not have all the information.

The Discovery Server v2 functionality allows every node running as a **Server** (a kind of *Participant type*)
to know and share all the participants and topics information with every other server matched.
In this sense, a server can be configured alongside ROS 2 introspection, since then the introspection tool will
be able to discover every entity in the network that is using the Discovery Server protocol.


Daemon's related commands
^^^^^^^^^^^^^^^^^^^^^^^^^

The ROS 2 Daemon is used in several ROS 2 CLI introspection commands. It adds a ROS 2 Node to the network in order
to receive all the data sent.
This section will explain how to use ROS 2 CLI with ROS 2 Daemon running as a **Server**.
This will allow the Daemon to know all the Node's graph and to receive every topic and endpoint information.

Fast DDS CLI can execute a Discovery Server, but it will spawn the Server in the actual process (or in a new one),
and we want to run ROS 2 Daemon process as the Discovery Server.
For this purpose, Fast DDS XML configuration can be used in order to preconfigure every new node that is created
with this configuration exported.

Below you can find a XML configuration file which will configure every new participant as a Discovery Server.
It is important to notice that, in order to create a Discovery Server, port and a GUID (id) must be specified.
so only one participant at a time can be created with this configuration file.
Creating more than one participant with the same file will lead to an error.

* :download:`XML Discovery Server configuration file <discovery_server_configuration_file.xml>`

First of all, instantiate a ROS 2 Daemon using this configuration (remember to source ROS 2 installation in every
new terminal).

.. code-block:: console

    export FASTRTPS_DEFAULT_PROFILES_FILE=discovery_server_configuration_file.xml
    ros2 daemon stop
    ros2 daemon start

Run a talker and a listener that will discover each other by the Server/Daemon (notice that ``ROS_DISCOVERY_SERVER``
configuration is the same as the one in `discovery_server_configuration_file.xml`).

.. code-block:: console

    export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
    ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener

.. code-block:: console

    export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
    ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker

Now the Daemon can be used to introspect the network (``ROS_DISCOVERY_SERVER`` must be exported because new nodes
are created within this tools' executions).

.. code-block:: console

    export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
    ros2 topic list
    ros2 node info /talker
    ros2 topic info /chatter
    ros2 topic echo /chatter

Be careful to use a different terminal than that of the Daemon for each execution, as some of the introspection
tools instantiate their own nodes, and only one node could be instantiated with
`discovery_server_configuration_file.xml`
exported.

We can also see the Node's Graph using the ROS 2 tool `rqt_graph` as follows.

.. code-block:: console

    export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
    rqt_grap


No Daemon commands
^^^^^^^^^^^^^^^^^^

Some ROS 2 CLI tools can be executed without the ROS 2 Daemon.
In order for these tools to connect with a Discovery Server and receive all the topics information
they need to be instantiated as a Server different than the main one, because they are
volatile nodes.

We can configure a Discovery Server that is connected to the main Server using a similar
configuration file than the one in the previous section.

* :download:`XML Secondary Discovery Server configuration file <secondary_discovery_server_configuration_file.xml>`

Following the previous configuration, build a simple system with a talker and a listener.

.. code-block:: console

    export FASTRTPS_DEFAULT_PROFILES_FILE=discovery_server_configuration_file.xml
    ros2 daemon stop
    ros2 daemon start

.. code-block:: console

    export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
    ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener

.. code-block:: console

    export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
    ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker

Continue using the ROS 2 CLI with ``--no-daemon`` option with the new configuration.
New nodes will connect with the existing Server and will know every topic.
Exporting ``ROS_DISCOVERY_SERVER`` is not needed as the remote server has been configured in the xml file.

.. code-block:: console

    export FASTRTPS_DEFAULT_PROFILES_FILE=secondary_discovery_server_configuration_file.xml
    ros2 topic list --no-daemon
    ros2 node info /talker --no-daemon
    ros2 topic info /chatter --no-daemon
    ros2 topic echo /chatter --no-daemon


Compare Discovery Server with Simple Discovery
-----------------------------------------------

In order to compare the ROS2 execution using *Simple Discovery* or *Discovery Service*, two scripts that
execute a talker and many listeners and analyze the network traffic during this time are provided.
For this experiment, ``tshark`` is required to be installed on your system.
The configuration file is mandatory in order to avoid using intra-process mode.

.. note::

    These scripts require a Discovery Server closure feature that is only available from
    Fast DDS v2.1.0 and forward.
    In order to use this functionality, compile ROS 2 with Fast DDS v2.1.0 or higher.

These scripts' functionalities are references for advance purpose and their study is left to the user.

* :download:`bash network traffic generator <generate_discovery_packages.bash>`

* :download:`python3 graph generator <discovery_packets.py>`

* :download:`XML configuration <no_intraprocess_configuration.xml>`

Run the bash script with the *setup* path to source ROS2 as argument.
This will generate the traffic trace for simple discovery.
Executing the same script with second argument ``SERVER``, it will generates the trace for service discovery.

.. note::

    Depending on your configuration of ``tcpdump``, this script may require ``sudo`` privileges to read traffic across
    your network device.

After both executions are done, run the python script to generates a graph similar to the one below:

.. code-block:: console

    $ export FASTRTPS_DEFAULT_PROFILES_FILE="no_intraprocess_configuration.xml"
    $ sudo bash generate_discovery_packages.bash ~/ros2_foxy/install/local_setup.bash
    $ sudo bash generate_discovery_packages.bash ~/ros2_foxy/install/local_setup.bash SERVER
    $ python3 discovery_packets.py

.. image:: /01-figures/fast_dds/discovery/discovery_packets.svg
    :align: center

This graph is the result of a  is a specific example, the user can execute the scripts and watch their own results.
It can easily be seen how the network traffic is reduced when using *Discovery Service*.

The reduction in traffic is a result of avoiding every node announcing itself and waiting a response from every other
node in the net.
This creates a huge amount of traffic in large architectures.
This reduction from this method increases with the number of Nodes, making this architecture more scalable than the
simple one.

Since *Fast DDS* v2.0.2 the new Discovery Server v2 is available, substituting the old Discovery Server.
In this new version, those nodes that do not share topics will not know each other, saving the whole discovery data
required to connect them and their endpoints.
Notice that this is not this example case, but even though the massive reduction could be appreciate
due to the hidden architecture topics of ROS 2 nodes.
