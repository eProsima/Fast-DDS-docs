<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="ros2-discovery-server"></a>

<a id="use-ros-2-with-fast-dds-discovery-server"></a>

# Use ROS 2 with Fast-DDS Discovery Server

This section explains how to run some ROS 2 examples using the Discovery Servers
as discovery communication.
In order to get more information about the specific use of this configuration,
please check the [Discovery Server Documentation](../../discovery/discovery_server.md#discovery-server)
or read the [common use cases](../../use_cases/wifi/discovery_server_use_case.md#discovery-server-use-case) for this configuration.

The following tutorial gathers the steps to check this functionality and learn how to use it with ROS 2.

The [Simple Discovery Protocol](../../discovery/simple.md#simple-disc-settings) is the
standard protocol defined in the [DDS standard](https://www.omg.org/omg-dds-portal/).
However, it has certain known disadvantages in some scenarios, mainly:

* It does not **Scale** efficiently, as the number of exchanged packets highly increases as new nodes are added.
* It requires **Multicasting** capabilities that may not work reliably in some scenarios, e.g. WiFi.

The **Discovery Server** provides a Client-Server Architecture that allows
the nodes to connect with each other using an intermediate server.
Each node will work as a *Client*, sharing its info with the *Discovery Server* and receiving
the discovery information from it.
This means that the network traffic is highly reduced in big systems, and it does not require *Multicasting*.

![image](01-figures/fast_dds/discovery/ds_explanation.svg)

These **Discovery Servers** can be independent, duplicated or connected with each other in order to create
redundancy over the network and avoid having a *Single-Point-Of-Failure*.

<a id="ros2-discovery-server-versions"></a>

<a id="ros2-discovery-server-v2"></a>

<a id="discovery-server-in-fast-dds-and-fast-dds-pro"></a>

## Discovery Server in Fast DDS and Fast DDS Pro

The Discovery Server is available both in the open-source version of *Fast DDS* and in *Fast DDS Pro*.
The client-server architecture described above, and therefore the removal of the *Multicasting* requirement, is
common to both of them.
What changes is how much discovery information the *server* sends to each of its *clients*:

|                                              | Fast DDS (open source)                                                                                                                                                                                                                                                                           | Fast DDS Pro **[Pro]**                                                                                                                                                                                                            |
|----------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Discovery information received by a *client* | All the information known by the *server*.<br/>Every *client* behaves as a [`SUPER_CLIENT`](../../api_reference/rtps/attributes/DiscoveryProtocol.md#_CPPv4N8eprosima7fastdds4rtps17DiscoveryProtocol12SUPER_CLIENTE) (see [Key concepts](../../discovery/discovery_server.md#ds-key-concepts)). | Only the information the *client* actually needs.<br/>The *server* runs a “matching” algorithm that uses the *topic* of the different nodes to decide if two<br/>nodes must be connected, or they could be left unmatched.        |
| Discovery traffic                            | Highly reduced with respect to [Simple Discovery](../../discovery/simple.md#simple-disc-settings), as nodes no longer announce<br/>themselves to, and wait for a response from, every other node in the network.                                                                                 | Further reduced, as the discovery data of nodes that do not share a topic is never sent.<br/>This is especially relevant in large systems, where each node typically matches only a small subset of<br/>the nodes in the network. |
| ROS 2 introspection tools                    | Receive the whole node graph as soon as they join the *server* network.                                                                                                                                                                                                                          | Need to be configured as **Super Client** to receive the whole node graph<br/>(see [ROS 2 Introspection](#ros2-discovery-server-introspection)).                                                                                  |

The following schema represents the decrease of the discovery packages achieved by the *Fast DDS Pro* filtering:

![image](01-figures/fast_dds/discovery/ds1vs2.svg)

This architecture reduces the number of packages sent between the server and the different clients dramatically.

#### NOTE
From *Fast DDS* **[Pro]** v3.8.0 onwards, this filter is also applied between servers: each server announces the topics
used by the nodes it serves directly, and receives only the discovery information matching those topics.
Please refer to [Topic interest propagation](../../discovery/discovery_server.md#ds-topic-interest)  for further details.

In the following graph, the reduction in traffic network over the discovery phase for a
RMF Clinic demo use case, is shown:

![image](01-figures/fast_dds/discovery/discovery_server_v2_performance.svg)

#### NOTE
This tutorial uses the open-source version of *Fast DDS*, which is the one distributed with ROS 2.
Everything explained from this point on applies to it, and no *Fast DDS Pro* feature is required to follow it.

In order to use it, the **Fast-DDS Discovery Server** can be set using
the XML configuration for Participants.
Furthermore, Fast DDS provides an easier way to set a **Discovery Server** communication using
the `fastdds` [CLI tool](../../../fastddscli/cli/cli.md#ffastddscli-cli) and an [environment variable](../../env_vars/env_vars.md#env-vars),
which are going to be used along this tutorial.
For a more detailed explanation about the configuration of the Discovery Server,
visit [Discovery Server](../../discovery/discovery_server.md#discovery-server).

<a id="prerequisites"></a>

## Prerequisites

This tutorial assumes you have at least a
[working Foxy ROS 2 installation](https://index.ros.org/doc/ros2/Installation/).
In case your installation is using a Fast DDS version lower than v2.0.2 you could not use the `fastdds` tool.
You could update your repository to use a different Fast DDS version,
or [set the discovery server by Fast-DDS XML QoS configuration](../../discovery/discovery_server.md#discovery-server).

#### NOTE
This tutorial can also be run in *Galactic* exporting the environment variable that selects *Fast DDS* as the
middleware layer:

```console
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

<a id="run-the-demo"></a>

## Run the demo

The `talker-listener` ROS 2 demo allows to create a *talker* node that publishes a *Hello World* message every second,
and a *listener* node that listens to these messages.

By [Sourcing ROS 2](https://index.ros.org/doc/ros2/Tutorials/Configuring-ROS2-Environment/)
you will get access to the CLI of *Fast DDS*: `fastdds`.
This CLI gives access to the [discovery tool](../../../fastddscli/cli/cli.md#cli-discovery),
which allows to launch a server. This server will manage the discovery process for the nodes that connect to it.

#### IMPORTANT
Do not forget to [source ROS 2](https://index.ros.org/doc/ros2/Tutorials/Configuring-ROS2-Environment/)
in every new terminal opened.

<a id="setup-discovery-server"></a>

### Setup Discovery Server

Start by launching a server with port 11811 and listening on all available interfaces, which is the default
configuration for the `fastdds` [CLI tool](../../../fastddscli/cli/cli.md#ffastddscli-cli).

Open a new terminal and run:

```console
fastdds discovery
```

<a id="launch-node-listener"></a>

### Launch node listener

Execute the listener demo, that will listen in `/chatter` topic.

In a new terminal, set the environment variable `ROS_DISCOVERY_SERVER` to use *Discovery Server*.
(Do not forget to source ROS 2 in every new terminal)

```console
export ROS_DISCOVERY_SERVER=127.0.0.1:11811
```

Afterwards, launch the listener node. Use the argument `--remap __node:=listener_discovery_server`
to change the node’s name for future purpose.

```console
ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener_discovery_server
```

This process will create a ROS 2 node, that will automatically create a client for the *Discovery Server*
and use the server created previously to run the discovery protocol.

<a id="launch-node-talker"></a>

### Launch node talker

Open a new terminal and set the environment variable as before, so the node raises a client for the discovery protocol.

```console
export ROS_DISCOVERY_SERVER=127.0.0.1:11811
ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker_discovery_server
```

Now, we should see the talker publishing *Hello World* messages, and the listener receiving these messages.

<a id="demonstrate-discovery-server-execution"></a>

### Demonstrate Discovery Server execution

So far, there is not proof that this example and the standard talker-listener example run differently.
For this purpose, run another node that is not connected to our Discovery Server.
Just run a new listener (listening in `/chatter` topic by default) in a new terminal and check that it is
not connected to the talker already running.

```console
ros2 run demo_nodes_cpp listener --ros-args --remap __node:=simple_listener
```

In this case, we should not see the listener receiving the messages.

To finally verify that everything is running correctly, a new talker can be created using the
*simple discovery protocol*.

```console
ros2 run demo_nodes_cpp talker --ros-args --remap __node:=simple_talker
```

Now we should see the listener *simple_listener* receiving the messages from *simple_talker* but not the other
messages from *talker_discovery_server*.

<a id="advance-user-cases"></a>

## Advance user cases

The following paragraphs are going to show different features of the Discovery Server
that allows to hold a robust structure over the node’s network.

<a id="server-redundancy"></a>

### Server Redundancy

By using the Fast DDS tool, several servers can be created, and the nodes can be connected to as many
servers as desired. This allows to have a safe redundancy network that will work even if some servers or
nodes shut down unexpectedly.
Next schema shows a simple architecture that will work with server redundancy:

![image](01-figures/fast_dds/discovery/ds_redundancy_example.svg)

In different terminals, run the next code to establish a communication over redundant servers.

```console
fastdds discovery -l 127.0.0.1 -p 11811
```

```console
fastdds discovery -l 127.0.0.1 -p 11888
```

```console
export ROS_DISCOVERY_SERVER="127.0.0.1:11811;127.0.0.1:11888"
ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker
```

```console
export ROS_DISCOVERY_SERVER="127.0.0.1:11811;127.0.0.1:11888"
ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener
```

Now, if one of these servers fails, there would still be discovery communication between nodes.

<a id="backup-server"></a>

### Backup Server

*Fast DDS* Discovery Server allows to easily build a server with a **backup** functionality.
This allows the server to retake the last state it saved in case of a shutdown.

![image](01-figures/fast_dds/discovery/ds_backup_example.svg)

In different terminals, run the next code to establish a communication over a backup server.

```console
fastdds discovery -l 127.0.0.1 -p 11811 -b
```

```console
export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker
```

```console
export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener
```

Several backup files are created in the path the server has run.
Two `SQLite` files and two `json` files that contains the information required to
raise a new server in case of failure, avoiding the whole discovery process to happen again and
without losing information.

<a id="discovery-partitions"></a>

### Discovery partitions

The **Discovery Server** communication could be used with different servers to split in virtual
partitions the discovery info.
This means that two endpoints only would know each other if there is a server or a server network
between them.
We are going to execute an example with two different independent servers.
The following image shows a schema of the architecture desired:

![image](01-figures/fast_dds/discovery/ds_partition_example.svg)

With this schema *Listener 1* will be connected to *Talker 1* and *Talker 2*, as they
share *Server 1*.
*Listener 2* will connect with *Talker 1* as they share *Server 2*.
But *Listener 2* will not hear the messages from *Talker 2* because they do not
share any server or servers’ network that connect them.

Run the first server listening in localhost on default port 11811.

```console
fastdds discovery -l 127.0.0.1 -p 11811
```

In another terminal run the second server listening in localhost on another port, in this case 11888.

```console
fastdds discovery -l 127.0.0.1 -p 11888
```

Now, run each node in a different terminal. Use the *environment variable* `ROS_DISCOVERY_SERVER` to decide which
server they are connected to.

```console
export ROS_DISCOVERY_SERVER="127.0.0.1:11811;127.0.0.1:11888"
ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker_1
```

```console
export ROS_DISCOVERY_SERVER="127.0.0.1:11811;127.0.0.1:11888"
ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener_1
```

```console
export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker_2
```

```console
export ROS_DISCOVERY_SERVER=";127.0.0.1:11888"
ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener_2
```

We should see how *Listener 1* is receiving double messages, while *Listener 2* is in a different
partition from *Talker 2* and so it does not listen to it.

#### NOTE
Once two endpoints know each other, they do not need the server network between them to
listen to each other messages.

<a id="ros2-discovery-server-introspection"></a>

<a id="ros-2-introspection"></a>

## ROS 2 Introspection

ROS 2 Command Line Interface (CLI) implements several introspection features to analyze the behaviour of a ROS 2
execution.
These features (i.e. rosbag, topic list, etc.) are very helpful to understand a ROS 2 working network.

Most of these features use the DDS capability to share any topic information with every exiting participant.
However, when using the Discovery Server there are no multicast announcements to rely on, so these tools must join
the *server* network in order to be aware of the nodes running in it.
As most of ROS 2 CLI Introspection is executed by adding a node into the network (some of them use ROS 2 Daemon,
and some create their own nodes), those nodes must be configured to connect to the **Server**.

The Discovery Server allows every node running as a [`SUPER_CLIENT`](../../api_reference/rtps/attributes/DiscoveryProtocol.md#_CPPv4N8eprosima7fastdds4rtps17DiscoveryProtocol12SUPER_CLIENTE), a kind of **Client** that
connects to a [`SERVER`](../../api_reference/rtps/attributes/DiscoveryProtocol.md#_CPPv4N8eprosima7fastdds4rtps17DiscoveryProtocol6SERVERE), from which it receives all the available discovery information (instead of just what it needs).
In this sense, ROS 2 introspection tools can be configured as **Super Client**, thus being able to discover every entity
that is using the Discovery Server protocol within the network.
This also holds when several servers are connected to each other: from *Fast DDS* **[Pro]** v3.8.0 onwards, a server with a
directly connected **Super Client** requests the whole discovery information from its known servers
(see [Topic interest propagation](../../discovery/discovery_server.md#ds-topic-interest) ).

<a id="daemon-s-related-commands"></a>

### Daemon’s related commands

The ROS 2 Daemon is used in several ROS 2 CLI introspection commands.
It adds a ROS 2 Node to the network in order to receive all the data sent.
In order for the ROS 2 CLI to work when using Discover Server discovery mechanism, the ROS 2 Daemon needs to be
configured as **Super Client**.
Therefore, this section is devoted to explain how to use ROS 2 CLI with ROS 2 Daemon running as a **Super Client**.
This will allow the Daemon to discover the entire Node graph, and to receive every topic and endpoint information.
To do so, a Fast DDS XML configuration file is used to configure the ROS 2 Daemon and CLI tools.

#### WARNING
Although it is possible to run the ROS 2 Daemon as a **Server**, this is not recommended since the daemon will stop
after two hours of inactivity, taking the **Server** down with it.

Below you can find a XML configuration file which will configure every new participant as a **Super Client**.

* [`XML Super Client configuration file`](../../../_downloads/9f9e92b14612364b742c8ecde24b3d24/super_client_configuration_file.xml)

First of all, instantiate a Discovery Server using [Fast DDS CLI](../../../fastddscli/cli/cli.md#cli-discovery)

```console
fastdds discovery -l 127.0.0.1 -p 11811
```

Run a talker and a listener that will discover each other through the Server (notice that `ROS_DISCOVERY_SERVER`
configuration is the same as the one in super_client_configuration_file.xml).

```console
export ROS_DISCOVERY_SERVER=127.0.0.1:11811
ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener
```

```console
export ROS_DISCOVERY_SERVER=127.0.0.1:11811
ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker
```

Then, instantiate a ROS 2 Daemon using the **Super Client** configuration (remember to source ROS 2 installation in
every new terminal).

```console
export FASTDDS_DEFAULT_PROFILES_FILE=super_client_configuration_file.xml
ros2 daemon stop
ros2 daemon start
ros2 topic list
ros2 node info /talker
ros2 topic info /chatter
ros2 topic echo /chatter
```

We can also see the Node’s Graph using the ROS 2 tool rqt_graph as follows (you may need to press the refresh button):

```console
export FASTDDS_DEFAULT_PROFILES_FILE=super_client_configuration_file.xml
rqt_graph
```

<a id="no-daemon-commands"></a>

### No Daemon commands

Some ROS 2 CLI tools can be executed without the ROS 2 Daemon.
In order for these tools to connect with a Discovery Server and receive all the topics information
they need to be instantiated as a **Super Client** that connects to the **Server**.

Following the previous configuration, build a simple system with a talker and a listener.
First, run a **Server**:

```console
fastdds discovery -l 127.0.0.1 -p 11811
```

Then, run the talker and listener is separate terminals:

```console
export ROS_DISCOVERY_SERVER=127.0.0.1:11811
ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener
```

```console
export ROS_DISCOVERY_SERVER=127.0.0.1:11811
ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker
```

Continue using the ROS 2 CLI with `--no-daemon` option with the new configuration.
New nodes will connect with the existing Server and will know every topic.
Exporting `ROS_DISCOVERY_SERVER` is not needed as the remote server has been configured in the xml file.

```console
export FASTDDS_DEFAULT_PROFILES_FILE=super_client_configuration_file.xml
ros2 topic list --no-daemon
ros2 node info /talker --no-daemon --spin-time 2
```

<a id="compare-discovery-server-with-simple-discovery"></a>

## Compare Discovery Server with Simple Discovery

In order to compare the ROS 2 execution using *Simple Discovery* or *Discovery Server*, two scripts that
execute a talker and many listeners and analyze the network traffic during this time are provided.
For this experiment, `tshark` is required to be installed on your system.
The configuration file is mandatory in order to avoid using intra-process mode.

#### NOTE
These scripts require a Discovery Server closure feature that is only available from
Fast DDS v2.1.0 and forward.
In order to use this functionality, compile ROS 2 with Fast DDS v2.1.0 or higher.

These scripts’ functionalities are references for advance purpose and their study is left to the user.

* [`bash network traffic generator`](../../../_downloads/9ce9d7ec6c35a66eb3009a1d50a00efe/generate_discovery_packages.bash)
* [`python3 graph generator`](../../../_downloads/8d87241c358b7fcab77725d1d612adfd/discovery_packets.py)
* [`XML configuration`](../../../_downloads/7d9dda06c005bddc3c47f6d2cc9865e2/no_intraprocess_configuration.xml)

Run the bash script with the *setup* path to source ROS 2 as argument.
This will generate the traffic trace for simple discovery.
Executing the same script with second argument `SERVER`, it will generates the trace for service discovery.

#### NOTE
Depending on your configuration of `tcpdump`, this script may require `sudo` privileges to read traffic across
your network device.

After both executions are done, run the python script to generates a graph similar to the one below:

```console
$ export FASTDDS_DEFAULT_PROFILES_FILE="no_intraprocess_configuration.xml"
$ sudo bash generate_discovery_packages.bash ~/ros2_foxy/install/local_setup.bash
$ sudo bash generate_discovery_packages.bash ~/ros2_foxy/install/local_setup.bash SERVER
$ python3 discovery_packets.py
```

![image](01-figures/fast_dds/discovery/discovery_packets.svg)

This graph is the result of a  is a specific example, the user can execute the scripts and watch their own results.
It can easily be seen how the network traffic is reduced when using *Discovery Service*.

The reduction in traffic is a result of avoiding every node announcing itself and waiting a response from every other
node in the net.
This creates a huge amount of traffic in large architectures.
This reduction from this method increases with the number of Nodes, making this architecture more scalable than the
simple one.

This reduction can be appreciated even in a small example like this one, due to the hidden architecture topics of
ROS 2 nodes.
