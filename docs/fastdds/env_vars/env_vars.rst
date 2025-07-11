.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include
.. include:: ../../03-exports/roles.include

.. _env_vars:

Environment variables
=====================

This is the list of environment variables that affect the behavior of *Fast DDS*:

.. _env_vars_fastdds_default_profiles_file:

``FASTDDS_DEFAULT_PROFILES_FILE``
----------------------------------

Defines the location of the default profile configuration XML file.
If this variable is set and its value corresponds with an existing file, *Fast DDS* will load its profiles.
For more information about XML profiles, please refer to :ref:`xml_profiles`.

+------------------------------------------------------------------+
| **Linux**                                                        |
+------------------------------------------------------------------+
| .. code-block:: bash                                             |
|                                                                  |
|     export FASTDDS_DEFAULT_PROFILES_FILE=/home/user/profiles.xml |
+------------------------------------------------------------------+
| **Windows**                                                      |
+------------------------------------------------------------------+
| .. code-block:: bash                                             |
|                                                                  |
|     set FASTDDS_DEFAULT_PROFILES_FILE=C:\profiles.xml            |
+------------------------------------------------------------------+

.. _env_vars_skip_default_xml:

``SKIP_DEFAULT_XML``
--------------------

Skips looking for a default profile configuration XML file.
If this variable is set to `1`, *Fast DDS* will load the configuration parameters directly from the classes'
definitions without looking for the *DEFAULT_FASTDDS_PROFILES.xml* in the working directory.
For more information about XML profiles, please refer to :ref:`xml_profiles`.

+------------------------------------------------------------------+
| **Linux**                                                        |
+------------------------------------------------------------------+
| .. code-block:: bash                                             |
|                                                                  |
|    export SKIP_DEFAULT_XML=1                                     |
+------------------------------------------------------------------+
| **Windows**                                                      |
+------------------------------------------------------------------+
| .. code-block:: bash                                             |
|                                                                  |
|    set SKIP_DEFAULT_XML=1                                        |
+------------------------------------------------------------------+

.. _env_vars_builtin_transports:

``FASTDDS_BUILTIN_TRANSPORTS``
----------------------------------

Setting this variable allows to modify the builtin transports that are initialized during the |DomainParticipant|
creation. It is a simple way of changing the default configuration of the :ref:`comm-transports-configuration`
and it directly affects how DDS entities communicate between them.

All existing values, along with a brief description, are shown below:

+----------------------------+------------------------------------------------------------------------------+
| Builtin Transports Options | Description                                                                  |
+============================+==============================================================================+
| ``NONE``                   | No transport will be instantiated. Hence, the user must manually add         |
|                            | the desired  transports. Otherwise, the participant creation will fail.      |
+----------------------------+------------------------------------------------------------------------------+
| ``DEFAULT``                | UDPv4 and SHM transports will be instantiated. SHM transport has priority    |
|                            | over the UDPv4  transport. Meaning that SHM will always be used              |
|                            | when possible.                                                               |
+----------------------------+------------------------------------------------------------------------------+
| ``DEFAULTv6``              | UDPv6 and SHM transports will be instantiated. SHM transport has priority    |
|                            | over the UDPv4  transport. Meaning that SHM will always be used              |
|                            | when possible.                                                               |
+----------------------------+------------------------------------------------------------------------------+
| ``SHM``                    | Only a SHM transport will be instantiated.                                   |
+----------------------------+------------------------------------------------------------------------------+
| ``UDPv4``                  | Only a UDPv4 transport will be instantiated.                                 |
+----------------------------+------------------------------------------------------------------------------+
| ``UDPv6``                  | Only a UDPv6 transport will be instantiated.                                 |
+----------------------------+------------------------------------------------------------------------------+
| ``LARGE_DATA``             | UDPv4, TCPv4, and SHM transports will be instantiated. However, UDP will     |
|                            | only be used  for multicast announcements during the participant             |
|                            | discovery phase (see :ref:`disc_phases`)  while the participant              |
|                            | liveliness and the application data delivery occurs over TCP or SHM.         |
|                            | This configuration is useful when working with large data.(See               |
|                            | :ref:`use-case-tcp`).                                                        |
+----------------------------+------------------------------------------------------------------------------+

.. note::
    The environment variable is only used in the case where |TransportConfigQos::use_builtin_transports-api| is set
    to ``TRUE``. In any other case, the environment variable has no effect.

.. note::
     TCPv4 transport is initialized with the following configuration:

     * |TCPTransportDescriptor::calculate_crc-api|, |TCPTransportDescriptor::check_crc-api| and
       |TCPTransportDescriptor::apply_security-api| are set to false.
     * |TCPTransportDescriptor::enable_tcp_nodelay-api| is set to true.
     * |TCPTransportDescriptor::keep_alive_thread-api| and
       |TCPTransportDescriptor::accept_thread-api| use the default configuration.

.. _env_vars_large_data_options:

Configuring builtin transports options
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Fast DDS offers a straightforward method to configure three main parameters of every builtin transport via the
environment variable.
However, this feature proves particularly valuable when employing the ``LARGE_DATA`` builtin
transports option.
The ``LARGE_DATA`` mode has been designed to improve performance when working with large data.
However, according to each specific use case, the user might want to configure several options to better fit their
needs.
This mode can also be configured with the ``tcp_negotiation_timeout`` parameter:

+----------------------------+--------------------------------------------------------------------+-------------------+
| Builtin Transports Options | Description                                                        | Type              |
+============================+====================================================================+===================+
| ``max_msg_size``           | It determines the maximum message size that will be specified      | uint32_t          |
|                            | in the transport layer. Selecting a message size large             |                   |
|                            | enough to accommodate the largest data message will                |                   |
|                            | prevent fragmentation, which can significantly enhance             |                   |
|                            | the overall sending rate.                                          |                   |
+----------------------------+--------------------------------------------------------------------+-------------------+
| ``sockets_size``           | It determines the size of the send and receive socket buffers.     | uint32_t          |
|                            | This parameter needs to be higher or equal to the maximum          |                   |
|                            | message size specified in order to be valid.                       |                   |
+----------------------------+--------------------------------------------------------------------+-------------------+
| ``non_blocking``           | It determines whether to use non-blocking send calls or not.       | bool              |
|                            | When activated, the transport will discard messages if the         |                   |
|                            | socket buffers are full.                                           |                   |
+----------------------------+--------------------------------------------------------------------+-------------------+
| ``tcp_negotiation_timeout``| It determines the time to wait for logical port negotiation.       | uint32_t          |
|                            | Only valid if the ``LARGE_DATA`` mode is being used.               |                   |
|                            | It only accepts milliseconds.                                      |                   |
+----------------------------+--------------------------------------------------------------------+-------------------+

The environment variable accepts several types of units to specify the values of the parameters.
Also, both lowercase and uppercase letters are valid.
The following list shows the available units and their corresponding symbols:

+ ``B``: Bytes. This is the default unit, so it is not necessary to specify it.
+ ``KB``: Kilobytes.
+ ``MB``: Megabytes.
+ ``GB``: Gigabytes.
+ ``KIB``: Kibibyte.
+ ``MIB``: Mebibyte.
+ ``GIB``: Gibibyte.

.. code-block:: bash

    export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA?max_msg_size=200KB&sockets_size=1MB&non_blocking=true&tcp_negotiation_timeout=50

.. note::
    When working with ``LARGE_DATA``, it is recommended to set the ``max_msg_size`` and ``sockets_size`` to a value
    large enough to accommodate the largest data message and to set the ``non_blocking`` to ``TRUE``.
    Note that activating the ``non_blocking`` option with a small message size (with fragmentation)
    can lead to an increase of messages drop rate and produce undesired results.
    For more information, please refer to :ref:`use-case-large-data-options`.

.. warning::
    Setting a ``max_msg_size`` higher than 65500 KB is only possible when using the ``LARGE_DATA`` builtin transports
    option.
    Trying to set it with any other builtin transports will result in an error and the creation of the participant
    will fail.


.. _env_vars_ros_discovery_server:

``ROS_DISCOVERY_SERVER``
------------------------

.. warning::
    The environment variable is only used in the case where :ref:`discovery protocol<discovery_protocol>`
    is set to |SIMPLE|, |SERVER|, or |BACKUP|.
    In any other case, the environment variable has no effect.

Setting this variable configures the :ref:`DomainParticipant<dds_layer_domainParticipant>` to connect to one or more
*servers* using the :ref:`Discovery Server<discovery_server>` discovery mechanism.

* If ``ROS_DISCOVERY_SERVER`` is defined, and the ``DomainParticipant``'s :ref:`discovery protocol<discovery_protocol>`
  is set to |SIMPLE|, then Fast DDS will instead configure it as |CLIENT| of the given *server*.
* If ``ROS_DISCOVERY_SERVER`` is defined, and the ``DomainParticipant``'s :ref:`discovery protocol<discovery_protocol>`
  is set to |SERVER| or |BACKUP|, then the variable is used to add remote *servers* to the given *server*, leaving the
  :ref:`discovery protocol<discovery_protocol>` as |SERVER| or |BACKUP| respectively.

* The value of the variable must list the locator of the server in the form of:

  + An IPv4 address like ``192.168.2.23``. The UDP protocol is used by default. The UDP port can be appended using `:`
    as in ``192.168.2.23:35665``.
  + An IPv6 address that follows RFC3513_ address convention like ``1080::8:800:200C:417A``. Again, it uses the UDP
    protocol by default. An UDP port can be appended like in ``[1080::8:800:200C:417A]:35665``. Note the use of square
    brackets to avoid ambiguities.
  + TCPv4 specifier + IPv4 address like ``TCPv4:[127.0.0.1]``. The TCP protocol is used to communicate with the server.
    The TCP port can be appended using `:` as in ``TCPv4:[127.0.0.1]:42100``.
  + TCPv6 specifier + IPv6 address like ``TCPv6:[::1]``. The TCP protocol is used to communicate with the server. The
    TCP port can be appended using `:` as in ``TCPv6:[::1]:42100``.
  + A DNS name can be specified. This name will be used to query known hosts and available DNS servers to try to
    resolve valid IP addresses. Several formats are acceptable:

    - Plain domain name: ``eprosima.com``. This will include all available IP addresses.
    - Domain name + port: ``eprosima.com:35665``. As above but using a specific port.
    - UDPv4 specifier + domain name: ``UDPv4:[eprosima.com]``. Only the first IPv4 address resolved will be used.
    - UDPv4 specifier + domain name + port: ``UDPv4:[eprosima.com]:35665``. As above but using a specific port.
    - UDPv6 specifier + domain name: ``UDPv6:[<dns>]``. Only the first IPv6 address resolved will be used.
    - UDPv6 specifier + domain name + port: ``UDPv6:[<dns>]:35665``. As above but using a specific port.
    - TCPv4 specifier + domain name: ``TCPv4:[eprosima.com]``. Only the first IPv4 address resolver will be used.
    - TCPv4 specifier + domain name + port: ``TCPv4:[eprosima.com]:42100``. As above but using a specific port.
    - TCPv6 specifier + domain name: ``TCPv6:[<dns>]``. Only the first IPv4 address resolver will be used.
    - TCPv6 specifier + domain name + port: ``TCPv6:[<dns>]:42100``. As above but using a specific port.

* If no port is specified when using default UDP transport, the default port 11811 is used.
* If no port is specified when using TCP transport, the default port 42100 is used.
* To set more than one *server*'s address, they must be separated by semicolons.
* When using IPv6 with DNS, the specified domain name space (*<dns>*) must be able to resolve to an IPv6
  address. Otherwise an error will be raised.

The following example shows how to set the address of two remote discovery servers with addresses
'84.22.259.329:8888' and 'localhost:1234'.

+----------------------------------------------------------------------------+
| **Linux**                                                                  |
+----------------------------------------------------------------------------+
| .. code-block:: bash                                                       |
|                                                                            |
|    export ROS_DISCOVERY_SERVER="84.22.259.329:8888;localhost:1234"         |
+----------------------------------------------------------------------------+
| **Windows**                                                                |
+----------------------------------------------------------------------------+
| .. code-block:: bash                                                       |
|                                                                            |
|    set ROS_DISCOVERY_SERVER=84.22.259.329:8888;localhost:1234              |
+----------------------------------------------------------------------------+

.. important::
    IP addresses specified in ``ROS_DISCOVERY_SERVER`` must be either valid IPv4/IPv6 addresses or domain names.
    If a name can be resolved into several addresses, it is possible to either use them all or restrict the selection to
    the first IPv4 using the `UDPv4` or `TCPv4` prefixes or to the first IPv6 address using the `UDPv6` or `TCPv6`
    prefixes.

.. important::
    This environment variable can be changed at runtime adding new remote servers to a |SERVER|, |BACKUP| or |CLIENT|
    (that has been initialized with this environment variable previously) if loaded from an environment file using
    :ref:`env_vars_fastdds_environment_file`.

.. _env_vars_easy_mode:

``ROS2_EASY_MODE``
------------------

Setting ``ROS2_EASY_MODE`` to an IP value allows a participant to automatically enter the
`Discovery Server Easy Mode <https://docs.vulcanexus.org/en/latest/rst/enhancements/easy_mode/easy_mode.html>`__.
This mode completely disables **multicast communication**, and relies on Discovery Servers for discovery purposes.

With ``ROS2_EASY_MODE`` a new Discovery Server will be automatically spawned locally in the given
:ref:`domain<dds_layer_domain>`, pointing to another Discovery Server located in the specified IP.
If the specified IP belongs to the same host, it will only work in localhost, until another host connects to it.
If there exists a Discovery Server for that domain, the spawn process will be skipped, relying on the existing server
for discovery purposes.
Therefore, only one Discovery Server per host will be present in the domain.

In order for this variable to take effect, the participant must have its
:ref:`discovery protocol<discovery_protocol>` set to |SIMPLE| (default), to automatically enter the
`Discovery Server Easy Mode <https://docs.vulcanexus.org/en/latest/rst/enhancements/easy_mode/easy_mode.html>`__.
If this happens, the participant will be configured as a |SUPER_CLIENT| pointing to the local server.

The following example will configure participants as |SUPER_CLIENT| pointing to a local Discovery Server,
which will try to connect to another Discovery Server located in the host ``10.0.0.1``.

.. code-block:: bash

    export ROS2_EASY_MODE=10.0.0.1

The port of the Discovery Server is calculated using the rules explained in the :ref:`listening_locators_defaultPorts`.
The transports configured in this new mode include :ref:`UDP<transport_udp_udp>` unicast for discovery and
:ref:`TCP<transport_tcp_tcp>` and :ref:`Shared Memory<transport_sharedMemory_sharedMemory>` for user data.

A detailed tutorial can be found in the
`Vucanexus Easy Mode Tutorial <https://docs.vulcanexus.org/en/latest/rst/tutorials/core/wifi/easy_mode/easy_mode.html>`__ documentation.

.. note::
    When ``ROS2_EASY_MODE`` is enabled, Fast DDS automatically loads a custom XML profile named ``service``.
    This profile increases the server's response timeout for ROS 2 services by modifying the
    |ReliabilityQosPolicy::max_blocking_time-api|.
    However, if the user provides an XML file that already contains a profile with the same name, Fast DDS will not
    load any extra profile.
    Instead, the |ReliabilityQosPolicy::max_blocking_time-api| value defined in the user's XML file will be used.

.. warning::
    Discovery Server ``ROS2_EASY_MODE`` is not yet available for Windows platforms.

.. _env_vars_ros_super_client:

``ROS_SUPER_CLIENT``
----------------------

If the ``DomainParticipant``'s :ref:`discovery protocol<discovery_protocol>` is set to |SIMPLE|,
and ``ROS_SUPER_CLIENT`` is set to TRUE, the participant is automatically promoted to a |SUPER_CLIENT|.

.. important::
    This environment variable is meant to be used in combination with ``ROS_DISCOVERY_SERVER`` to promote a participant from |SIMPLE| to |SUPER_CLIENT|.
    The participant will have the *servers* list defined in ``ROS_DISCOVERY_SERVER``.

The possible values are: **TRUE**, **true**, **True**, **1**, **FALSE**, **false**, **False**, **0**.

.. important::
    If the variable is not set, the default behavior of Fast DDS is equivalent to the case in which the variable is set to false.

The following example shows how to set the environment variable to true.

+----------------------------------------------------------------------------+
| **Linux**                                                                  |
+----------------------------------------------------------------------------+
| .. code-block:: bash                                                       |
|                                                                            |
|    export ROS_SUPER_CLIENT=TRUE                                            |
+----------------------------------------------------------------------------+
| **Windows**                                                                |
+----------------------------------------------------------------------------+
| .. code-block:: bash                                                       |
|                                                                            |
|    set ROS_SUPER_CLIENT=TRUE                                               |
+----------------------------------------------------------------------------+

.. _env_vars_fastdds_statistics:

``FASTDDS_STATISTICS``
----------------------

.. warning::
    The environment variable is only used in the case where the CMake option `FASTDDS_STATISTICS` has been enabled.
    In any other case, the environment variable has no effect.
    Please, refer to :ref:`cmake_options` for more information.

Setting this variable configures the :ref:`DomainParticipant<dds_layer_domainParticipant>` to enable the statistics
DataWriters which topics are contained in the list set in this environment variable.
The elements of the list should be separated by semicolons and match the
:ref:`statistics topic name aliases<statistics_topic_names>`.

For example, to enable the statistics DataWriters that report the latency measurements, the environment variable should
be set as follows:

+----------------------------------------------------------------------------+
| **Linux**                                                                  |
+----------------------------------------------------------------------------+
| .. code-block:: bash                                                       |
|                                                                            |
|    export FASTDDS_STATISTICS="HISTORY_LATENCY_TOPIC;NETWORK_LATENCY_TOPIC" |
+----------------------------------------------------------------------------+
| **Windows**                                                                |
+----------------------------------------------------------------------------+
| .. code-block:: bash                                                       |
|                                                                            |
|    set FASTDDS_STATISTICS=HISTORY_LATENCY_TOPIC;NETWORK_LATENCY_TOPIC      |
+----------------------------------------------------------------------------+

.. important::
    This environment variable can be used together with the XML profiles
    (for more information please refer to :ref:`auto_enabling_statistics_datawriters`).
    The statistics DataWriters that will be enabled is the union between the ones specified in the XML file (if loaded)
    and the ones stated in the environment variable (if set).

.. _env_vars_fastdds_environment_file:

``FASTDDS_ENVIRONMENT_FILE``
----------------------------

Setting this environment variable to an existing ``json`` file allows to load the environment variables from the file
instead of from the environment.
This allows to change the value of some environment variables at run time with just modifying and saving the changes to
the file.
The environment value can be either an absolute or relative path.
The file format is as follows:

.. literalinclude:: /../code/environment_file_format.json
    :language: JSON

.. important::
    The environment variables set in the environment file have precedence over the environment.

.. warning::
    Currently only ``ROS_DISCOVERY_SERVER`` environment variable allows for changes at run time. (see
    :ref:`DS_modify_server_list`)

.. _RFC3513: https://www.rfc-editor.org/rfc/rfc3513
