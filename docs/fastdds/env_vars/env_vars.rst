.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include
.. include:: ../../03-exports/roles.include

.. _env_vars:

Environment variables
=====================

This is the list of environment variables that affect the behavior of *Fast DDS*:

.. _env_vars_fastrtps_default_profiles_file:

``FASTRTPS_DEFAULT_PROFILES_FILE``
----------------------------------

Defines the location of the default profile configuration XML file.
If this variable is set and its value corresponds with an existing file, *Fast DDS* will load its profiles.
For more information about XML profiles, please refer to :ref:`xml_profiles`.

+------------------------------------------------------------------+
| **Linux**                                                        |
+------------------------------------------------------------------+
| .. code-block:: bash                                             |
|                                                                  |
|    export FASTRTPS_DEFAULT_PROFILES_FILE=/home/user/profiles.xml |
+------------------------------------------------------------------+
| **Windows**                                                      |
+------------------------------------------------------------------+
| .. code-block:: bash                                             |
|                                                                  |
|    set FASTRTPS_DEFAULT_PROFILES_FILE=C:\profiles.xml            |
+------------------------------------------------------------------+

.. _env_vars_skip_default_xml:

``SKIP_DEFAULT_XML``
--------------------

Skips looking for a default profile configuration XML file.
If this variable is set to `1`, *Fast DDS* will load the configuration parameters directly from the classes'
definitions without looking for the *DEFAULT_FASTRTPS_PROFILES.xml* in the working directory.
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
|                            | the desired |br| transports. Otherwise, the participant creation will fail.  |
+----------------------------+------------------------------------------------------------------------------+
| ``DEFAULT``                | UDPv4 and SHM transports will be instantiated. SHM transport has priority    |
|                            | over the UDPv4 |br| transport. Meaning that SHM will always be used          |
|                            | when possible.                                                               |
+----------------------------+------------------------------------------------------------------------------+
| ``DEFAULTv6``              | UDPv6 and SHM transports will be instantiated. SHM transport has priority    |
|                            | over the UDPv4 |br| transport. Meaning that SHM will always be used          |
|                            | when possible.                                                               |
+----------------------------+------------------------------------------------------------------------------+
| ``SHM``                    | Only a SHM transport will be instantiated.                                   |
+----------------------------+------------------------------------------------------------------------------+
| ``UDPv4``                  | Only a UDPv4 transport will be instantiated.                                 |
+----------------------------+------------------------------------------------------------------------------+
| ``UDPv6``                  | Only a UDPv6 transport will be instantiated.                                 |
+----------------------------+------------------------------------------------------------------------------+
| ``LARGE_DATA``             | UDPv4, TCPv4, and SHM transports will be instantiated. However, UDP will     |
|                            | only be used |br| for multicast announcements during the participant         |
|                            | discovery phase (see :ref:`disc_phases`) |br| while the participant          |
|                            | liveliness and the application data delivery occurs over TCP or SHM. |br|    |
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
* The server's ID is determined by their position in the list.
  Two semicolons together means the corresponding ID is free.
* When using IPv6 with DNS, the specified domain name space (*<dns>*) must be able to resolve to an IPv6
  address. Otherwise an error will be raised.

The following example shows how to set the address of two remote discovery servers with addresses
'84.22.259.329:8888' and 'localhost:1234' and IDs 0 and 2 respectively.

    +----------------------------------------------------------------------------+
    | **Linux**                                                                  |
    +----------------------------------------------------------------------------+
    | .. code-block:: bash                                                       |
    |                                                                            |
    |    export ROS_DISCOVERY_SERVER="84.22.259.329:8888;;localhost:1234"        |
    +----------------------------------------------------------------------------+
    | **Windows**                                                                |
    +----------------------------------------------------------------------------+
    | .. code-block:: bash                                                       |
    |                                                                            |
    |    set ROS_DISCOVERY_SERVER=84.22.259.329:8888;;localhost:1234             |
    +----------------------------------------------------------------------------+

.. important::
    IP addresses specified in ``ROS_DISCOVERY_SERVER`` must be either valid IPv4/IPv6 addresses or domain names.
    If a name can be resolved into several addresses, it is possible to either use them all or restrict the selection to
    the first IPv4 using the `UDPv4:` or `TCPv4` prefixes or to the first IPv6 address using the `UDPv6:` or `TCPv6`
    prefixes.

.. important::
    This environment variable is meant to be used in combination with :ref:`Fast DDS discovery CLI<cli_discovery>`.
    The *server*'s ID is used by *Fast DDS* to derived the |GuidPrefix_t-api| of the *server*.
    If the *server* is not instantiated using the CLI, the *server*'s GUID prefix should adhere to the same schema
    as the one generated from the CLI.
    Else, the *clients* configured with this environment variable will not be able to establish a connection with
    the *server*, thus not being able to connect to other *clients* either.
    The *server*'s GUID prefixes generated by the CLI comply with the following schema:
    ``44.53.<server-id-in-hex>.5f.45.50.52.4f.53.49.4d.41``.
    This prefix schema has been chosen for its ASCII translation: ``DS<id_in_hex>_EPROSIMA``.

.. important::
    This environment variable can be changed at runtime adding new remote servers to a |SERVER|, |BACKUP| or |CLIENT|
    (that has been initialized with this environment variable previously) if loaded from an environment file using
    :ref:`env_vars_fastdds_environment_file`.

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
