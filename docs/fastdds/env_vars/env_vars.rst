.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include

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

.. _env_vars_ros_discovery_server:

``ROS_DISCOVERY_SERVER``
------------------------

.. warning::
    The environment variable is only used in the case where :ref:`discovery protocol<discovery_protocol>`
    is set to |SIMPLE|, |SERVER|, or |BACKUP|.
    In any other case, the environment variable has no effect.

Setting this variable configures the :ref:`DomainParticipant<dds_layer_domainParticipant>` to connect to one or more
*servers* using the :ref:`Discovery Server<discovery_server>` discovery mechanism.

* If ``ROS_DISCOVERY_SERVER`` is defined, and the ``DomainParticipant``'s :ref:`discovery protocol<discovery_protocol>`,
  is set to |SIMPLE|, then Fast DDS will instead configure it as |CLIENT| of the given *server*.
* If ``ROS_DISCOVERY_SERVER`` is defined, and the ``DomainParticipant``'s :ref:`discovery protocol<discovery_protocol>`
  is |SERVER| or |BACKUP|, then the variable is used to add remote *servers* to the given *server*, leaving the
  :ref:`discovery protocol<discovery_protocol>` as |SERVER| or |BACKUP| respectively.

* The value of the variable must list the locator of the server in the form of the IPv4 address (e.g., '192.168.2.23')
  or IP-port pair (e.g., '192.168.2.23:24353').
  Instead of an IPv4 address, a name can be specified (e.g., 'localhost', 'localhost:12345').
  This name would be used to query known hosts and available DNS servers to try to resolve a valid IPv4 address (see
  :ref:`DS_dns_name`).
* If no port is specified, the default port 11811 is used.
* To set more than one *server*'s address, they must be separated by semicolons.
* The server's ID is determined by their position in the list.
  Two semicolons together means the corresponding ID is free.

The following example shows how to set the address of two remote discovery servers with addresses
'84.22.259.329:8888' and 'localhost:1234' and IDs 0 and 2 respectively.

    +----------------------------------------------------------------------------+
    | **Linux**                                                                  |
    +----------------------------------------------------------------------------+
    | .. code-block:: bash                                                       |
    |                                                                            |
    |    export ROS_DISCOVERY_SERVER=84.22.259.329:8888;;localhost:1234          |
    +----------------------------------------------------------------------------+
    | **Windows**                                                                |
    +----------------------------------------------------------------------------+
    | .. code-block:: bash                                                       |
    |                                                                            |
    |    set ROS_DISCOVERY_SERVER=84.22.259.329:8888;;localhost:1234             |
    +----------------------------------------------------------------------------+

.. important::
  IP addresses specified in ``ROS_DISCOVERY_SERVER`` must be either valid IPv4 addresses or names.
  If a name which can be translated into an address is specified, the first valid IPv4 returned from the query will be
  used.

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
    |    export FASTDDS_STATISTICS=HISTORY_LATENCY_TOPIC;NETWORK_LATENCY_TOPIC   |
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
