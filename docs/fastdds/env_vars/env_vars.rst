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
    is set to |SIMPLE|.
    In any other case the environment variable has no effect.

When setting this variable, the :ref:`DomainParticipant<dds_layer_domainParticipant>` is configured as a *Client* of
the given *Server*, implementing the :ref:`Discovery Server<discovery_server>` mechanism, provided its
:ref:`dds_layer_domainParticipant`'s |discoveryProtocol| setting has been left configured as default
(:ref:`Simple discovery<simple_disc_settings>`).
For more information on how to configure the discovery mechanism in *Fast DDS*, please refer to :ref:`discovery`.

* The value of the variable must list the locator of the server in the form of the IP address (e.g., '192.168.2.23')
  or IP-port pair (e.g., '192.168.2.23:24353').
* If no port is specified, the default port 11811 is used.
* To set more than one *server*'s address, they must be separated by semicolons.
* The server's ID is determined by their position in the list.
  A blank space between semicolons means the corresponding ID is free.

The following example shows how to set the address of two remote discovery servers with addresses
'84.22.259.329:8888' and '81.41.17.102:1234' and IDs 0 and 2 respectively.

    +----------------------------------------------------------------------------+
    | **Linux**                                                                  |
    +----------------------------------------------------------------------------+
    | .. code-block:: bash                                                       |
    |                                                                            |
    |    export ROS_DISCOVERY_SERVER=84.22.259.329:8888;;81.41.17.102:1234       |
    +----------------------------------------------------------------------------+
    | **Windows**                                                                |
    +----------------------------------------------------------------------------+
    | .. code-block:: bash                                                       |
    |                                                                            |
    |    set ROS_DISCOVERY_SERVER=84.22.259.329:8888;;81.41.17.102:1234          |
    +----------------------------------------------------------------------------+

.. important::
    This environment variable is meant to be used in combination with :ref:`Fast DDS discovery CLI<cli_discovery>`.
    The *server*'s ID is used by *Fast DDS* to derived the |GuidPrefix_t-api| of the *server*.
    If the *server* is not instantiated using the CLI, the *server*'s GUID prefix should adhere to the same schema
    as the one generated from the CLI.
    Else, the *clients* configured with this environment variable will not be able to establish a connection with
    the *server*, thus not being able to connect to other *clients* either.
    The *server*'s GUID prefixes generated by the CLI comply with the following schema:
    ``44.53.<server-id-in-hex>.5f.45.50.52.4f.53.49.4d.41``
