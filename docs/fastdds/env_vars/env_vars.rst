.. _env_vars:

Environment variables
=====================

This is the list of environment variables that affect the behavior of *Fast DDS*:

``FASTRTPS_DEFAULT_PROFILES_FILE``
    Defines the location of the default profile configuration XML file.
    If this variable is set and its value corresponds with an existing file, *Fast DDS* will load its profiles.
    For more information about XML profiles, please refer to :ref:`xml_profiles`.

+--------------------------------------------------------------+
| **linux**                                                    |
+--------------------------------------------------------------+
| .. code-block::                                              |
|                                                              |
|    FASTRTPS_DEFAULT_PROFILES_FILE=/home/user/profiles.xml    |
+--------------------------------------------------------------+
| **windows**                                                  |
+--------------------------------------------------------------+
| .. code-block::                                              |
|                                                              |
|    set FASTRTPS_DEFAULT_PROFILES_FILE=C:\profiles.xml        |
+--------------------------------------------------------------+


``ROS2_AUTO_CLIENT_SERVER``
    When this variable is set, it automatically activates the :ref:`Server-Client Discovery<discovery_server>`.
    The value of the variable must represent the locator of the server,
    in the form of the IP address (e.g., '192.168.2.23') or address-port pair (e.g., '192.168.2.23:24353').
    If no port is specified, the default port 11311 is used.

    If the specified address is a local address of the current machine, it will start the
    :ref:`dds_layer_domainParticipant` as a server.
    If the address is a remote one, or if there is already a server at the given port,
    it will start the :ref:`dds_layer_domainParticipant` as a client,
    connecting to the server at the specified locator.

+--------------------------------------------------------------+
| **linux**                                                    |
+--------------------------------------------------------------+
| .. code-block::                                              |
|                                                              |
|    ROS2_AUTO_CLIENT_SERVER=192.168.2.23:24353                |
+--------------------------------------------------------------+
| **windows**                                                  |
+--------------------------------------------------------------+
| .. code-block::                                              |
|                                                              |
|    set ROS2_AUTO_CLIENT_SERVER=192.168.2.23:24353            |
+--------------------------------------------------------------+


``FASTDDS_AUTO_CLIENT_SERVER``
    An alias for ``ROS2_AUTO_CLIENT_SERVER``.
    If both variables are defined, only the value of ``ROS2_AUTO_CLIENT_SERVER`` is used.
