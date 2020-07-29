.. include:: ../../03-exports/aliases-api.include

.. _env_vars:

Environment variables
=====================

This is the list of environment variables that affect the behavior of *Fast DDS*:

``FASTRTPS_DEFAULT_PROFILES_FILE``
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


``ROS_DISCOVERY_SERVER``
    When this variable is set, if the :ref:`dds_layer_domainParticipant`'s |discoveryProtocol| is configured as default
    or |SIMPLE|, it automatically configures it as either a client or a server of
    :ref:`Server-Client Discovery<discovery_server>`.
    The value of the variable must represent the locator of the server,
    in the form of the IP address (e.g., '192.168.2.23') or address-port pair (e.g., '192.168.2.23:24353').
    If no port is specified, the default port 11811 is used.

    To set more than one address they mus be separated by semicolons.
    Their id will be determined by their position in the variable.
    A blank space between semicolons means the corresponding id is free.

    If the specified address is a local address of the current machine, it will start the
    :ref:`dds_layer_domainParticipant` as a server.
    If the address is a remote one, or if there is already a server at the given port,
    it will start the DomainParticipant as a client,
    connecting to the server at the specified locator.


    **Example**

    Two remote discovery servers with addresses 84.22.259.329:8888 and 81.41.17.102:1234 and ids 0 and 2.

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



