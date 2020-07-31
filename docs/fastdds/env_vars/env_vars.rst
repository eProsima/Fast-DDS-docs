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
    When this variable is set it configures the participant as a client of
    :ref:`Server-Client Discovery<discovery_server>` provided the
    :ref:`dds_layer_domainParticipant`'s |discoveryProtocol| has been left configured as default.
    The value of the variable must list the locator of the server
    in the form of the IP address (e.g., '192.168.2.23') or IP-port pair (e.g., '192.168.2.23:24353').
    If no port is specified, the default port 11811 is used. For more information on how to configure the discovery
    mechanism in *Fast DDS*, please refer to ::ref:`.. _discovery:`.

    To set more than one address they must be separated by semicolons.
    The server's Id is determined by their position in the list.
    A blank space between semicolons means the corresponding Id is free.


    **Example**

    The following example shows how to give the address two remote discovery servers with addresses '84.22.259.329:8888'
    and '81.41.17.102:1234' and Ids 0 and 2.

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



