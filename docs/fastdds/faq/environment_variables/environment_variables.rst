.. include:: ../../../03-exports/aliases.include
.. include:: ../../../03-exports/aliases-api.include

.. _freq_env_variables_questions:

Environment Variables Frequently Asked Questions
================================================


.. collapse::  What are the most important environment variables that affect the behavior of Fast DDS?




    :Answer:

    ``FASTDDS_DEFAULT_PROFILES_FILE``, ``SKIP_DEFAULT_XML``, ``FASTDDS_BUILTIN_TRANSPORTS``, ``ROS_DISCOVERY_SERVER``, ``ROS_SUPER_CLIENT``, ``FASTDDS_STATISTICS``, ``FASTDDS_ENVIRONMENT_FILE``.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What is the purpose of the "FASTDDS_DEFAULT_PROFILES_FILE" environment variable?




    :Answer:

    Defines the location of the default profile configuration XML file.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What happens when the variable "SKIP_DEFAULT_XML" is set to 1?




    :Answer:

    Skips looking for a default profile configuration XML file. If this variable is set to 1, Fast DDS will load the configuration parameters directly from the classes' definitions without looking for the **DEFAULT_FASTDDS_PROFILES.xml** in the working directory.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What is the primary purpose of the FASTDDS_BUILTIN_TRANSPORTS environment variable?




    :Answer:

    Setting this variable allows to modify the builtin transports that are initialized during the DomainParticipant creation.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What is the purpose of the "ROS_DISCOVERY_SERVER" environment variable?




    :Answer:

    Setting this variable configures the DomainParticipant to connect to one or more servers using the Discovery Server discovery mechanism.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What happens to a "DomainParticipant" when its discovery protocol is set to "SIMPLE" and "ROS_SUPER_CLIENT" is set to TRUE?




    :Answer:

    If the DomainParticipant's discovery protocol is set to ``SIMPLE``, and ``ROS_SUPER_CLIENT`` is set to ``TRUE``, the participant is automatically promoted to a ``SUPER_CLIENT``.


-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What is the purpose of setting the "FASTDDS_STATISTICS" environment variable, according to the provided information?




    :Answer:

    Setting this variable configures the **DomainParticipant** to enable the statistics **DataWriters** which topics are contained in the list set in this environment variable.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What happens when you set the "FASTDDS_ENVIRONMENT_FILE" environment variable to a JSON file?




    :Answer:

    Setting this environment variable to an existing ``json`` file allows to load the environment variables from the file instead of from the environment.

|
