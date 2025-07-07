.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include
.. include:: ../../03-exports/roles.include

.. _logprofiles:

Log profiles
------------

*eProsima Fast DDS* allows for registering and configuring :ref:`Log consumers <dds_layer_log_consumer>` using XML
configuration files.
Please refer to :ref:`dds_layer_log_intro` for more information on *Fast DDS* extensible Logging built-in module.
The logging profiles are defined within the ``<log>`` XML tags.
The ``<log>`` element has two child elements: ``<use_default>`` and ``<consumer>``.
These are described in the following table.

.. list-table::
   :header-rows: 1
   :align: left

   * - Name
     - Description
     - Values
     - Default
   * - ``<use_default>``
     - If set to ``FALSE``, a call to |Log::ClearConsumers-api| is 
       |Log::ClearConsumers-api| is 
       performed. See :ref:`dds_layer_log_register_consumers`.
     - ``bool``
     - ``true``
   * - ``<consumer>``
     - Defines the class and configuration of the consumer to 
       be registered. Multiple consumers can be registered 
       this way. See :ref:`dds_layer_log_consumer`.
     - :ref:`xmllogconsumer`
     -
   * - ``<thread_settings>``
     - |ThreadSettings| for the logging thread.
     - |ThreadSettings|
     -



The following constitutes an example of an XML configuration file that sets the |Log-api| to use one
|StdoutConsumer-api|, one |StdoutErrConsumer-api|, and one |FileConsumer-api|:

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->LOG-CONFIG<-->
    :end-before: <!--><-->
    :lines: 2-3, 5-53, 55

.. _xmllogconsumer:

ConsumerDataType
^^^^^^^^^^^^^^^^

+-------------------+--------------------------------------------------------------------------+-----------------------+
| Name              | Description                                                              | Values                |
+===================+==========================================================================+=======================+
| ``<class>``       | The class of the consumer.                                               | ``StdoutConsumer``    |
|                   |                                                                          +-----------------------+
|                   |                                                                          | ``StdoutErrConsumer`` |
|                   |                                                                          +-----------------------+
|                   |                                                                          | ``FileConsumer``      |
+-------------------+--------------------------------------------------------------------------+-----------------------+
| ``<property>``    | This element is used to configure the log consumer and only applies      | :ref:`xmllogprop`     |
|                   | if ``<class>`` is set to ``StdoutErrConsumer`` or ``FileConsumer``.      |                       |
+-------------------+--------------------------------------------------------------------------+-----------------------+

.. _xmllogprop:

PropertyType
^^^^^^^^^^^^

+-------------+----------------------------------------------------------+----------------------+----------------------+
| Name        | Description                                              | Values               | Default              |
+=============+==========================================================+======================+======================+
| ``<name>``  | Name of the property to be configured.                   | ``filename``         |                      |
|             |                                                          +----------------------+----------------------+
|             |                                                          | ``append``           |                      |
|             |                                                          +----------------------+----------------------+
|             |                                                          | ``stderr_threshold`` |                      |
+-------------+----------------------------------------------------------+----------------------+----------------------+
| ``<value>`` | The value of the property.                               |                      |                      |
|             +----------------------------------------------------------+----------------------+----------------------+
|             | * If ``<name>`` is set to ``filename``, then this        | ``string``           | `output.log`         |
|             |   element contains the name of the log file. This        |                      |                      |
|             |   property only applies if ``<class>`` is set to         |                      |                      |
|             |   ``FileConsumer``                                       |                      |                      |
|             +----------------------------------------------------------+----------------------+----------------------+
|             | * If ``<name>`` is set to ``append``, then this          | ``Boolean``          | ``false``            |
|             |   element defines whether the consumer should, upon      |                      |                      |
|             |   creation, open the file for appending or               |                      |                      |
|             |   overriding. This property only applies if              |                      |                      |
|             |   ``<class>`` is set to ``FileConsumer``                 |                      |                      |
|             +----------------------------------------------------------+----------------------+----------------------+
|             | * If ``<name>`` is set to ``stderr_threshold``, then     | ``Log::Kind``        |``Log::Kind::Warning``|
|             |   this element defines the threshold used by the         |                      |                      |
|             |   :ref:`Log consumers <dds_layer_log_consumer>`.         |                      |                      |
|             |   This property only applies if ``<class>`` is set       |                      |                      |
|             |   to ``StdoutErrConsumer``                               |                      |                      |
+-------------+----------------------------------------------------------+----------------------+----------------------+
