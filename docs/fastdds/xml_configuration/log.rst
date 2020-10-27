.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include
.. include:: ../../03-exports/roles.include

.. _logprofiles:

Log profiles
----------------

*eProsima Fast DDS* allows for registering and configuring :ref:`Log consumers <dds_layer_log_consumer>` using XML
configuration files.
Please refer to :ref:`dds_layer_log_intro` for more information on *Fast DDS* extensible Logging built-in module.
The logging profiles are defined within the ``<log>`` XML tags.
The ``<log>`` element has two child elements: ``<use_default>`` and ``<consumer>``.
These are described in the following table.

+-------------------+---------------------------------------------------------------+-----------------------+----------+
| Name              | Description                                                   | Values                | Default  |
+===================+===============================================================+=======================+==========+
| ``<use_default>`` | If set to ``FALSE``, a call to                                | ``bool``              | ``true`` |
|                   | |Log::ClearConsumers-api| is |br|                             |                       |          |
|                   | performed. See :ref:`dds_layer_log_register_consumers`.       |                       |          |
+-------------------+---------------------------------------------------------------+-----------------------+----------+
| ``<consumer>``    | Defines the class and configuration of the consumer to |br|   | :ref:`xmllogconsumer` |          |
|                   | be registered. Multiple consumers can be registered |br|      |                       |          |
|                   | this way. See :ref:`dds_layer_log_consumer`.                  |                       |          |
+-------------------+---------------------------------------------------------------+-----------------------+----------+

The following constitutes an example of an XML configuration file that sets the |Log-api| to use one
|StdoutConsumer-api|, one |StdoutErrConsumer-api| and one |FileConsumer-api|:

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->LOG-CONFIG<-->
    :end-before: <!--><-->
    :lines: 2-3, 5-43, 45

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
| ``<property>``    | This element is used to configured the log consumer and only applies|br| | :ref:`xmllogprop`     |
|                   | if ``<class>`` is set to ``StdoutErrConsumer`` or ``FileConsumer``.      |                       |
+-------------------+--------------------------------------------------------------------------+-----------------------+

.. _xmllogprop:

PropertyType
^^^^^^^^^^^^

+-------------+------------------------------------------------------------------+----------------------+--------------+
| Name        | Description                                                      | Values               | Default      |
+=============+==================================================================+======================+==============+
| ``<name>``  | Name of the property to be configured.                           | ``filename``         |              |
|             |                                                                  +----------------------+--------------+
|             |                                                                  | ``append``           |              |
|             |                                                                  +----------------------+--------------+
|             |                                                                  | ``stderr_threshold`` |              |
+-------------+------------------------------------------------------------------+----------------------+--------------+
| ``<value>`` | The value of the property.                                       |                      |              |
|             +------------------------------------------------------------------+----------------------+--------------+
|             | * If ``<name>`` is set to ``filename``, then this element        | ``string``           | `output.log` |
|             |   contains  |br|                                                 |                      |              |
|             |   the name of the log file. This property only applies if |br|   |                      |              |
|             |   ``<class>`` is set to ``FileConsumer``                         |                      |              |
|             +------------------------------------------------------------------+----------------------+--------------+
|             | * If ``<name>`` is set to ``append``, then this element          | ``Boolean``          | ``false``    |
|             |   defines |br|                                                   |                      |              |
|             |   whether the consumer should, upon creation, open the file |br| |                      |              |
|             |   for appending or overriding. This property only applies if |br||                      |              |
|             |   ``<class>`` is set to ``FileConsumer``                         |                      |              |
|             +------------------------------------------------------------------+----------------------+--------------+
|             | * If ``<name>`` is set to ``stderr_threshold``, then this element|                      | ``Log::Kind::Warning``|
|             |   defines |br|                                                   |                      |              |
|             |   the threshold used by the                                      |                      |              |
|             |   :ref:`Log consumers <dds_layer_log_consumer>`. This |br|       |                      |              |
|             |   property only applies if ``class`` is set to                   |                      |              |
|             |   ``StdoutErrConsumer``                                          |                      |              |
+-------------+------------------------------------------------------------------+----------------------+--------------+
