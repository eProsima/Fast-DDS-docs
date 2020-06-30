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
| ``<use_default>`` | |Log::ClearConsumers-api| is |br|                             | ``bool``              | ``true`` |
|                   | performed. See :ref:`dds_layer_log_register_consumers`.       |                       |          |
+-------------------+---------------------------------------------------------------+-----------------------+----------+
| ``<consumer>``    | Defines the class and configuration of the consumer to |br|   | :ref:`xmllogconsumer` |          |
|                   | be registered. Multiple consumers can be registered |br|      |                       |          |
|                   | this way. See :ref:`dds_layer_log_consumer`.                  |                       |          |
+-------------------+---------------------------------------------------------------+-----------------------+----------+

The following constitutes an example of an XML configuration file that sets the |Log-api| to use one
|StdoutConsumer-api| and one |FileConsumer-api|:

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->LOG-CONFIG<-->
    :end-before: <!--><-->
    :lines: 2-3, 5-32, 34

.. _xmllogconsumer:

ConsumerDataType
^^^^^^^^^^^^^^^^

+-------------------+--------------------------------------------------------------------------+-----------------------+
| Name              | Description                                                              | Values                |
+===================+==========================================================================+=======================+
| ``<class>``       | The class of the consumer.                                               | ``StdoutConsumer``    |
|                   |                                                                          +-----------------------+
|                   |                                                                          | ``FileConsumer``      |
+-------------------+--------------------------------------------------------------------------+-----------------------+
| ``<property>``    | This element is used to configured the file consumer and therefore |br|  | :ref:`xmllogprop`     |
|                   | only applies if ``<class>`` is set to ``FileConsumer``.                  |                       |
+-------------------+--------------------------------------------------------------------------+-----------------------+

.. _xmllogprop:

PropertyType
^^^^^^^^^^^^

+-------------+--------------------------------------------------------------------------+--------------+--------------+
| Name        | Description                                                              | Values       | Default      |
+=============+==========================================================================+==============+==============+
| ``<name>``  | Name of the property to be configured.                                   | ``filename`` |              |
|             |                                                                          +--------------+--------------+
|             |                                                                          | ``append``   |              |
+-------------+--------------------------------------------------------------------------+--------------+--------------+
| ``<value>`` | The value of the property.                                               |              |              |
|             +--------------------------------------------------------------------------+--------------+--------------+
|             | * If ``<name>`` is set to ``filename``, then this element contains  |br| | ``string``   | `output.log` |
|             |   the name of the log file.                                              |              |              |
|             +--------------------------------------------------------------------------+--------------+--------------+
|             | * If ``<name>`` is set to ``append``, then this element defines  |br|    | ``Boolean``  | ``false``    |
|             |   whether the consumer should, upon creation, open the file for  |br|    |              |              |
|             |   appending or overriding.                                               |              |              |
+-------------+--------------------------------------------------------------------------+--------------+--------------+
