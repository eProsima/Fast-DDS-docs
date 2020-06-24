.. include:: ../../../../03-exports/aliases.include
.. include:: ../../../../03-exports/aliases-api.include

.. _dds_layer_topic_fastddsgen:

Fast DDS-Gen for data types source code generation
--------------------------------------------------

*eProsima Fast DDS* comes with a built-in source code generation tool, *Fast DDS-Gen*, which eases the process of
translating an IDL specification of a data type to a functional implementation.
Thus, this tool automatically generates the source code of a data type defined using IDL.
A basic use of the tool is described below.
To learn about all the features that *Fast DDS* offers, please refer to :ref:`Fast DDS-Gen <fastddsgen_intro>` section.

Basic usage
^^^^^^^^^^^

*Fast DDS* can be executed by calling *fastrtpsgen* on Linux or *fastrtpsgen.bat* on Windows.
The IDL file containing the data type definition is given with the ``<IDLfile>`` argument.

+----------------------------------------------------------------------------------------------------------------------+
| **Linux**                                                                                                            |
+----------------------------------------------------------------------------------------------------------------------+
| .. code-block:: bash                                                                                                 |
|                                                                                                                      |
|    fastrtpsgen [<options>] <IDLfile> [<IDLfile> ...]                                                                 |
+----------------------------------------------------------------------------------------------------------------------+
| **Windows**                                                                                                          |
+----------------------------------------------------------------------------------------------------------------------+
| .. code-block:: bash                                                                                                 |
|                                                                                                                      |
|    fastrtpsgen.bat [<options>] <IDLfile> [<IDLfile> ...]                                                             |
+----------------------------------------------------------------------------------------------------------------------+

Among the available arguments defined in :ref:`fastddsgen_usage`, the main *Fast DDS-Gen* options for data type source
code generation are the following:

*   ``-replace``: It replaces existing files in case the data type files have been previously generated.
*   ``-help``: It lists the currently supported platforms and Visual Studio versions.
*   ``-typeobject``: It builds additional files for |TypeObject-api| generation and management (see
    :ref:`dynamictypes_discovery_typeobject`).
*   ``-example``: It generates a basic example of a DDS application and the files to build it for
    the given ``platform``.
    Thus, *Fast DDS-Gen* tool can generate a sample application using the provided data type, together with a
    `Makefile`, to compile it on Linux distributions, and a Visual Studio project for Windows.
    To see an example of this please refer to tutorial :ref:`fastddsgen_pubsub_app`.

Output files
^^^^^^^^^^^^

*Fast DDS-Gen* outputs several files.
Assuming the IDL file had the name *“Mytype”*, and none of the above options have been defined, these files are:

*   MyType.cxx/.h: Type definition.
*   MyTypePubSubType.cxx/.h: Serialization and deserialization source code for the data type.
    It also defines the |TopicDataType::getKey-api| member function of the :class:`MyTypePubSubType` class in case the
    topic implements keys (see :ref:`dds_layer_topic_keyed_data_types`).

If the ``-typeobject`` argument was used, `MyType.cxx` is modified to register the |TypeObject-api| representation in
the |TypeObjectFactory-api|, and these files will also be generated:

*   MyTypeTypeObject.cxx/.h: TypeObject representation for `MyType` IDL.
