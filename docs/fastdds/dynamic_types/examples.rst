.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include

.. _dynamictypes_examples:

Dynamic HelloWorld Examples
===========================

.. _Fast DDS GitHub repository: https://github.com/eProsima/Fast-DDS

These are complete working examples that make use of dynamic types.
You can explore them to find how this feature connects to the rest of *Fast DDS*,
and learn how to integrate it in your own application.

DynamicHelloWorldExample
------------------------

This example is in folder
`examples/cpp/dds/DynamicHelloWorldExample <https://github.com/eProsima/Fast-DDS/tree/master/examples/cpp/dds/DynamicHelloWorldExample>`_
of the `Fast DDS GitHub repository`_.
It shows the use of DynamicType generation to provide the |TopicDataType|.
This example is compatible with the classic HelloWorldExample.

As a quick reference, the following piece of code shows how the HelloWorld type is created using DynamicTypes:

.. literalinclude:: /../code/CodeTester.cpp
   :language: c++
   :start-after: //DYNAMIC_HELLO_WORLD_API
   :end-before: //!--
   :dedent: 8


DDSDynamicHelloWorldExample
---------------------------

This example uses the DDS API, and can be retrieve from folder
`examples/cpp/dds/DynamicHelloWorldExample <https://github.com/eProsima/Fast-DDS/tree/master/examples/cpp/dds/DynamicHelloWorldExample>`_
of the `Fast DDS GitHub repository`_.
It shows a publisher that loads a type from an XML file, and shares it during discovery.
The subscriber discovers the type using :ref:`discovery-time-data-typing`, and registers the
discovered type on the listener.

TypeLookupService
-----------------

This example uses the DDS API, and it is located in folder
`examples/cpp/dds/TypeLookupService <https://github.com/eProsima/Fast-DDS/tree/master/examples/cpp/dds/TypeLookupService>`_
of the `Fast DDS GitHub repository`_.
It is very similar to DDSDynamicHelloWorldExample, but the shared type is complex enough to require the
TypeLookup Service due to the dependency of inner struct types.
