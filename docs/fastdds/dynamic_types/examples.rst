.. _dynamictypes_examples:

Dynamic HelloWorld Examples
===========================

DynamicHelloWorldExample
------------------------

This example is located in folder ``examples/C++/DynamicHelloWorldExample``.
It shows the use of DynamicType generation to provide the TopicDataType.
This example is compatible with the classic HelloWorldExample.

As a quick reference, the following piece of code shows how the HelloWorld type is created using DynamicTypes:

.. literalinclude:: /../code/CodeTester.cpp
   :language: c++
   :start-after: //DYNAMIC_HELLO_WORLD_API
   :end-before: //!--
   :dedent: 8


DDSDynamicHelloWorldExample
---------------------------

This example uses the DDS API, and it is located in the ``examples/C++/DDS/DynamicHelloWorldExample`` folder.
It shows a publisher that loads a type from an XML file, and shares it during discovery.
The subscriber discovers the type using :ref:`discovery-time-data-typing`, and registers the
discovered type on the ``on_type_discovery`` listener function.

TypeLookupService
-----------------

This example uses the DDS API, and it is located in the ``examples/C++/DDS/TypeLookupService`` folder.
It's very similar to DDSDynamicHelloWorldExample, but the shared type is complex enough to require the
TypeLookup Service due to the dependency of inner struct types.
Specifically, it uses the ``register_remote_type`` approach with a callback.
