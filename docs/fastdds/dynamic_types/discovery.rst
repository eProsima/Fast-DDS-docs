.. _dynamictypes_discovery:


Dynamic Types Discovery and Endpoint Matching
=============================================

.. _DDS-XTypes V1.2: http://www.omg.org/spec/DDS-XTypes/1.2

When using Dynamic Types support, Fast DDS checks the optional ``TypeObjectV1`` and ``TypeIdV1`` values
during endpoint matching.
Currently, the matching only verifies that both endpoints are using the same topic data type,
but will not negotiate about it.

The matching checks ``CompleteTypeObject`` first.
If one or both endpoints do not define the ``CompleteTypeObject``, it tries with ``MinimalTypeObject``.
If one or both endpoints do not define ``MinimalTypeObject`` either, it compares the ``TypeIdentifier``.
If none is defined, then just the type name is checked.

If one of the endpoints  transmits a ``CompleteTypeObject``, :ref:`discovery-time-data-typing` can be performed.

TypeObject (TypeObjectV1)
-------------------------

There are two kinds of ``TypeObject``: ``MinimalTypeObject`` and ``CompleteTypeObject``.

 - ``MinimalTypeObject`` is used to check compatibility between types.
 - ``CompleteTypeObject`` fully describes the type.

``TypeObject`` is an IDL union with both *Minimal* and *Complete* representation.
Both are described in the annexes of `DDS-XTypes V1.2`_ document,
please refer to this document for details.

TypeIdentifier (TypeIdV1)
-------------------------

``TypeIdentifier`` represents a full description of basic types and has an :class:`EquivalenceKind` for complex ones.

``TypeIdentifier`` is described in the annexes of `DDS-XTypes V1.2`_ document,
please refer to this document for details.

TypeObjectFactory
-----------------

*Singleton* class that manages the creation and access for all registered ``TypeObjects`` and ``TypeIdentifiers``.
It can generate a full ``DynamicType`` from a basic ``TypeIdentifier`` (i.e., a ``TypeIdentifier`` whose discriminator
isn't:class:`EK_MINIMAL` or :class:`EK_COMPLETE`) can generate a full ``DynamicType``.

Fastrtpsgen
-----------

*FastRTPSGen* supports the generation of :class:`XXXTypeObject.h` and :class:`XXXTypeObject.cxx` files,
taking :class:`XXX` as our IDL type.
These files provide a small Type Factory for the type :class:`XXX`.
Generally, these files are not used directly, as now the type :class:`XXX` will register itself through its factory to
``TypeObjectFactory`` in its constructor, making it very easy to use static types with dynamic types.

.. _discovery-time-data-typing:

Discovery-Time Data Typing
--------------------------

Using the Fast DDS API, when a participant discovers a remote endpoint that sends a complete ``TypeObject`` or a simple
``TypeIdentifier`` describing a type that the participant doesn't know, the participant listener's
function ``on_type_discovery`` is called with the received ``TypeObject`` or ``TypeIdentifier provided``,
and, when possible, a ``DynamicType_ptr`` ready to be used.

Discovery-Time Data Typing allows the discovering of simple DynamicTypes. A TypeObject that depends on other
TypeObjects, cannot be built locally using Discovery-Time Data Typing and should use :ref:`TypeLookup-Service` instead.

To ease the sharing of the TypeObject and TypeIdentifier used by Discovery-Time Data Typing, there exists
an attribute in ``TopicAttributes`` named ``auto_fill_type_object``. If set to true, on discovery time,
the local participant will try to fill ``type`` and ``type_id`` fields in the correspond ``ReaderProxyData``
or ``WriterProxyData`` to be sent to the remote endpoint.


.. _typelookup-service:

TypeLookup Service
------------------

Using the Fast DDS API, when a participant discovers an endpoint that sends a TypeInformation
describing a type that the participant doesn't know, the participant listener's
function ``on_type_information_received`` is called with the received TypeInformation.
The user can then try to retrieve the full TypeObject hierarchy to build the remote type locally, using the
TypeLookup Service.

To enable this builtin TypeLookup Service, the user must enable it in the Participant's RTPS builtin attributes:

.. literalinclude:: /../code/CodeTester.cpp
   :language: c++
   :start-after: //TYPELOOKUP_SERVICE_ENABLING
   :end-before: //!--
   :dedent: 4

A participant can be enabled to act as a TypeLookup server, client, or both.

The process of retrieving the remote type from its TypeInformation, and then registering it, can be simplified
using the  ``register_remote_type`` function on the :ref:`dds_layer_domainParticiant`.
This function takes the name of the type, the TypeInformation, and a callback function.
Internally it uses the TypeLookup Service to retrieve the full TypeObject, and, if successful, it will
call the callback.

This callback has the following signature:

.. code-block:: c

    void(std::string& type_name, const DynamicType_ptr type)

- | type_name: Is the name given to the type when calling ``register_remote_type`` function,
  | to allow the same callback to be used across different calls.

- | type: If the ``register_remote_type`` was able to build and register a DynamicType,
  | this parameter contains a pointer to the type.
  | Otherwise it contains ``nullptr``.
  | In the latter case, the user can still try to build the type manually using the factories, but it is very
  | likely that the build process will fail.

``TopicAttributes`` contains an  attribute named ``auto_fill_type_information``.
If set to true, the local participant will try to fill the ``type_information`` field in the correspond
``ReaderProxyData`` or ``WriterProxyData`` to be sent to the remote endpoint during discovery.


