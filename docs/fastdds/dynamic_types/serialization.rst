.. _dynamictypes_serialization:

Serialization
=============

Dynamic Types have their own :class:`pubsub` type like any class generated with an IDL, and
their management is pretty similar to them.

.. literalinclude:: /../code/CodeTester.cpp
   :language: c++
   :start-after: //DYNAMIC_TYPES_SERIALIZATION
   :end-before: //!--
   :dedent: 8

A member can be marked to be ignored by serialization with the annotation ``@non_serialized``.

