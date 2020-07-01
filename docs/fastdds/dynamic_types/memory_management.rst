.. include:: ../../03-exports/aliases.include

.. _dynamictypes_memorymanagement:

Memory management
=================

Memory management is critical for Dynamic Types since
every dynamic type and dynamic data is managed with pointers.
Every object stored inside of a dynamic object is managed by its owner, and users
must delete every object they create using the factories.

.. literalinclude:: /../code/CodeTester.cpp
   :language: c++
   :start-after: //DYNAMIC_TYPES_NOTES_1
   :end-before: //!--
   :dedent: 8

To ease this management, the library defines smart pointers (``DynamicTypeBuilder_ptr``,
``DynamicType`` and  ``DynamicData_ptr``) that will delete the objects automatically when they are not
needed anymore.
``DynamicType`` will always be returned as ``DynamicType_ptr`` because there is no internal management of its memory.

.. literalinclude:: /../code/CodeTester.cpp
   :language: c++
   :start-after: //DYNAMIC_TYPES_NOTES_2
   :end-before: //!--
   :dedent: 8

The only case where these smart pointers cannot be used is with functions ``loan_value`` and ``return_loaned_value``.
Raw pointers should be used with these functions, because the returned value should not be deleted, and using
a smart pointer with them will cause a crash.


