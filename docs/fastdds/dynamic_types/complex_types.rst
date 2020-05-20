.. _dynamictypes_complextypes:

Complex Types
=============

If the application's data model is complex, it is possible to combine the
:ref:`basic types<dynamictypes_supportedtypes>` to create complex types,
including nested composed types (structures within structures within unions).
Types can also be extended using inheritance, improving the flexibility of the definition
of the data types to fit the model.

Nested structures
-----------------

Structures can contain other structures as members.
The access to these compound members is restricted and managed by the ``DynamicData`` instance.
Users must request access calling ``loan_value`` before using them, and release
them with ``return_loaned_value`` once they finished.
The loan operation will fail if the member is already loaned and has not been released yet.

.. literalinclude:: /../code/CodeTester.cpp
   :language: c++
   :start-after: //DYNAMIC_TYPES_CREATE_NESTED_STRUCTS
   :end-before: //!--
   :dedent: 8

Structure inheritance
---------------------

To inherit a structure from another one, use the ``create_child_struct_type`` function from
``DynamicTypeBuilderFactory``.
The resultant type contains all members from the base class and the new ones added to the child.

Structures support several levels of inheritance, so the base class can be another derived type itself.

.. literalinclude:: /../code/CodeTester.cpp
   :language: c++
   :start-after: //DYNAMIC_TYPES_CREATE_INHERITANCE_STRUCTS
   :end-before: //!--
   :dedent: 8

Alias of an alias
-----------------

Alias types support recursion, simply use an alias name as base type for ``create_alias_type``.

.. literalinclude:: /../code/CodeTester.cpp
   :language: c++
   :start-after: //DYNAMIC_TYPES_CREATE_NESTED_ALIAS
   :end-before: //!--
   :dedent: 8

Unions with complex types
-------------------------

Unions support complex type fields.
The access to these complex type fields is restricted and managed by the ``DynamicData`` instance.
Users must request access calling ``loan_value`` before using them, and release
them with ``return_loaned_value`` once they finished.
The loan operation will fail if the fields is already loaned and has not been released yet.

.. literalinclude:: /../code/CodeTester.cpp
   :language: c++
   :start-after: //DYNAMIC_TYPES_CREATE_NESTED_UNIONS
   :end-before: //!--
   :dedent: 8

