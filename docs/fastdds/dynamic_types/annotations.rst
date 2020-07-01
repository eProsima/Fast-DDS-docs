.. include:: ../../03-exports/aliases.include

.. _dynamictypes_annotations:

Annotations
===========


|DynamicTypeBuilder| allows applying an annotation
to both current type and inner members with the functions:

- :func:`apply_annotation`

- :func:`apply_annotation_to_member`

Both functions take the name,  the key and the value of the annotation.
:func:`apply_annotation_to_member` additionally receives the ``MemberId`` of the inner member.

For example, if we define an annotation like:

.. code-block:: idl

    @annotation MyAnnotation
    {
        long value;
        string name;
    };

And then we apply it through IDL to a struct:

.. code-block:: idl

    @MyAnnotation(5, "length")
    struct MyStruct
    {
    ...

The equivalent code using |DynamicTypes| will be:

.. literalinclude:: /../code/CodeTester.cpp
   :language: c++
   :start-after: //DYNAMIC_TYPES_CREATE_ANNOTATION
   :end-before: //!--
   :dedent: 8

Builtin annotations
-------------------

The following annotations modifies the behavior of DynamicTypes:

- | ``@position``: When applied to |Bitmask|, sets the position of the flag,
  | as expected in the IDL annotation.
  | If applied to |Bitset|, sets the base position of the bitfield,
  | useful to identify unassigned bits.

- | ``@bit_bound``: Applies to |Bitset|. Sets the size in bits of the bitfield.

- | ``@key``: Alias for ``@Key``. See :ref:`dds_layer_topic_keyed_data_types` section for more details.

- | ``@default``: Sets a default value for the member.

- | ``@non_serialized``: Excludes a member from being serialized.

