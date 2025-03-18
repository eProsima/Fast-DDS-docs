.. include:: ../../../../03-exports/aliases.include
.. include:: ../../../../03-exports/aliases-api.include

.. _dds_layer_topic_contentFilteredTopic_default_filter:

The default SQL-like filter
===========================

Filter expressions used by :ref:`dds_layer_topic_contentFilteredTopic` API may use a subset of SQL syntax, extended with
the possibility to use program variables in the SQL expression.
This section shows this default SQL-like syntax and how to use it.

* :ref:`default_sql_filter_grammar`
* :ref:`default_sql_filter_like`
* :ref:`default_sql_filter_match`
* :ref:`default_sql_filter_type_comparisons`
* :ref:`default_sql_filter_example`

.. _default_sql_filter_grammar:

Grammar
-------

The allowed SQL expressions are defined with the BNF-grammar below.

The following conventions are made:

- "Terminals" are quoted.
- ``TOKENS`` are typeset in code block with black font color.

.. productionlist::
    Expression: FilterExpression
    FilterExpression: Condition
    Condition: Predicate |
             : Condition "AND" Condition |
             : Condition "OR" Condition |
             : "NOT" Condition |
             : "(" Condition ")"
    Predicate: ComparisonPredicate |
             : BetweenPredicate
    ComparisonPredicate: FIELDNAME RelOp Parameter |
                       : Parameter RelOp FIELDNAME |
                       : FIELDNAME RelOp FIELDNAME
    BetweenPredicate: FIELDNAME "BETWEEN" Range |
                    : FIELDNAME "NOT BETWEEN" Range
    RelOp: "=" | ">" | ">=" | "<" | "<=" |
         : "<>" | "!=" | `like` | `match`
    Range: Parameter "AND" Parameter
    Parameter: BOOLEANVALUE |
             : INTEGERVALUE |
             : CHARVALUE |
             : FLOATVALUE |
             : STRINGVALUE |
             : ENUMERATEDVALUE |
             : PARAMETER

"Terminals" and ``TOKENS`` are case sensitive but both uppercase and lowercase are supported.

The syntax and meaning of the tokens used in the SQL grammar is described as follows:

- **FIELDNAME**: is a reference to a field in the data-structure.
  The dot ``.`` is used to navigate through nested structures.
  The number of dots that may be used in a FIELDNAME is unlimited.
  The FIELDNAME can refer to fields at any depth in the data structure.
  The names of the field are those specified in the IDL definition of the corresponding structure.

  .. productionlist::
      FIELDNAME: FieldNamePart ( "." FieldNamePart )*
      FieldNamePart: Identifier ( "[" Integer "]" )?

  An example of FIELDNAMEs:

  .. tab-set::

      .. tab-item:: Filter expression
          :sync: expr

          .. code::

              "points[0] = 0 AND color.red < 100"

      .. tab-item:: Associated IDL
          :sync: idl

          .. code:: IDL

              struct Color
              {
                  octet red;
                  octet green;
                  octet blue;
              };

              struct Shape
              {
                  long points[4];
                  Color color;
              };

- **BOOLEANVALUE**: Can either be `true` of `false`, case sensitive.

  .. productionlist::
      BOOLEANVALUE: ["TRUE", "true", "FALSE", "false"]

- **INTEGERVALUE**: Any series of digits, optionally preceded by a plus or minus sign, representing a decimal integer
  value within the range of the system.
  A hexadecimal number is preceded by ``0x`` and must be a valid hexadecimal expression.

  .. productionlist::
      INTEGERVALUE: (["+","-"])? Integer
      Integer: (["0"-"9"])+ | ["0x","0X"](["0"-"9", "A"-"F", "a"-"f"])+

  An example of INTEGERVALUE:

  .. code::

      value = -10

- **CHARVALUE**: A single character enclosed between single quotes.

  .. productionlist::
      CHARVALUE: "'" Character "'"
      Character: ~["\n"]

  An example of CHARVALUE:

  .. code::

      value = 'c'

- **FLOATVALUE**: Any series of digits, optionally preceded by a plus or minus sign and optionally including a floating
  point (``.``).
  A power-of-ten expression may be postfixed, which has the syntax e:sup:`n`, where ``n`` is a number, optionally
  preceded by a plus or minus sign.

  .. productionlist::
      FLOATVALUE: (["+"], "-"])? (Integer Exponent | Integer Fractional | Integer Fractional Exponent)
      Fractional: "." Integer
      Exponent: ["e","E"] (["+"], "-"])? Integer

  An example of FLOATVALUE:

  .. code::

      value = 10.1e-10

- **STRINGVALUE**: Any series of characters encapsulated in single quotes, except a new-line character or a right quote.
  A string starts with a left or right quote, but ends with a right quote.

  .. productionlist::
      STRINGVALUE: ["'"] ~["'", "\r", "\n"] ["'"]

  An example of STRINGVALUE:

  .. code::

      value = 'This is a string'


- **ENUMERATEDVALUE**: An enumerated value is a reference to a value declared within an enumeration.
  Enumerated values consist of the name of the enumeration label enclosed in single quotes.
  The name used for the enumeration label must correspond to the label names specified in the IDL definition of the
  enumeration.

  .. productionlist::
      ENUMERATEDVALUE: ["'"] ~["'", "\r", "\n"] ["'"]

  An example of ENUMERATEDVALUE:

  .. tab-set::

      .. tab-item:: Filter expression
          :sync: expr

          .. code::

              value = 'ENUM_VALUE_1'

      .. tab-item:: Associated IDL
          :sync: idl

          .. code::

              enum MyEnum
              {
                  ENUM_VALUE_1,
                  ENUM_VALUE_2,
                  ENUM_VALUE_3
              };

              struct Enumerators
              {
                  MyEnum value;
              };

- **PARAMETER**: A parameter is of the form ``%n``, where ``n`` represents a natural number (zero included) smaller than
  100.
  It refers to the ``n + 1 th`` argument in the given context.

  .. productionlist::
      PARAMETER: ["%"] ["0"-"9"] (["0"-"9"])?

  An example of PARAMETER:

  .. code::

      value = %1


.. _default_sql_filter_like:

Like condition
--------------

The `like <https://www.w3schools.com/sql/sql_like.asp>`_ operator is similar as the one defined by SQL.
This operator can only be used with strings.
There are two wildcards that could be used in conjunction with this operator

- The percent sign ``%`` (or its alias ``*``) represents zero, one, or multiple characters.
- The underscore sign ``_`` (or its alias ``?``) represents one single character.

All wildcards can also be used in combinations.

An example of ``like`` operator

.. tab-set::

    .. tab-item:: Filter expression
        :sync: expr

        .. code::

            "str like '%bird%'"

    .. tab-item:: Associated IDL
        :sync: idl

        .. code:: IDL

            struct Like
            {
                string str;
            };

where string ``There are birds flying`` will return ``true``.

.. _default_sql_filter_match:

Match condition
---------------

The ``match`` operator performs a full-text search using a regular expression.
This operator can only be used with strings.
It uses the `Basic Regular Expression (BRE) defined by POSIX`_.

An example of ``match`` operator

.. tab-set::

    .. tab-item:: Filter expression
        :sync: expr

        .. code::

            "str match '^The'"

    .. tab-item:: Associated IDL
        :sync: idl

        .. code:: IDL

            struct Like
            {
                string str;
            };

where string ``There are birds flying`` will return ``true``.

.. _default_sql_filter_type_comparisons:

Type comparisons
-----------------

For the supported operators in the grammar, next table shows the type compatibility.

.. list-table::
    :header-rows: 1
    :stub-columns: 1

    * - Operator1 | Operator2
      - BOOLEAN
      - INTEGER
      - FLOAT
      - CHAR
      - STRING
      - ENUM
    * - BOOLEAN
      - ✅
      - ✅
      - ❌
      - ❌
      - ❌
      - ❌
    * - INTEGER
      - ✅
      - ✅
      - ✅
      - ❌
      - ❌
      - ❌
    * - FLOAT
      - ❌
      - ✅
      - ✅
      - ❌
      - ❌
      - ❌
    * - CHAR
      - ❌
      - ❌
      - ❌
      - ✅
      - ✅
      - ❌
    * - STRING
      - ❌
      - ❌
      - ❌
      - ✅
      - ✅
      - ❌
    * - ENUM
      - ❌
      - ✅
      - ❌
      - ❌
      - ❌
      - ✅ *

**(*)** Only for the same enumerated type.

.. _default_sql_filter_example:

Example
-------

Assuming Topic ``Shape`` has next IDL definition.

.. code:: IDL

   struct Shape
   {
       long x,
       long y,
       long z,
       long width,
       long height
   };

An example of filter expression would be:

.. code::

    "x < 23 AND y > 50 AND width BETWEEN %0 AND %1"

A :ref:`dds_layer_topic_contentFilteredTopic` may be created using this filter expression as explained in section
:ref:`dds_layer_topic_contentFilteredTopic_creation`.

.. literalinclude:: /../code/DDSCodeTester.cpp
    :language: c++
    :start-after: //DDS_CONTENT_FILTERED_TOPIC_SQL_EXAMPLE
    :end-before: //!
    :dedent: 8

In this example parameters are used.
Internally the :ref:`dds_layer_topic_contentFilteredTopic` will be created with the filter expression below, after
setting the provided parameters.

.. code::

    "x < 23 AND y > 50 AND width BETWEEN 10 AND 20"


.. _Basic Regular Expression (BRE) defined by POSIX:
   https://pubs.opengroup.org/onlinepubs/9699919799/basedefs/V1_chap09.html#tag_09_03
