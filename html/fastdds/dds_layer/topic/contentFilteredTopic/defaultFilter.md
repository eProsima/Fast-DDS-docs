<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="dds-layer-topic-contentfilteredtopic-default-filter"></a>

<a id="the-default-sql-like-filter"></a>

# The default SQL-like filter

Filter expressions used by [ContentFilteredTopic](contentFilteredTopic.md#dds-layer-topic-contentfilteredtopic) API may use a subset of SQL syntax, extended with
the possibility to use program variables in the SQL expression.
This section shows this default SQL-like syntax and how to use it.
Apart from the syntax described in this section, certain restrictions apply to the allowed SQL expressions.
These restrictions can be relaxed or tightened by the user, as explained in
[SQL Filter Expression limits](../../../property_policies/non_consolidated_qos.md#properties-sql-filter-expression-limits).

* [Grammar](#default-sql-filter-grammar)
* [Like condition](#default-sql-filter-like)
* [Match condition](#default-sql-filter-match)
* [Type comparisons](#default-sql-filter-type-comparisons)
* [Example](#default-sql-filter-example)

<a id="default-sql-filter-grammar"></a>

<a id="grammar"></a>

## Grammar

The allowed SQL expressions are defined with the BNF-grammar below.

The following conventions are made:

- “Terminals” are quoted.
- `TOKENS` are typeset in code block with black font color.

```
 FilterExpression

 Condition

 Predicate |

 Condition "AND" Condition |

 Condition "OR" Condition |

 "NOT" Condition |

 "(" Condition ")"

 ComparisonPredicate |

 BetweenPredicate

 FIELDNAME RelOp Parameter |

 Parameter RelOp FIELDNAME |

 FIELDNAME RelOp FIELDNAME

 FIELDNAME "BETWEEN" Range |

 FIELDNAME "NOT BETWEEN" Range

 "=" | ">" | ">=" | "<" | "<=" |

 "<>" | "!=" | like | match

 Parameter "AND" Parameter

 BOOLEANVALUE |

 INTEGERVALUE |

 CHARVALUE |

 FLOATVALUE |

 STRINGVALUE |

 ENUMERATEDVALUE |

 PARAMETER
```

“Terminals” and `TOKENS` are case sensitive but both uppercase and lowercase are supported.

The syntax and meaning of the tokens used in the SQL grammar is described as follows:

- **FIELDNAME**: is a reference to a field in the data-structure.
  The dot `.` is used to navigate through nested structures.
  The number of dots that may be used in a FIELDNAME is unlimited.
  The FIELDNAME can refer to fields at any depth in the data structure.
  The names of the field are those specified in the IDL definition of the corresponding structure.
  ```
   FieldNamePart ( "." FieldNamePart )*

   Identifier ( "[" Integer "]" )?
  ```

  An example of FIELDNAMEs:

  ### Filter expression

  ```default
  "points[0] = 0 AND color.red < 100"
  ```

  ### Associated IDL

  ```IDL
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
  ```
- **BOOLEANVALUE**: Can either be true of false, case sensitive.
  ```
   ["TRUE", "true", "FALSE", "false"]
  ```
- **INTEGERVALUE**: Any series of digits, optionally preceded by a plus or minus sign, representing a decimal integer
  value within the range of the system.
  A hexadecimal number is preceded by `0x` and must be a valid hexadecimal expression.
  ```
   (["+","-"])? Integer

   (["0"-"9"])+ | ["0x","0X"](["0"-"9", "A"-"F", "a"-"f"])+
  ```

  An example of INTEGERVALUE:
  ```default
  value = -10
  ```
- **CHARVALUE**: A single character enclosed between single quotes.
  ```
   "'" Character "'"

   ~["\n"]
  ```

  An example of CHARVALUE:
  ```default
  value = 'c'
  ```
- **FLOATVALUE**: Any series of digits, optionally preceded by a plus or minus sign and optionally including a floating
  point (`.`).
  A power-of-ten expression may be postfixed, which has the syntax e:sup:n, where `n` is a number, optionally
  preceded by a plus or minus sign.
  ```
   (["+"], "-"])? (Integer Exponent | Integer Fractional | Integer Fractional Exponent)

   "." Integer

   ["e","E"] (["+"], "-"])? Integer
  ```

  An example of FLOATVALUE:
  ```default
  value = 10.1e-10
  ```
- **STRINGVALUE**: Any series of characters encapsulated in single quotes, except a new-line character or a right quote.
  A string starts with a left or right quote, but ends with a right quote.
  ```
   ["'"] ~["'", "\r", "\n"] ["'"]
  ```

  An example of STRINGVALUE:
  ```default
  value = 'This is a string'
  ```
- **ENUMERATEDVALUE**: An enumerated value is a reference to a value declared within an enumeration.
  Enumerated values consist of the name of the enumeration label enclosed in single quotes.
  The name used for the enumeration label must correspond to the label names specified in the IDL definition of the
  enumeration.
  ```
   ["'"] ~["'", "\r", "\n"] ["'"]
  ```

  An example of ENUMERATEDVALUE:

  ### Filter expression

  ```default
  value = 'ENUM_VALUE_1'
  ```

  ### Associated IDL

  ```default
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
  ```
- **PARAMETER**: A parameter is of the form `%n`, where `n` represents a natural number (zero included) smaller than
  100.
  It refers to the `n + 1 th` argument in the given context.
  ```
   ["%"] ["0"-"9"] (["0"-"9"])?
  ```

  An example of PARAMETER:
  ```default
  value = %1
  ```

<a id="default-sql-filter-like"></a>

<a id="like-condition"></a>

## Like condition

The [like](https://www.w3schools.com/sql/sql_like.asp) operator is similar as the one defined by SQL.
This operator can only be used with strings.
There are two wildcards that could be used in conjunction with this operator

- The percent sign `%` (or its alias `*`) represents zero, one, or multiple characters.
- The underscore sign `_` (or its alias `?`) represents one single character.

All wildcards can also be used in combinations.

An example of `like` operator

### Filter expression

```default
"str like '%bird%'"
```

### Associated IDL

```IDL
struct Like
{
    string str;
};
```

where string `There are birds flying` will return `true`.

<a id="default-sql-filter-match"></a>

<a id="match-condition"></a>

## Match condition

The `match` operator performs a full-text search using a regular expression.
This operator can only be used with strings.
It uses the [Basic Regular Expression (BRE) defined by POSIX](https://pubs.opengroup.org/onlinepubs/9699919799/basedefs/V1_chap09.html#tag_09_03).

An example of `match` operator

### Filter expression

```default
"str match '^The'"
```

### Associated IDL

```IDL
struct Like
{
    string str;
};
```

where string `There are birds flying` will return `true`.

<a id="default-sql-filter-type-comparisons"></a>

<a id="type-comparisons"></a>

## Type comparisons

For the supported operators in the grammar, next table shows the type compatibility.

| Operator1 | Operator2   | BOOLEAN   | INTEGER   | FLOAT   | CHAR   | STRING   | ENUM   |
|-------------------------|-----------|-----------|---------|--------|----------|--------|
| BOOLEAN                 | ✅         | ✅         | ❌       | ❌      | ❌        | ❌      |
| INTEGER                 | ✅         | ✅         | ✅       | ❌      | ❌        | ❌      |
| FLOAT                   | ❌         | ✅         | ✅       | ❌      | ❌        | ❌      |
| CHAR                    | ❌         | ❌         | ❌       | ✅      | ✅        | ❌      |
| STRING                  | ❌         | ❌         | ❌       | ✅      | ✅        | ❌      |
| ENUM                    | ❌         | ✅         | ❌       | ❌      | ❌        | ✅ \*   |

**(\*)** Only for the same enumerated type.

<a id="default-sql-filter-example"></a>

<a id="example"></a>

## Example

Assuming Topic `Shape` has next IDL definition.

```IDL
struct Shape
{
    long x,
    long y,
    long z,
    long width,
    long height
};
```

An example of filter expression would be:

```default
"x < 23 AND y > 50 AND width BETWEEN %0 AND %1"
```

A [ContentFilteredTopic](contentFilteredTopic.md#dds-layer-topic-contentfilteredtopic) may be created using this filter expression as explained in section
[Creating a ContentFilteredTopic](createContentFilteredTopic.md#dds-layer-topic-contentfilteredtopic-creation).

```c++
ContentFilteredTopic* sql_filter_topic =
        participant->create_contentfilteredtopic("Shape", topic,
                "x < 23 AND y > 50 AND width BETWEEN %0 AND %1",
                {"10", "20"});
```

In this example parameters are used.
Internally the [ContentFilteredTopic](contentFilteredTopic.md#dds-layer-topic-contentfilteredtopic) will be created with the filter expression below, after
setting the provided parameters.

```default
"x < 23 AND y > 50 AND width BETWEEN 10 AND 20"
```
