.. _fastddsgen_interfaces_exceptions:

Exceptions
==========

Exceptions are user-defined structures that can be raised by members of an interface.
They are declared similarly to ``struct`` types; using the ``exception`` keyword,
followed by the exception name and a body enclosed in curly braces, which can be empty or contain members:

.. code-block:: omg-idl

    module Example {

        exception MyException {

            long my_exception_member;

        };

        exception MyOtherException {};
    };

.. note::
    In case of an exception being raised, user can access to the value of each member.
    In this case, ``out`` / ``inout`` parameters and the return result are undefined.

A list of exceptions can be specified in the member declarations of an interface
using ``raises`` keyword for operations or ``readonly`` attributes,
and ``getraises`` or ``setraises`` for plain attributes:

.. code-block:: omg-idl

    module Example {

        interface MyInterface {

            // Operation that raises an exception
            my_return_type my_operation(in long param1, out string param2, inout myType param3)
                raises (MyException, MyOtherException);

            // Read-only attribute that raises an exception
            readonly attribute my_type my_readonly_attr
                raises (MyException);

            // Plain attribute that raises exceptions
            attribute my_type my_plain_attr
                getraises (MyException)
                setraises (MyException, MyOtherException);

        };
    };

Defining exceptions for attributes makes sense because an attribute declaration
implicitly represents read and write operations. For read-only attributes, ``raises`` keyword
is used to specify the exceptions that may be raised when the getter is called.
Similarly, ``getraises`` and ``setraises`` keywords are used in plain attributes to specify
the exceptions that may be raised when the getter and setter are called, respectively.

.. note::
    The list of exceptions must be enclosed in parentheses and separated by commas.

.. note::
    All exception specifications are optional. In no one is specified, the operation should not raise any exception.

    In case of specifying both ``getraises`` and ``setraises`` in the same attribute,
    ``getraises`` expression should be declared in first place.

.. warning::
    In addition to the exceptions specified in the interface, other middleware
    exceptions may be raised.
