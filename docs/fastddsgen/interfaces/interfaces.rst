.. _fastddsgen_interfaces_definition:

Defining an IDL interface
=========================

*eProsima Fast DDS-Gen* allows the generation of the code required by both the client and the server
to use the *Fast DDS* Request-Reply internal API (see :ref:`request_reply_api_intro`),
given an IDL interface provided by the user.
The following subsections will describe how to define an IDL interface for simple asynchronous operations
and data streaming.

IDL specification overview
--------------------------

The `OMG IDL specification <https://www.omg.org/spec/IDL/4.2/PDF>`_ defines interfaces
that client and server objects may use, for example, in the context of a *Remote Procedure Calls* communication
(see :ref:`rpc_dds_intro`).

Each interface represents a set of operations and attributes, and it is constituted by a header and a body:

* An interface header is defined by the keyword ``interface``, followed by the interface name
  and an optional inheritance list, in case of being derived from other interfaces.

  For example, a valid interface header is: :code:`interface MyInterface : MyBaseInterface, ...`

* The interface body is enclosed in curly braces and contains the interface members.
  Each member is defined by its type, name and an optional list of parameters.

  * An operation is declared by specifying the return type, which can be any primitive type, any type
    previously declared by the user in the IDL file or ``void`` if the operation does no return a result;
    the name of the operation and an optional list of parameters, enclosed by parentheses and separated by commas.
    Each parameter is specified by its type and name, preceded by an attribute that specifies its direction:

    * ``in`` indicates that the parameter is used only for input.
    * ``out`` indicates that the parameter is used only for output.
    * ``inout`` indicates that the parameter can be used for both input and output.

  * An attribute can be declared by specifying the ``attribute`` keyword
    (preceded by the ``readonly`` keyword for read-only attributes), followed by a valid type and its name.
    Declaring an attribute is logically equivalent to declaring a getter and a setter for plain attributes
    (*i.e:* mutable attributes), and a getter for read-only attributes.

Interfaces can also be forward declared, for example :code:`interface MyInterface;`.

.. _fastddsgen_interfaces_data_streaming:

Defining data streaming operations
----------------------------------

*eProsima Fast DDS-Gen* allows to generate code for data streaming in each use case
described in :ref:`rpc_data_streaming_intro`, using the ``@feed`` builtin annotation in operations:

* Input feeds can be specified by adding ``@feed`` before an *in* parameter, for example:
  :code:`return_type input_feed_operation(@feed in long param);`. When a parameter ``param`` is marked
  as ``@feed`` parameter, an input feed will be expected to process the values associated to ``param``.

* Output feeds can be specified by adding ``@feed`` before a return type, for example:
  :code:`@feed return_type output_feed_operation(in long param);`. When a return type is marked
  as ``@feed`` parameter, an output feed will be expected to process the values associated to result of the operation.

Client-side, Server-side or bidirectional data streamings can be specified by marking
with ``@feed`` annotations input parameters, return types or both, respectively.

.. _note:
  ``@feed`` annotated parameters can be combined with non ``@feed`` annotated, for example
  :code:`return_type mixed_operation(@feed in long param1, in long param2);`


Example
-------

The following example shows how to define an interfaces, addressing the cases described before:

.. code-block:: omg-idl

    module Example {

        // forward declaration
        interface MyInterface;

        interface MyBaseInterface {

            typedef long my_type;
            typedef long my_return_type;

            // Operation with in, out and inout parameters
            my_return_type my_operation (
                    in long param1,
                    out string param2,
                    inout my_type param3);

            // Operation with no parameters and no return
            void my_void_operation();

            // Operation describing a Server-side data streaming
            @feed my_return_type my_server_streaming_operation(
                    in long param);

            // Operation describing a Client-side data streaming
            my_return_type my_client_streaming_operation(
                    @feed in long param);

            // Operation describing a Bidirectional data streaming
            @feed my_return_type my_bidirectional_streaming_operation(
                    in long param1,
                    @feed in long param2);
        };

        interface MyInterface : MyBaseInterface {

            // Read-only attribute
            readonly attribute my_type my_readonly_attr;

            // Plain attribute
            attribute my_type my_plain_attr;

        };

    };

.. warning::
    For now, *Fast DDS-Gen* tool does not support the generation of code for interfaces
    which contain attributes. Only operations are supported.
