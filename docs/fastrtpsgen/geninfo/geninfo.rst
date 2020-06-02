.. _fastrtpsgen_intro:

Introduction
============

eProsima Fast DDS-Gen is a Java application that generates source code using the data types defined in an IDL file.
This generated source code can be used in your applications in order to publish and subscribe to a topic of your defined
type.

To declare your structured data, you have to use IDL (Interface Definition Language) format.
IDL is a specification language, made by OMG (Object Management Group), which describes an interface in a
language-independent way, enabling communication between software components that do not share the same language.

eProsima Fast DDS-Gen is a tool that reads IDL files and parses a subset of the OMG IDL specification to generate
serialization source code.
This subset includes the data type descriptions included in :ref:`idl-types`.
The rest of the file content is ignored.

eProsima Fast DDS-Gen generated source code uses `Fast CDR <https://github.com/eProsima/Fast-CDR>`_: a C++11 library that
provides a serialization mechanism.
In this case, as indicated by the RTPS specification document, the serialization mechanism used is CDR.
The standard CDR (Common Data Representation) is a transfer syntax low-level representation for transfer between agents,
mapping from data types defined in OMG IDL to byte streams.

One of the main features of eProsima Fast DDS-Gen is to avoid the users the trouble of knowing anything about
serialization or deserialization procedures.
It also provides an initial implementation of a publisher and a subscriber using eProsima RTPS library.

.. _compile-fastrtpsgen:

Compile
-------

In order to compile *Fast DDS-Gen* you first need to have `gradle <https://gradle.org/install>`_ and
`java JDK <http://www.oracle.com/technetwork/java/javase/downloads/index.html>`_ installed
(please, check the JDK recommended version for the gradle version you have installed).

To compile *Fast DDS-Gen* java application, you will need to download its source code from
the `Fast-RPTS-Gen <https://github.com/eProsima/Fast-RTPS-Gen>`_ repository and with ``--recursive`` option and
compile it calling ``gradle assemble``. For more details see :ref:`compile-fastrtpsgen`.

.. code-block:: bash

    > git clone --recursive https://github.com/eProsima/Fast-RTPS-Gen.git
    > cd Fast-DDS-Gen
    > gradle assemble

The generated java application can be found at ``share/fastrtps`` and more user friendly scripts at ``scripts`` folders.
If you want to make these scripts available from anywhere you can add the ``scripts`` folder path to your ``PATH``
environment variable.
