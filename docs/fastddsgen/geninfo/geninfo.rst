.. _fastrtpsgen_intro:

Introduction
============

eProsima Fast DDS-Gen is a Java application that generates *eProsima Fast DDS* source code using the data types defined
in an IDL file.
This generated source code can be used in any *Fast DDS* application in order to define the data type of a topic,
which will later be used to publish or subscribe.

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

For installing Fast DDS-Gen from sources, please refer to the Installation Manual.
