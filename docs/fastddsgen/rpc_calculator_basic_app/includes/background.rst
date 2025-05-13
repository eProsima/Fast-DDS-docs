Background
-----------

*eProsima Fast DDS-Gen* is a Java application that generates eProsima Fast DDS source code using
the data types and interfaces defined in an IDL (Interface Definition Language) file.
This generated source code can be used in any Fast DDS application in order to define the data type of a topic,
or in a RPC application (see :ref:`RPC over DDS <rpc_dds_intro>`) to define the service and its operations.

In a RPC over DDS application (see :ref:`RPC over DDS <rpc_dds_intro>`), user provides
through the IDL file the common interfaces that will be used by the client and server applications.
An interface contains the collection of operations that can be invoked by the client on the server.
Please refer to :ref:`IDL interfaces introduction <fastddsgen_interfaces_introduction>` for more information.

Each interface defines a service, and *eProsima Fast DDS-Gen* generates the source code required to communicate
a client with a server through that service using the internal
*eProsima Fast DDS* :ref:`request-reply API <request_reply_api_intro>`.
