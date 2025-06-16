.. include:: ../../03-exports/aliases-api.include
.. include:: ../../03-exports/roles.include

.. _fastddsgen_interfaces_introduction:

IDL interfaces
==============

*eProsima FastDDS-Gen* tool allows the generation of source code for a :ref:`RPC over DDS <rpc_dds_intro>` application
from an IDL file. The IDL file must contain the operation that can be called on both the client and server sides,
and the parameters that can be passed to them. These operations are specified using the concept of interfaces
defined in the OMG IDL `specification <https://www.omg.org/spec/IDL/4.2/PDF>`_.

The next subsections serves as a guide to how to define your own IDL interface, how to define exceptions for its operations,
and how to develop RPC Client/Server applications.


.. toctree::
   :maxdepth: 2

   /fastddsgen/interfaces/interfaces
   /fastddsgen/interfaces/exceptions
   /fastddsgen/rpc_calculator_basic_app/intro
   /fastddsgen/rpc_calculator_feed_app/intro
   /fastddsgen/rpc_server_schedule/intro
