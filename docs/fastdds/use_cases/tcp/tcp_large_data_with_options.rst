.. include:: ../../../03-exports/aliases.include
.. include:: ../../../03-exports/aliases-api.include

.. _use-case-large-data-options:

LARGE_DATA with configuration options
=====================================

As it has been observed in :ref:`use-case-tcp-multicast`, ``LARGE_DATA`` builtin transports option offers an easy
and efficient way to improve performance when working with large data. Nonetheless, custom configuration can help
to enhance the performance even further.

The ``LARGE_DATA`` mode can be configured by adjusting the following parameters:

+ ``max_msg_size``: Maximum size of the message that can be sent over the transport. Sending messages larger
  than this size will result in fragmentation. Its maximum value is 2^32-1.
+ ``sockets_size``: Size of the send and receive socket buffers. This value must be higher or equal than the
  ``max_msg_size`` to obtain a valid configuration. Its maximum value is 2^64-1.
+ ``non_blocking``: If set to true, the transport will use non-blocking sockets. This can be useful to avoid
  blocking the application if the socket buffers are full. However, some messages will be lost. Its default value is
  false.

Adjusting the maximum message and the socket buffer sizes to a value large enough to accommodate the data to be sent
and setting the transport to non-blocking mode can help to improve the performance with large messages. In this
way, it is possible to take advantage of the TCP transport to avoid fragmentation and use the non-blocking mode to
avoid blocking the application when the socket buffers are full. This configuration can be used, for example, when
streaming video, which will result in a significant increase in fluidity.

The following snippets show how to configure the ``LARGE_DATA`` mode:

.. tabs::

   .. tab:: Environment Variable

      .. code-block:: bash

         export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA?max_msg_size=1MB&sockets_size=1MB&non_blocking=true

   .. tab:: XML

      .. literalinclude:: /../code/XMLTester.xml
         :language: xml
         :start-after: <!-->LARGE_DATA_BUILTIN_TRANSPORTS_OPTIONS<-->
         :end-before: <!--><-->
         :lines: 2-4, 6-12, 14-15

   .. tab:: C++

      .. literalinclude:: ../../../../code/DDSCodeTester.cpp
         :language: c++
         :dedent: 8
         :start-after: //LARGE_DATA_BUILTIN_TRANSPORTS_OPTIONS
         :end-before: //!

.. note::

  To learn how to check and modify the default maximum system value for the socket buffers size, please refer to
  :ref:`finding-out-maximum-socket-values`.

