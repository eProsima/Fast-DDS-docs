.. _troubleshooting:

Troubleshooting
===============

This section offers hints and pointers to help users with navigating through the documentation while troubleshooting
issues.

* Although UDP/SHM default transports of Fast DDS are designed to work in most network environments, they may encounter
  certain limitations when operating over WiFi or within lossy network conditions. In these cases, it is advisable to
  set up the ``LARGE_DATA`` configuration, which has been specifically optimized for these scenarios. The
  ``LARGE_DATA`` profile limits the use of UDP solely to the :ref:`PDP discovery<disc_phases>` phase, employing the more
  reliable TCP/SHM for the remainder of the communication process. Its implementation can be accomplished by simply
  configuring the ``FASTDDS_BUILTIN_TRANSPORTS`` environment variable, or alternatively through XML profiles
  or via code. For more information, please refer to :ref:`use-case-tcp-multicast`.

  .. tabs::

   .. tab:: Environment Variable

      .. code-block:: bash

          export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA

   .. tab:: XML

      .. literalinclude:: /../code/XMLTester.xml
          :language: xml
          :start-after: <!-->LARGE_DATA_BUILTIN_TRANSPORTS<-->
          :end-before: <!--><-->
          :lines: 2-4, 6-13, 15-16

   .. tab:: C++

      .. literalinclude:: ../../../code/DDSCodeTester.cpp
        :language: c++
        :dedent: 8
        :start-after: //LARGE_DATA_BUILTIN_TRANSPORTS
        :end-before: //!

* Problems with transmitting **large samples such as video or point clouds**? Please refer to
  :ref:`use-case-largeData`.
