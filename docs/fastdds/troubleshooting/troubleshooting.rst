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

  .. tab-set::

      .. tab-item:: Environment Variable

          .. code-block:: bash

              export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA

      .. tab-item:: XML

          .. literalinclude:: /../code/XMLTester.xml
              :language: xml
              :start-after: <!-->LARGE_DATA_BUILTIN_TRANSPORTS<-->
              :end-before: <!--><-->
              :lines: 2-4, 6-13, 15-16

      .. tab-item:: C++

          .. literalinclude:: ../../../code/DDSCodeTester.cpp
              :language: c++
              :dedent: 8
              :start-after: //LARGE_DATA_BUILTIN_TRANSPORTS
              :end-before: //!

* If having problems with transmitting **large samples** when using the ``LARGE_DATA`` mode, try to use the builtin
  transports configuration options to adjust ``LARGE_DATA`` to your specific use case.
  Please refer to :ref:`use-case-large-data-options` for more information.

* If having problems with transmitting **large samples such as video or point clouds**, please refer to
  :ref:`use-case-largeData`.

* Fast DDS v3 introduced the new feature :ref:`XTypes<dynamic-types>`, which allows to discover remote types.
  In consequence, discovery traffic can be increased during start up.
  If you are experiencing high load during discovery, try disabling the new feature.
  Please refer to :ref:`disable type propagation<property_type_propagation>` to learn how to do it.

* Applications that load **libunwind** at runtime (e.g., through profiling tools such as
  ``gperftools``, ``heaptrack``, or sanitizer runtimes) may experience segmentation faults during
  C++ exception handling within Fast DDS. This is caused by a symbol conflict between ``libunwind``
  and ``libgcc_s``: both export ``_Unwind_*`` symbols with incompatible internal layouts, and if
  ``libunwind`` is resolved first, it interferes with the GCC exception-handling ABI. The crash
  typically manifests as a ``SIGABRT`` or ``SIGSEGV`` inside the ``_Unwind_RaiseException`` call
  chain. This is especially common in Docker containers where ``libunwind`` is pulled in
  transitively by GStreamer or other multimedia frameworks.

  **Workaround**: ensure ``libgcc_s`` is loaded before ``libunwind`` by preloading it:

  .. code-block:: bash

      # Shell
      export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libgcc_s.so.1

      # Dockerfile
      ENV LD_PRELOAD=/lib/x86_64-linux-gnu/libgcc_s.so.1

  On ``aarch64`` systems, adjust the library path accordingly
  (e.g., ``/usr/lib/aarch64-linux-gnu/libgcc_s.so.1``).

  .. note::

      This is not a Fast DDS bug but a known Linux ecosystem issue affecting any C++ application
      that uses exceptions when both unwinder implementations are loaded. See
      `Ubuntu bug #1960005 <https://bugs.launchpad.net/ubuntu/+source/libunwind/+bug/1960005>`__
      and `LLVM issue #90041 <https://github.com/llvm/llvm-project/issues/90041>`__ for upstream
      discussions.
