.. _tuning-socket-buffer:

Increasing socket buffers size
------------------------------

In high rate scenarios or large data scenarios, network packages can be dropped because
the transmitted amount of data fills the socket buffer before it can be processed.
Using |RELIABLE_RELIABILITY_QOS-api| mode,
*Fast DDS* will try to recover lost samples, but with the penalty of
retransmission.
With |BEST_EFFORT_RELIABILITY_QOS-api| mode,
samples will be definitely lost.

By default *eProsima Fast DDS* creates socket buffers with the system default size.
However, these sizes can be modified using the :ref:`dds_layer_domainParticipantQos`,
as shown in the example below.

.. tab-set-code::

    .. literalinclude:: /../code/DDSCodeTester.cpp
       :language: c++
       :start-after: //CONF-QOS-INCREASE-SOCKETBUFFERS
       :end-before: //!--
       :dedent: 8

    .. literalinclude:: /../code/XMLTester.xml
       :language: xml
       :start-after: <!-->CONF-QOS-INCREASE-SOCKETBUFFERS
       :end-before: <!--><-->
       :lines: 2-3,5-
       :append: </profiles>

.. _finding-out-maximum-socket-values:

Finding out system maximum values
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Operating systems set a maximum value for socket buffer sizes.
If the buffer sizes are tuned with DomainParticipantQos, the values set
cannot exceed the maximum value of the system.

Linux
.....

The maximum buffer size values can be retrieved with the command ``sysctl``.
For socket buffers used to send data, use the following command:

.. code-block:: bash

   $> sudo sysctl -a | grep net.core.wmem_max
   net.core.wmem_max = 1048576

For socket buffers used to receive data the command is:

.. code-block:: bash

   $> sudo sysctl -a | grep net.core.rmem_max
   net.core.rmem_max = 4194304

However, these maximum values are also configurable and can be increased if needed.
The following command increases the maximum buffer size of sending sockets:

.. code-block:: bash

    $> sudo sysctl -w net.core.wmem_max=12582912

For receiving sockets, the command is:

.. code-block:: bash

    $> sudo sysctl -w net.core.rmem_max=12582912

Linux also defines per-socket TCP buffer sizes as triplets:

.. code-block:: text

    $> net.ipv4.tcp_wmem = <min> <default> <max> (TCP send buffer)
    $> net.ipv4.tcp_rmem = <min> <default> <max> (TCP receive buffer)

The middle value is the default used for most connections. If only the global maxima are raised, sockets may still
use a small default and saturate during bursts.
Set the current values for sending sockets with:

.. code-block:: bash

    $> sudo sysctl -w net.ipv4.tcp_wmem="4096 12582912 12582912"

For receiving sockets, the command is:

.. code-block:: bash

    $> sudo sysctl -w net.ipv4.tcp_rmem="4096 12582912 12582912"

Windows
.......

The following command changes the maximum buffer size of sending sockets:

.. code-block::

    C:\> reg add HKLM\SYSTEM\CurrentControlSet\services\AFD\Parameters /v DefaultSendWindow /t REG_DWORD /d 12582912

For receiving sockets, the command is:

.. code-block::

    C:\> reg add HKLM\SYSTEM\CurrentControlSet\services\AFD\Parameters /v DefaultReceiveWindow /t REG_DWORD /d 12582912


Increasing the Transmit Queue Length of an interface (Linux only)
-----------------------------------------------------------------

The Transmit Queue Length (``txqueuelen``) is a TCP/UDP/IP stack network interface value.
This value sets the number of packets allowed per kernel transmit queue of a network interface device.
By default, the ``txqueuelen`` value for Ethernet interfaces is set to ``1000`` in Linux.
This value is adequate for most Gigabit network devices.
However, in some specific cases, the ``txqueuelen`` setting should be increased to avoid overflows that drop packets.
Similarly, choosing a value that is too large can cause added overhead resulting in higher network latencies.

Note that this information only applies to the `sending` side, and not the `receiving` side.
Also increasing the ``txqueuelen`` should go together with increasing the buffer sizes of the UDP and/or TCP buffers.
(this must be applied for both the `sending` and `receiving` sides).

The settings for a specific network adapter can be viewed using the one of the following commands:

.. tab-set::

    .. tab-item:: ``ip``

        .. code-block:: bash

            ip link show ${interface}

    .. tab-item:: ``ifconfig``

        .. code-block:: bash

            ifconfig ${interface}

This will display the configuration of the adapter, and among the parameters the ``txqueuelen``.
This parameter can be a value between 1000 and 20000.

.. important::

  If the ``ip`` command is used, the Transmit Queue Length parameter is called ``qlen``.

The ``txqueuelen`` can be modified for the current session using either the ``ifconfig`` or ``ip`` commands.
However, take into account that after rebooting the default values will be configured again.

.. tab-set::

    .. tab-item:: ``ip``

        .. code-block:: bash

            ip link set txqueuelen ${value} dev ${interface}

    .. tab-item:: ``ifconfig``

        .. code-block:: bash

            ifconfig ${interface} txqueuelen ${size}
