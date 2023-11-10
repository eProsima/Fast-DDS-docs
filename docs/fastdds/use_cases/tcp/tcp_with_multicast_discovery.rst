.. include:: ../../../03-exports/aliases.include
.. include:: ../../../03-exports/aliases-api.include

.. _use-case-tcp-multicast:

TCP Communication with Multicast Discovery
==========================================

The following snippets show how to configure *Fast DDS* |DomainParticipants| to run the
:ref:`PDP discovery<disc_phases>` phase over UDP multicast, and communicate application data over a
:ref:`transport_tcp_tcp` transport.
With this approach, applications managing large samples can benefit from transmitting their data over TCP, while
at the same time have the flexibility of automatic discovery.

.. tabs::

   .. tab:: C++

      .. literalinclude:: ../../../../code/DDSCodeTester.cpp
        :language: c++
        :dedent: 8
        :start-after: //PDP-MULTICAST-DATA-TCP
        :end-before: //!

   .. tab:: XML

      .. literalinclude:: /../code/XMLTester.xml
          :language: xml
          :start-after: <!-->PDP-MULTICAST-DATA-TCP<-->
          :end-before: <!--><-->
          :lines: 2-4, 6-80, 82-83
