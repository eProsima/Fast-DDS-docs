eProsima Fast RTPS
==================

.. image:: logo.png
   :height: 100px
   :width: 100px
   :align: left
   :alt: eProsima
   :target: http://www.eprosima.com/

*eprosima Fast RTPS* is a C++ implementation of the RTPS (Real Time Publish Subscribe) protocol, which provides publisher-subscriber communications over unreliable transports such as UDP, 
as defined and maintained by the Object Management Group (OMG) consortium. RTPS is also the wire interoperability protocol defined for the Data Distribution
Service (DDS) standard, again by the OMG. *eProsima Fast RTPS* holds the benefit of being standalone and up-to-date, as most vendor solutions either implement RTPS as a tool to implement 
DDS or use past versions of the specification.

Some of the main features of this library are:

* Configurable best-effort and reliable publish-subscribe communication policies for real-time applications.
* Plug and play connectivity so that any new applications are automatically discovered by any other members of the network.
* Modularity and scalability to allow continuous growth with complex and simple devices in the network.
* Configurable network behavior and interchangeable transport layer: Choose the best protocol and system input/output channel combination for each deployment.
* Two API Layers: a high-level Publisher-Subscriber one focused on usability and a lower-level Writer-Reader one that provides finer access to the inner workings of the RTPS protocol.

Installation Guide
------------------

You can get either a binary distribution of *eprosima Fast RTPS* or compile the library yourself from source.

Installation from Binaries
^^^^^^^^^^^^^^^^^^^^^^^^^^

The latest, up to date binary release of *eprosima Fast RTPS* can be obtained from the `company website <https://www.eprosima.com/>`_.


Installation from Source
^^^^^^^^^^^^^^^^^^^^^^^^

To compile *eprosima Fast RTPS* from source, at least Cmake version 2.8.12 and Boost 1.61 are needed.
Clone the project from GitHub: ::

    $ git clone https://github.com/eProsima/Fast-RTPS

If you are on Linux, execute: ::

    $ cmake ../ -DEPROSIMA_BUILD=ON -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=install
    $ make
    $ make install 

If you are on Windows, choose your version of Visual Studio: ::

    > cmake ../  -G"Visual Studio 14 2015 Win64" -DEPROSIMA_BUILD=ON -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=installationpath
    > cmake --build . --target install
	
If you want to compile the performance tests, you will need to add the argument `-DPERFORMANCE_TESTS=ON` when calling Cmake.

Getting Help
------------

If you need support you can reach us by mail at `support@eProsima.com` or by phone at `+34 91 804 34 48`.


