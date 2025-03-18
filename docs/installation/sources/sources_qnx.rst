.. _qnx_sources:

QNX 7.1 installation from sources
=================================

The instructions for installing :ref:`Fast DDS library <fastdds_lib_qnx>`
and running examples and tests on QNX 7.1 are provided in this page.
It is organized as follows:

.. _fastdds_lib_qnx:

Fast DDS library installation
"""""""""""""""""""""""""""""

This section provides the instructions for installing *eProsima Fast DDS* for QNX 7.1 in a Ubuntu environment from
sources. The following packages will be installed:

* :code:`foonathan_memory_vendor`, an STL compatible C++ memory allocator
  `library <https://github.com/foonathan/memory>`_.
* :code:`fastdds_gen`, a Java application that generates source code using the data types defined in an IDL file.
* :code:`fastcdr`, a C++ library that serializes according to the
  `standard CDR <https://www.omg.org/cgi-bin/doc?formal/02-06-51>`_ serialization mechanism.
* :code:`fastdds`, the core library of *eProsima Fast DDS* library.

.. seealso::

    For further information about Fast DDS library dependencies, as well as for the corresponding versions of other
    related products, please refer to the Fast DDS :ref:`dependencies_compatibilities_library_dependencies` section.

The :ref:`requirements_sw_qnx` detailed below needs to be met first.

.. _requirements_sw_qnx:

Requirements
------------

Users must be in a Ubuntu environment to cross-compile for QNX 7.1. It is recommended that users use Ubuntu 20.04.
The installation of *eProsima Fast DDS* in a Ubuntu environment from sources requires the following tools to be
installed in the system:

* :ref:`qnx_sdp_7.1_sw`
* :ref:`cmake_pip3_git_sw`

.. _qnx_sdp_7.1_sw:

QNX SDP 7.1
^^^^^^^^^^^^^

`QNX SDP 7.1 <https://www.qnx.com/download/>`_ is required to be installed in the user's Ubuntu environment.
QNX SDP is QNX's Software Development Platform which contains tools and files which are needed to cross-compile for QNX.

`QNX SDP Installation Guide <https://www.qnx.com/developers/docs/7.0.0/#com.qnx.doc.qnxsdp.quickstart/topic/install_host.html>`_

For the purpose of these instructions, QNX SDP 7.1 is assumed to be installed at ~/qnx710.
If this is not the case, please adjust the paths accordingly.

.. seealso::

    For further information about Fast DDS build system dependencies regarding QNX 7.1, please refer to the Fast DDS
    :ref:`dependencies_compatibilities_build_system_dependencies` section.

.. _cmake_pip3_git_sw:

CMake, pip3, git, dos2unix, and automake
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

These packages provide the tools required to install *eProsima Fast DDS* and its dependencies from command line.

.. code-block:: bash

    sudo apt install cmake python3-pip git dos2unix automake

.. _build_and_installation_sw:

Build and Installation
----------------------

#. Open a terminal and clone *eProsima Fast DDS*:

   .. code-block:: bash

       git clone https://github.com/eProsima/Fast-DDS.git && cd Fast-DDS
       WORKSPACE=$PWD

#. Initialize git submodules for Asio, Fast-CDR and TinyXML2 and apply QNX patches to them:

   .. note::
       OpenSSL is already installed in the QNX SDP 7.1.

   .. code-block:: bash

       cd $WORKSPACE

       # Initialize git submodules
       git submodule update --init $WORKSPACE/thirdparty/asio/ $WORKSPACE/thirdparty/fastcdr $WORKSPACE/thirdparty/tinyxml2/

       # Apply QNX patch to Asio.
       cd $WORKSPACE/thirdparty/asio
       git apply $WORKSPACE/build_qnx/qnx_patches/asio_qnx.patch

       # Apply QNX patch to Fast-CDR.
       cd $WORKSPACE/thirdparty/fastcdr
       git apply $WORKSPACE/build_qnx/qnx_patches/fastcdr_qnx.patch

       # Apply QNX patch to TinyXML2.
       # TinyXML2's CMakeLists.txt has CRLF, so use unix2dos to convert the patch to CRLF.
       cd $WORKSPACE/thirdparty/tinyxml2
       unix2dos $WORKSPACE/build_qnx/qnx_patches/tinyxml2_qnx.patch
       git apply $WORKSPACE/build_qnx/qnx_patches/tinyxml2_qnx.patch

#. Get foonathan_memory_vendor:

   .. code-block:: bash

       cd $WORKSPACE
       git clone https://github.com/eProsima/foonathan_memory_vendor.git

#. Optional: clone GoogleTest and apply QNX patch to it:

   .. note::

      GoogleTest is required for building Fast-DDS tests.

   .. code-block:: bash

       cd $WORKSPACE
       git clone https://github.com/google/googletest.git && cd googletest
       git checkout v1.13.0
       git apply $WORKSPACE/build_qnx/qnx_patches/googletest_qnx.patch

#. Source the QNX environment script:

   .. code-block:: bash

       source ~/qnx710/qnxsdp-env.sh

#. Build and install Fast-DDS and its dependencies:

   .. note::

       | To build examples, set COMPILE_EXAMPLES to ON in $WORKSPACE/build_qnx/common.mk.
       | To build tests, set EPROSIMA_BUILD_TESTS to ON in $WORKSPACE/build_qnx/common.mk.

   .. note::

       | All libraries will be installed to $(QNX_TARGET)/$(CPUVARDIR)/usr/lib.
       | All examples will be installed to $(QNX_TARGET)/$(CPUVARDIR)/usr/examples.
       | All tests will be installed to $(QNX_TARGET)/$(CPUVARDIR)/usr/bin/Fast-DDS_test.

       | QNX_TARGET is where the QNX SDP 7.1 installation's target folder is.
       | If QNX SDP 7.1 is installed at ~/qnx710, the QNX_TARGET will be at ~/qnx710/target/qnx7.
       | CPUVARDIR is a directory for a specific target architecture e.g. aarch64le and x86_64.

       | For example, libraries compiled for an aarch64 target will be at ~/qnx710/target/qnx7/aarch64le/usr/lib assuming QNX SDP 7.1 is installed at ~/qnx710.

   .. code-block:: bash

       cd $WORKSPACE/build_qnx
       make install -j 4

.. _run_examples_and_tests:

Run Examples and Tests on a QNX 7.1 Target
""""""""""""""""""""""""""""""""""""""""""

| Because examples and tests are compiled for QNX, they can only be run on a QNX target, not Ubuntu.

Move Libraries, Examples, and Tests to the QNX Target
------------------------------------------------------

#. Move the built libraries to the QNX target:

   The following steps assume that $(QNX_TARGET) is ~/qnx710/target/qnx7 and that $(CPUVARDIR) is aarch64le.
   Adjust the values if this is not the case.

   .. code-block:: bash

       # Move Fast-CDR library to the QNX target
       scp ~/qnx710/target/qnx7/aarch64le/usr/lib/libfastcdr.so* root@<target-ip-address>:/usr/lib

       # Move Fast-DDS library to the QNX target
       scp ~/qnx710/target/qnx7/aarch64le/usr/lib/libfastdds.so* root@<target-ip-address>:/usr/lib

       # Move Foonathan Memory library to the QNX target
       scp ~/qnx710/target/qnx7/aarch64le/usr/lib/libfoonathan_memory* root@<target-ip-address>:/usr/lib

       # Move TinyXML2 library to the QNX target
       scp ~/qnx710/target/qnx7/aarch64le/usr/lib/libtinyxml2.so* root@<target-ip-address>:/usr/lib

       # Move GoogleTest library to the QNX target
       scp ~/qnx710/target/qnx7/aarch64le/usr/lib/libgtest* root@<target-ip-address>:/usr/lib
       scp ~/qnx710/target/qnx7/aarch64le/usr/lib/libgmock* root@<target-ip-address>:/usr/lib

#. Move examples and tests to the QNX target:

   .. code-block:: bash

       # Move Fast-CDR library to the QNX target
       scp -r ~/qnx710/target/qnx7/aarch64le/usr/examples root@<target-ip-address>:/var

       # Move Fast-DDS library to the QNX target
       scp -r ~/qnx710/target/qnx7/aarch64le/usr/bin/Fast-DDS_test root@<target-ip-address>:/var

Run Hello World
------------------------

#. Open a terminal and run a subscriber:

   .. code-block:: bash

       # ssh into the QNX target
       ssh root@<target-ip-address>

       # Run a subscriber
       /var/examples/cpp/hello_world/bin/hello_world subscriber

#. Open another terminal and run a publisher:

   .. code-block:: bash

       # ssh into the QNX target
       ssh root@<target-ip-address>

       # Run a publisher
       /var/examples/cpp/hello_world/bin/hello_world publisher

The following output will be shown in the subscriber terminal:

.. code-block:: console

    Starting
    Subscriber running. Please press enter to stop the Subscriber
    Subscriber matched.
    Message HelloWorld 1 RECEIVED
    Message HelloWorld 2 RECEIVED
    Message HelloWorld 3 RECEIVED
    Message HelloWorld 4 RECEIVED
    Message HelloWorld 5 RECEIVED
    Message HelloWorld 6 RECEIVED
    Message HelloWorld 7 RECEIVED
    Message HelloWorld 8 RECEIVED
    Message HelloWorld 9 RECEIVED
    Message HelloWorld 10 RECEIVED
    Subscriber unmatched.

The following output will be shown for the publisher:

.. code-block:: console

    Starting
    Publisher running 10 samples.
    Publisher matched.
    Message: HelloWorld with index: 1 SENT
    Message: HelloWorld with index: 2 SENT
    Message: HelloWorld with index: 3 SENT
    Message: HelloWorld with index: 4 SENT
    Message: HelloWorld with index: 5 SENT
    Message: HelloWorld with index: 6 SENT
    Message: HelloWorld with index: 7 SENT
    Message: HelloWorld with index: 8 SENT
    Message: HelloWorld with index: 9 SENT
    Message: HelloWorld with index: 10 SENT

Run a Test
----------

Because test binaries compiled for QNX cannot be run on Ubuntu,
test binaries must be run on a target which is running QNX.

.. code-block:: bash

    # ssh into the QNX target
    ssh root@<target-ip-address>

    # Run a test
    cd /var/Fast-DDS_test/unittest/dds/core/entity
    ./EntityTests

The following test output for EntityTests will be shown:

.. code-block:: console

    [==========] Running 5 tests from 1 test suite.
    [----------] Global test environment set-up.
    [----------] 5 tests from EntityTests
    [ RUN      ] EntityTests.entity_constructor
    [       OK ] EntityTests.entity_constructor (0 ms)
    [ RUN      ] EntityTests.entity_enable
    [       OK ] EntityTests.entity_enable (0 ms)
    [ RUN      ] EntityTests.entity_get_instance_handle
    [       OK ] EntityTests.entity_get_instance_handle (0 ms)
    [ RUN      ] EntityTests.entity_equal_operator
    [       OK ] EntityTests.entity_equal_operator (0 ms)
    [ RUN      ] EntityTests.get_statuscondition
    [       OK ] EntityTests.get_statuscondition (0 ms)
    [----------] 5 tests from EntityTests (0 ms total)

    [----------] Global test environment tear-down
    [==========] 5 tests from 1 test suite ran. (0 ms total)
    [  PASSED  ] 5 tests.
