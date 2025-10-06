.. _linux_binaries:

Linux installation from binaries
================================

The instructions for installing *eProsima Fast DDS* in a Linux environment from
binaries are provided in this page.

.. _install_bl:

Install
-------

The latest release of *eProsima Fast DDS* for Linux is available at the eProsima website
`Downloads tab <https://eprosima.com/index.php/downloads-all>`_.
Once downloaded, install the packagen using `apt`.

.. code-block:: bash
   :substitutions:

   sudo apt install ./fastdds-|ProjectVersion|_amd64.deb

.. _contents_bl:

.. _contents_bl:

Contents
^^^^^^^^

By default, the installation will download all the available packages, namely:

* :code:`foonathan_memory_vendor`, an STL compatible C++ memory allocator
  `library <https://github.com/foonathan/memory>`_.
* :code:`fastcdr`, a C++ library that serializes according to the
  `standard CDR <https://www.omg.org/cgi-bin/doc?formal/02-06-51>`_ serialization mechanism.
* :code:`fastdds`, the core library of *eProsima Fast DDS* library.
* :code:`fastddsgen`, a Java application that generates source code using the data types defined in an IDL file.

.. seealso::

    For further information about Fast DDS dependencies, as well as for the corresponding versions of other related
    products, please refer to the Fast DDS :ref:`dependencies_compatibilities` section.

.. _linking_bl:

Including Fast-DDS in a CMake project
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The installer deploys *CMake config* files that simplify to incorporate **Fast-DDS** to any CMake project via
the *find_package* CMake API.

Shared and static libraries are provided by the installer. The user can select which one will be used in the CMake
project using next mechanisms.

1. Through CMake package components when calling `find_package()`.

   .. code-block:: cmake

       find_package(fastdds shared) # Load shared library target
       find_package(fastdds static) # Load static library target

2. Through the custom CMake variable `fastdds_SHARED_LIBS`.

   .. code-block:: bash

       cmake -Dfastdds_SHARED_LIBS=ON .. # Load shared library target
       cmake -Dfastdds_SHARED_LIBS=OFF .. # Load static library target

3. Through the built-in CMake variable `BUILD_SHARED_LIBS`.

   .. code-block:: bash

       cmake -DBUILD_SHARED_LIBS=ON .. # Load shared library target
       cmake -DBUILD_SHARED_LIBS=OFF .. # Load static library target

4. In case no previous mechanism is used, CMake will try to load static library target.
   If it fails then CMake will try to load shared library target.

For example in order to build the examples dynamically linked to **Fast-DDS** do:

.. code-block:: bash

 cmake -Bbuildexample -DBUILD_SHARED_LIBS=ON .
 cmake --build buildexample --target install


.. _cli_bl:

Fast DDS CLI (optional)
-----------------------

The :ref:`Fast DDS CLI<ffastddscli_cli>` (Command Line Interface) is a tool that provides a set commands and
sub-commands to perform, Fast DDS related, maintenance and configuration tasks.
As an optional tool, its dependencies are not installed by default, but they can be installed by running the
following command:

.. code-block:: bash

    pip install xmlschema

Python3 is required to run the CLI tool, and the `xmlschema <https://pypi.org/project/xmlschema/>`_ dependency is
needed to use the :ref:`XML validation command<cli_xml>`.
