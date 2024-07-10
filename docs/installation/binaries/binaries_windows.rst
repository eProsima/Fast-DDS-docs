.. _windows_binaries:

Windows installation from binaries
==================================

The instructions for installing *eProsima Fast DDS* in a Windows environment from
binaries are provided in this page.
It is organized as follows:

.. contents::
    :local:
    :backlinks: none
    :depth: 2

First of all, the :ref:`requirements_bw` detailed below need to be met.


.. _requirements_bw:

Requirements
------------

The installation of *eProsima Fast DDS* in a Windows environment from binaries requires the following tools to be
installed in the system:

* :ref:`visual_studio_bw`

.. _visual_studio_bw:

Visual Studio
^^^^^^^^^^^^^

`Visual Studio <https://visualstudio.microsoft.com/>`_ is required to
have a C++ compiler in the system. For this purpose, make sure to check the
:code:`Desktop development with C++` option during the Visual Studio installation process.

If Visual Studio is already installed but the Visual C++ Redistributable packages are not,
open Visual Studio and go to :code:`Tools` -> :code:`Get Tools and Features` and in the :code:`Workloads` tab enable
:code:`Desktop development with C++`. Finally, click :code:`Modify` at the bottom right.

.. _install_bw:

Install
-------

The latest release of *eProsima Fast DDS* for Windows is available at the company website
`downloads page <https://eprosima.com/index.php/downloads-all>`_.
Once downloaded, execute the installer and follow the instructions, choosing the preferred Visual Studio
version and architecture when prompted.


.. note::

    By default, *eProsima Fast DDS* does not compile tests. To activate them, please refer to the
    :ref:`windows_sources` page.

    To use the :ref:`cli_xml` validation tool, please refer to the :ref:`windows_sources` page.

.. _contents_bw:

Contents
^^^^^^^^

By default, the installation will download all the available packages, namely:

* :code:`foonathan_memory_vendor`, an STL compatible C++ memory allocator
  `library <https://github.com/foonathan/memory>`_.
* :code:`fastcdr`, a C++ library that serializes according to the
  `standard CDR <https://www.omg.org/cgi-bin/doc?formal/02-06-51>`_ serialization mechanism.
* :code:`fastrtps`, the core library of *eProsima Fast DDS* library.
* :code:`fastddsgen`, a Java application that generates source code using the data types defined in an IDL file.

.. _env_vars_bw:

Environment variables
^^^^^^^^^^^^^^^^^^^^^

*eProsima Fast DDS* requires the following environment variable setup in order to function properly:

* :code:`FASTRTPSHOME`: Root folder where *eProsima Fast DDS* is installed.
* Additions to the ``PATH``: The location of *eProsima Fast DDS* scripts and libraries should be
  appended to the ``PATH``.

These variables are set automatically by checking the corresponding box during the installation process.

.. _linking_bw:

Including Fast-DDS in a CMake project
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The installer deploys *CMake config* files that simplify to incorporate **Fast-DDS** to any CMake project via
the *find_package* CMake API.

By setting the CMake variable **FASTDDS_STATIC** is possible to choose the desired linkage (dynamic or static library)
in the CMake generator stage. If the variable is missing defaults to dynamic linking.

For example in order to build the examples statically linked to **Fast-DDS** do:

   .. code-block:: console

    > cmake -Bbuildexample -DFASTDDS_STATIC=ON .
    > cmake --build buildexample --target install


.. _cli_bw:

Fast DDS CLI (optional)
-----------------------

The :ref:`Fast DDS CLI<ffastddscli_cli>` (Command Line Interface) is a tool that provides a set commands and
sub-commands to perform, Fast DDS related, maintenance and configuration tasks.
As an optional tool, its dependencies are not installed by default, but they can be installed by running the
following command:

.. code-block:: bash

    choco install python
    python -m pip install --upgrade pywin32 xmlschema

Python3 is required to run the CLI tool, and the `xmlschema <https://pypi.org/project/xmlschema/>`_ dependency is
needed to use the :ref:`XML validation command<cli_xml>`.
