.. _windows_binaries:

Windows installation from binaries
==================================

In this page, we provide the instructions for installing *eProsima Fast DDS* in a Windows environment from
binaries.

It is organized as follows:

.. contents::
    :local:
    :backlinks: none
    :depth: 2

First of all, the :ref:`requirements_bw` and :ref:`dependencies_bw` detailed below need to be met.


.. _requirements_bw:

Requirements
------------

The installation of *eProsima Fast DDS* in a Windows environment from binaries requires the following tools to be
installed in the system:

* :ref:`visual_studio_bw`
* :ref:`chocolatey_bw`

.. _visual_studio_bw:

Visual Studio
^^^^^^^^^^^^^

`Visual Studio <https://visualstudio.microsoft.com/>`_ is required to
have a C++ compiler in the system. For this purpose, make sure to check the
:code:`Desktop development with C++` option during the Visual Studio installation process.

If Visual Studio is already installed but the Visual C++ Redistributable packages are not,
open Visual Studio and go to :code:`Tools` -> :code:`Get Tools and Features` and in the :code:`Workloads` tab enable
:code:`Desktop development with C++`. Finally, click :code:`Modify` at the bottom right.

.. _chocolatey_bw:

Chocolatey
^^^^^^^^^^

Chocolatey is a Windows package manager. It is needed to install some of *eProsima Fast DDS*'s dependencies.
Download and install it directly from the `website <https://chocolatey.org/>`_.


.. _dependencies_bw:


Dependencies
------------

*eProsima Fast RTPS* has the following dependencies, when installed from binaries in a Windows environment:

* :ref:`tinyxml2_bw`
* :ref:`openssl_bw`

.. _tinyxml2_bw:

TinyXML2
^^^^^^^^

TinyXML2 is a simple, small and efficient C++ XML parser.
It can be downloaded directly from
`here <https://github.com/ros2/choco-packages/releases/download/2020-02-24/tinyxml2.6.0.0.nupkg>`_.

After downloading this package, open an administrative shell with *PowerShell* and execute the following command:

.. code-block:: bash

    choco install -y -s <PATH_TO_DOWNLOADS> tinyxml2

where :code:`<PATH_TO_DOWNLOADS>` is the folder into which the package has been downloaded.

.. _openssl_bw:

OpenSSL
^^^^^^^

OpenSSL is a robust toolkit for the TLS and SSL protocols and a general-purpose cryptography library.
Download and install the latest OpenSSL version for Windows at this
`link <https://slproweb.com/products/Win32OpenSSL.html>`_.
After installing, add the environment variable :code:`OPENSSL_ROOT_DIR` pointing to the installation root directory.

For example:

.. code-block:: bash

   OPENSSL_ROOT_DIR=C:\Program Files\OpenSSL-Win64

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


.. _contents_bw:

Contents
^^^^^^^^

By default, the installation will download all the available packages, namely:

* :code:`foonathan_memory_vendor`, an STL compatible C++ memory allocator
  `library <https://github.com/foonathan/memory>`_.
* :code:`fastcdr`, a C++ library that serializes according to the
  `standard CDR <https://www.omg.org/cgi-bin/doc?formal/02-06-51>`_ serialization mechanism.
* :code:`fastrtps`, the core library of *eProsima Fast DDS* library.
* :code:`fastrtpsgen`, a Java application that generates source code using the data types defined in an IDL file.

.. _env_vars_bw:

Environment variables
^^^^^^^^^^^^^^^^^^^^^

eProsima Fast RTPS requires the following environment variable setup in order to function properly:

* :code:`FASTRTPSHOME`: Root folder where *eProsima Fast DDS* is installed.
* Additions to the PATH: the location of *eProsima Fast DDS* scripts and libraries should be
  appended to the PATH.

These variables are set automatically by checking the corresponding box during the installation process.
