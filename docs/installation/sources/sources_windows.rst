.. _windows_sources:

Windows installation from sources
=================================

The instructions for installing both the :ref:`Fast DDS library <fastdds_lib_sl>`
and the :ref:`Fast DDS-Gen <fastddsgen_sl>` generation tool from sources are provided in this page.
It is organized as follows:

.. contents::
    :local:
    :backlinks: none
    :depth: 2

.. _fastdds_lib_sw:

Fast DDS library installation
"""""""""""""""""""""""""""""

This section provides the instructions for installing *eProsima Fast DDS* in a Windows environment from
sources. The following packages will be installed:

* :code:`foonathan_memory_vendor`, an STL compatible C++ memory allocator
  `library <https://github.com/foonathan/memory>`_.
* :code:`fastcdr`, a C++ library that serializes according to the
  `standard CDR <https://www.omg.org/cgi-bin/doc?formal/02-06-51>`_ serialization mechanism.
* :code:`fastrtps`, the core library of *eProsima Fast DDS* library.

First of all, the :ref:`requirements_sw` and :ref:`dependencies_sw` detailed below need to be met.
Afterwards, the user can choose whether to follow either the :ref:`colcon <colcon_installation_windows>`)
or the :ref:`CMake <cmake_installation_windows>`) installation instructions.


.. _requirements_sw:

Requirements
------------

The installation of *eProsima Fast DDS* in a Windows environment from sources requires the following tools to be
installed in the system:

* :ref:`visual_studio_sw`
* :ref:`chocolatey_sw`
* :ref:`cmake_pip3_wget_git_sw`
* :ref:`gtest_sw` [optional]

.. _visual_studio_sw:

Visual Studio
^^^^^^^^^^^^^

`Visual Studio <https://visualstudio.microsoft.com/>`_ is required to
have a C++ compiler in the system. For this purpose, make sure to check the
:code:`Desktop development with C++` option during the Visual Studio installation process.

If Visual Studio is already installed but the Visual C++ Redistributable packages are not,
open Visual Studio and go to :code:`Tools` -> :code:`Get Tools and Features` and in the :code:`Workloads` tab enable
:code:`Desktop development with C++`. Finally, click :code:`Modify` at the bottom right.

.. _chocolatey_sw:

Chocolatey
^^^^^^^^^^

Chocolatey is a Windows package manager. It is needed to install some of *eProsima Fast DDS*'s dependencies.
Download and install it directly from the `website <https://chocolatey.org/>`_.

.. _cmake_pip3_wget_git_sw:

CMake, pip3, wget and git
^^^^^^^^^^^^^^^^^^^^^^^^^

These packages provide the tools required to install *eProsima Fast DDS* and its dependencies from command line.
Download and install CMake_, pip3_, wget_ and git_ by following the instructions detailed in the respective
websites.
Once installed, add the path to the executables to the :code:`PATH` from the
*Edit the system environment variables* control panel.

.. _gtest_sw:

Gtest
^^^^^

GTest is a unit testing library for C++.
By default, *eProsima Fast DDS* does not compile tests.
It is possible to activate them with the opportune
`CMake configuration options <https://cmake.org/cmake/help/v3.6/manual/cmake.1.html#options>`_
when calling colcon_ or CMake_.
For more details, please refer to the :ref:`cmake_options` section.
Also add the `Gtest repository <https://github.com/google/googletest>`_ into the workspace directory.

.. code-block:: bash

    git clone https://github.com/google/googletest src/googletest-distribution

and add next argument to the `colcon` call

.. code-block:: bash

    colcon build --cmake-args -Dgtest_force_shared_crt=ON


.. _dependencies_sw:

Dependencies
------------

*eProsima Fast RTPS* has the following dependencies, when installed from sources in a Windows environment:

* :ref:`asiotinyxml2_sw`
* :ref:`openssl_sw`

.. _asiotinyxml2_sw:

Asio and TinyXML2 libraries
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Asio is a cross-platform C++ library for network and low-level I/O programming, which provides a consistent
asynchronous model.
TinyXML2 is a simple, small and efficient C++ XML parser.
They can be downloaded directly from the links below:

* `Asio <https://github.com/ros2/choco-packages/releases/download/2020-02-24/asio.1.12.1.nupkg>`_
* `TinyXML2 <https://github.com/ros2/choco-packages/releases/download/2020-02-24/tinyxml2.6.0.0.nupkg>`_

After downloading these packages, open an administrative shell with *PowerShell* and execute the following command:

.. code-block:: bash

    choco install -y -s <PATH_TO_DOWNLOADS> asio tinyxml2

where :code:`<PATH_TO_DOWNLOADS>` is the folder into which the packages have been downloaded.

.. _openssl_sw:

OpenSSL
^^^^^^^

OpenSSL is a robust toolkit for the TLS and SSL protocols and a general-purpose cryptography library.
Download and install the latest OpenSSL version for Windows at this
`link <https://slproweb.com/products/Win32OpenSSL.html>`_.
After installing, add the environment variable :code:`OPENSSL_ROOT_DIR` pointing to the installation root directory.

For example:

.. code-block:: bash

   OPENSSL_ROOT_DIR=C:\Program Files\OpenSSL-Win64


.. _colcon_installation_windows:

Colcon installation
-------------------

colcon_ is a command line tool based on CMake_ aimed at building sets of software packages.
This section explains how to use it to compile *eProsima Fast DDS* and its dependencies.

.. important::

    Run colcon within a Visual Studio prompt. To do so, launch a *Developer Command Prompt* from the
    search engine.

#. Install the ROS 2 development tools (colcon_ and vcstool_) by executing the following command:

   .. code-block:: bash

       pip3 install -U colcon-common-extensions vcstool

   and add the path to the :code:`vcs` executable to the :code:`PATH` from the
   *Edit the system environment variables* control panel.

   .. note::

       If this fails due to an Environment Error, add the :code:`--user` flag to the :code:`pip3` installation command.

#. Create a :code:`Fast-DDS` directory and download the repos file that will be used to install
   *eProsima Fast DDS* and its dependencies:

   .. code-block:: bash

       mkdir ~\Fast-DDS
       cd ~\Fast-DDS
       wget https://raw.githubusercontent.com/eProsima/Fast-DDS/master/fastrtps.repos -output fastrtps.repos
       mkdir src
       vcs import src --input fastrtps.repos

   Finally, use colcon_ to compile all software:

   .. code-block:: bash

       colcon build

.. note::

    Being based on CMake_, it is possible to pass the CMake configuration options to the :code:`colcon build`
    command. For more information on the specific syntax, please refer to the
    `CMake specific arguments <https://colcon.readthedocs.io/en/released/reference/verb/build.html#cmake-specific-arguments>`_
    page of the colcon_ manual.

.. _run_app_colcon_sw:

Run an application
^^^^^^^^^^^^^^^^^^

When running an instance of an application using *eProsima Fast DDS*, the colcon overlay built in the
dedicated :code:`Fast-DDS` directory must be sourced.
There are two possibilities:

* Every time a new shell is opened, prepare the environment locally by typing the
  command:

  .. code-block:: bash

      setup.bat

* Add the sourcing of the colcon overlay permanently, by opening the
  *Edit the system environment variables* control panel, and adding :code:`~/Fast-DDS/install/setup.bat`
  to the :code:`PATH`.


.. _cmake_installation_windows:

CMake installation
-------------------

This section explains how to compile *eProsima Fast DDS* with CMake_, either :ref:`locally <local_installation_sw>` or
:ref:`globally <global_installation_sw>`.

.. _local_installation_sw:

Local installation
^^^^^^^^^^^^^^^^^^

#. Open a command prompt, and create a :code:`Fast-DDS` directory where to download and build *eProsima Fast DDS* and
   its dependencies:

   .. code-block:: bash

       mkdir ~\Fast-DDS

#. Clone the following dependencies and compile them using CMake_.

   * `Foonathan memory <https://github.com/foonathan/memory>`_

     .. code-block:: bash

         cd ~\Fast-DDS
         git clone https://github.com/eProsima/foonathan_memory_vendor.git
         cd foonathan_memory_vendor
         mkdir build && cd build
         cmake ..  -DBUILD_SHARED_LIBS=ON
         cmake --build . --target install

   * `Fast CDR <https://github.com/eProsima/Fast-CDR.git>`_

     .. code-block:: bash

         cd ~\Fast-DDS
         git clone https://github.com/eProsima/Fast-CDR.git
         cd Fast-CDR
         mkdir build && cd build
         cmake ..
         cmake --build . --target install

#. Once all dependencies are installed, install *eProsima Fast DDS*:

   .. code-block:: bash

       cd ~\Fast-DDS
       git clone https://github.com/eProsima/Fast-DDS.git
       cd Fast-DDS
       mkdir build && cd build
       cmake ..
       cmake --build . --target install

.. _global_installation_sw:

Global installation
^^^^^^^^^^^^^^^^^^^

To install *eProsima Fast DDS* system-wide instead of locally, remove all the flags that
appear in the configuration steps of ``Fast-CDR`` and ``Fast-DDS``.

.. note::

    By default, *eProsima Fast DDS* does not compile tests.
    However, they can be activated by downloading and installing `Gtest <https://github.com/google/googletest>`_.

.. _run_app_cmake_sw:

Run an application
^^^^^^^^^^^^^^^^^^

When running an instance of an application using *eProsima Fast DDS*, it must be linked with the library where the
packages have been installed. This can be done by opening the *Edit system environment variables* control panel and
adding to the ``PATH`` the *Fast DDS* and *Fast CDR* installation directories:

*   *Fast DDS*: C:\\Program Files\\fastrtps
*   *Fast CDR*: C:\\Program Files\\fastcdr


.. _fastddsgen_sw:

Fast DDS-Gen installation
"""""""""""""""""""""""""

This section outlines the instructions for installing *Fast DDS-Gen* in a Windows environment from
sources.
*Fast DDS-Gen* is a Java application that generates source code using the data types defined in an IDL file.
Please refer to :ref:`fastddsgen_intro` for more information.

Requirements
------------

In order to compile *Fast DDS-Gen*, the following packages need to be installed in the system:

* :ref:`java_sb`
* :ref:`gradle_sb`

.. _java_sb:

Java JDK
^^^^^^^^

The JDK is a development environment for building applications and components using the Java language.
Download and install it at the following the steps given in the
`Oracle website <https://www.oracle.com/java/technologies/javase-downloads.html>`_.

.. _gradle_sb:

Gradle
^^^^^^

Gradle is an open-source build automation tool.
Download and install the last stable version of `Gradle <https://gradle.org/install>`_ in the preferred way.

Compiling Fast DDS-Gen
----------------------

Once the requirements above are met, install *Fast DDS-Gen* by following the steps below:

.. code-block:: bash

    cd ~
    git clone --recursive https://github.com/eProsima/Fast-DDS-Gen.git
    cd Fast-DDS-Gen
    gradle assemble

Contents
^^^^^^^^

The ``Fast-DDS-Gen`` folder contains the following packages:

* ``share/fastddsgen``, where the generated Java application is.
* ``scripts``, containing some user friendly scripts.

  .. note::

      To make these scripts accessible from any directory, add the ``scripts`` folder path to the
      ``PATH`` environment variable.

.. External links

.. _colcon: https://colcon.readthedocs.io/en/released/
.. _CMake: https://cmake.org
.. _pip3: https://docs.python.org/3/installing/index.html
.. _wget: https://www.gnu.org/software/wget/
.. _git: https://git-scm.com/
.. _vcstool: https://pypi.org/project/vcstool/
.. _Gtest: https://github.com/google/googletest
