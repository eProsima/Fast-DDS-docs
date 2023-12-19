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

This section provides the instructions for installing *eProsima Fast DDS* in a Windows environment from sources.
The following packages will be installed:

* :code:`foonathan_memory_vendor`, an STL compatible C++ memory allocator
  `library <https://github.com/foonathan/memory>`_.
* :code:`fastcdr`, a C++ library that serializes according to the
  `standard CDR <https://www.omg.org/cgi-bin/doc?formal/02-06-51>`_ serialization mechanism.
* :code:`fastrtps`, the core library of *eProsima Fast DDS* library.

First of all, the :ref:`requirements_sw` and :ref:`dependencies_sw` detailed below need to be met.
Afterwards, the user can choose whether to follow either the :ref:`colcon <colcon_installation_windows>`
or the :ref:`CMake <cmake_installation_windows>` installation instructions.


.. _requirements_sw:

Requirements
------------

The installation of *eProsima Fast DDS* in a Windows environment from sources requires the following tools to be
installed in the system:

* :ref:`visual_studio_sw`
* :ref:`chocolatey_sw`
* :ref:`cmake_pip3_wget_git_sw`
* :ref:`gtest_sw` [optional]
* :ref:`pythonreq_sw` [optional]

.. _visual_studio_sw:

Visual Studio
^^^^^^^^^^^^^

`Visual Studio <https://visualstudio.microsoft.com/>`_ is required to have a C++ compiler in the system.
For this purpose, make sure to check the :code:`Desktop development with C++` option during the Visual Studio
installation process.

If Visual Studio is already installed but the Visual C++ Redistributable packages are not,
open Visual Studio and go to :code:`Tools` -> :code:`Get Tools and Features` and in the :code:`Workloads` tab enable
:code:`Desktop development with C++`.
Finally, click :code:`Modify` at the bottom right.

.. _chocolatey_sw:

Chocolatey
^^^^^^^^^^

Chocolatey is a Windows package manager.
It is needed to install some of *eProsima Fast DDS*'s dependencies.
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

    git clone --branch release-1.11.0 https://github.com/google/googletest src/googletest-distribution

and add next argument to the `colcon` call

.. code-block:: bash

    colcon build --cmake-args -Dgtest_force_shared_crt=ON


.. _pythonreq_sw:

XML validation tool
^^^^^^^^^^^^^^^^^^^
XML validation is a new command introduced to validate the XML profiles against an XSD schema through Fast DDS CLI.
That ensures the proper characterization of the entities using the xml profiles.

For more details, please refer to the :ref:`cli_xml` section.

Install the xmlschema_ dependency to be able to use this optional tool.

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
Install it by running the following command inside an administrative shell with *PowerShell* and execute the following command:

.. code-block:: bash

   choco install -y openssl

.. _libp11_sw:

Libp11 and SoftHSM libraries
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Libp11 provides PKCS#11 support for OpenSSL.
This is an optional dependency, that is needed only when *eprosima Fast DDS* is used with security and PKCS#11 URIs.

Download the latest libp11_ version for Windows from this
`repository <https://github.com/OpenSC/libp11>`__
and follow the `installation instructions <https://github.com/OpenSC/libp11/blob/master/INSTALL.md>`_

SoftHSM is a software implementation of an HSM (Hardware Security Module).
If *eProsima Fast DDS* tests are activated and *libp11* is installed
on the system, SoftHSM is additionally required to run tests of PKCS#11 features.

Download the SoftHSM_ for Windows installer from this
`repository <https://github.com/disig/SoftHSM2-for-Windows>`__.
Execute the installer and follow the installation instructions.

OpenSSL access HSM and other hardware devices through its engine functionality.
In order to set up a new engine the OpenSSL configuration files must be updated specifying the
libp11_ and hardware module (here SoftHSM_) dynamic libraries location.

OpenSSL on Windows references its default configuration file through the `OPENSSL_CONF`
environment variable.
By default OpenSSL installs two identical default configuration files:

* `C:\\Program Files\\OpenSSL-Win64\\bin\\cnf\\openssl.cnf` mimics the Linux distributions one.

* `C:\\Program Files\\OpenSSL-Win64\\bin\\openssl.cfg` kept for backward compatibility.

Neither of them are loaded by default.
In order to direct OpenSSL to load one of them or any other we must set the variable:

.. code-block:: console

   cmd> set OPENSSL_CONF=C:\Program Files\OpenSSL-Win64\bin\cnf\openssl.cnf
   powershell> $Env:OPENSSL_CONF="C:\Program Files\OpenSSL-Win64\bin\cnf\openssl.cnf"

Once we have hinted OpenSSL the configuration file to use we must modify it to set up the
new PKCS#11 engine following the
`OpenSSL guidelines <https://www.openssl.org/docs/man1.1.1/man5/config.html#Engine-Configuration-Module>`_
replacing the binaries path with the proper ones.
For example, before any section in the configuration file we introduce:

.. code-block:: cfg

    openssl_conf = openssl_init

at the end of the file we include the engine devoted sections.
Note to use POSIX path separator instead of the windows one.

.. code-block:: cfg

    [openssl_init]
        engines = engine_section

    [engine_section]
        pkcs11 = pkcs11_section

        [pkcs11_section]
        engine_id = pkcs11
        dynamic_path = C:/Program Files/libp11/src/pkcs11.dll
        MODULE_PATH = C:/Program Files (x86)/SoftHSM2/lib/softhsm2-x64.dll
        init = 0

A proper set up can be verified using OpenSSL command line tool:

.. code-block:: console

    openssl engine pkcs11 -t

.. _colcon_installation_windows:

Colcon installation
-------------------

colcon_ is a command line tool based on CMake_ aimed at building sets of software packages.
This section explains how to use it to compile *eProsima Fast DDS* and its dependencies.

.. important::

    Run colcon within a Visual Studio prompt.
    To do so, launch a *Developer Command Prompt* from the search engine.

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

    Being based on CMake_, it is possible to pass the CMake configuration options to the :code:`colcon build` command.
    For more information on the specific syntax, please refer to the
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

       mkdir %USERPROFILE%\Fast-DDS

#. Clone the following dependencies and compile them using CMake_.

   * Fast DDS depends on `Foonathan memory <https://github.com/foonathan/memory>`_.
     To ease the dependency management, *eProsima* provides a vendor package
     `Foonathan memory vendor <https://github.com/eProsima/foonathan_memory_vendor>`_, which downloads and builds a
     specific revision of *Foonathan memory* if the library is not found in the system.

     .. code-block:: bash

         cd %USERPROFILE%\Fast-DDS
         git clone https://github.com/eProsima/foonathan_memory_vendor.git
         cd foonathan_memory_vendor
         mkdir build && cd build
         cmake -DCMAKE_INSTALL_PREFIX=%USERPROFILE%/Fast-DDS/install ..
         cmake --build . --target install

   * `Fast CDR <https://github.com/eProsima/Fast-CDR.git>`_

     .. code-block:: bash

         cd %USERPROFILE%\Fast-DDS
         git clone https://github.com/eProsima/Fast-CDR.git
         cd Fast-CDR
         mkdir build && cd build
         cmake -DCMAKE_INSTALL_PREFIX=%USERPROFILE%/Fast-DDS/install ..
         cmake --build . --target install

#. Once all dependencies are installed, install *eProsima Fast DDS*:

   .. code-block:: bash

       cd %USERPROFILE%\Fast-DDS
       git clone https://github.com/eProsima/Fast-DDS.git
       cd Fast-DDS
       mkdir build && cd build
       cmake -DCMAKE_INSTALL_PREFIX=%USERPROFILE%/Fast-DDS/install ..
       cmake --build . --target install

.. _global_installation_sw:

Global installation
^^^^^^^^^^^^^^^^^^^

To install *eProsima Fast DDS* system-wide instead of locally, remove the ``CMAKE_INSTALL_PREFIX`` flags that
appear in the configuration steps of ``Fast-CDR`` and ``Fast-DDS``.

.. note::

    By default, *eProsima Fast DDS* does not compile tests.
    However, they can be activated by downloading and installing `Gtest <https://github.com/google/googletest>`_.

.. _run_app_cmake_sw:

Run an application
^^^^^^^^^^^^^^^^^^

When running an instance of an application using *eProsima Fast DDS*, it must be linked with the library where the
packages have been installed.
This can be done by opening the *Edit system environment variables* control panel and adding to the ``PATH`` the
*Fast DDS* and *Fast CDR* installation directories:

*   *Fast DDS*: C:\\Program Files\\fastrtps
*   *Fast CDR*: C:\\Program Files\\fastcdr

.. _fastdds_python_sw:

Fast DDS Python bindings installation
"""""""""""""""""""""""""""""""""""""

This section provides the instructions for installing *Fast DDS Python bindings* in a Windows environment from sources.
*Fast DDS Python bindings* is an extension of *Fast DDS* which provides access to the Fast DDS API through Python.
Therefore, its installation is an extension of the installation of :ref:`Fast DDS <fastdds_lib_sw>`.

*Fast DDS Python bindings* source code consists on several `.i` files which will be processed by SWIG_.
Then C++ files (for connecting C++ and Python) and Python files (Python module for Fast DDS) will be generated.

First of all, the :ref:`requirements_python_sw` and :ref:`dependencies_python_sw` detailed below need to be met.
Afterwards, the user can choose whether to follow either the :ref:`colcon <colcon_installation_python_windows>` or the
:ref:`CMake <cmake_installation_python_windows>` installation instructions.

.. _requirements_python_sw:

Requirements
------------

The installation of *Fast DDS Python bindings* in a Windows environment from sources requires the following tools to be
installed in the system:

- :ref:`Fast DDS requirements <requirements_sw>`
- :ref:`swig_python_sw`

.. _swig_python_sw:

.. include:: ../../04-common/python_requirements.rst
   :start-after: .. begin-swig
   :end-before: .. end-windows-swig

.. _dependencies_python_sw:

Dependencies
------------

*Fast DDS Python bindings* has the following dependencies, when installed from sources in a Windows environment:

- :ref:`Fast DDS dependencies <dependencies_sw>`

.. _colcon_installation_python_windows:

Colcon installation
-------------------

colcon_ is a command line tool based on CMake_ aimed at building sets of software packages.
This section explains how to use it to compile *Fast DDS Python bindings* and its dependencies.

.. important::

    Run colcon within a Visual Studio prompt.
    To do so, launch a *Developer Command Prompt* from the search engine.

#. Install the ROS 2 development tools (colcon_ and vcstool_) by executing the following command:

   .. code-block:: bash

       pip3 install -U colcon-common-extensions vcstool

   and add the path to the :code:`vcs` executable to the :code:`PATH` from the
   *Edit the system environment variables* control panel.

   .. note::

       If this fails due to an Environment Error, add the :code:`--user` flag to the :code:`pip3` installation command.

#. Create a :code:`Fast-DDS-python` directory and download the repos file that will be used to install
   *Fast DDS Python bindings* and its dependencies:

   .. code-block:: bash

       mkdir ~\Fast-DDS-python
       cd ~\Fast-DDS-python
       wget https://raw.githubusercontent.com/eProsima/Fast-DDS-python/main/fastdds_python.repos
       mkdir src
       vcs import src --input fastdds_python.repos

#. Build the packages:

   .. code-block:: bash

       colcon build

.. note::

    Being based on CMake_, it is possible to pass CMake configuration options to the :code:`colcon build` command.
    For more information on the specific syntax, please refer to the
    `CMake specific arguments <https://colcon.readthedocs.io/en/released/reference/verb/build.html#cmake-specific-arguments>`_
    page of the colcon_ manual.

Run an application
^^^^^^^^^^^^^^^^^^

When running an instance of an application using *Fast DDS Python bindings*, the colcon overlay built in the
dedicated :code:`Fast-DDS-python` directory must be sourced.
There are two possibilities:

* Every time a new shell is opened, prepare the environment locally by typing the
  command:

  .. code-block:: bash

      setup.bat

* Add the sourcing of the colcon overlay permanently, by opening the
  *Edit the system environment variables* control panel, and adding :code:`~/Fast-DDS/install/setup.bat`
  to the :code:`PATH`.

.. _cmake_installation_python_windows:

CMake installation
------------------

This section explains how to compile *Fast DDS Python bindings* with CMake_, either
:ref:`locally <local_installation_python_sw>` or :ref:`globally <global_installation_python_sw>`.

.. _local_installation_python_sw:

Local installation
^^^^^^^^^^^^^^^^^^

#. Open a command prompt, and create a :code:`Fast-DDS-python` directory where to download and build
   *Fast DDS Python bindings* and its dependencies:

   .. code-block:: bash

       mkdir %USERPROFILE%\Fast-DDS-python

#. Clone the following dependencies and compile them using CMake_.

   * Fast DDS depends on `Foonathan memory <https://github.com/foonathan/memory>`_.
     To ease the dependency management, *eProsima* provides a vendor package
     `Foonathan memory vendor <https://github.com/eProsima/foonathan_memory_vendor>`_, which downloads and builds a
     specific revision of *Foonathan memory* if the library is not found in the system.

     .. code-block:: bash

         cd %USERPROFILE%\Fast-DDS-python
         git clone https://github.com/eProsima/foonathan_memory_vendor.git
         cd foonathan_memory_vendor
         mkdir build && cd build
         cmake -DCMAKE_INSTALL_PREFIX=%USERPROFILE%/Fast-DDS-python/install ..
         cmake --build . --target install

   * `Fast CDR <https://github.com/eProsima/Fast-CDR.git>`_

     .. code-block:: bash

         cd %USERPROFILE%\Fast-DDS-python
         git clone https://github.com/eProsima/Fast-CDR.git
         cd Fast-CDR
         mkdir build && cd build
         cmake -DCMAKE_INSTALL_PREFIX=%USERPROFILE%/Fast-DDS-python/install ..
         cmake --build . --target install

   * `Fast DDS <https://github.com/eProsima/Fast-DDS.git>`_

     .. code-block:: bash

         cd %USERPROFILE%\Fast-DDS-python
         git clone https://github.com/eProsima/Fast-DDS.git
         cd Fast-DDS
         mkdir build && cd build
         cmake -DCMAKE_INSTALL_PREFIX=%USERPROFILE%/Fast-DDS-python/install ..
         cmake --build . --target install

#. Once all dependencies are installed, install *Fast DDS Python bindings*:

   .. code-block:: bash

       cd ~/Fast-DDS-python
       git clone https://github.com/eProsima/Fast-DDS-python.git
       cd Fast-DDS-python
       mkdir build && cd build
       cmake -DCMAKE_INSTALL_PREFIX=%USERPROFILE%/Fast-DDS-python/install ..
       cmake --build . --target install

.. _global_installation_python_sw:

Global installation
^^^^^^^^^^^^^^^^^^^

To install *Fast DDS Python bindings* system-wide instead of locally, remove all the flags that
appear in the configuration steps of :code:`Fast-CDR`, :code:`Fast-DDS` and :code:`Fast-DDS-python`, and change the
first in the configuration step of :code:`foonathan_memory_vendor` to the following:

.. code-block:: bash

    -DCMAKE_INSTALL_PREFIX=/usr/local/ -DBUILD_SHARED_LIBS=ON

.. note::

    Installation on system directories may need of permissions.
    Maybe permissions have to be granted through :code:`sudo`.

    .. code-block:: bash

        sudo cmake --build . --target install

Run an application
^^^^^^^^^^^^^^^^^^

When running an instance of an application using *Fast DDS Python bindings*, it must be linked with the library where
the packages have been installed.
This can be done by opening the *Edit system environment variables* control panel and adding to the ``PATH`` the
*Fast DDS python*, *Fast CDR* and *Fast DDS* installation directories:

*   *Fast DDS python*: C:\\Program Files\\fastdds_python
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

*Fast DDS-Gen* is built using Gradle.
Gradle is an open-source build automation tool which requires a Java version to be executed (see
`Gradle-Java compatibility matrix <https://docs.gradle.org/current/userguide/compatibility.html>`_).

.. important::

    Even though earlier versions of Gradle support Java 8, *Fast DDS-Gen* stopped supporting Java versions previous to
    Java 11 since release v2.4.0.

.. important::

    *Fast DDS-Gen* introduced support for Gradle 7 in release v2.2.0.
    Gradle 8 is not yet supported.

.. _java_sb:

Java JDK
^^^^^^^^

The JDK is a development environment for building applications and components using the Java language.
Download and install it following the steps given in the
`Oracle website <https://www.oracle.com/java/technologies/javase-downloads.html>`_.

.. note::

    *Fast DDS-Gen* supports Java versions from 11 to 19.

Compiling Fast DDS-Gen
----------------------

In order to compile *Fast DDS-Gen*, an executable script is included in the repository which will download Gradle
temporarily for the compilation step.
Please, follow the steps below to build *Fast DDS-Gen*:

.. note::

    If Fast DDS has already been installed following :ref:`colcon_installation_windows`, skip cloning *Fast DDS-Gen*'s
    repository, as it can already be found under the :code:`src` directory within the colcon workspace.

.. code-block:: bash

    cd ~
    git clone --recursive https://github.com/eProsima/Fast-DDS-Gen.git
    cd Fast-DDS-Gen
    gradlew.bat assemble

.. note::

    In case that a supported Gradle version is already installed in the system, *Fast DDS-Gen* can also be built running
    directly:

    .. code-block:: bash

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
.. _libp11: https://github.com/OpenSC/libp11/
.. _SoftHSM: https://www.opendnssec.org/softhsm/
.. _xmlschema: https://pypi.org/project/xmlschema/
