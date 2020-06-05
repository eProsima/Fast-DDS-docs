.. _windows_sources:

Windows installation from sources
=================================

In this page, we provide the instructions for installing both the :ref:`Fast DDS library <fastdds_lib_sl>`
and the :ref:`Fast DDS-Gen <fastddsgen_sl>` generation tool from sources.
It is organized as follows:

.. contents::
    :local:
    :backlinks: none
    :depth: 2

.. _fastdds_lib_sw:

Fast DDS library installation
"""""""""""""""""""""""""""""

In this section, we provide the instructions for installing *eProsima Fast DDS* in a Windows environment from
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
* :ref:`cmake_pip_wget_git_sw`
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

.. _cmake_pip_wget_git_sw:

CMake, pip, wget and git
^^^^^^^^^^^^^^^^^^^^^^^^

These packages provide the tools required to install *eProsima Fast DDS* and its dependencies from command line.
Download and install CMake_, pip_, wget_ and git_ by following the instructions detailed in the respective
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
For more details, please refer to the :ref:`parameters_cmake` section.

For a detailed description of the Gtest installation process, please refer to the
`Gtest Installation Guide <https://github.com/google/googletest>`_.


.. _dependencies_sw:

Dependencies
------------

*eProsima Fast RTPS* has the following dependencies, when installed from sources in a Windows environment:

* :ref:`asiotinyxml2_sw`
* :ref:`openssl_sw`

.. _asiotinyxml2_sw:

Asio and TinyXML2 libraries
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Asio is a cross-platform C++ library for network and low-level I/O programming providing with a consistent
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

       pip install -U colcon-common-extensions vcstool

   and add the path to the :code:`vcs` executable to the :code:`PATH` from the
   *Edit the system environment variables* control panel.

   .. note::

       If this fails due to an Environment Error, add the :code:`--user` flag to the :code:`pip` installation command.

#. Create a :code:`Fast-DDS` directory and download the repos file that will be used to install
   *eProsima Fast DDS* and its dependencies:

   .. code-block:: bash

       mkdir ~/Fast-DDS
       cd ~/Fast-DDS
       wget https://raw.githubusercontent.com/eProsima/Fast-DDS/master/fastrtps.repos
       mkdir src
       vcs import src < fastrtps.repos

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

* Every time the :code:`Fast-DDS` directory is opened in a new shell, prepare the environment locally by typing the
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

       mkdir ~/Fast-DDS

#. Clone the following dependencies and compile them using CMake_.

   * `Foonathan memory <https://github.com/foonathan/memory>`_

     .. code-block:: bash

         cd ~/Fast-DDS
         git clone https://github.com/eProsima/foonathan_memory_vendor.git
         cd foonathan_memory_vendor
         mkdir build && cd build
         cmake .. -DCMAKE_INSTALL_PREFIX=~/Fast-DDS/install -DBUILD_SHARED_LIBS=ON
         cmake --build . --target install

   * `Fast CDR <https://github.com/eProsima/Fast-CDR.git>`_

     .. code-block:: bash

         cd ~/Fast-DDS
         git clone https://github.com/eProsima/Fast-CDR.git
         cd Fast-CDR
         mkdir build && cd build
         cmake .. -DCMAKE_INSTALL_PREFIX=~/Fast-DDS/install
         cmake --build . --target install

#. Once all dependencies are installed, install *eProsima Fast DDS*:

   .. code-block:: bash

       cd ~/Fast-DDS
       git clone https://github.com/eProsima/Fast-RTPS.git
       cd Fast-RTPS
       mkdir build && cd build
       cmake ..  -DCMAKE_INSTALL_PREFIX=~/Fast-DDS/install -DCMAKE_PREFIX_PATH=~/Fast-DDS/install
       cmake --build . --target install

.. _global_installation_sw:

Global installation
^^^^^^^^^^^^^^^^^^^

To install *eProsima Fast DDS* system-wide instead of locally, remove all the flags that
appear in the configuration steps of :code:`Fast-CDR` and :code:`Fast-RTPS`, and change the first in the
configuration step of :code:`foonathan_memory_vendor` to the following:

.. code-block:: bash

    -DCMAKE_INSTALL_PREFIX=??

.. note::

    By default, *eProsima Fast DDS* does not compile tests.
    However, they can be activated by downloading and installing `Gtest <https://github.com/google/googletest>`_.

.. _run_app_cmake_sw:

Run an application
^^^^^^^^^^^^^^^^^^

When running an instance of an application using *eProsima Fast DDS*, it must be linked with the library where the
packages have been installed. There are two possibilities:

* Prepare the environment locally by typing the command:

  .. code-block:: bash

      C:\> PATH=~/Fast-DDS/install/bin

* Link it permanently, by opening the *Edit the system environment variables* control panel, and adding
  :code:`~/Fast-DDS/install/bin` to the :code:`PATH`.


.. _fastddsgen_sw:

Fast DDS-Gen installation
"""""""""""""""""""""""""

In this section, we provide the instructions for installing *Fast DDS-Gen* in a Windows environment from
sources.
*Fast DDS-Gen* is a Java application that generates source code using the data types defined in an IDL file.

Requirements
------------

In order to compile *Fast DDS-Gen*, the following packages need to be installed in the system:

* :ref:`java_sb`
* :ref:`gradle_sb`

.. _java_sb:

Java JDK
^^^^^^^^

The JDK is a development environment for building applications and components using the Java language.
Download and install it at the following `page <https://www.oracle.com/java/technologies/javase-downloads.html>`_.

.. _gradle_sb:

Gradle
^^^^^^

Gradle is an open-source build automation tool. Download and install `Gradle <https://gradle.org/install>`_
in the preferred way.

Install
-------

Once the requirements above are met, install *Fast DDS-Gen* by following the steps below:

.. code-block:: bash

    cd ~
    git clone --recursive https://github.com/eProsima/Fast-RTPS-Gen.git
    cd Fast-RTPS-Gen
    gradle assemble

Contents
^^^^^^^^

The :code:`Fast-DDS-Gen` folder contains the following packages:

* :code:`share/fastrtps`, where the generated Java application is.
* :code:`scripts`, containing some user friendly scripts.

  .. note::

      To make these scripts available from anywhere, add the :code:`scripts` folder path to the
      :code:`PATH` environment variable.

.. External links

.. _colcon: https://colcon.readthedocs.io/en/released/
.. _CMake: https://cmake.org
.. _pip: https://pypi.org/project/pip/
.. _wget: https://www.gnu.org/software/wget/
.. _git: https://git-scm.com/
.. _vcstool: https://pypi.org/project/vcstool/
.. _Gtest: https://github.com/google/googletest
