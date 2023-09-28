.. _linux_sources:

Linux installation from sources
===============================

The instructions for installing the :ref:`Fast DDS library <fastdds_lib_sl>`,
the :ref:`Fast DDS Python bindings <fastdds_python_sl>`
and the :ref:`Fast DDS-Gen <fastddsgen_sl>` generation tool from sources are provided in this page.
It is organized as follows:

.. contents::
    :local:
    :backlinks: none
    :depth: 2

.. _fastdds_lib_sl:

Fast DDS library installation
"""""""""""""""""""""""""""""

This section describes the instructions for installing *eProsima Fast DDS* in a Linux environment from sources.
The following packages will be installed:

* :code:`foonathan_memory_vendor`, an STL compatible C++ memory allocator
  `library <https://github.com/foonathan/memory>`_.
* :code:`fastcdr`, a C++ library that serializes according to the
  `standard CDR <https://www.omg.org/cgi-bin/doc?formal/02-06-51>`_ serialization mechanism.
* :code:`fastrtps`, the core library of *eProsima Fast DDS* library.

First of all, the :ref:`requirements_sl` and :ref:`dependencies_sl` detailed below need to be met.
Afterwards, the user can choose whether to follow either the :ref:`colcon <colcon_installation_linux>`
or the :ref:`CMake <cmake_installation_linux>` installation instructions.

.. _requirements_sl:


Requirements
------------

The installation of *eProsima Fast DDS* in a Linux environment from sources requires the following tools to be
installed in the system:

* :ref:`cmake_gcc_pip3_wget_git_sl`

.. _cmake_gcc_pip3_wget_git_sl:

CMake, g++, pip3, wget and git
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

These packages provide the tools required to install *eProsima Fast DDS* and its dependencies from command line.
Install CMake_, `g++ <https://gcc.gnu.org/>`_, pip3_, wget_ and git_ using the package manager of the appropriate
Linux distribution.
For example, on Ubuntu use the command:

.. code-block:: bash

    sudo apt install cmake g++ python3-pip wget git

.. _dependencies_sl:

Dependencies
------------

*eProsima Fast DDS* has the following dependencies, when installed from sources in a Linux environment:

* :ref:`asiotinyxml2_sl`
* :ref:`openssl_sl`
* :ref:`libp11_sl`
* :ref:`gtest_sl` [optional]
* :ref:`pythonreq_sl` [optional]

.. _asiotinyxml2_sl:

Asio and TinyXML2 libraries
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Asio is a cross-platform C++ library for network and low-level I/O programming, which provides a consistent
asynchronous model.
TinyXML2 is a simple, small and efficient C++ XML parser.
Install these libraries using the package manager of the appropriate Linux distribution.
For example, on Ubuntu use the command:

.. code-block:: bash

    sudo apt install libasio-dev libtinyxml2-dev

.. _openssl_sl:

OpenSSL
^^^^^^^

OpenSSL is a robust toolkit for the TLS and SSL protocols and a general-purpose cryptography library.
Install OpenSSL_ using the package manager of the appropriate Linux distribution.
For example, on Ubuntu use the command:

.. code-block:: bash

   sudo apt install libssl-dev

.. _libp11_sl:

Libp11 and SoftHSM libraries
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Libp11 provides PKCS#11 support for OpenSSL.
This is an optional dependency, that is needed only when *eprosima Fast DDS* is used with security and PKCS#11 URIs.

Install libp11_ using the package manager of the appropriate Linux distribution.
For example, on Ubuntu use the command:

.. code-block:: bash

   sudo apt install libp11-dev libengine-pkcs11-openssl

SoftHSM is a software implementation of an HSM (Hardware Security Module).
If *eProsima Fast DDS* tests are activated and *libp11* is installed
on the system, SoftHSM is additionally required to run tests of PKCS#11 features.

Install SoftHSM_ using the package manager of the appropriate Linux distribution.
For example, on Ubuntu use the command:

.. code-block:: bash

   sudo apt install softhsm2

Note that the *softhsm2* package creates a new group called *softhsm*.
In order to grant access to the HSM module a user must belong to this group.

.. code-block:: bash

   sudo usermod -a -G softhsm <user>

OpenSSL access HSM and other hardware devices through its engine functionality.
In order to set up a new engine the OpenSSL configuration files (usually `/etc/ssl/openssl.cnf`)
must be updated specifying the libp11_ and hardware module (here SoftHSM_) dynamic libraries
location.

This configuration step can be avoided using p11kit_ which allows OpenSSL to find PKCS#11
devices on runtime without static configuration.
This kit is often available through the Linux distribution package manager.
On Ubuntu, for example:

.. code-block:: bash

   sudo apt install libengine-pkcs11-openssl

Once installed, to check p11kit_ is able to find the SoftHSM_ module use:

.. code-block:: bash

   p11-kit list-modules

In order to check if OpenSSL is able to access PKCS#11 engine use:

.. code-block:: bash

    openssl engine pkcs11 -t

.. _gtest_sl:

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


.. _pythonreq_sl:

XML validation tool
^^^^^^^^^^^^^^^^^^^
XML validation is a new command introduced to validate the XML profiles against an XSD schema through Fast DDS CLI.
That ensures the proper characterization of the entities using the xml profiles.

For more details, please refer to the :ref:`cli_xml` section.

Install the xmlschema_ dependency to be able to use this optional tool.

.. _colcon_installation_linux:

Colcon installation
-------------------

colcon_ is a command line tool based on CMake_ aimed at building sets of software packages.
This section explains how to use it to compile *eProsima Fast DDS* and its dependencies.

#. Install the ROS 2 development tools (colcon_ and vcstool_) by executing the following command:

   .. code-block:: bash

       pip3 install -U colcon-common-extensions vcstool

   .. note::

       Mind that under non-root users, :code:`pip3` may install python :code:`colcon` and :code:`vcs` executables in
       :code:`$HOME/.local/bin`, for instance when running with :code:`--user`.
       To be able to run these applications, make sure that :code:`pip3` binary installation directory is in your
       :code:`$PATH` (:code:`$HOME/.local/bin` is normally introduced while login on an interactive non-root shell).

#. Create a :code:`Fast-DDS` directory and download the repos file that will be used to install
   *eProsima Fast DDS* and its dependencies:

   .. code-block:: bash

       mkdir ~/Fast-DDS
       cd ~/Fast-DDS
       wget https://raw.githubusercontent.com/eProsima/Fast-DDS/master/fastrtps.repos
       mkdir src
       vcs import src < fastrtps.repos

#. Build the packages:

   .. code-block:: bash

       colcon build

.. note::

    Being based on CMake_, it is possible to pass CMake configuration options to the :code:`colcon build` command.
    For more information on the specific syntax, please refer to the
    `CMake specific arguments <https://colcon.readthedocs.io/en/released/reference/verb/build.html#cmake-specific-arguments>`_
    page of the colcon_ manual.

.. _run_app_colcon_sl:

Run an application
^^^^^^^^^^^^^^^^^^

When running an instance of an application using *eProsima Fast DDS*, the colcon overlay built in the
dedicated :code:`Fast-DDS` directory must be sourced.
There are two possibilities:

* Every time a new shell is opened, prepare the environment locally by typing the
  command:

  .. code-block:: bash

      source ~/Fast-DDS/install/setup.bash

* Add the sourcing of the colcon overlay permanently to the :code:`PATH`, by typing the following:

  .. code-block:: bash

      echo 'source ~/Fast-DDS/install/setup.bash' >> ~/.bashrc


.. _cmake_installation_linux:

CMake installation
------------------

This section explains how to compile *eProsima Fast DDS* with CMake_, either :ref:`locally <local_installation_sl>` or
:ref:`globally <global_installation_sl>`.

.. _local_installation_sl:

Local installation
^^^^^^^^^^^^^^^^^^

#. Create a :code:`Fast-DDS` directory where to download and build *eProsima Fast DDS* and its dependencies:

   .. code-block:: bash

       mkdir ~/Fast-DDS

#. Clone the following dependencies and compile them using CMake_.

   * `Foonathan memory <https://github.com/foonathan/memory>`_

     .. code-block:: bash

         cd ~/Fast-DDS
         git clone https://github.com/eProsima/foonathan_memory_vendor.git
         mkdir foonathan_memory_vendor/build
         cd foonathan_memory_vendor/build
         cmake .. -DCMAKE_INSTALL_PREFIX=~/Fast-DDS/install -DBUILD_SHARED_LIBS=ON
         cmake --build . --target install

   * `Fast CDR <https://github.com/eProsima/Fast-CDR.git>`_

     .. code-block:: bash

         cd ~/Fast-DDS
         git clone https://github.com/eProsima/Fast-CDR.git
         mkdir Fast-CDR/build
         cd Fast-CDR/build
         cmake .. -DCMAKE_INSTALL_PREFIX=~/Fast-DDS/install
         cmake --build . --target install

#. Once all dependencies are installed, install *eProsima Fast DDS*:

   .. code-block:: bash

       cd ~/Fast-DDS
       git clone https://github.com/eProsima/Fast-DDS.git
       mkdir Fast-DDS/build
       cd Fast-DDS/build
       cmake ..  -DCMAKE_INSTALL_PREFIX=~/Fast-DDS/install
       cmake --build . --target install

.. note::

    By default, *eProsima Fast DDS* does not compile tests.
    However, they can be activated by downloading and installing `Gtest <https://github.com/google/googletest>`_.


.. _global_installation_sl:

Global installation
^^^^^^^^^^^^^^^^^^^

To install *eProsima Fast DDS* system-wide instead of locally, remove all the flags that
appear in the configuration steps of :code:`Fast-CDR` and :code:`Fast-DDS`, and change the first in the
configuration step of :code:`foonathan_memory_vendor` to the following:

.. code-block:: bash

    -DCMAKE_INSTALL_PREFIX=/usr/local/ -DBUILD_SHARED_LIBS=ON

.. note::

    Installation on system directories may need of permissions.
    Maybe permissions have to be granted through :code:`sudo`.

    .. code-block:: bash

        sudo cmake --build . --target install

.. _run_app_cmake_sl:

Run an application
^^^^^^^^^^^^^^^^^^

When running an instance of an application using *eProsima Fast DDS*, it must be linked with the library where the
packages have been installed, which in the case of system-wide installation  is: :code:`/usr/local/lib/` (if local
installation is used, adjust for the correct directory).
There are two possibilities:

* Prepare the environment locally by typing the command:

  .. code-block:: bash

      export LD_LIBRARY_PATH=/usr/local/lib/

* Add it permanently it to the :code:`PATH`, by typing:

  .. code-block:: bash

      echo 'export LD_LIBRARY_PATH=/usr/local/lib/' >> ~/.bashrc

.. _fastdds_python_sl:

Fast DDS Python bindings installation
"""""""""""""""""""""""""""""""""""""

This section provides the instructions for installing *Fast DDS Python bindings* in a Linux environment from sources.
*Fast DDS Python bindings* is an extension of *Fast DDS* which provides access to the Fast DDS API through Python.
Therefore, its installation is an extension of the installation of :ref:`Fast DDS <fastdds_lib_sl>`.

*Fast DDS Python bindings* source code consists on several `.i` files which will be processed by SWIG_.
Then C++ files (for connecting C++ and Python) and Python files (Python module for Fast DDS) will be generated.

First of all, the :ref:`requirements_python_sl` and :ref:`dependencies_python_sl` detailed below need to be met.
Afterwards, the user can choose whether to follow either the :ref:`colcon <colcon_installation_python_linux>` or the
:ref:`CMake <cmake_installation_python_linux>` installation instructions.

.. _requirements_python_sl:

Requirements
------------

The installation of *Fast DDS Python bindings* in a Linux environment from sources requires the following tools to be
installed in the system:

- :ref:`Fast DDS requirements <requirements_sl>`
- :ref:`swig_python_sl`
- :ref:`libpython_dev_sl`

.. _swig_python_sl:

.. include:: ../../04-common/python_requirements.rst
   :start-after: .. begin-swig
   :end-before: .. end-swig

.. _libpython_dev_sl:

.. include:: ../../04-common/python_requirements.rst
   :start-after: .. begin-libpython-dev
   :end-before: .. end-libpython-dev

.. _dependencies_python_sl:

Dependencies
------------

*Fast DDS Python bindings* has the following dependencies, when installed from sources in a Linux environment:

- :ref:`Fast DDS dependencies <dependencies_sl>`

.. _colcon_installation_python_linux:

Colcon installation
-------------------

colcon_ is a command line tool based on CMake_ aimed at building sets of software packages.
This section explains how to use it to compile *Fast DDS Python bindings* and its dependencies.

#. Install the ROS 2 development tools (colcon_ and vcstool_) by executing the following command:

   .. code-block:: bash

       pip3 install -U colcon-common-extensions vcstool

   .. note::

       If this fails due to an Environment Error, add the :code:`--user` flag to the :code:`pip3` installation command.

#. Create a :code:`Fast-DDS-python` directory and download the repos file that will be used to install
   *Fast DDS Python bindings* and its dependencies:

   .. code-block:: bash

       mkdir ~/Fast-DDS-python
       cd ~/Fast-DDS-python
       wget https://raw.githubusercontent.com/eProsima/Fast-DDS-python/main/fastdds_python.repos
       mkdir src
       vcs import src < fastdds_python.repos

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

      source ~/Fast-DDS-python/install/setup.bash

* Add the sourcing of the colcon overlay permanently to the :code:`PATH`, by typing the following:

  .. code-block:: bash

      echo 'source ~/Fast-DDS-python/install/setup.bash' >> ~/.bashrc

.. _cmake_installation_python_linux:

CMake installation
------------------

This section explains how to compile *Fast DDS Python bindings* with CMake_, either
:ref:`locally <local_installation_python_sl>` or :ref:`globally <global_installation_python_sl>`.

.. _local_installation_python_sl:

Local installation
^^^^^^^^^^^^^^^^^^

#. Create a :code:`Fast-DDS-python` directory where to download and build *Fast DDS Python bindings* and its
   dependencies:

   .. code-block:: bash

       mkdir ~/Fast-DDS-python

#. Clone the following dependencies and compile them using CMake_.

   * `Foonathan memory <https://github.com/foonathan/memory>`_

     .. code-block:: bash

         cd ~/Fast-DDS-python
         git clone https://github.com/eProsima/foonathan_memory_vendor.git
         mkdir foonathan_memory_vendor/build
         cd foonathan_memory_vendor/build
         cmake .. -DCMAKE_INSTALL_PREFIX=~/Fast-DDS-python/install -DBUILD_SHARED_LIBS=ON
         cmake --build . --target install

   * `Fast CDR <https://github.com/eProsima/Fast-CDR.git>`_

     .. code-block:: bash

         cd ~/Fast-DDS-python
         git clone https://github.com/eProsima/Fast-CDR.git
         mkdir Fast-CDR/build
         cd Fast-CDR/build
         cmake .. -DCMAKE_INSTALL_PREFIX=~/Fast-DDS-python/install
         cmake --build . --target install

   * `Fast DDS <https://github.com/eProsima/Fast-DDS.git>`_

     .. code-block:: bash

         cd ~/Fast-DDS-python
         git clone https://github.com/eProsima/Fast-DDS.git
         mkdir Fast-DDS/build
         cd Fast-DDS/build
         cmake .. -DCMAKE_INSTALL_PREFIX=~/Fast-DDS-python/install
         cmake --build . --target install

#. Once all dependencies are installed, install *Fast DDS Python bindings*:

   .. code-block:: bash

       cd ~/Fast-DDS-python
       git clone https://github.com/eProsima/Fast-DDS-python.git
       mkdir -p Fast-DDS-python/fastdds_python/build
       cd Fast-DDS-python/fastdds_python/build
       cmake ..  -DCMAKE_INSTALL_PREFIX=~/Fast-DDS-python/install
       cmake --build . --target install

.. _global_installation_python_sl:

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
the packages have been installed, which in the case of system-wide installation is: :code:`/usr/local/lib/` (if local
installation is used, adjust for the correct directory).
There are two possibilities:

* Prepare the environment locally by typing the command:

  .. code-block:: bash

      export LD_LIBRARY_PATH=/usr/local/lib/

* Add it permanently it to the :code:`PATH`, by typing:

  .. code-block:: bash

      echo 'export LD_LIBRARY_PATH=/usr/local/lib/' >> ~/.bashrc

.. _fastddsgen_sl:

Fast DDS-Gen installation
"""""""""""""""""""""""""

This section provides the instructions for installing *Fast DDS-Gen* in a Linux environment from
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

.. _java_sl:

Java JDK
^^^^^^^^

The JDK is a development environment for building applications and components using the Java language.
There are several versions of Java available.
For instance, to install Java 11 JDK, run the following command:

.. code-block:: bash

    sudo apt install openjdk-11-jdk

.. note::

    *Fast DDS-Gen* supports Java versions from 11 to 19.

Compiling Fast DDS-Gen
----------------------

In order to compile *Fast DDS-Gen*, an executable script is included in the repository which will download Gradle
temporarily for the compilation step.
Please, follow the steps below to build *Fast DDS-Gen*:

.. note::

    If Fast DDS has already been installed following :ref:`colcon_installation_linux`, skip cloning *Fast DDS-Gen*'s
    repository, as it can already be found under the :code:`src` directory within the colcon workspace.

.. code-block:: bash

    cd ~
    git clone --recursive https://github.com/eProsima/Fast-DDS-Gen.git
    cd Fast-DDS-Gen
    ./gradlew assemble

.. note::

    In case that a supported Gradle version is already installed in the system, *Fast DDS-Gen* can also be built running
    directly:

    .. code-block:: bash

        gradle assemble

Contents
^^^^^^^^

The :code:`Fast-DDS-Gen` folder contains the following packages:

* :code:`share/fastddsgen`, where the generated Java application is.
* :code:`scripts`, containing some user friendly scripts.

  .. note::

      To make these scripts accessible from any shell session and directory, add the :code:`scripts` folder path to the
      :code:`PATH` environment variable.

.. External links

.. _colcon: https://colcon.readthedocs.io/en/released/
.. _CMake: https://cmake.org
.. _pip3: https://docs.python.org/3/installing/index.html
.. _wget: https://www.gnu.org/software/wget/
.. _git: https://git-scm.com/
.. _OpenSSL: https://www.openssl.org/
.. _Gtest: https://github.com/google/googletest
.. _vcstool: https://pypi.org/project/vcstool/
.. _libp11: https://github.com/OpenSC/libp11/
.. _SoftHSM: https://www.opendnssec.org/softhsm/
.. _p11kit: https://github.com/p11-glue/p11-kit
.. _SWIG: http://www.swig.org/
.. _xmlschema: https://pypi.org/project/xmlschema/
