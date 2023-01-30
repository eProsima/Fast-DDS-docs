.. _mac_sources:

Mac OS installation from sources
================================

The instructions for installing both the :ref:`Fast DDS library <fastdds_lib_sm>`
and the :ref:`Fast DDS-Gen <fastddsgen_sm>` generation tool from sources are provided in this page.
It is organized as follows:

.. contents::
    :local:
    :backlinks: none
    :depth: 2

.. _fastdds_lib_sm:

Fast DDS library installation
"""""""""""""""""""""""""""""

This section describes the instructions for installing *eProsima Fast DDS* in a Mac OS environment from
sources. The following packages will be installed:

* :code:`foonathan_memory_vendor`, an STL compatible C++ memory allocator
  `library <https://github.com/foonathan/memory>`_.
* :code:`fastcdr`, a C++ library that serializes according to the
  `standard CDR <https://www.omg.org/cgi-bin/doc?formal/02-06-51>`_ serialization mechanism.
* :code:`fastrtps`, the core library of *eProsima Fast DDS* library.

First of all, the :ref:`requirements_sm` and :ref:`dependencies_sm` detailed below need to be met.
Afterwards, the user can choose whether to follow either the :ref:`colcon <colcon_installation_mac>`)
or the :ref:`CMake <cmake_installation_mac>`) installation instructions.

.. _requirements_sm:


Requirements
------------

The installation of *eProsima Fast DDS* in a MacOS environment from sources requires the following tools to be
installed in the system:

* :ref:`homebrew_sm`
* :ref:`xcode_cli_sm`
* :ref:`cmake_gcc_pip3_wget_git_sm`
* :ref:`gtest_sm` [optional]
* :ref:`pythonreq_sm` [optional]

.. _homebrew_sm:

Homebrew
^^^^^^^^
Homebrew is a macOS package manager, it is needed to install some of *eProsima Fast DDS*'s dependencies.
To install it open a terminal window and run the following command.

.. code-block:: bash

        /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install.sh)"

.. _xcode_cli_sm:

Xcode Command Line Tools
^^^^^^^^^^^^^^^^^^^^^^^^
The Xcode command line tools package is separate from Xcode and allows for command line development in mac.
The previous step should have installed Xcode CLI, to check the correct installation run the following command:

.. code-block::

        gcc --version

.. _cmake_gcc_pip3_wget_git_sm:

CMake, g++, pip3, wget and git
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

These packages provide the tools required to install *eProsima Fast DDS* and its dependencies from command line.
Install CMake_, pip3_ and wget_ using the Homebrew package manager:

.. code-block:: bash

        brew install cmake python3 wget

.. _gtest_sm:

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


.. _pythonreq_sm:

XML validation tool
^^^^^^^^^^^^^^^^^^^
XML validation is a new command introduced to validate the XML profiles against an XSD schema.
That ensures the proper characterization of the entities through the xml profiles.

For more details, please refer to the :ref:`cli_xml` section.

.. code-block:: bash

    pip3 install xmlschema


.. _dependencies_sm:

Dependencies
------------

*eProsima Fast DDS* has the following dependencies, when installed from binaries in a Linux environment:

* :ref:`asiotinyxml2_sm`
* :ref:`openssl_sm`

.. _asiotinyxml2_sm:

Asio and TinyXML2 libraries
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Asio is a cross-platform C++ library for network and low-level I/O programming, which provides a consistent
asynchronous model.
TinyXML2 is a simple, small and efficient C++ XML parser.
Install these libraries using Homebrew:

.. code-block:: bash

        brew install asio tinyxml2

.. _openssl_sm:

OpenSSL
^^^^^^^

OpenSSL is a robust toolkit for the TLS and SSL protocols and a general-purpose cryptography library.
Install OpenSSL_ using Homebrew:

.. code-block:: bash

        brew install openssl@1.1


.. _colcon_installation_mac:

Colcon installation
-------------------

colcon_ is a command line tool based on CMake_ aimed at building sets of software packages.
This section explains how to use it to compile *eProsima Fast DDS* and its dependencies.

#. Install the ROS 2 development tools (colcon_ and vcstool_) by executing the following command:

   .. code-block:: bash

       pip3 install -U colcon-common-extensions vcstool


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

    The :code:`--cmake-args` option allows to pass the CMake configuration options to the :code:`colcon build` command.
    In Mac OS the location of OpenSSL is not found automatically and therefore has to be passed explicitly:
    :code:`--cmake-args -DOPENSSL_ROOT_DIR=/usr/local/opt/openssl -DOPENSSL_LIBRARIES=/usr/local/opt/openssl/lib`.
    This is only required when building with :ref:`security`.
    For more information on the specific syntax, please refer to the `CMake specific arguments
    <https://colcon.readthedocs.io/en/released/reference/verb/build.html#cmake-specific-arguments>`_ page of the colcon_
    manual.

.. _run_app_colcon_sm:

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

      touch ~/.bash_profile
      echo 'source ~/Fast-DDS/install/setup.bash' >> ~/.bash_profile


.. _cmake_installation_mac:

CMake installation
------------------

This section explains how to compile *eProsima Fast DDS* with CMake_, either :ref:`locally <local_installation_sm>` or
:ref:`globally <global_installation_sm>`.

.. _local_installation_sm:

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
         sudo cmake --build . --target install

   * `Fast CDR <https://github.com/eProsima/Fast-CDR.git>`_

     .. code-block:: bash

         cd ~/Fast-DDS
         git clone https://github.com/eProsima/Fast-CDR.git
         mkdir Fast-CDR/build
         cd Fast-CDR/build
         cmake ..  -DCMAKE_INSTALL_PREFIX=~/Fast-DDS/install
         sudo cmake --build . --target install

#. Once all dependencies are installed, install *eProsima Fast DDS*:

   .. code-block:: bash

       cd ~/Fast-DDS
       git clone https://github.com/eProsima/Fast-DDS.git
       mkdir Fast-DDS/build
       cd Fast-DDS/build
       cmake ..  -DCMAKE_INSTALL_PREFIX=~/Fast-DDS/install -DCMAKE_PREFIX_PATH=~/Fast-DDS/install
       sudo cmake --build . --target install

.. note::

    By default, *eProsima Fast DDS* does not compile tests.
    However, they can be activated by downloading and installing `Gtest <https://github.com/google/googletest>`_.


.. _global_installation_sm:

Global installation
^^^^^^^^^^^^^^^^^^^

To install *eProsima Fast DDS* system-wide instead of locally, remove all the flags that
appear in the configuration steps of :code:`Fast-CDR` and :code:`Fast-DDS`, and change the first in the
configuration step of :code:`foonathan_memory_vendor` to the following:

.. code-block:: bash

    -DCMAKE_INSTALL_PREFIX=/usr/local/ -DBUILD_SHARED_LIBS=ON


.. _run_app_cmake_sm:

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

      touch ~/.bash_profile
      echo 'export LD_LIBRARY_PATH=/usr/local/lib/' >> ~/.bash_profile


.. _fastddsgen_sm:

Fast DDS-Gen installation
"""""""""""""""""""""""""

This section provides the instructions for installing *Fast DDS-Gen* in a Mac OS environment from
sources.
*Fast DDS-Gen* is a Java application that generates source code using the data types defined in an IDL file.
Please refer to :ref:`fastddsgen_intro` for more information.

Requirements
------------

In order to compile *Fast DDS-Gen*, the following packages need to be installed in the system:

* :ref:`java_sm`
* :ref:`gradle_sm`

.. _java_sm:

Java JDK
^^^^^^^^

The JDK is a development environment for building applications and components using the Java language.
Download and install it at the following the steps given in the
`Oracle website <https://www.oracle.com/java/technologies/javase-downloads.html>`_.

.. _gradle_sm:

Gradle
^^^^^^

Gradle is an open-source build automation tool.
Download and install the last stable version of `Gradle <https://gradle.org/install>`_ in the preferred way.
with Homebrew it would be running the command:

.. code-block:: bash

        brew install gradle

.. note::

    If errors occur during compilation or you do not wish to install gradle, an executable script is included which will
    download gradle temporarily for the compilation step.

    .. code-block:: bash

        ./gradlew assemble

Compiling Fast DDS-Gen
----------------------

Once the requirements above are met, compile *Fast DDS-Gen* by following the steps below:

.. code-block:: bash

    cd ~
    git clone --recursive https://github.com/eProsima/Fast-DDS-Gen.git
    cd Fast-DDS-Gen
    gradle assemble

Contents
^^^^^^^^

The :code:`Fast-DDS-Gen` folder contains the following packages:

* :code:`share/fastddsgen`, where the generated Java application is.
* :code:`scripts`, containing some user friendly scripts.

  .. note::

      To make these scripts accessible from any shell session and directory, add the :code:`scripts` folder path to the
      :code:`PATH` environment variable using the method described above.

.. External links

.. _colcon: https://colcon.readthedocs.io/en/released/
.. _CMake: https://cmake.org
.. _pip3: https://docs.python.org/3/installing/index.html
.. _wget: https://www.gnu.org/software/wget/
.. _git: https://git-scm.com/
.. _OpenSSL: https://www.openssl.org/
.. _Gtest: https://github.com/google/googletest
.. _vcstool: https://pypi.org/project/vcstool/
