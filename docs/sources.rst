.. _installation-from-sources:

Installation from Sources
#########################

Dependencies
************

Asio and TinyXML2 libraries
===========================

On Linux, you can install these libraries using the package manager of your Linux distribution.
For example, on Ubuntu you can install them by using its package manager with the next command.

.. code-block:: bash

   sudo apt install libasio-dev libtinyxml2-dev

On Windows, you can install these libraries using Chocolatey_.
First, download the following chocolatey packages from this
`ROS2 Github repository <https://github.com/ros2/choco-packages/releases/latest>`_.

* asio.1.12.1.nupkg
* tinyxml2.6.0.0.nupkg

Once these packages are downloaded, open an administrative shell and execute the following command:

.. code-block:: batch

   choco install -y -s <PATH\TO\DOWNLOADS\> asio tinyxml2

Please replace ``<PATH\TO\DOWNLOADS>`` with the folder you downloaded the packages to.

Colcon installation
*******************

colcon_ is a command line tool to build sets of software packages.
This section explains to use it to compile easily Fast-RTPS and its dependencies.
First install ROS2 development tools (colcon_ and vcstool):

.. code-block:: bash

    pip install -U colcon-common-extensions vcstool

Download the repos file that will be used to download Fast RTPS and its dependencies:

.. code-block:: bash

    $ wget https://raw.githubusercontent.com/eProsima/Fast-RTPS/master/fastrtps.repos
    $ mkdir src
    $ vcs import src < fastrtps.repos

Finally, use colcon_ to compile all software:

.. code-block:: bash

    $ colcon build

Manual installation
*******************

Before compiling manually Fast RTPS you need to clone the following dependencies and compile them using CMake_.

* `Fast CDR <https://github.com/eProsima/Fast-CDR.git>`_

    .. code-block:: bash

        $ git clone https://github.com/eProsima/Fast-CDR.git
        $ mkdir Fast-CDR/build && cd Fast-CDR/build
        $ cmake ..
        $ cmake --build . --target install

* `Foonathan memory <https://github.com/foonathan/memory>`_

    .. code-block:: bash

        $ git clone https://github.com/eProsima/foonathan_memory_vendor.git
        $ cd foonathan_memory_vendor
        $ mkdir build && cd build
        $ cmake ..
        $ cmake --build . --target install

Once all dependencies are installed, you will be able to compile and install Fast RTPS.

.. code-block:: bash

   $ git clone https://github.com/eProsima/Fast-RTPS.git
   $ mkdir Fast-RTPS/build && cd Fast-RTPS/build
   $ cmake ..
   $ cmake --build . --target install

If you want to compile *fastrtpsgen* java application, you will need to add the argument ``-DBUILD_JAVA=ON`` when
calling CMake (see :ref:`compile-fastrtpsgen`).

If you want to compile the examples, you will need to add the argument ``-DCOMPILE_EXAMPLES=ON`` when calling CMake.

If you want to compile the performance tests, you will need to add the argument ``-DPERFORMANCE_TESTS=ON`` when calling
CMake.

For generate *fastrtpsgen* please see :ref:`compile-fastrtpsgen`.

Security
********

By default, Fast RTPS doesn't compile security support.
You can activate it adding ``-DSECURITY=ON`` at CMake configuration step.
More information about security on Fast RTPS, see :ref:`security`.

When security is activated on compilation Fast RTPS builds several built-in security plug-ins.
Some of them have the dependency of OpenSSL library.

OpenSSL installation on Linux
=============================

Surely you can install OpenSSL using the package manager of your Linux distribution.
For example, on Ubuntu you can install OpenSSL using its package manager with next command.

.. code-block:: bash

   sudo apt install libssl-dev

OpenSSL installation on Windows
===============================

.. _webpage: https://slproweb.com/products/Win32OpenSSL.html

You can download OpenSSL 1.0.2 for Windows in this webpage_.
This is the OpenSSL version tested by our team.
Download and use the installer that fits your requirements.
After installing, add the environment variable ``OPENSSL_ROOT_DIR`` pointing to the installation root directory.
For example:

.. code-block:: bash

   OPENSSL_ROOT_DIR=C:\OpenSSL-Win64



.. External links
.. _Chocolatey: https://chocolatey.org
.. _CMake: https://cmake.org
.. _colcon: https://colcon.readthedocs.io
