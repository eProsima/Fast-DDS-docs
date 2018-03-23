.. _installation-from-sources:

Installation from Sources
=========================

Clone the project from Github: ::

    $ git clone https://github.com/eProsima/Fast-RTPS
    $ mkdir Fast-RTPS/build && cd Fast-RTPS/build

If you are on Linux, execute: ::

    $ cmake -DTHIRDPARTY=ON ..
    $ make
    $ sudo make install

If you are on Windows, choose your version of Visual Studio using CMake option *-G*: ::

    > cmake -G "Visual Studio 14 2015 Win64" -DTHIRDPARTY=ON ..
    > cmake --build . --target install

If you want to compile *fastrtpsgen* java application, you will need to add the argument ``-DBUILD_JAVA=ON`` when
calling CMake (see :ref:`compile-fastrtpsgen`).

If you want to compile the examples, you will need to add the argument ``-DCOMPILE_EXAMPLES=ON`` when calling CMake.

If you want to compile the performance tests, you will need to add the argument ``-DPERFORMANCE_TESTS=ON`` when calling CMake.

For generate *fastrtpsgen* please see :ref:`compile-fastrtpsgen`.

Security
--------

By default Fast RTPS doesn't compile security support. You can activate it adding ``-DSECURITY=ON`` at CMake
configuration step. More information about security on Fast RTPS, see :ref:`security`.

When security is activated on compilation Fast RTPS builds two built-in security plugins. Both have the dependency of
OpenSSL library.

OpenSSL installation on Linux
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Surely you can install OpenSSL using the package manager of your Linux distribution. For example on Fedora you can
install OpenSSL using its package manager with next command.

.. code-block:: bash

   sudo yum install openssl-devel

OpenSSL installation on Windows
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. _webpage: https://slproweb.com/products/Win32OpenSSL.html

You can download OpenSSL 1.0.2 for Windows in this webpage_. This is the OpenSSL version tested by our team. Download the
installer that fits your requirements and install it. After installing, add the environment variable
``OPENSSL_ROOT_DIR`` pointing to the installation root directory. For example:

.. code-block:: bash

   OPENSSL_ROOT_DIR=C:\OpenSSL-Win64
