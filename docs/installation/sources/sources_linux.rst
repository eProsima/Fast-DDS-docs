.. _linux_sources:

Linux installation from sources
===============================

To install *eProsima Fast DDS* from sources, you first need to meet the required dependencies (see Requirements - REF)
and then choose whether to follow either the colcon_ (see colcon - REF) or the CMake_ (see cmake -REF)
installation instructions.

Requirements
------------

*eProsima Fast RTPS* requires the following dependencies when building from sources in a Linux environment.

Asio and TinyXML2 libraries
^^^^^^^^^^^^^^^^^^^^^^^^^^^

You can install these libraries using the package manager of your Linux distribution.
For example, on Ubuntu you can install them with the command:

.. code-block:: bash

    sudo apt install libasio-dev libtinyxml2-dev

Cmake and g++
^^^^^^^^^^^^^

You can install Cmake_ and g++ using the package manager of your Linux distribution.
For example, on Ubuntu you can install them with the command:

.. code-block:: bash

    sudo apt install cmake g++

In addition, you may need to set :code:`LD_LIBRARY_PATH`:

.. code-block:: bash

    export LD_LIBRARY_PATH=/usr/local/lib/

OpenSSL
^^^^^^^

You can install OpenSSL using the package manager of your Linux distribution.
For example, on Ubuntu you can install it with the command:

.. code-block:: bash

   sudo apt install libssl-dev

Gtest
^^^^^

.. note::

    By default, *eProsima Fast DDS* doesn’t compile tests.
    You can activate them by adding the :code:`-DPERFORMANCE_TESTS=ON` flag when calling colcon_ or CMake_
    (for details, see below - REF).

You can find information on how to install Gtest at this `link <https://github.com/google/googletest>`_.

Installation with Colcon
------------------------

colcon_ is a command line tool to build sets of software packages.
This section explains how to use it to compile easily *eProsima Fast DDS* and its dependencies.

.. note::

    In order to run the commands below, you need to have the following packages installed in your Linux system:

    * python
    * pip
    * wget
    * vcs

You can then install the ROS2 development tools (colcon_ and vcstool):

.. code-block:: bash

    pip install -U colcon-common-extensions vcstool

.. note::

    If this fails due to an Environment Error, add the :code:`--user` flag to your installation.

Now, create a colcon_ workspace, and then download the repos file that will be used to install *eProsima Fast DDS* and
its dependencies:

.. code-block:: bash

    $ mkdir Fast-DDS-ws && cd Fast-DDS-ws
    $ wget https://raw.githubusercontent.com/eProsima/Fast-RTPS/master/fastrtps.repos
    $ mkdir src
    $ vcs import src < fastrtps.repos

Finally, use colcon_ to compile all software:

.. code-block:: bash

    colcon build

Once that’s finished building, you can source the new colcon overlay:

.. code-block:: bash

    source install/setup.bash

.. note::

    The sourcing of the local colcon overlay is required every time the colcon workspace is opened in a new shell
    environment to run an *eProsima Fast DDS* instance.
    As an alternative, you can add it permanently to you path by typing the following:

    .. code-block:: bash

        echo 'source PATH_TO_WORKSPACE/Fast-DDS-ws/install/setup.bash' >> ~/.bashrc

    Where :code:`PATH_TO_WORKSPACE` is the path to the :code:`Fast-DDS-ws` worskspace.

If you want to compile the examples, you will need to add the flag
:code:`--cmake-args "-DCOMPILE_EXAMPLES=ON"` when running :code:`colcon build`.
If you want to compile the performance tests, you will need to add the flag
:code:`--cmake-args "--DPERFORMANCE_TESTS=ON"` when running :code:`colcon build`.
For this step, you need Gtest_ as explained in the Requirements section above (REF).


Manual Installation
-------------------

First of all, create a Fast-DDS directory where to download and build *eProsima Fast DDS* and its dependencies:

.. code-block:: bash

    mkdir Fast-DDS && cd Fast-DDS

Now, before compiling *eProsima Fast DDS*, you need to clone the following dependencies and compile them using CMake.

* `Fast CDR <https://github.com/eProsima/Fast-CDR.git>`_

  .. code-block:: bash

      $ git clone https://github.com/eProsima/Fast-CDR.git
      $ mkdir Fast-CDR/build && cd Fast-CDR/build
      $ cmake -DCMAKE_INSTALL_PREFIX=install ..
      $ cmake --build . --target install
      $ cd ../..

* `Foonathan memory <https://github.com/foonathan/memory>`_

  .. code-block:: bash

      $ git clone https://github.com/eProsima/foonathan_memory_vendor.git
      $ mkdir foonathan_memory_vendor/build && cd foonathan_memory_vendor/build
      $ cmake -DCMAKE_INSTALL_PREFIX=install ..
      $ cmake --build . --target install
      $ cd ../..

Once all dependencies are installed, you will be able to compile and install *eProsima Fast DDS*:

.. code-block:: bash

    $ git clone https://github.com/eProsima/Fast-RTPS.git
    $ mkdir Fast-RTPS/build && cd Fast-RTPS/build
    $ cmake -DCMAKE_INSTALL_PREFIX=install ..
    $ cmake --build . --target install

If you want to compile the examples, you will need to add the argument :code:`-DCOMPILE_EXAMPLES=ON` when calling
the configuration CMake_.
If you want to compile the performance tests, you will need to add the argument
:code:`-DPERFORMANCE_TESTS=ON` when calling the configuration CMake_.
For this step, you need Gtest_ as explained in the Requirements section above (REF).

.. External links

.. _colcon: https://colcon.readthedocs.io/en/released/
.. _CMake: https://cmake.org
.. _Gtest: https://github.com/google/googletest
