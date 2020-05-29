.. _linux_sources:

Linux installation from sources
===============================

To install *eProsima Fast DDS* from sources, you first need to meet the required dependencies
(see :ref:`requirements_linux_sources`)
and then choose whether to follow either the colcon_ (see colcon :ref:`colcon_installation_linux`) or the CMake_
(see :ref:`cmake_installation_linux`) installation instructions.

.. _requirements_linux_sources:

Requirements
------------

*eProsima Fast DDS* requires the following dependencies when building from sources in a Linux environment.

Asio and TinyXML2 libraries
^^^^^^^^^^^^^^^^^^^^^^^^^^^

You can install these libraries using the package manager of your Linux distribution.
For example, on Ubuntu you can install them with the command:

.. code-block:: bash

    sudo apt install libasio-dev libtinyxml2-dev

Python
^^^^^^

Download and install the Python_ version that better fits your requirements.

Cmake, g++, pip, wget and git
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

You can install Cmake_, g++, pip_, wget_ and git_ using the package manager of your Linux distribution.
For example, on Ubuntu you can install them with the command:

.. code-block:: bash

    sudo apt install cmake g++ pip wget git

OpenSSL
^^^^^^^

You can install OpenSSL_ using the package manager of your Linux distribution.
For example, on Ubuntu you can install it with the command:

.. code-block:: bash

   sudo apt install libssl-dev

Gtest
^^^^^

.. note::

    By default, *eProsima Fast DDS* doesn’t compile tests.
    You can activate them by adding the :code:`-DPERFORMANCE_TESTS=ON` flag when calling colcon_ or CMake_
    (for details, see below).

You can find information on how to install Gtest at this `link <https://github.com/google/googletest>`_.

.. _colcon_installation_linux:

Colcon Installation
-------------------

colcon_ is a command line tool to build sets of software packages.
This section explains how to use it to compile easily *eProsima Fast DDS* and its dependencies.
First, install the ROS2 development tools (colcon_ and vcstool_):

.. code-block:: bash

    pip install -U colcon-common-extensions vcstool

.. note::

    If this fails due to an Environment Error, add the :code:`--user` flag to your installation.

Now, create a colcon_ workspace and download the repos file that will be used to install *eProsima Fast DDS* and
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

.. important::

    The sourcing of the local colcon overlay is required every time the colcon workspace is opened in a new shell
    environment to run an *eProsima Fast DDS* instance.
    As an alternative, you can add it permanently to you path by typing the following:

    .. code-block:: bash

        echo 'source PATH_TO_WORKSPACE/Fast-DDS-ws/install/setup.bash' >> ~/.bashrc

    Where :code:`PATH_TO_WORKSPACE` is the path to the :code:`Fast-DDS-ws` worskspace.

.. note::

    If you want to compile the examples, you will need to add the flag
    :code:`--cmake-args "-DCOMPILE_EXAMPLES=ON"` when running :code:`colcon build`.
    If you want to compile the performance tests, you will need to add the flag
    :code:`--cmake-args "--DPERFORMANCE_TESTS=ON"` when running :code:`colcon build`.
    For this step, you need Gtest_ as explained in the :ref:`requirements_linux_sources` section above.


.. _cmake_installation_linux:

CMake Installation
------------------

This section explains how to compile *eProsima Fast DDS* locally with CMake_.
First of all, create a Fast-DDS directory where to download and build *eProsima Fast DDS* and its dependencies:

.. code-block:: bash

    mkdir Fast-DDS && cd Fast-DDS

Now clone the following dependencies and compile them using CMake_.

* `Foonathan memory <https://github.com/foonathan/memory>`_

  .. code-block:: bash

      $ git clone https://github.com/eProsima/foonathan_memory_vendor.git
      $ mkdir foonathan_memory_vendor/build && cd foonathan_memory_vendor/build
      $ cmake .. -DCMAKE_INSTALL_PREFIX=../../install
      $ sudo cmake --build . --target install
      $ cd ../..

* `Fast CDR <https://github.com/eProsima/Fast-CDR.git>`_

  .. code-block:: bash

      $ git clone https://github.com/eProsima/Fast-CDR.git
      $ mkdir Fast-CDR/build && cd Fast-CDR/build
      $ cmake .. -DCMAKE_INSTALL_PREFIX=../../install
      $ sudo cmake --build . --target install
      $ cd ../..

Once all dependencies are installed, you will be able to compile and install *eProsima Fast DDS*:

.. code-block:: bash

    $ git clone https://github.com/eProsima/Fast-RTPS.git
    $ mkdir Fast-RTPS/build && cd Fast-RTPS/build
    $ sudo cmake ..  -DCMAKE_INSTALL_PREFIX=../../install -DCMAKE_PREFIX_PATH=../../install
    $ cmake --build . --target install



If you want to install *eProsima Fast DDS* system-wide instead of locally, you need to remove all the flags that
appear in the configuration steps of :code:`Fast-CDR` and :code:`Fast-RTPS`, and change the one in the
configuration step of :code:`foonathan_memory_vendor` to the following:

.. code-block:: bash

    -DCMAKE_INSTALL_PREFIX=/usr/local/ -DBUILD_SHARED_LIBS=ON

.. note::

    If you want to compile the examples, you will need to add the argument :code:`-DCOMPILE_EXAMPLES=ON` when calling
    the configuration CMake_.
    If you want to compile the performance tests, you will need to add the argument
    :code:`-DPERFORMANCE_TESTS=ON` when calling the configuration CMake_.
    For this step, you need Gtest_ as explained in the :ref:`requirements_linux_sources` section above..

.. important::

    When running an *eProsima Fast DDS* application, you need to link it with the library :code:`/usr/local/lib/`
    where the pakages have been installed. You can either prepare the environment locally by typing the command:

    .. code-block:: bash

        export LD_LIBRARY_PATH=/usr/local/lib/

    in the console you use to run the *eProsima Fast DDS* instance, or permanently add it to your path, by typing:

    .. code-block:: bash

        echo 'export LD_LIBRARY_PATH=/usr/local/lib/' >> ~/.bashrc

.. External links

.. _colcon: https://colcon.readthedocs.io/en/released/
.. _Python: https://www.python.org/
.. _CMake: https://cmake.org
.. _pip: https://pypi.org/project/pip/
.. _wget: https://www.gnu.org/software/wget/
.. _git: https://git-scm.com/
.. _OpenSSL: https://www.openssl.org/
.. _Gtest: https://github.com/google/googletest
.. _vcstool: https://pypi.org/project/vcstool/
