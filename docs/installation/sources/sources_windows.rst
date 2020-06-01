.. _windows_sources:

Windows installation from sources
=================================

To install *eProsima Fast DDS* from sources, you first need to meet the required dependencies
(see requirements_windows_sources_)
and then choose whether to follow either the colcon_ (see colcon colcon_installation_windows_) or the CMake_
(see cmake_installation_windows_) installation instructions.

.. _requirements_windows_sources:

Requirements
------------

*eProsima Fast RTPS* requires the following dependencies when building from sources in a Windows environment.

Visual Studio
^^^^^^^^^^^^^

To install *eProsima Fast DDS*, you need to have `Visual Studio <https://visualstudio.microsoft.com/>`_ installed in
your system. Make sure you check the :code:`Desktop development with C++` option during the installation process.
If you have Visual Studio but you haven’t the Visual C++ Redistributable packages installed,
open Visual Studio and go to :code:`Tools` -> :code:`Get Tools and Features` and in the :code:`Workloads` tab enable
:code:`Desktop development with C++`. Click :code:`Modify` at the bottom right when you're done.

Chocolatey
^^^^^^^^^^

In order to install some of *eProsima Fast DDS*'s dependencies, you need to have the Windows package
manager Chocolatey_ installed in your system.

Asio and TinyXML2 libraries
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Once you have Chocolatey installed, download the following Chocolatey packages from this
`ROS2 Github repository <https://github.com/ros2/choco-packages/releases/tag/2020-02-24>`_.

* asio.1.12.1.nupkg
* tinyxml2.6.0.0.nupkg

After downloading these packages, open an administrative shell with *PowerShell* and execute the following command:

.. code-block:: bash

    choco install -y -s <PATH_TO_DOWNLOADS> asio tinyxml2

where :code:`<PATH_TO_DOWNLOADS>` is the folder you downloaded the packages into.

Python
^^^^^^

Download and install the Python_ version that better fits your requirements.

CMake, pip, wget and git
^^^^^^^^^^^^^^^^^^^^^^^^

You can download and install CMake_, pip_, wget_ and git_ by following the instructions on the respective
websites.
Once installed, you need to add the path to these executables to your :code:`PATH` from the
*Edit the system environment variables* control panel.

OpenSSL
^^^^^^^

You can download the latest OpenSSL version for Windows at this webpage_.
Download and use the installer that fits your requirements.
After installing, add the environment variable :code:`OPENSSL_ROOT_DIR` pointing to the installation root directory.

For example:

.. code-block:: bash

   OPENSSL_ROOT_DIR=C:\Program Files\OpenSSL-Win64

Gtest
^^^^^

.. note::

    By default, *eProsima Fast DDS* does not compile tests.
    You can activate them by adding the :code:`-DPERFORMANCE_TESTS=ON` flag when calling colcon_ or CMake_
    (for details, see below).

You can find information on how to install Gtest at this `link <https://github.com/google/googletest>`_.

.. _colcon_installation_windows:

Colcon Installation
-------------------

colcon_ is a command line tool to build sets of software packages.
This section explains how to use it to compile easily *eProsima Fast DDS* and its dependencies.

.. important::

    You need to run colcon within a Visual Studio prompt. To do so, launch a *Developer Command Prompt* from the
    search engine.

You can then install the ROS2 development tools (colcon_ and vcstool_):

.. code-block:: bash

    pip install -U colcon-common-extensions vcstool

and add the path to the :code:`vcs` executable to your :code:`PATH` from the
*Edit the system environment variables* control panel.

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

To run an *eProsima Fast DDS* instance, you need to source the colcon overlay with the command:

.. code-block:: bash

    setup.bat

.. important::

    The sourcing of the local colcon overlay is required every time the colcon workspace is opened in a new shell
    environment to run an *eProsima Fast DDS* instance.
    As an alternative, you can add it permanently to your :code:`PATH`.

.. note::

    If you want to compile the examples, you will need to add the flag
    :code:`--cmake-args "-DCOMPILE_EXAMPLES=ON"` when running :code:`colcon build`.
    If you want to compile the performance tests, you will need to add the flag
    :code:`--cmake-args "--DPERFORMANCE_TESTS=ON"` when running :code:`colcon build`.
    For this step, you need Gtest_ as explained in the requirements_windows_sources_ section above.


.. _cmake_installation_windows:

CMake Installation
-------------------

This section explains how to compile *eProsima Fast DDS* locally with CMake_.
Open a command prompt, and create a :code:`Fast-DDS` directory where to download and build *eProsima Fast DDS* and
its dependencies:

.. code-block:: bash

    mkdir Fast-DDS && cd Fast-DDS

Now clone the following dependencies and compile them using CMake_.

* `Foonathan memory <https://github.com/foonathan/memory>`_

  .. code-block:: bash

      $ git clone https://github.com/eProsima/foonathan_memory_vendor.git
      $ cd foonathan_memory_vendor
      $ mkdir build && cd build
      $ cmake .. -DCMAKE_INSTALL_PREFIX=../../install
      $ cmake --build . --target install
      $ cd ../..

* `Fast CDR <https://github.com/eProsima/Fast-CDR.git>`_

  .. code-block:: bash

      $ git clone https://github.com/eProsima/Fast-CDR.git
      $ cd Fast-CDR
      $ mkdir build && cd build
      $ cmake .. -DCMAKE_INSTALL_PREFIX=../../install
      $ cmake --build . --target install
      $ cd ../..

Once all dependencies are installed, you will be able to compile and install *eProsima Fast DDS*:

.. code-block:: bash

    $ git clone https://github.com/eProsima/Fast-RTPS.git
    $ cd Fast-RTPS
    $ mkdir build && cd build
    $ cmake ..  -DCMAKE_INSTALL_PREFIX=../../install -DCMAKE_PREFIX_PATH=../../install
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
    For this step, you need Gtest_ as explained in the requirements_windows_sources_ section above.


.. important::

    When running an *eProsima Fast DDS* application, you need to link it with the library
    where the packages have been installed. You can either prepare the environment locally by typing the command:

    .. code-block:: bash

        C:\> PATH=<PATH_TO_Fast-DDS_INSTALLATION>/Fast-DDS/install/bin

    in the console you use to run the *eProsima Fast DDS* instance, or permanently add it to your path, by opening the
    *Edit the system environment variables* control panel, and adding
    :code:`<PATH_TO_Fast-DDS_INSTALLATION>/Fast-DDS/install/bin`
    to the :code:`PATH`.

.. External links

.. _colcon: https://colcon.readthedocs.io/en/released/
.. _CMake: https://cmake.org
.. _Chocolatey: https://chocolatey.org
.. _webpage: https://slproweb.com/products/Win32OpenSSL.html
.. _Python: https://www.python.org/
.. _pip: https://pypi.org/project/pip/
.. _wget: https://www.gnu.org/software/wget/
.. _git: https://git-scm.com/
.. _vcstool: https://pypi.org/project/vcstool/
.. _Gtest: https://github.com/google/googletest
