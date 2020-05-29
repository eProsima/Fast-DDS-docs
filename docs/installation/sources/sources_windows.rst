.. _windows_sources:

Windows installation from sources
=================================

To install *eProsima Fast DDS* from sources, you first need to meet the required dependencies (see Requirements - REF)
and then choose whether to follow either the colcon_ (see colcon - REF) or the CMake_ (see cmake -REF)
installation instructions.

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

After downloading these packages, open an administrative shell and execute the following command:

.. code-block:: bash

    choco install -y -s <PATH_TO_DOWNLOADS> asio tinyxml2

where :code:`<PATH\TO\DOWNLOADS>` is the folder you downloaded the packages into.

Cmake
^^^^^

You can download and install Cmake_ by following the instrucctions on the website.

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

    By default, *eProsima Fast DDS* doesn’t compile tests.
    You can activate them by adding the :code:`-DPERFORMANCE_TESTS=ON` flag when calling colcon_ or CMake_
    (for details, see below - REF).

You can find information on how to install Gtest at this `link <https://github.com/google/googletest>`_.

Installation with Colcon
------------------------

colcon_ is a command line tool to build sets of software packages.
This section explains how to use it to compile easily *eProsima Fast DDS* and its dependencies.

.. important::

    You need to run colcon within a Visual Studio prompt. To do so, launch a "Developer Command Prompt" in the
    search engine/from the Start button.

.. note::

    In order to run the commands below, you need to have the following packages installed in your Windows system:

    * python_
    * pip_, with the :code:`get-pip.py` command. After installing it, add the environment variable :code:`pip`
      pointing to the installation directory.
    * wget_. After installing it, add the environment variable :code:`wget` pointing to the directory where the
      executable is.
    * vcs. After installing it, add the environment variable :code:`vcs` pointing to the directory where the
      executable is.

Once installed, you need to add the path to these executables to your path from the
'Edit the system environment variables' control panel.

You can then install the ROS2 development tools (colcon_ and vcstool_):

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

    colcon build --cmake-args -DTHIRDPARTY=ON

To run an *eProsima Fast DDS* instance, you need source the colcon overlay with the command:

.. code-block:: bash

    setup.bat

.. note::

    The sourcing of the local colcon overlay is required every time the colcon workspace is opened in a new shell
    environment to run an *eProsima Fast DDS* instance.
    As an alternative, you can add it permanently to you path from the 'Edit the system environment variables' control
    panel.


.. note::

    If you want to compile the examples, you will need to add the flag
    :code:`--cmake-args "-DCOMPILE_EXAMPLES=ON"` when running :code:`colcon build`.
    If you want to compile the performance tests, you will need to add the flag
    :code:`--cmake-args "--DPERFORMANCE_TESTS=ON"` when running :code:`colcon build`.
    For this step, you need Gtest_ as explained in the Requirements section above (REF).


Manual Installation
-------------------

To install *eProsima Fast DDS*, you need to have git_ installed. After installing it, add the environment variable
:code:`git` pointing to the directory where the executable is.

Now open a command prompt, and create a :code:`Fast-DDS` directory where to download and build *eProsima Fast DDS* and
its dependencies:

.. code-block:: bash

    mkdir Fast-DDS && cd Fast-DDS

Now, before compiling *eProsima Fast DDS*, you need to clone the following dependencies and compile them using CMake.

* `Fast CDR <https://github.com/eProsima/Fast-CDR.git>`_

  .. code-block:: bash

      $ git clone https://github.com/eProsima/Fast-CDR.git
      $ cd Fast-CDR
      $ mkdir build && cd build
      $ cmake -DCMAKE_INSTALL_PREFIX=install ..
      $ cmake --build . --target install
      $ cd ../..

* `Foonathan memory <https://github.com/foonathan/memory>`_

  .. code-block:: bash

      $ git clone https://github.com/eProsima/foonathan_memory_vendor.git
      $ cd foonathan_memory_vendor
      $ mkdir build && cd build
      $ cmake -DCMAKE_INSTALL_PREFIX=install ..
      $ cmake --build . --target install
      $ cd ../..

Once all dependencies are installed, you will be able to compile and install *eProsima Fast DDS*:

.. code-block:: bash

    $ git clone https://github.com/eProsima/Fast-RTPS.git
    $ cd Fast-RTPS
    $ mkdir build && cd build
    $ cmake -DCMAKE_INSTALL_PREFIX=install ..
    $ cmake --build . --target install

.. note::

    If you want to compile the examples, you will need to add the argument :code:`-DCOMPILE_EXAMPLES=ON` when calling
    the configuration CMake_.
    If you want to compile the performance tests, you will need to add the argument
    :code:`-DPERFORMANCE_TESTS=ON` when calling the configuration CMake_.
    For this step, you need Gtest_ as explained in the Requirements section above (REF).



.. External links

.. _colcon: https://colcon.readthedocs.io/en/released/
.. _CMake: https://cmake.org
.. _Gtest: https://github.com/google/googletest
.. _Chocolatey: https://chocolatey.org
.. _webpage: https://slproweb.com/products/Win32OpenSSL.html
.. _python: https://www.python.org/downloads/windows/
.. _pip: ??
.. _wget: https://www.gnu.org/software/wget/
.. _vcstool: https://pypi.org/project/vcstool/
.. _git: https://git-scm.com/download/win
