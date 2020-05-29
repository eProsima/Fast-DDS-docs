.. _windows_binaries:

Windows installation from binaries
==================================

You can download the latest release of *eProsima Fast DDS* for Windows from the company website
`downloads page <https://eprosima.com/index.php/downloads-all>`_.
Once downloaded, execute the installer and follow the instructions, choosing your preferred Visual Studio
version and architecture when prompted.

Requirements
------------

*eProsima Fast RTPS* requires the following dependencies when installing from binaries in a Windows environment.

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

TinyXML2
^^^^^^^^

Once you have Chocolatey installed, download the following Chocolatey package from this
`ROS2 Github repository <https://github.com/ros2/choco-packages/releases/tag/2020-02-24>`_.

* tinyxml2.6.0.0.nupkg

After downloading these package, open an administrative shell and execute the following command:

.. code-block:: bash

    choco install -y -s <PATH_TO_DOWNLOADS> tinyxml2

where :code:`<PATH_TO_DOWNLOADS>` is the folder you downloaded the packages into.

OpenSSL
^^^^^^^

You can download the latest OpenSSL version for Windows at this webpage_.
Download and use the installer that fits your requirements.
After installing, add the environment variable :code:`OPENSSL_ROOT_DIR` pointing to the installation root directory.

For example:

.. code-block:: bash

   OPENSSL_ROOT_DIR=C:\Program Files\OpenSSL-Win64

Packages
--------

By default, the installation will download all the available packages, namely:

- :code:`fastcdr`
- :code:`fastrtps`
- :code:`fastrtpsgen`
- :code:`foonathan-memory-vendor`

Components
----------

By default, the installation will download the following components:

- :code:`Libraries`
- :code:`Documentation`
- :code:`Examples`
- :code:`C++ Headers`
- :code:`Java application`

If you wish, you can uncheck the components you don't want to install.

Environmental Variables
-----------------------

eProsima Fast RTPS requires the following environmental variable setup in order to function properly:

* :code:`FASTRTPSHOME`: Root folder where *eProsima Fast DDS* is installed.
* Additions to the PATH: the location of *eProsima Fast DDS* scripts and libraries should be
  appended to the PATH.

These variables are set automatically by checking the corresponding box during the installation process.

.. External links

.. _Chocolatey: https://chocolatey.org/
.. _webpage: https://slproweb.com/products/Win32OpenSSL.html
