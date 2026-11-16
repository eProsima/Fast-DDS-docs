Create the application workspace
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The application workspace will have the following structure at the end of the project.
Files ``HelloWorldPublisher.py`` and ``HelloWorldSubscriber.py`` are the Publisher application and
Subscriber application respectively.

.. code-block:: shell-session

    .
    ├── CMakeCache.txt
    ├── CMakeFiles
    ├── CMakeLists.txt
    ├── HelloWorld.hpp
    ├── HelloWorld.i
    ├── HelloWorld.idl
    ├── HelloWorld.py
    ├── HelloWorldCdrAux.hpp
    ├── HelloWorldCdrAux.ipp
    ├── HelloWorldPubSubTypes.cxx
    ├── HelloWorldPubSubTypes.hpp
    ├── HelloWorldPubSubTypes.i
    ├── HelloWorldPublisher.py
    ├── HelloWorldSubscriber.py
    ├── HelloWorldTypeObjectSupport.cxx
    ├── HelloWorldTypeObjectSupport.hpp
    ├── Makefile
    ├── _HelloWorldWrapper.so
    ├── cmake_install.cmake
    └── libHelloWorld.so


Let's create the directory tree first.

.. code-block:: bash

    mkdir workspace_HelloWorld && cd workspace_HelloWorld
