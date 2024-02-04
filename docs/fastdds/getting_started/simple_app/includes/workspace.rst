Create the application workspace
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The application workspace will have the following structure at the end of the project.
Files ``build/DDSHelloWorldPublisher`` and ``build/DDSHelloWorldSubscriber`` are the Publisher application and
Subscriber application respectively.

.. code-block:: shell-session

    .
    └── workspace_DDSHelloWorld
        ├── build
        │   ├── CMakeCache.txt
        │   ├── CMakeFiles
        │   ├── cmake_install.cmake
        │   ├── DDSHelloWorldPublisher
        │   ├── DDSHelloWorldSubscriber
        │   └── Makefile
        ├── CMakeLists.txt
        └── src
            ├── HelloWorld.hpp
            ├── HelloWorld.idl
            ├── HelloWorldCdrAux.hpp
            ├── HelloWorldCdrAux.ipp
            ├── HelloWorldPublisher.cpp
            ├── HelloWorldPubSubTypes.cxx
            ├── HelloWorldPubSubTypes.h
            └── HelloWorldSubscriber.cpp

Let's create the directory tree first.

.. code-block:: bash

    mkdir workspace_DDSHelloWorld && cd workspace_DDSHelloWorld
    mkdir src build
