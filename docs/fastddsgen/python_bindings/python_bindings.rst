.. _fastddsgen_python_bindings:

Building Python auxiliary libraries
===================================

.. note:: Currently only supported in Linux environments.

*eProsima Fast DDS-Gen* can generate the required source files and CMake project to build the Python modules that
allow the use of the IDL defined data types within a *Fast DDS* Python-based application.
Each IDL file will result in a new Python module that will contain all the data types defined in the file.
The proper Python binding is generated building the provided solution using SWIG_.

Calling *eProsima Fast DDS-Gen* with the option `-python` will generate these files.
*eProsima Fast DDS-Gen* will generate a `.i` file which will be processed by SWIG_ and a CMake project to call SWIG_
first generating C++ files (for connecting C++ and Python) and Python files (Python module for your type) and
then compiling the C++ sources.

Before calling CMake, the :ref:`fastddsgen_python_build` process needs several :ref:`fastddsgen_python_deps` to be met.

.. _fastddsgen_python_deps:

Dependencies
""""""""""""

.. include:: ../../04-common/python_requirements.rst
   :start-after: .. begin-swig
   :end-before: .. end-swig

.. include:: ../../04-common/python_requirements.rst
   :start-after: .. begin-libpython-dev
   :end-before: .. end-libpython-dev

.. _fastddsgen_python_build:

Building
""""""""

Call CMake:

.. code:: bash

    mkdir build
    cd build
    cmake ..
    cmake --build .

This will create the Python files (`.py`) with the modules (one per each IDL file) that have to be imported within the
Python script.

.. External links

.. _SWIG: http://www.swig.org/
