.. _fastddsgen_python_bindings:

Building Python auxiliary libraries
===================================

*eProsima Fast DDS-Gen* can generate the required source files and CMake project to build the Python modules that
allow the use of the IDL defined data types within a *Fast DDS* Python-based application.
Each IDL file will result in a new Python module that will contain all the data types defined in the file.
The proper Python binding is generated building the provided solution using `SWIG <(http://www.swig.org/)>`_.

After generating the files using *eProsima Fast DDS-Gen* with the `-python` option your
:ref:`workspace <fastddsgen_pubsub_app_workspace>` will include the SWIG interface files (`.i`) required to build the
data type's Python modules; use the generated CMakeFile to build the data types and create the Python bindings:

.. code:: bash

    mkdir build
    cd build
    cmake ..
    cmake --build .

This will create the Python files (`.py`) with the modules (one per each IDL file) that have to be imported within the
Python script.
