Here you can find the source code used in the documentation and also the XML examples.
They are located here to test they are right.


# Testing

## Source code

Source code is located in the file `CodeTester.cpp`.
CMake is used to compile the source code.

```
mkdir build && cd build
cmake ..
```

By default it will try to use the same git branch's name to clone Fast DDS.
If you want to use another Fast DDS's branch you have the option `FASTDDS_BRANCH`.

```bash
mkdir build && cd build
cmake -DFASTDDS_BRANCH=develop ..
```

## XML examples

XML examples are located in the file `XMLTester.xml`.
The generated application has the functionality to check the XML file against Fast DDS XML parser.
You can use CTest to run the application and check the XML examples.

```bash
ctest -VV
```
