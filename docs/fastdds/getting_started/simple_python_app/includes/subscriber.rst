Write the Fast DDS subscriber
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

From the workspace, run the following command to download the HelloWorldSubscriber.py file.

.. code-block:: bash

    wget -O HelloWorldSubscriber.py \
        https://raw.githubusercontent.com/eProsima/Fast-RTPS-docs/master/code/Examples/Python/HelloWorld/HelloWorldSubscriber.py

This is the Python source code for the subscriber application.
The application runs a subscriber until the user press *Ctrl+C* receiving samples under the topic HelloWorldTopic.

.. literalinclude:: /../code/Examples/Python/HelloWorld/HelloWorldSubscriber.py
    :language: python
    :linenos:

Examining the code
""""""""""""""""""

Since the source code of both the publisher and subscriber applications is mostly identical, this document will focus on
the main differences between them, omitting the parts of the code that have already been explained.

Following the same structure as in the publisher explanation, the first step is the implementation of the data reader
listener.
The first overridden callback of the ReaderListener is the |DataReaderListener::on_subscription_matched-python-api|,
which is the analog of the |DataWriterListener::on_publication_matched-python-api| callback of the DataWriter.

.. literalinclude:: /../code/Examples/Python/HelloWorld/HelloWorldSubscriber.py
    :language: python
    :lines: 36-40
    :dedent: 4

The second overridden callback is |DataReaderListener::on_data_available-python-api|.
In this, the next received sample that the data reader can access is taken and processed to display its content.
It is here that the object of the |SampleInfo-python-api| class is defined, which determines whether a sample has
already been read or taken.

.. literalinclude:: /../code/Examples/Python/HelloWorld/HelloWorldSubscriber.py
    :language: python
    :lines: 43-46
    :dedent: 4

The next line defines the :class:`Reader` class that implements a subscriber.

.. literalinclude:: /../code/Examples/Python/HelloWorld/HelloWorldSubscriber.py
    :language: python
    :lines: 51

Next comes the subscriber initialization public member function.
This is the same as the initialization public member function defined for the :class:`Writer`.

.. literalinclude:: /../code/Examples/Python/HelloWorld/HelloWorldSubscriber.py
    :language: python
    :lines: 54-76
    :dedent: 4

The public member function :func:`run` ensures that the subscriber runs until the user press *Ctrl+C*.

.. literalinclude:: /../code/Examples/Python/HelloWorld/HelloWorldSubscriber.py
    :language: python
    :lines: 85-89
    :dedent: 4

Finally, the participant that implements a subscriber is initialized and run in main.

.. literalinclude:: /../code/Examples/Python/HelloWorld/HelloWorldSubscriber.py
    :language: python
    :lines: 92-96
