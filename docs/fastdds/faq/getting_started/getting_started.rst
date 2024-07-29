.. include:: ../../../03-exports/aliases.include
.. include:: ../../../03-exports/aliases-api.include

.. _freq_getting_started_questions:

Frequently Asked Getting Started Questions
==========================================

   
.. collapse::  What is DDS?
   
    \ 


    :Answer:

    The **Data Distribution Service (DDS)** is a data-centric communication protocol used for distributed software application communications.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
   
.. collapse::  What are the key entities of a DDS? 
   
    \ 


    :Answer:

    There are four basic entities: **Publisher**, **Subscriber**, **Topic**, **Domain**. 
   
-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
   
.. collapse::  What is the primary purpose of using Quality of Service (QoS) in DDS?
   
    \ 


    :Answer:

    DDS uses QoS to define the behavioral characteristics of DDS Entities. QoS are comprised of individual QoS policies (objects of type deriving from QoSPolicy).
   
-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What is the primary responsibility of a DDS Publisher entity?
   
    \ 


    :Answer:

    **Publisher**. It is the DCPS entity in charge of the creation and configuration of the **DataWriters** it implements. The  **DataWriter** is the entity in charge of the actual publication of the messages. Each one will have an assigned **Topic** under which the messages are published.
   
-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What is the primary function of a DDS Domain?
   
    \ 


    :Answer:
        
    **Domain**. This is the concept used to link all publishers and subscribers, belonging to one or more applications, which exchange data under different topics. These individual applications that participate in a domain are called **DomainParticipant**. The DDS Domain is identified by a domain ID.
   
-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What is RTPS?
   
    \ 


    :Answer:

    The **Real-Time Publish Subscribe (RTPS)** protocol, developed to support DDS applications, is a publication-subscription communication middleware over transports such as UDP/IP.  

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
   
.. collapse::  What type of communication does RTPS support?
   
    \ 


    :Answer:

    It is designed to support both unicast and multicast communications.
   
-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What is the maximum number of endpoints that a single RTPSParticipant can have?
   
    \ 


    :Answer:

    A RTPSParticipant can have any number of writer and reader endpoints.
   
-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What is the primary mechanism by which RTPS participants exchange data, and what role do topics play in this process?
   
    \ 

    
    :Answer:

    Communication revolves around **Topics**, which define and label the data being exchanged. The topics do not belong to a specific participant. The participant, through the **RTPSWriters**, makes changes in the data published under a topic, and through the **RTPSReaders** receives the data associated with the topics to which it subscribes. The communication unit is called **Change**, which represents an update in the data that is written under a Topic. **RTPSReaders/RTPSWriters** register these changes on their **History**, a data structure that serves as a cache for recent changes.
   
|
