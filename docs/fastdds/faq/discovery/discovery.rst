.. _freq_discovery_questions:

Discovery Frequently Asked Questions
====================================

.. raw:: html

   <style>
       .question-box {
           background-color: #e7f3fe;
           border-left: 6px solid #2196F3;
           padding: 10px;
           margin: 10px 0;
       }
       .answer-box {
           background-color: #f9f9f9;
           border-left: 6px solid #4CAF50;
           padding: 10px;
           margin: 10px 0 20px 0;
       }
   </style>

.. raw:: html

   <div class="question-box">
       <b>Q: What are the two main phases involved in the discovery process of Fast DDS?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> The Participant Discovery Phase (PDP) involves DomainParticipants recognizing each other by sending periodic announcement messages with their unicast addresses. Matching occurs when they are in the same DDS Domain, using multicast by default, though unicast and announcement frequency can be customized. In the Endpoint Discovery Phase (EDP), DataWriters and DataReaders acknowledge each other by sharing information about topics and data types over the established channels. Matching endpoints with the same topic and data type are then ready to exchange user data.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: What discovery mechanisms does FastDDS provide?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> There are four discovery mechanisms in DDS: Simple Discovery, which follows the RTPS standard for both PDP and EDP, ensuring compatibility with other DDS implementations; Static Discovery, which uses the Simple Participant Discovery Protocol (SPDP) but skips the Endpoint Discovery phase if endpoint details are pre-known; Discovery Server, which employs a centralized server for meta traffic discovery; and Manual Discovery, which disables the PDP and requires users to manually match RTPS participants and endpoints using external meta-information channels.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: What is an initial peer list?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> An initial peer list contains one or more IP-port address pairs corresponding to remote DomainParticipants PDP discovery listening resources, so that the local DomainParticipant will not only send its PDP traffic to the default multicast address-port specified by its domain, but also to all the IP-port address pairs specified in the initial peers list.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: When could a static configuration of peers be used?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> When all DataWriters and DataReaders, and their Topics and data types, are known beforehand, the EDP phase can be replaced with a static configuration of peers.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: What is the primary difference between the Discovery Server mechanism and Simple discovery mechanism in terms of managing metatraffic?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> The Discovery Server mechanism is based on a client-server discovery paradigm, the metatraffic is managed by one or several server DomainParticipants, as opposed to simple discovery, where metatraffic is exchanged using a message broadcast mechanism like an IP multicast protocol.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: What is the primary function of a Discovery Server in the DDS architecture?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> The role of the server is to redistribute its clients' discovery information to its known clients and servers.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: What is the primary purpose of a "BACKUP" server in the Discovery Server mechanism?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> A "BACKUP" server is a server that persists its discovery database into a file. This type of server can load the network graph from a file on start-up without the need of receiving any client's information.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: What is a client in this context?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> A "CLIENT" is a participant that connects to one or more servers from which it receives only the discovery information they require to establish communication with matching endpoints.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: What is the difference between a CLIENT and a SUPER_CLIENT?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> A SUPER_CLIENT is a client that receives the discovery information known by the server, in opposition to clients, which only receive the information they need.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: What is the purpose of each server specifying its own locator list in the context of discovery configuration?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> Each client must keep a list of locators associated to the servers to which it wants to link. Each server specifies its own locator list which must be populated with "RemoteServerAttributes" objects with a valid "metatrafficUnicastLocatorList" or "metatrafficMulticastLocatorList". In XML the server list and its elements are simultaneously specified.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: What is the typical interval of time between discovery messages sent by clients to servers, as described in the text?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> As explained above the clients send discovery messages to the servers at regular intervals (ping period) until they receive message reception acknowledgement. The default value for this period is 450 ms.
   </div>
