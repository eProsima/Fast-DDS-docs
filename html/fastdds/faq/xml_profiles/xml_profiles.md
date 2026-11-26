<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="freq-xml-profiles-questions"></a>

<a id="xml-profiles-frequently-asked-questions"></a>

# XML PROFILES Frequently Asked Questions

**How are XML profiles defined?**

<br/> The XML profiles are defined within the `<dds>` element, and in turn, within the `<profiles>` XML elements.
For further information, refer to [Creating an XML profiles file](../../xml_configuration/making_xml_profiles.md#making-xml-profiles).

---

**What happens if an XML file is loaded multiple times?**

<br/> If the same XML profile file is loaded multiple times, the second loading of the file will result in an error together with the consequent error log.
For further information, refer to [Loading and applying profiles](../../xml_configuration/making_xml_profiles.md#loadingapplyingprofiles).

---

**What is stand-alone profiles definition?**

<br/> The element defining the XML profile is the root element of the XML file.
For further information, refer to [Rooted vs Standalone profiles definition](../../xml_configuration/making_xml_profiles.md#rootedvsstandalone).

---

**What is rooted profile definition?**

<br/> The element defining the XML profile is the child element of another element.
For further information, refer to [Rooted vs Standalone profiles definition](../../xml_configuration/making_xml_profiles.md#rootedvsstandalone).

---

**What do DomainParticipant profiles do?**

<br/> The [DomainParticipant](../../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant) profiles allow the definition of the configuration of DomainParticipants through XML files.
These profiles are defined within the `<participant>` XML tags.
For further information, refer to [DomainParticipant profiles](../../xml_configuration/domainparticipant.md#participantprofiles).

---

**What are the domain participant XML attributes?**

<br/> The `<participant>` element has two attributes defined: `profile_name` and `is_default_profile`.
For further information, refer to [DomainParticipant XML attributes](../../xml_configuration/domainparticipant.md#domainparticipantattributes).

---

**How is the DomainParticipant configured?**

<br/> The `<participant>` element has two child elements: `<domainId>` and `<rtps>` .
All the [DomainParticipant](../../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant) configuration options belong to the `<rtps>` element, except for the DDS DomainId which is defined by the `<domainId>` element.
For further information, refer to [DomainParticipant configuration](../../xml_configuration/domainparticipant.md#domainparticipantconfig).

---

**What do DataWriter profiles do?**

<br/> The [DataWriter](../../dds_layer/publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter) profiles allow for configuring DataWriters from an XML file.
These profiles are defined within the `<data_writer>` XML tags.
For further information, refer to [DataWriter profiles](../../xml_configuration/datawriter.md#publisherprofiles).

---

**What are the XML attributes of the DataWriter?**

<br/> The `<data_writer>` element has two attributes defined: `profile_name` and `is_default_profile`.
For further information, refer to [DataWriter profiles](../../xml_configuration/datawriter.md#publisherprofiles).

---

**What do DataReader profiles do?**

<br/> The [DataReader](../../dds_layer/subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader) profiles allow declaring DataReaders from an XML file.
These profiles are defined within the `<data_reader>` XML tags.
For further information, refer to [DataReader profiles](../../xml_configuration/datareader.md#subscriberprofiles).

---

**What are the XML attributes of the DataReader?**

<br/> The `<data_reader>` element has two attributes defined: `profile_name` and `is_default_profile`.
For further information, refer to [DataReader profiles](../../xml_configuration/datareader.md#subscriberprofiles).

---

**What do Topic profiles do?**

<br/> The `<topic>` element has two attributes defined: `profile_name` and `is_default_profile`.
For further information, refer to [Topic profiles](../../xml_configuration/topic.md#topic).

---

**What are the available XML elements for configuring transport layer parameters in Fast DDS, and what is their purpose?**

<br/> This section defines the XML elements available for configuring the transport layer parameters in Fast DDS.
These elements are defined within the XML tag `<transports_descriptors>`.
The `<transport_descriptors>` can contain one or more `<transport_descriptor>` XML elements. Each `<transport_descriptor>` element defines a configuration for a specific type of transport protocol.
For further information, refer to [Transport descriptors](../../xml_configuration/transports.md#transportdescriptors).

---

**Are log profiles available?**

<br/> eProsima Fast DDS allows for registering and configuring Log consumers using XML configuration files.
The logging profiles are defined within the `<log>` XML tags. The `<log>` element has two child elements: `<use_default>` and `<consumer>`.
For further information, refer to [Log profiles](../../xml_configuration/log.md#logprofiles).

---

**What is the advantage of having Dynamic Language Binding support in XML files?**

<br/> The topic data types can be modified without the need to modify the source code of the DDS application.
For further information, refer to [Dynamic Types profiles](../../xml_configuration/dynamic_types.md#xmldynamictypes).

---

**How are Dynamic Types configured with XML?**

<br/> Dynamic Types in Fast DDS are configured using XML files, where data types are defined within `<types>` elements.
Each `<types>` element can contain one or more `<type>` definitions. Members of these types, such as primitive or complex types like structures, unions, sequences, arrays, and maps, are specified using the `<member>` tag along with appropriate attributes for the type and its characteristics.
Once the XML file is created, it can be loaded into the Fast DDS application using the [`load_XML_profiles_file()`](../../api_reference/dds_pim/domain/domainparticipantfactory.md#_CPPv4N8eprosima7fastdds3dds24DomainParticipantFactory22load_XML_profiles_fileERKNSt6stringE) function, allowing the types to be dynamically used without modifying the application’s source code.
For further information, refer to [Dynamic Types profiles](../../xml_configuration/dynamic_types.md#xmldynamictypes).

<br/>
