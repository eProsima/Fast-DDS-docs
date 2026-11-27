<a id="api-pim-icontentfilterfactory"></a>

<a id="icontentfilterfactory"></a>

# IContentFilterFactory

### struct IContentFilterFactory

The interface that a factory of [IContentFilter](icontentfilter.md#structeprosima_1_1fastdds_1_1dds_1_1IContentFilter) objects should implement. 

### Public Functions

### virtual ReturnCode_t create_content_filter(const char \*filter_class_name, const char \*type_name, const [TopicDataType](topicdatatype.md#_CPPv4N8eprosima7fastdds3dds13TopicDataTypeE) \*data_type, const char \*filter_expression, const ParameterSeq &filter_parameters, [IContentFilter](icontentfilter.md#_CPPv4N8eprosima7fastdds3dds14IContentFilterE) \*&filter_instance) = 0

Create or update an [IContentFilter](icontentfilter.md#structeprosima_1_1fastdds_1_1dds_1_1IContentFilter) instance.

* **Parameters:**
  * **filter_class_name** – **[in]** Filter class name for which the factory is being called. Allows using the same factory for different filter classes. 
  * **type_name** – **[in]** Type name of the topic being filtered. 
  * **data_type** – **[in]** Type support object of the topic being filtered. 
  * **filter_expression** – **[in]** Content filter expression. May be nullptr when updating the parameters of a filter instance. 
  * **filter_parameters** – **[in]** Values to set for the filter parameters (n on the filter expression). 
  * **filter_instance** – **[inout]** When a filter is being created, it will be nullptr on input, and will have the pointer to the created filter instance on output. The caller takes ownership of the filter instance returned. When a filter is being updated, it will have a previously returned pointer on input. The method takes ownership of the filter instance during its execution, and can update the filter instance or even destroy it and create a new one. The caller takes ownership of the filter instance returned. It should always have a valid pointer upon return. The original state of the filter instance should be preserved when an error is returned.
* **Returns:**
  A return code indicating the result of the operation. 

### virtual ReturnCode_t delete_content_filter(const char \*filter_class_name, [IContentFilter](icontentfilter.md#_CPPv4N8eprosima7fastdds3dds14IContentFilterE) \*filter_instance) = 0

Delete an [IContentFilter](icontentfilter.md#structeprosima_1_1fastdds_1_1dds_1_1IContentFilter) instance.

* **Parameters:**
  * **filter_class_name** – **[in]** Filter class name for which the factory is being called. Allows using the same factory for different filter classes. 
  * **filter_instance** – **[in]** A pointer to a filter instance previously returned by create_content_filter. The factory takes ownership of the filter instance, and can decide to destroy it or keep it for future use. In case of deletion, note this pointer must be downcasted to the derived class.
* **Returns:**
  A return code indicating the result of the operation.
