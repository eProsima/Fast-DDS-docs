<a id="contentfilterproperty"></a>

# ContentFilterProperty

### class ContentFilterProperty

Information about the content filter being applied by a reader. 

### Public Functions

### inline explicit ContentFilterProperty(const [AllocationConfiguration](#_CPPv4N8eprosima7fastdds4rtps21ContentFilterProperty23AllocationConfigurationE) &config)

Construct a [ContentFilterProperty](#classeprosima_1_1fastdds_1_1rtps_1_1ContentFilterProperty).

* **Parameters:**
  **config** – Allocation configuration for the new object. 

### Public Members

### fastcdr::string_255 content_filtered_topic_name

Name of the content filtered topic on which the reader was created. 

### fastcdr::string_255 related_topic_name

Name of the related topic being filtered. 

### fastcdr::string_255 filter_class_name

Class name of the filter being used. May be empty to indicate the [ContentFilterProperty](#classeprosima_1_1fastdds_1_1rtps_1_1ContentFilterProperty) is not present. 

### std::string filter_expression

Filter expression indicating which content the reader wants to receive. May be empty to indicate the [ContentFilterProperty](#classeprosima_1_1fastdds_1_1rtps_1_1ContentFilterProperty) is not present. 

### fastdds::ResourceLimitedVector<fastcdr::string_255, std::true_type> expression_parameters

List of values for the parameters present on the filter expression. 

### struct AllocationConfiguration

Allocation configuration for a [ContentFilterProperty](#classeprosima_1_1fastdds_1_1rtps_1_1ContentFilterProperty). 

### Public Members

### std::size_t expression_initial_size = 0

Preallocated size of the filter expression. 

### fastdds::ResourceLimitedContainerConfig expression_parameters = {0, 100, 1}

Allocation configuration for the list of expression parameters.
