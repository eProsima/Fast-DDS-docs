<a id="api-pim-verbatimtextdescriptor"></a>

<a id="verbatimtextdescriptor"></a>

# VerbatimTextDescriptor

### class VerbatimTextDescriptor

### Public Functions

### virtual std::string &placement() = 0

Returns the location within the generated output at which the output text should be inserted. 

* **Returns:**
  The location. 

### virtual const std::string &placement() const = 0

Returns the location within the generated output at which the output text should be inserted. 

* **Returns:**
  The location. 

### virtual void placement(const std::string &placement) = 0

Sets the location within the generated output at which the output text should be inserted. 

* **Parameters:**
  **placement** – **[in]** The location. 

### virtual void placement(std::string &&placement) = 0

Sets the location within the generated output at which the output text should be inserted. 

* **Parameters:**
  **placement** – **[in]** The location. 

### virtual std::string &text() = 0

Returns the literal output text. 

* **Returns:**
  The text. 

### virtual const std::string &text() const = 0

Returns the literal output text. 

* **Returns:**
  The text. 

### virtual void text(const std::string &text) = 0

Sets the literal output text. 

* **Parameters:**
  **text** – **[in]** The text. 

### virtual void text(std::string &&text) = 0

Sets the literal output text. 

* **Parameters:**
  **text** – **[in]** The text. 

### virtual ReturnCode_t copy_from(traits<[VerbatimTextDescriptor](#_CPPv4N8eprosima7fastdds3dds22VerbatimTextDescriptorE)>::ref_type descriptor) = 0

Overwrites the contents of this descriptor with those of another descriptor. 

* **Parameters:**
  **descriptor** – **[in]** reference. 
* **Return values:**
  * **RETCODE_OK** – when the copy was successful. 
  * **RETCODE_BAD_PARAMETER** – when descriptor reference is nil. 
* **Returns:**
  ReturnCode_t

### virtual bool equals(traits<[VerbatimTextDescriptor](#_CPPv4N8eprosima7fastdds3dds22VerbatimTextDescriptorE)>::ref_type descriptor) = 0

Compares. 

* **Parameters:**
  **descriptor** – **[in]** reference to compare to. 
* **Returns:**
  **bool** `true` on equality 

### virtual bool is_consistent() = 0

Indicates whether the states of all of this descriptor’s properties are consistent. 

* **Returns:**
  **bool** `true` if consistent.
