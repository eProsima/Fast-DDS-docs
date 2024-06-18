.. rst-class:: api-ref

InstanceHandle_t Operators
--------------------------------

.. doxygenfunction:: eprosima::fastdds::rtps::operator==(const InstanceHandle_t &ihandle1, const InstanceHandle_t &ihandle2) noexcept
    :project: FastDDS

.. doxygenfunction:: eprosima::fastdds::rtps::operator!=(const InstanceHandle_t &ihandle1, const InstanceHandle_t &ihandle2) noexcept
    :project: FastDDS

.. doxygenfunction:: eprosima::fastdds::rtps::operator<(const InstanceHandle_t &h1, const InstanceHandle_t &h2) noexcept
    :project: FastDDS

.. doxygenfunction:: eprosima::fastdds::rtps::operator<<(std::ostream &output, const InstanceHandle_t &iHandle)
    :project: FastDDS

.. doxygenfunction:: eprosima::fastdds::rtps::operator>>(std::istream &input, InstanceHandle_t &iHandle)
    :project: FastDDS

.. doxygenfunction:: eprosima::fastdds::rtps::iHandle2GUID(GUID_t &guid, const InstanceHandle_t &ihandle) noexcept
    :project: FastDDS

.. doxygenfunction:: eprosima::fastdds::rtps::iHandle2GUID(const InstanceHandle_t &ihandle) noexcept
    :project: FastDDS
