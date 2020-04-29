#!/bin/bash
HEADER_FILE=$1
DIR=$2

CLASSES=$(cat ${HEADER_FILE} | grep -e "class" | grep -v -e "friend" -e "iterator" -e "//" | awk '{print $2}')

for CLASS in ${CLASSES}
do
    CLASS_LOWER=$(echo "${CLASS}" | awk '{print tolower($0)}')
    FILENAME=${DIR}/${CLASS_LOWER}.rst
    if [[ ! -f ${FILENAME} ]]
    then
        echo "Creating page for ${CLASS} in ${FILENAME}"

        echo ".. _api_pim_${CLASS_LOWER}:" > ${FILENAME}
        echo "" >> ${FILENAME}
        echo "${CLASS}" >> ${FILENAME}
        echo "-----------" >> ${FILENAME}
        echo "" >> ${FILENAME}
        echo ".. doxygenclass:: eprosima::fastdds::dds::${CLASS}" >> ${FILENAME}
        echo "    :project: Fast RTPS" >> ${FILENAME}
        echo "    :members:" >> ${FILENAME}
        echo "" >> ${FILENAME}

        IDX_FILE=$(basename ${DIR})
        IDX_FILE=${DIR}/${IDX_FILE}.rst
        "Adding ${FILENAME} to ${IDX_FILE}"
        echo "   ${FILENAME}" >> ${IDX_FILE}
    fi
done