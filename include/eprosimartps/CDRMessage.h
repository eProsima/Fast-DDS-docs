/*************************************************************************
 * Copyright (c) 2014 eProsima. All rights reserved.
 *
 * This copy of eProsima RTPS is licensed to you under the terms described in the
 * EPROSIMARTPS_LIBRARY_LICENSE file included in this distribution.
 *
 *************************************************************************/

/**
 * @file CDRMessage.h
 *	CDR Message initialization, adding and reading elements.
 *  Created on: Feb 24, 2014
 *      Author: Gonzalo Rodriguez Canosa
 *      email:  gonzalorodriguez@eprosima.com
 */

#include "eprosimartps/rtps_all.h"

#include "eprosimartps/dds/ParameterTypes.h"

using eprosima::dds::ParameterId_t;

#ifndef CDRMESSAGE_H_
#define CDRMESSAGE_H_

namespace eprosima {
namespace rtps {

/**
 * Class CDRMessage, contains   inline methods to initialize CDRMessage_t and add or read different data types.
 @ingroup COMMONMODULE
 */
namespace CDRMessage{

	/** @name Read from a CDRMessage_t.
	 * Methods to read different data types from a CDR message. Pointers to the message and to the data types are provided.
	 * The read position is updated in the message. It fails if you attempt to read outside the
	 * boundaries of the message.
	 * @param[in] msg Pointer to message.
	 * @param[out] data_ptr Pointer to data.
	 * @param[in] size Number of bytes (if necessary).
	 * @return True if correct.
	 */

	/// @{
	  inline bool readEntityId(CDRMessage_t* msg,EntityId_t*id);
	  inline bool readData(CDRMessage_t* msg,octet* o,uint16_t length);
	  inline bool readDataReversed(CDRMessage_t* msg,octet* o,uint16_t length);
	  inline bool readInt32(CDRMessage_t* msg,int32_t* lo);
	  inline bool readUInt32(CDRMessage_t* msg,uint32_t* ulo);
	  inline bool readSequenceNumber(CDRMessage_t* msg,SequenceNumber_t* sn);
	  inline bool readInt16(CDRMessage_t* msg,int16_t* i16);
	  inline bool readUInt16(CDRMessage_t* msg,uint16_t* i16);
	  inline bool readLocator(CDRMessage_t* msg,Locator_t* loc);
	  inline bool readOctet(CDRMessage_t* msg,octet* o);
	  inline bool readSequenceNumberSet(CDRMessage_t* msg,SequenceNumberSet_t* snset);
	  inline bool readTimestamp(CDRMessage_t*msg,Time_t* ts);

	///@}
//	/**
//	 * @brief Initialize given CDR message with a given byte size. It frees the memory already allocated and reserves new one.
//	 * @param[in,out] msg Pointer to the message to initialize.
//	 * @param[in] size Maximum size of the message.
//	 * @return True if correct.
//	 */
//	  inline bool initCDRMsg(CDRMessage_t* msg,uint size);
	/**
	 * Initialize given CDR message with default size. It frees the memory already allocated and reserves new one.
	 * @param[in,out] msg Pointer to the message to initialize.
	 * @return True if correct.
	 */
	  inline bool initCDRMsg(CDRMessage_t* msg);
	/**
	 * Append given CDRMessage to existing CDR Message. Joins two messages into the first one if it has space.
	 * @param[out] first Pointer to first message.
	 * @param[in] second Pointer to second message.
	 * @return True if correct.
	 */
	  inline bool appendMsg(CDRMessage_t* first,CDRMessage_t* second);


	/** @name Add to a CDRMessage_t.
	 * Methods to add different data types to a CDR message. Pointers to the message and to the data types are provided.
	 * The write position is updated in the message. It fails if you attempt to write outside the
	 * boundaries of the message.
	 * @param[in,out] Pointer to message.
	 * @param[in] data Data to add (might be a pointer).
	 * @param[in] byteSize Number of bytes (if necessary).
	 * @return True if correct.
	 */
	/// @{

	  inline bool addData(CDRMessage_t*,octet*,uint number_bytes);
	  inline bool addDataReversed(CDRMessage_t*,octet*,uint byte_number);
	  inline bool addOctet(CDRMessage_t*msg,octet o);
	  inline bool addUInt16(CDRMessage_t*msg,uint16_t us);
	  inline bool addInt32(CDRMessage_t*msg,int32_t lo);
	  inline bool addUInt32(CDRMessage_t*msg,uint32_t lo);
	  inline bool addInt64(CDRMessage_t*msg,int64_t lo);
	  inline bool addEntityId(CDRMessage_t*msg,EntityId_t* id);
	  inline bool addSequenceNumber(CDRMessage_t*msg,SequenceNumber_t* sn);
	  inline bool addSequenceNumberSet(CDRMessage_t*msg,SequenceNumberSet_t* sns);
	  inline bool addLocator(CDRMessage_t*msg,Locator_t*loc);
	  inline bool addParameterStatus(CDRMessage_t*msg,octet status);
	  inline bool addParameterKey(CDRMessage_t*msg,InstanceHandle_t* iHandle);
	  inline bool addParameterSentinel(CDRMessage_t*msg);
	  inline bool addParameterId(CDRMessage_t*msg,ParameterId_t pid);
	///@}

}
} /* namespace rtps */
} /* namespace eprosima */

#include "CDRMessage.hpp"


#endif /* CDRMESSAGE_H_ */
