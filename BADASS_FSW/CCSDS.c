#include "ccsds.h"


bool CCSDS_ValidCheckSum (uint8_t array[], uint8_t size) {

	return (CCSDS_ComputeCheckSum(array,size) == 0);

} /* END CCSDS_ValidCheckSum() */


uint8_t CCSDS_ComputeCheckSum (uint8_t array[], uint8_t size) {
	uint8_t    CheckSum;
	checksum = 0xFF;

	for (uint8_t i = 0; i < size; i++) {
		checksum ^= array[i];
	}

	return checksum;

} /* END CCSDS_ComputeCheckSum() */
