#include "ccsds.h"


bool CCSDS_ValidCheckSum (uint8_t array[], uint8_t size) {

	return (CCSDS_ComputeCheckSum(array,size) == 0);
	// This works because when you XOR something with itself, it will be zero. Checksum part becomes zero, and XOR with zero is identity, so it is as if that byte was not there for the initial checksum.

} /* END CCSDS_ValidCheckSum() */


uint8_t CCSDS_ComputeCheckSum (uint8_t array[], uint8_t size) {
	uint8_t    CheckSum;
	checksum = 0xFF;	

	// iterate through all data in array and bytewise XOR
	for (uint8_t i = 0; i < size; i++) {
		checksum ^= array[i];
	}

	return checksum;

} /* END CCSDS_ComputeCheckSum() */
