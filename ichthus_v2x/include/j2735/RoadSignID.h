/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "../asn/J2735_201603DA_copyright_updated 12.2.ASN"
 * 	`asn1c -f compound-names -pdu=all`
 */

#ifndef	_RoadSignID_H_
#define	_RoadSignID_H_


#include <asn_application.h>

/* Including external dependencies */
#include "Position3D.h"
#include "HeadingSlice.h"
#include "MUTCDCode.h"
#include "MsgCRC.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* RoadSignID */
typedef struct RoadSignID {
	Position3D_t	 position;
	HeadingSlice_t	 viewAngle;
	MUTCDCode_t	*mutcdCode	/* OPTIONAL */;
	MsgCRC_t	*crc	/* OPTIONAL */;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} RoadSignID_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_RoadSignID;
extern asn_SEQUENCE_specifics_t asn_SPC_RoadSignID_specs_1;
extern asn_TYPE_member_t asn_MBR_RoadSignID_1[4];

#ifdef __cplusplus
}
#endif

#endif	/* _RoadSignID_H_ */
#include <asn_internal.h>
