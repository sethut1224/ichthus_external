/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "../asn/J2735_201603DA_copyright_updated 12.2.ASN"
 * 	`asn1c -f compound-names -pdu=all`
 */

#ifndef	_SnapshotDistance_H_
#define	_SnapshotDistance_H_


#include <asn_application.h>

/* Including external dependencies */
#include "GrossDistance.h"
#include "GrossSpeed.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* SnapshotDistance */
typedef struct SnapshotDistance {
	GrossDistance_t	 distance1;
	GrossSpeed_t	 speed1;
	GrossDistance_t	 distance2;
	GrossSpeed_t	 speed2;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} SnapshotDistance_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_SnapshotDistance;
extern asn_SEQUENCE_specifics_t asn_SPC_SnapshotDistance_specs_1;
extern asn_TYPE_member_t asn_MBR_SnapshotDistance_1[4];

#ifdef __cplusplus
}
#endif

#endif	/* _SnapshotDistance_H_ */
#include <asn_internal.h>
