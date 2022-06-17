/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "../asn/J2735_201603DA_copyright_updated 12.2.ASN"
 * 	`asn1c -f compound-names -pdu=all`
 */

#ifndef	_Snapshot_H_
#define	_Snapshot_H_


#include <asn_application.h>

/* Including external dependencies */
#include "FullPositionVector.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct VehicleSafetyExtensions;
struct VehicleStatus;

/* Snapshot */
typedef struct Snapshot {
	FullPositionVector_t	 thePosition;
	struct VehicleSafetyExtensions	*safetyExt	/* OPTIONAL */;
	struct VehicleStatus	*dataSet	/* OPTIONAL */;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} Snapshot_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_Snapshot;
extern asn_SEQUENCE_specifics_t asn_SPC_Snapshot_specs_1;
extern asn_TYPE_member_t asn_MBR_Snapshot_1[3];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "VehicleSafetyExtensions.h"
#include "VehicleStatus.h"

#endif	/* _Snapshot_H_ */
#include <asn_internal.h>
