/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "../asn/J2735_201603DA_copyright_updated 12.2.ASN"
 * 	`asn1c -f compound-names -pdu=all`
 */

#ifndef	_SignalStatusPackageList_H_
#define	_SignalStatusPackageList_H_


#include <asn_application.h>

/* Including external dependencies */
#include <asn_SEQUENCE_OF.h>
#include <constr_SEQUENCE_OF.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct SignalStatusPackage;

/* SignalStatusPackageList */
typedef struct SignalStatusPackageList {
	A_SEQUENCE_OF(struct SignalStatusPackage) list;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} SignalStatusPackageList_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_SignalStatusPackageList;
extern asn_SET_OF_specifics_t asn_SPC_SignalStatusPackageList_specs_1;
extern asn_TYPE_member_t asn_MBR_SignalStatusPackageList_1[1];
extern asn_per_constraints_t asn_PER_type_SignalStatusPackageList_constr_1;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "SignalStatusPackage.h"

#endif	/* _SignalStatusPackageList_H_ */
#include <asn_internal.h>
