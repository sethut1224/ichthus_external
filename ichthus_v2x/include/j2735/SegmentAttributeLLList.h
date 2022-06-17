/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "../asn/J2735_201603DA_copyright_updated 12.2.ASN"
 * 	`asn1c -f compound-names -pdu=all`
 */

#ifndef	_SegmentAttributeLLList_H_
#define	_SegmentAttributeLLList_H_


#include <asn_application.h>

/* Including external dependencies */
#include "SegmentAttributeLL.h"
#include <asn_SEQUENCE_OF.h>
#include <constr_SEQUENCE_OF.h>

#ifdef __cplusplus
extern "C" {
#endif

/* SegmentAttributeLLList */
typedef struct SegmentAttributeLLList {
	A_SEQUENCE_OF(SegmentAttributeLL_t) list;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} SegmentAttributeLLList_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_SegmentAttributeLLList;
extern asn_SET_OF_specifics_t asn_SPC_SegmentAttributeLLList_specs_1;
extern asn_TYPE_member_t asn_MBR_SegmentAttributeLLList_1[1];
extern asn_per_constraints_t asn_PER_type_SegmentAttributeLLList_constr_1;

#ifdef __cplusplus
}
#endif

#endif	/* _SegmentAttributeLLList_H_ */
#include <asn_internal.h>
