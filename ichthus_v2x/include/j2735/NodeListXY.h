/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "../asn/J2735_201603DA_copyright_updated 12.2.ASN"
 * 	`asn1c -f compound-names -pdu=all`
 */

#ifndef	_NodeListXY_H_
#define	_NodeListXY_H_


#include <asn_application.h>

/* Including external dependencies */
#include "NodeSetXY.h"
#include "ComputedLane.h"
#include <constr_CHOICE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum NodeListXY_PR {
	NodeListXY_PR_NOTHING,	/* No components present */
	NodeListXY_PR_nodes,
	NodeListXY_PR_computed
	/* Extensions may appear below */
	
} NodeListXY_PR;

/* NodeListXY */
typedef struct NodeListXY {
	NodeListXY_PR present;
	union NodeListXY_u {
		NodeSetXY_t	 nodes;
		ComputedLane_t	 computed;
		/*
		 * This type is extensible,
		 * possible extensions are below.
		 */
	} choice;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} NodeListXY_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_NodeListXY;
extern asn_CHOICE_specifics_t asn_SPC_NodeListXY_specs_1;
extern asn_TYPE_member_t asn_MBR_NodeListXY_1[2];
extern asn_per_constraints_t asn_PER_type_NodeListXY_constr_1;

#ifdef __cplusplus
}
#endif

#endif	/* _NodeListXY_H_ */
#include <asn_internal.h>
