/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "AddGrpB"
 * 	found in "../asn/J2735_201603DA_copyright_updated 12.2.ASN"
 * 	`asn1c -fcompound-names -pdu=all`
 */

#include "LaneDataAttribute-addGrpB.h"

static const ber_tlv_tag_t asn_DEF_LaneDataAttribute_addGrpB_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static asn_SEQUENCE_specifics_t asn_SPC_LaneDataAttribute_addGrpB_specs_1 = {
	sizeof(struct LaneDataAttribute_addGrpB),
	offsetof(struct LaneDataAttribute_addGrpB, _asn_ctx),
	0,	/* No top level tags */
	0,	/* No tags in the map */
	0, 0, 0,	/* Optional elements (not needed) */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_LaneDataAttribute_addGrpB = {
	"LaneDataAttribute-addGrpB",
	"LaneDataAttribute-addGrpB",
	&asn_OP_SEQUENCE,
	asn_DEF_LaneDataAttribute_addGrpB_tags_1,
	sizeof(asn_DEF_LaneDataAttribute_addGrpB_tags_1)
		/sizeof(asn_DEF_LaneDataAttribute_addGrpB_tags_1[0]), /* 1 */
	asn_DEF_LaneDataAttribute_addGrpB_tags_1,	/* Same as above */
	sizeof(asn_DEF_LaneDataAttribute_addGrpB_tags_1)
		/sizeof(asn_DEF_LaneDataAttribute_addGrpB_tags_1[0]), /* 1 */
	{ 0, 0, SEQUENCE_constraint },
	0, 0,	/* No members */
	&asn_SPC_LaneDataAttribute_addGrpB_specs_1	/* Additional specs */
};

