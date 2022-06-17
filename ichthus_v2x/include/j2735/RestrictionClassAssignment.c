/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "../asn/J2735_201603DA_copyright_updated 12.2.ASN"
 * 	`asn1c -f compound-names -pdu=all`
 */

#include "RestrictionClassAssignment.h"

asn_TYPE_member_t asn_MBR_RestrictionClassAssignment_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct RestrictionClassAssignment, id),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_RestrictionClassID,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"id"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct RestrictionClassAssignment, users),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_RestrictionUserTypeList,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"users"
		},
};
static const ber_tlv_tag_t asn_DEF_RestrictionClassAssignment_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_RestrictionClassAssignment_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* id */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 } /* users */
};
asn_SEQUENCE_specifics_t asn_SPC_RestrictionClassAssignment_specs_1 = {
	sizeof(struct RestrictionClassAssignment),
	offsetof(struct RestrictionClassAssignment, _asn_ctx),
	asn_MAP_RestrictionClassAssignment_tag2el_1,
	2,	/* Count of tags in the map */
	0, 0, 0,	/* Optional elements (not needed) */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_RestrictionClassAssignment = {
	"RestrictionClassAssignment",
	"RestrictionClassAssignment",
	&asn_OP_SEQUENCE,
	asn_DEF_RestrictionClassAssignment_tags_1,
	sizeof(asn_DEF_RestrictionClassAssignment_tags_1)
		/sizeof(asn_DEF_RestrictionClassAssignment_tags_1[0]), /* 1 */
	asn_DEF_RestrictionClassAssignment_tags_1,	/* Same as above */
	sizeof(asn_DEF_RestrictionClassAssignment_tags_1)
		/sizeof(asn_DEF_RestrictionClassAssignment_tags_1[0]), /* 1 */
	{ 0, 0, SEQUENCE_constraint },
	asn_MBR_RestrictionClassAssignment_1,
	2,	/* Elements count */
	&asn_SPC_RestrictionClassAssignment_specs_1	/* Additional specs */
};

