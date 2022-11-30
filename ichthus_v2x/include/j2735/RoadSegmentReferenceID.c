/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "../asn/J2735_201603DA_copyright_updated 12.2.ASN"
 * 	`asn1c -fcompound-names -pdu=all`
 */

#include "RoadSegmentReferenceID.h"

asn_TYPE_member_t asn_MBR_RoadSegmentReferenceID_1[] = {
	{ ATF_POINTER, 1, offsetof(struct RoadSegmentReferenceID, region),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_RoadRegulatorID,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"region"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct RoadSegmentReferenceID, id),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_RoadSegmentID,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"id"
		},
};
static const int asn_MAP_RoadSegmentReferenceID_oms_1[] = { 0 };
static const ber_tlv_tag_t asn_DEF_RoadSegmentReferenceID_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_RoadSegmentReferenceID_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* region */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 } /* id */
};
asn_SEQUENCE_specifics_t asn_SPC_RoadSegmentReferenceID_specs_1 = {
	sizeof(struct RoadSegmentReferenceID),
	offsetof(struct RoadSegmentReferenceID, _asn_ctx),
	asn_MAP_RoadSegmentReferenceID_tag2el_1,
	2,	/* Count of tags in the map */
	asn_MAP_RoadSegmentReferenceID_oms_1,	/* Optional members */
	1, 0,	/* Root/Additions */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_RoadSegmentReferenceID = {
	"RoadSegmentReferenceID",
	"RoadSegmentReferenceID",
	&asn_OP_SEQUENCE,
	asn_DEF_RoadSegmentReferenceID_tags_1,
	sizeof(asn_DEF_RoadSegmentReferenceID_tags_1)
		/sizeof(asn_DEF_RoadSegmentReferenceID_tags_1[0]), /* 1 */
	asn_DEF_RoadSegmentReferenceID_tags_1,	/* Same as above */
	sizeof(asn_DEF_RoadSegmentReferenceID_tags_1)
		/sizeof(asn_DEF_RoadSegmentReferenceID_tags_1[0]), /* 1 */
	{ 0, 0, SEQUENCE_constraint },
	asn_MBR_RoadSegmentReferenceID_1,
	2,	/* Elements count */
	&asn_SPC_RoadSegmentReferenceID_specs_1	/* Additional specs */
};

