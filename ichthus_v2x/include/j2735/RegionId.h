/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "../asn/J2735_201603DA_copyright_updated 12.2.ASN"
 * 	`asn1c -f compound-names -pdu=all`
 */

#ifndef	_RegionId_H_
#define	_RegionId_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeInteger.h>

#ifdef __cplusplus
extern "C" {
#endif

/* RegionId */
typedef long	 RegionId_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_RegionId_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_RegionId;
asn_struct_free_f RegionId_free;
asn_struct_print_f RegionId_print;
asn_constr_check_f RegionId_constraint;
ber_type_decoder_f RegionId_decode_ber;
der_type_encoder_f RegionId_encode_der;
xer_type_decoder_f RegionId_decode_xer;
xer_type_encoder_f RegionId_encode_xer;
oer_type_decoder_f RegionId_decode_oer;
oer_type_encoder_f RegionId_encode_oer;
per_type_decoder_f RegionId_decode_uper;
per_type_encoder_f RegionId_encode_uper;

#ifdef __cplusplus
}
#endif

#endif	/* _RegionId_H_ */
#include <asn_internal.h>