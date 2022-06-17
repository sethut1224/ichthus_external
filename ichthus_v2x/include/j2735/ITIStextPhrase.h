/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "../asn/J2735_201603DA_copyright_updated 12.2.ASN"
 * 	`asn1c -f compound-names -pdu=all`
 */

#ifndef	_ITIStextPhrase_H_
#define	_ITIStextPhrase_H_


#include <asn_application.h>

/* Including external dependencies */
#include <IA5String.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ITIStextPhrase */
typedef IA5String_t	 ITIStextPhrase_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_ITIStextPhrase_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_ITIStextPhrase;
asn_struct_free_f ITIStextPhrase_free;
asn_struct_print_f ITIStextPhrase_print;
asn_constr_check_f ITIStextPhrase_constraint;
ber_type_decoder_f ITIStextPhrase_decode_ber;
der_type_encoder_f ITIStextPhrase_encode_der;
xer_type_decoder_f ITIStextPhrase_decode_xer;
xer_type_encoder_f ITIStextPhrase_encode_xer;
oer_type_decoder_f ITIStextPhrase_decode_oer;
oer_type_encoder_f ITIStextPhrase_encode_oer;
per_type_decoder_f ITIStextPhrase_decode_uper;
per_type_encoder_f ITIStextPhrase_encode_uper;

#ifdef __cplusplus
}
#endif

#endif	/* _ITIStextPhrase_H_ */
#include <asn_internal.h>
