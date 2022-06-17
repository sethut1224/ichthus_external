/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "../asn/J2735_201603DA_copyright_updated 12.2.ASN"
 * 	`asn1c -f compound-names -pdu=all`
 */

#ifndef	_TireData_H_
#define	_TireData_H_


#include <asn_application.h>

/* Including external dependencies */
#include "TireLocation.h"
#include "TirePressure.h"
#include "TireTemp.h"
#include "WheelSensorStatus.h"
#include "WheelEndElectFault.h"
#include "TireLeakageRate.h"
#include "TirePressureThresholdDetection.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* TireData */
typedef struct TireData {
	TireLocation_t	*location	/* OPTIONAL */;
	TirePressure_t	*pressure	/* OPTIONAL */;
	TireTemp_t	*temp	/* OPTIONAL */;
	WheelSensorStatus_t	*wheelSensorStatus	/* OPTIONAL */;
	WheelEndElectFault_t	*wheelEndElectFault	/* OPTIONAL */;
	TireLeakageRate_t	*leakageRate	/* OPTIONAL */;
	TirePressureThresholdDetection_t	*detection	/* OPTIONAL */;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} TireData_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_TireData;
extern asn_SEQUENCE_specifics_t asn_SPC_TireData_specs_1;
extern asn_TYPE_member_t asn_MBR_TireData_1[7];

#ifdef __cplusplus
}
#endif

#endif	/* _TireData_H_ */
#include <asn_internal.h>
