/*
 * Mymc_type.h
 *
 *  Created on: Jul 15, 2021
 *      Author: fenglimin1
 */

#ifndef INC_MYMC_TYPE_H_
#define INC_MYMC_TYPE_H_

#include <stdint.h>
#include <stdbool.h>
 /* @brief Two components q, d type definition
  */
#define MAX(a,b) (((a)>(b))?(a):(b))
#define MIN(a,b) (((a)<(b))?(a):(b))
#define LIMIT(u, ulim, llim)    (((u)>=(ulim))? (ulim) : (((u)<=(llim))? (llim) : (u)))
#define SIGN(u)                 ((u)>(0)?(1):((u)<(0)?(-1):(0)))
#define ABS( u )                (((u)>=0)  ? (u) : -(u))
typedef struct
{
  float q;
  float d;
} qd_t;
/**
  * @brief Two components a,b type definition
  */
typedef struct
{
  float a;
  float b;
  float c;
} abc_t;
/**
  * @brief Two components alpha, beta type definition
  */
typedef struct
{
  float alpha;
  float beta;
} alphabeta_t;

typedef struct
{
  abc_t Iabc;         /**< @brief Stator current on stator reference frame abc */
  alphabeta_t Ialphabeta;  /**< @brief Stator current on stator reference frame alfa-beta*/
  qd_t IqdHF;       /**< @brief Stator current on stator reference frame alfa-beta*/
  qd_t Iqd;         /**< @brief Stator current on rotor reference frame qd */
  qd_t Iqdref;      /**< @brief Stator current on rotor reference frame qd */
  int16_t UserIdref;           /**< @brief User value for the Idref stator current */
  qd_t Vqd;         /**< @brief Phase voltage on rotor reference frame qd */
  alphabeta_t Valphabeta;  /**< @brief Phase voltage on stator reference frame alpha-beta*/
  int16_t hTeref;              /**< @brief Reference torque */
  int16_t hElAngle;            /**< @brief Electrical angle used for reference frame transformation  */
  uint16_t hCodeError;         /**< @brief error message */

} FOCVars_t;

#endif /* INC_MYMC_TYPE_H_ */
