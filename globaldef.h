/*
 * globaldef.h
 *
 *  Created on: 27.4.2014
 *      Author: Ville
 */

#ifndef GLOBALDEF_H_
#define GLOBALDEF_H_

#ifndef __cplusplus
#undef false
#undef true
#undef bool
typedef enum {false, true} bool;
#endif

#define __enable_irq()          sei()
#define __disable_irq()         cli()

#endif /* GLOBALDEF_H_ */
