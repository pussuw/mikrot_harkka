/*
 * serial.h
 *
 *  Created on: 27.4.2014
 *      Author: Ville
 */

#ifndef SERIAL_H_
#define SERIAL_H_

void serial_init(void);
void serial_write(void * buf,
                  uint8_t len);
int16_t serial_read(void);

#endif /* SERIAL_H_ */
