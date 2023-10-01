/* 
 * File:   bootload.h
 * Author: tenda
 *
 * Created on 2023/09/17, 10:07
 */

#ifndef BOOTLOAD_H
#define	BOOTLOAD_H

#ifdef	__cplusplus
extern "C" {
#endif
void manualboot(void);
void monitor(int monitor_mode);

#ifdef	__cplusplus
}
#endif

#endif	/* BOOTLOAD_H */

