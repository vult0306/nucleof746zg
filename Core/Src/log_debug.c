/*
 * log_debug.c
 *
 *  Created on: May 24, 2021
 *      Author: vle
 */
#include <string.h>
#include "main.h"
#include "log_debug.h"


#if defined(__GNUC__)
int _write(int fd, char * ptr, int len)
{
    for (; len > 0; --len) {
        LL_USART_TransmitData8(USART3, *(ptr++));
        while (!LL_USART_IsActiveFlag_TXE(USART3)) {}
    }
    while (!LL_USART_IsActiveFlag_TC(USART3)) {}
    return len;
}
#endif