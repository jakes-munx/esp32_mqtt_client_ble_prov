#ifndef __APP_GLOBALS_H
#define __APP_GLOBALS_H


#define FOREACH(item, array) \
    for (int keep = 1, \
             count = 0,\
             size = sizeof (array) / sizeof *(array); \
         keep && count != size; \
         keep = !keep, count++) \
            for (item = (array) + count; keep; keep = !keep) 

#define SEND_TELEM_MAC  1

#define CERTS_NAMESPACE "certs"
#define SEND_TELEMTRY_PERIOD_SECONDS    10


#endif // __APP_GLOBALS_H