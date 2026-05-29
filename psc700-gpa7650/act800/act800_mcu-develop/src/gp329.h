#ifndef GP329_H
#define	GP329_H

#include <stdbool.h>

#ifdef	__cplusplus
extern "C" {
#endif


    void GP329_Initialize(void);
    bool gp329_enable(bool enable);
    bool gp329_poweron(void);
    bool gp329_poweroff(void);
    bool gp329_fw_update(void);
    void gp329_reset(void);

#ifdef	__cplusplus
}
#endif

#endif	/* GP329_H */

