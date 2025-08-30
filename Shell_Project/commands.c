
#include "Uart0.h"
// TBD: Reboots the microcontroller
void reboot(void);
{

}

// Displays process thread status
void ps()
{

}
void ipcs();
void kill(uint32_t pid);
void pkill(const char name[]);
void pi(bool on);
void preempt(bool on);
void sched(bool prio_on);
void pidof(const char name[]);
void run(const char name[]);