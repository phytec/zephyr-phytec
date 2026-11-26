#include <zephyr/kernel.h>

#ifndef COUNTER_HOUR_H
#define COUNTER_HOUR_H

struct counter_clock_t {
    int year;
    int month;
    int day;
    int hour;
    int minutes;
    int seconds;
};

struct counter_clock_t getClock(void);
void setClock(struct counter_clock_t received_clock);
void updateTime(void);
int days_in_month(int month, int year);

#endif /* COUNTER_HOUR_H */