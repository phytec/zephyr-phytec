#include "counter_hour.h"
static struct counter_clock_t counter_clock;

static int64_t time_start;
static int64_t time_now;

static bool init_time = false;

struct counter_clock_t getClock(){
    return counter_clock;
}

void setClock(struct counter_clock_t received_clock){
    counter_clock.year = received_clock.year;
    counter_clock.month = received_clock.month;
    counter_clock.day = received_clock.day;
    counter_clock.hour = received_clock.hour;
    counter_clock.minutes = received_clock.minutes;
    counter_clock.seconds = 0;
}

void updateTime(){
    if(!init_time){
        time_start = k_uptime_get();
        init_time=true;
    }
    time_now = k_uptime_get();

    if(time_now-time_start>=1000){
        counter_clock.seconds++;
        time_start=time_now;
    }

    if(counter_clock.seconds >= 60){
        counter_clock.seconds = 0;
        counter_clock.minutes++;

        if(counter_clock.minutes >= 60){
            counter_clock.minutes = 0;
            counter_clock.hour++;

            if(counter_clock.hour >= 24){
                counter_clock.hour = 0;
                counter_clock.day++;

                if(counter_clock.day > days_in_month(counter_clock.month+1,counter_clock.year)){
                    counter_clock.day = 1;

                    if(counter_clock.month == 11){
                        counter_clock.month = 0;
                        counter_clock.year++;
                    }else{
                        counter_clock.month++;
                    }
                }
            }
        }
    }
}

int days_in_month(int month, int year){
    static const int days[] = {31,28,31,30,31,30,31,31,30,31,30,31};
    if (month == 2 && (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0))) {
        return 29;
    }
    return days[month - 1];
}