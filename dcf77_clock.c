/*
 * DCF77 decoder for the RaspberryPi
 * acting directly on GPIO, hand over time to NTP per SHM.
 * by  Sascha Reißner  reiszner@novaplan.at
 * 
 */

#define _SVID_SOURCE
#define _POSIX_C_SOURCE 200112L

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <fcntl.h>
#include <errno.h>
#include <signal.h>
#include <wiringPi.h>

#ifndef SYS_WINNT
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#else
#include <windows.h>
#include <iostream.h>
#define sleep(x) Sleep(x*1000)
#endif

// #define TOLERANCE_MICRO 40000000L

// defined in ntp.h
#define NTPD_BASE 0x4e545030
#define LEAP_NOWARNING	0x0	// normal, no leap second warning
#define LEAP_ADDSECOND	0x1	// last minute of day has 61 seconds
#define LEAP_DELSECOND	0x2	// last minute of day has 59 seconds
#define LEAP_NOTINSYNC	0x3	// overload, clock is free running

struct shmTime {
	int    mode;	// 0 - if valid set: use values, clear valid
					// 1 - if valid set: if count before and after read of values is equal, use values, clear valid
	int    count;
	time_t clockTimeStampSec;
	int    clockTimeStampUSec;
	time_t receiveTimeStampSec;
	int    receiveTimeStampUSec;
	int    leap;
	int    precision;
	int    nsamples;
	int    valid;
	int    dummy[10];
};

typedef struct {
	struct timespec time;
	struct timespec clock;
} time_info_t;

typedef struct {
	int8_t min;
	int8_t min_chk;
	int8_t hour;
	int8_t hour_chk;
	int8_t day;
	int8_t day_chk;
	int8_t wday;
	int8_t wday_chk;
	int8_t mon;
	int8_t mon_chk;
	int8_t year;
	int8_t year_chk;
	int8_t tz;
	int8_t tz_chk;
	int8_t dst;
	int8_t lsec;
	int8_t alert;
	int8_t check;
	int8_t stamp_chk;
	time_t stamp;
} dcf77_time;

typedef struct {
	char string[128];
	int block;
} dcf77_data;

typedef void (sigfunk) (int);

char *weekday[8] = {
	" --none-- ", "Monday    ", "Tuesday   ", "Wednesday ", "Thursday  ", "Friday    ", "Saturday  ", "Sunday    "
};

static int flag_debug = 0;
static int flag_run = 1;
static time_info_t sig_now;

void edge_sig (void) {
	clock_gettime (CLOCK_MONOTONIC_RAW, &sig_now.time);
	clock_gettime (CLOCK_REALTIME, &sig_now.clock);
}

static void quit (int signr) {
	flag_run = 0;
	return;
}


void write_bcd (char *data, int8_t num) {

	uint8_t i, high, low;

	low = num % 10;
	high = num / 10;

	for (i = 0 ; i < 4 ; i++) {
		data[i]   = low  & (1 << i) ? '1' : '0';
		data[i+4] = high & (1 << i) ? '1' : '0';
	}
}



void get_diff (struct timespec *diff, const time_info_t const *old, const time_info_t const *new, const long tolerance) {

	diff->tv_sec = new->time.tv_sec - old->time.tv_sec;
	diff->tv_nsec = (new->time.tv_nsec - old->time.tv_nsec) + tolerance;

	if (diff->tv_nsec >= 1000000000L) {
		diff->tv_sec++;
		diff->tv_nsec -= 1000000000L;
	}

	if (diff->tv_nsec < 0L) {
		diff->tv_sec--;
		diff->tv_nsec += 1000000000L;
	}
}



int check_tolerance (struct timespec *diff, const long sec, const long nsec, const long tolerance) {

//	printf ("check tolerance %ld %10ld <-> %ld %10ld (%ld) ... ", diff->tv_sec, diff->tv_nsec, sec, nsec, tolerance);

	if (diff->tv_sec == sec && diff->tv_nsec >= nsec && diff->tv_nsec <= (nsec + (2 * tolerance))) {
//		printf ("ok\n");
		return 1;
	}
//	printf ("fail\n");
	return 0;
}



int get_second (const struct timespec const *diff, const time_info_t const *min, const time_info_t const *sig, const long tolerance) {

	struct timespec sec_diff;
	int second = 0;

	if (min->time.tv_sec) {
		get_diff (&sec_diff, min, sig, tolerance);
		second = sec_diff.tv_sec;
	}

	return second;
}



// return 1 if parity is okay
int check_parity (int8_t *data, size_t count) {

	if (data[0] < 0) return 0;

	size_t i;
	int  parity = 0, fail = 0;

// can't check without parity
	if (data[count - 1] < 0) return 0;

// count one's and fail's
	for (i = 0 ; i < count - 1 ; i++) {
		if (data[i] < 0) fail++;
		else parity += data[i];
	}

// can't check with more then one fail's
	if (fail > 1) return 0;

// correct only one fail
	if (fail > 0) {
		for (i = 0 ; i < count ; i++) {
			if (data[i] < 0) {
				if ((data[i] = (parity + data[count]) % 2)) parity++;
				break;
			}
		}
	}

	if (parity % 2 == data[count - 1]) {
		if (fail) return 0;
		else return 1;
	}
	return -1;
}



// return the number if it is in possible range from start to end
// otherwise return -1

int check_number (int8_t *data, size_t count, int start, int end) {

	if (data[0] < 0) return -1;

	size_t i = 0;
	int number = 0;

	for (i = 0 ; i < count ; i++) {
		if (data[i] < 0) break;
		if (data[i] == 1) number += (1 << (i % 4));
	}

	if (i < count || number < start || number > end) return -1;
	return number;
}



int check_data_sync (int8_t *data) {

	int check = 0;

	if (data[0] == 0) check++;
	else if (data[0] == 1) check--;

	return check;
}



int check_data_tz (int8_t *data, int8_t *tz) {

	int check = -1;

	if      (data[17] == 0 && data[18] == 1) *tz = 1;
	else if (data[17] == 1 && data[18] == 0) *tz = 2;
	else *tz = -1;

	if (*tz > 0) check = 1;

	return check;
}



int check_data_time (int8_t *data) {

	int check;

	if (data[20] == 1) check = 1;
	else check = -1;

	return check;
}



void check_data_min (int8_t *data, int8_t *min) {
	if (check_parity (&data[21], 8) > 0) {
		*min = check_number (&data[21], 4, 0, 9);
		if (*min >= 0) *min += check_number (&data[25], 3, 0, 5) * 10;
		if (*min < 0 || *min > 59) *min = -1;
	}
}



void check_data_hour (int8_t *data, int8_t *hour) {
	if (check_parity (&data[29], 7) > 0) {
		*hour = check_number (&data[29], 4, 0, 9);
		if (*hour >= 0) *hour += check_number (&data[33], 2, 0, 2) * 10;
		if (*hour < 0 || *hour > 23) *hour = -1;
	}
}



void check_data_dst (int8_t *data, int8_t hour, int8_t *dst) {
	*dst = data[16];
	if (*dst == 1 && hour < 1 && hour > 4) *dst = 0;
}



void check_data_day (int8_t *data, int8_t *day) {
	*day = check_number (&data[36], 4, 0, 9);
	if (*day >= 0) *day += check_number (&data[40], 2, 0, 3) * 10;
	if (*day < 1 || *day > 31) *day = -1;
}



void check_data_wday (int8_t *data, int8_t *wday) {
	*wday = check_number (&data[42], 3, 1, 7);
}



void check_data_mon (int8_t *data, int8_t *mon) {
	*mon = check_number (&data[45], 4, 0, 9);
	if (*mon >= 0 && data[49] > 0) *mon += data[49] * 10;
	if (*mon < 1 || *mon > 12) *mon = -1;
}



void check_data_year (int8_t *data, int8_t *year) {
	*year = check_number (&data[50], 4, 0, 9);
	if (*year >= 0) *year += check_number (&data[54], 4, 0, 9) * 10;
	if (*year < 0 || *year > 99) *year = -1;
}



int check_data_date (int8_t *data) {

	int check = 0;

	check += check_parity (&data[36], 23);

	return check;
}



int check_data_lsec (int8_t *data, int8_t *lsec, int8_t day, int8_t mon) {

	*lsec = data[19];

	if (*lsec == 1) {
		if ((mon == 6 && day == 30) || (mon == 12 && day == 31) || (mon == 3 && day == 31) || (mon == 9 && day == 30)) {
			return 1;
		}
		else {
			*lsec = -5;
			return -1;
		}
	}
	if (*lsec == 0) return 1;

	return 0;
}



void output_time (dcf77_time *time) {
	printf("Date   : %s, ", time->wday > 0 ? weekday[time->wday] : "-- n/a -- ");
	if (time->day > 0) printf("%02d.", time->day);
	else  printf("--.");
	if (time->mon > 0) printf("%02d.", time->mon);
	else  printf("--.");
	if (time->year >= 0) printf("%4d ", 2000 + time->year);
	else  printf("---- ");
	if (time->hour >= 0) printf("%02d:", time->hour);
	else  printf("--:");
	if (time->min  >= 0) printf("%02d ", time->min);
	else  printf("-- ");
	if (time->tz > 0) printf ("%s", time->tz == 1 ? "CET" : "CEST");
	else printf ("---");
	if (time->stamp != 0)
		printf(" (Stamp: %ld / Confirm: %d)", time->stamp, time->stamp_chk);
	else
		printf ("\nConfirm:    %2d       %2d %2d  %2d  %2d %2d %2d", time->wday_chk, time->day_chk, time->mon_chk, time->year_chk, time->hour_chk, time->min_chk, time->tz_chk);
	printf("\n");

	if (time->dst  == 1) printf("DST change at end of this hour!\n");
	if (time->lsec  > 0) printf("Leap-Second at end of this day!\n");
	if (time->alert)     printf("DCF77-Transmitter set ALERT!\n");
}



void add_minute (dcf77_time *dcf, time_info_t *info, const int count) {

	if (info->time.tv_sec) {
		info->time.tv_sec += count * 60;
		info->clock.tv_sec += count * 60;
	}

	if (dcf->stamp) dcf->stamp += count * 60;

	if (dcf->min  >= 0) {
		dcf->min += count;
		if (dcf->min / 60) {

			if (dcf->hour >= 0) {
				dcf->hour += dcf->min / 60;
				if (dcf->hour / 24) {

					if (dcf->wday > 0) {
						dcf->wday += dcf->hour / 24;
						if (dcf->wday > 7) dcf->wday -= 7;
					}

					if (dcf->day >= 0) {
						dcf->day  += dcf->hour / 24;
						if (dcf->mon > 0) {
							if      (dcf->mon == 2 && dcf->year >= 0 && dcf->day > 28 && dcf->year % 4) { dcf->day = 1; dcf->mon++; }
							else if (dcf->mon == 2 && dcf->year >= 0 && dcf->day > 29 && dcf->year % 4 == 0) { dcf->day = 1; dcf->mon++; }
							else if ((dcf->mon == 4 || dcf->mon == 6 || dcf->mon == 9 || dcf->mon == 11) && dcf->day > 30) { dcf->day = 1; dcf->mon++; }
							else if ((dcf->mon == 1 || dcf->mon == 3 || dcf->mon == 5 || dcf->mon == 7 || dcf->mon == 8 || dcf->mon == 10 || dcf->mon == 12) && dcf->day > 31) { dcf->day = 1; dcf->mon++; }
							if (dcf->year >= 0) {
								if (dcf->mon > 12) {
									dcf->mon = 1;
									dcf->year++;
									if (dcf->year > 99) dcf->year = 0;
								}
							}
						}
					}
					dcf->hour %= 24;
				}
			}
			dcf->min %= 60;
		}
	}
}



void check_data (int8_t *data, dcf77_time *now, dcf77_time *last) {

	struct tm dcf_time;
	int check = 0;

	now->check = 0;

	now->check += check_data_sync (data);
	now->check += check_data_time (data);
	now->check += check_data_tz   (data, &now->tz);
	check_data_min  (data, &now->min);
	check_data_hour (data, &now->hour);
	check_data_dst  (data, now->hour, &now->dst);
	check_data_day  (data, &now->day);
	check_data_wday (data, &now->wday);
	check_data_mon  (data, &now->mon);
	check_data_year (data, &now->year);
	now->check += check_data_date (data);
	now->check += check_data_lsec (data, &now->lsec, now->day, now->mon);
	if (now->lsec == -5) now->lsec = 0;
	if (data[15] == 1) now->alert = 1;

	if (flag_debug) {
		printf ("--- Split ---\n");
		output_time (now);
	}

	if (last->stamp == 0) {

		if (last->min >= 0) {
			now->min_chk = last->min_chk;
			if (now->min >= 0) {
				if (now->min == (last->min + 1) % 60) now->min_chk++;
				else if (last->min_chk > 0) {
					now->min = (last->min + 1) % 60;
					now->min_chk--;
				}
			}
			else now->min = (last->min + 1) % 60;
		}

		if (now->min == 0 && last->hour >= 0) last->hour = (last->hour + 1) % 24;

		if (last->hour >= 0) {
			now->hour_chk = last->hour_chk;
			if (now->hour >= 0) {
				if (now->hour == last->hour) now->hour_chk++;
				else if (last->hour_chk > 0) {
					now->hour = last->hour;
					now->hour_chk--;
				}
			}
			else now->hour = last->hour;
		}

		if (last->day > 0) {
			now->day_chk = last->day_chk;
			if (now->day > 0) {
				if  (now->day == last->day) now->day_chk++;
				else if (last->day_chk > 0) {
					now->day = last->day;
					now->day_chk--;
				}
			}
			else now->day = last->day;
		}

		if (last->wday > 0) {
			now->wday_chk = last->wday_chk;
			if (now->wday > 0) {
				if (now->wday == last->wday) now->wday_chk++;
				else if (last->wday_chk > 0) {
					now->wday = last->wday;
					now->wday_chk--;
				}
			}
			else now->wday = last->wday;
		}

		if (last->mon > 0) {
			now->mon_chk = last->mon_chk;
			if (now->mon > 0) {
				if  (now->mon == last->mon) now->mon_chk++;
				else if (last->mon_chk > 0) {
					now->mon = last->mon;
					now->mon_chk--;
				}
			}
			else now->mon = last->mon;
		}

		if (last->year >= 0) {
			now->year_chk = last->year_chk;
			if (now->year >= 0) {
				if (now->year == last->year) now->year_chk++;
				else if (last->year_chk > 0) {
					now->year = last->year;
					now->year_chk--;
				}
			}
			else now->year = last->year;
		}

		if (last->tz > 0) {
			now->tz_chk = last->tz_chk;
			if (now->tz > 0) {
				if   (now->tz == last->tz) now->tz_chk++;
				else if (last->tz_chk > 0) {
					now->tz = last->tz;
					now->tz_chk--;
				}
			}
			else now->tz = last->tz;
		}

		if (now->min_chk > 1 && now->hour_chk > 1 && now->day_chk > 1 && now->wday_chk > 1 && now->mon_chk > 1 && now->year_chk > 1 && now->tz_chk > 1) {
			dcf_time.tm_sec = 0;
			dcf_time.tm_min = now->min;
			dcf_time.tm_hour = now->hour;
			dcf_time.tm_mday = now->day;
			dcf_time.tm_mon = now->mon - 1;
			dcf_time.tm_year = now->year + 100;
			dcf_time.tm_wday = 0;
			dcf_time.tm_yday = 0;
			dcf_time.tm_isdst = now->tz - 1;
			now->stamp = mktime (&dcf_time);
			localtime_r (&now->stamp, &dcf_time);
			if (dcf_time.tm_wday != now->wday % 7) now->stamp = 0;
		}
	}
	else {

		now->stamp = last->stamp + 60;
		now->stamp_chk = last->stamp_chk;
		localtime_r (&now->stamp, &dcf_time);

		if (now->min != dcf_time.tm_min) {
			if (now->min >= 0) check++;
			now->min = dcf_time.tm_min;
		}

		if (now->hour != dcf_time.tm_hour) {
			if (now->hour >= 0) check++;
			now->hour = dcf_time.tm_hour;
		}

		if (now->day != dcf_time.tm_mday) {
			if (now->day > 0) check++;
			now->day = dcf_time.tm_mday;
		}

		if (now->mon != dcf_time.tm_mon + 1) {
			if (now->mon > 0) check++;
			now->mon = dcf_time.tm_mon + 1;
		}

		if (now->year != dcf_time.tm_year - 100) {
			if (now->year >= 0) check++;
			now->year = dcf_time.tm_year - 100;
		}

		if (now->wday % 7 != dcf_time.tm_wday) {
			if (now->wday > 0) check++;
			now->wday = dcf_time.tm_wday;
			if (now->wday == 0) now->wday = 7;
		}

		if (now->tz != dcf_time.tm_isdst + 1) {
			if (now->tz >= 0) check++;
			now->tz = last->tz;
		}

		if (now->min == 1) {
			last->dst = 0;
			last->lsec = 0;
		}

		if (now->dst) {
			last->dst += now->dst;
		}

		if (now->lsec) {
			last->lsec += now->lsec;
		}

		if (check) now->stamp_chk--;
		else now->stamp_chk++;

		if (now->stamp_chk > 10) now->stamp_chk = 10;
		if (now->stamp_chk < 0) {
			now->min_chk = 1;
			now->hour_chk = 1;
			now->tz_chk = 1;
			now->day_chk = 1;
			now->mon_chk = 1;
			now->wday_chk = 1;
			now->year_chk = 1;
			now->stamp = 0;
		}
	}
}



void init_dcf77_time (dcf77_time *time) {
	time->min = -2;
	time->min_chk = 0;
	time->hour = -2;
	time->hour_chk = 0;
	time->day = -2;
	time->day_chk = 0;
	time->wday = -2;
	time->wday_chk = 0;
	time->mon = -2;
	time->mon_chk = 0;
	time->year = -2;
	time->year_chk = 0;
	time->tz = -2;
	time->tz_chk = 0;
	time->dst = -2;
	time->check = -50;
	time->lsec = 0;
	time->alert = 0;
	time->stamp = 0;
	time->stamp_chk = 0;
}

void init_dcf77_data (dcf77_data *data) {
	memset (data->string, '\0', 128);
	data->block = 0;
}

void init_time_info (time_info_t *info) {
	info->time.tv_sec = 0;
	info->time.tv_nsec = 0;
	info->clock.tv_sec = 0;
	info->clock.tv_nsec = 0;
}



void gather_data (dcf77_data *data, const int8_t *clock_data, const dcf77_time *time, const char *fifo_name) {

	int i, fifo;

	if (strlen(fifo_name) == 0) return;
	if (time->stamp == 0 || time->tz < 0 || time->wday < 0) return;

	data->block = time->min % 3;
	if (data->block == 0) init_dcf77_data (data);

	for (i = 0 ; i < 14 ; i++) data->string[data->block * 14 + i] = '0' + clock_data[i + 1];

	if (data->block == 2) {
		write_bcd (&data->string[42], time->min);
		write_bcd (&data->string[50], time->hour);
		write_bcd (&data->string[58], time->day);
		write_bcd (&data->string[66], time->mon);
		write_bcd (&data->string[71], time->wday);
		write_bcd (&data->string[74], time->year);
		data->string[82] = '+';

		if (time->tz > 0)
			data->string[83] = '0' + time->tz;
		else
			data->string[83] = '0';

		data->string[84] = '\n';
		data->string[85] = '\0';
	}

	if (flag_debug) {
		for (i = 0 ; i < 82 ; i++) {
			if (i == 14 || i == 28 || i == 42) printf (" ");
			if (data->string[i] == '\0') printf ("_");
			else printf ("%c", data->string[i]);
		}
		printf ("\n");
		fflush (stdout);
	}

	if (data->block == 2) {
		if (data->string[0] && data->string[14] && data->string[28]) {
			if ((fifo = open (fifo_name, O_WRONLY | O_NONBLOCK)) >= 0) {
				write(fifo, data->string, strlen(data->string));
				close (fifo);
			}
		}
		data->string[0] = '\0';
	}
}



static volatile struct shmTime *getShmTime (int unit) {

	int shmid;
	volatile struct shmTime *ntp_shm;

	shmid = shmget (NTPD_BASE + unit, sizeof (struct shmTime), IPC_CREAT | 0777);
	if (shmid == -1) return NULL;

	ntp_shm = (struct shmTime *) shmat (shmid, NULL, 0);
	if (ntp_shm == (void *) -1) return NULL;

	ntp_shm->valid = 0;
	ntp_shm->mode = 1;
	ntp_shm->count = 0;

	return ntp_shm;

}



void set_ntp_shm (volatile struct shmTime *ntp_shm, const dcf77_time *now, const long min_dev, const long sig_avr) {

	static int precision = 5 * 16;

	int prec;
	long tmp = min_dev < 0 ? -min_dev : min_dev;

	if      (tmp <       950) prec = 20 * 16;
	else if (tmp <      1900) prec = 19 * 16;
	else if (tmp <      3800) prec = 18 * 16;
	else if (tmp <      7625) prec = 17 * 16;
	else if (tmp <     15250) prec = 16 * 16;
	else if (tmp <     30500) prec = 15 * 16;
	else if (tmp <     61025) prec = 14 * 16;
	else if (tmp <    122050) prec = 13 * 16;
	else if (tmp <    244125) prec = 12 * 16;
	else if (tmp <    488250) prec = 11 * 16;
	else if (tmp <    976500) prec = 10 * 16;
	else if (tmp <   1953125) prec =  9 * 16;
	else if (tmp <   3906250) prec =  8 * 16;
	else if (tmp <   7812500) prec =  7 * 16;
	else if (tmp <  15625000) prec =  6 * 16;
	else                      prec =  5 * 16;

	if (prec > precision) precision++;
	if (prec < precision) precision -= 2;

	if (flag_debug) {
		printf ("Prec_now : %d\n", prec);
		printf ("Precision: %d (%d)\n", precision, -(precision >> 4));
	}

	ntp_shm->valid = 0;

	ntp_shm->clockTimeStampSec = now->stamp;
	ntp_shm->clockTimeStampUSec = 0;

/*
	if (sig_avr < 0L) {
		ntp_shm->clockTimeStampUSec = (-sig_avr) / 1000;
	}
	else
		ntp_shm->clockTimeStampSec--;
		ntp_shm->clockTimeStampUSec = 1000000 - (sig_avr / 1000);
*/

	ntp_shm->receiveTimeStampSec = sig_now.clock.tv_sec;
	ntp_shm->receiveTimeStampUSec = sig_now.clock.tv_nsec / 1000;

/*
	if (min_dev < 0) {
		ntp_shm->receiveTimeStampUSec += (tmp / 1000);
		if (ntp_shm->receiveTimeStampUSec > 1000000) {
			ntp_shm->receiveTimeStampSec++;
			ntp_shm->receiveTimeStampUSec -= 1000000;
		}
	}
	else {
		if (ntp_shm->receiveTimeStampUSec < (tmp / 1000)) {
			ntp_shm->receiveTimeStampSec--;
			ntp_shm->receiveTimeStampUSec += 1000000;
		}
		ntp_shm->receiveTimeStampUSec -= (tmp / 1000);
	}
*/

	ntp_shm->precision = -(precision >> 4);

	if (now->lsec > 0)
		ntp_shm->leap = LEAP_ADDSECOND;
	else
		ntp_shm->leap = LEAP_NOWARNING;

	ntp_shm->count++;
	ntp_shm->valid = 1;
}



sigfunk *signal (int sig_nr, sigfunk signalhandler) {
	struct sigaction neu_sig, alt_sig;
	neu_sig.sa_handler = signalhandler;
	sigemptyset (&neu_sig.sa_mask);
	neu_sig.sa_flags = SA_RESTART;
	if (sigaction (sig_nr, &neu_sig, &alt_sig) < 0)
		return SIG_ERR;
	return alt_sig.sa_handler;
}



static void start_daemon (void) {

	int i;
	pid_t pid;

	if ((pid = fork ()) != 0) exit (EXIT_SUCCESS);
	if (setsid() < 0) {
		fprintf (stderr, "can't set sessionID. exit.\n");
		exit (EXIT_FAILURE);
	}

/* close all open filedescriptors */
	for (i = sysconf (_SC_OPEN_MAX); i > 0; i--) close (i);

}



int main (int argc, char *argv[])
{

	time_info_t min_last, sec_last, sig_last;
	struct timespec diff;
	int sec_cnt = 0, min_cnt = 0, edge_dir = 0, unit = -1, gpio[2] = {-1, -1}, sig_cnt = 0, noise, i, j;
	int8_t data[60];
	long min_dev = 0, tolerance = 25000000L, sig_stat[60], sig_avr;
	char fifo_name[256] = "";
	static volatile struct shmTime *ntp_shm = NULL;

	unsigned int sig_short = 0, sig_long = 0;

	while ((i = getopt (argc, argv, "g:Dhu:f:t:")) != -1) {
		switch (i) {

			case 'h':
				fprintf (stderr, "Usage: %s [-h] [-D] -g <pin> [-g <pin>] [-u <num>] [-f <name>] [-t <msec>]\n", argv[0]);
				fprintf (stderr, "    -h          this helptext\n");
				fprintf (stderr, "    -D          debbuging (don't fork to background)\n");
				fprintf (stderr, "    -g <pin>    GPIO-pin (or pins) that is connection to the receiver\n");
				fprintf (stderr, "    -u <num>    unit-number of NTP shared memory driver\n");
				fprintf (stderr, "    -f <name>   fifoname to send additional data (bit 1 to 14)\n");
				fprintf (stderr, "    -t <msec>   tolerance in milliseconds (default: 25)\n");
				return EXIT_FAILURE;

			case 'D':
				flag_debug = 1;
				break;

			case 'g':
				if (gpio[0] >= 0) {
					if (gpio[1] >= 0) gpio[0] = gpio[1];
					gpio[1] = atoi (optarg);
				}
				else {
					gpio[0] = atoi (optarg);
				}
				break;

			case 'u':
				unit = atoi (optarg);
				break;

			case 'f':
				strncpy (fifo_name, optarg, 255);
				break;

			case 't':
				tolerance = strtol (optarg, NULL, 10);
				if (tolerance < 5) {
					fprintf(stderr, "Tolerance can't be lower then 5! set it to 5.\n");
					tolerance = 5;
				}
				if (tolerance > 40) {
					fprintf(stderr, "Tolerance can't be greater then 40! set it to 40.\n");
					tolerance = 40;
				}
				tolerance *= 1000000;
				break;
				
			default:
				fprintf(stderr, "Unknown option '%s'! igrore it.\n", optarg);
				fprintf(stderr, "See '%s -h' for more information.\n", argv[0]);
				break;

		}
	}

	if (gpio[0] < 0) {
		fprintf (stderr, "no GPIO-pin given! exit.\n");
		return EXIT_FAILURE;
	}

	wiringPiSetup();

// fork to background
	if (flag_debug == 0) start_daemon();

	signal (SIGINT, quit);
	signal (SIGQUIT, quit);
	signal (SIGTERM, quit);

	if (unit >= 0) {
		if  ((ntp_shm = getShmTime(unit)) == NULL) {
			fprintf (stderr, "Can't attach shared memory with unit %d!\n", i);
			return EXIT_FAILURE;
		}
	}

	setenv("TZ", ":Europe/Berlin", 1);

	dcf77_time time_last, time_now;
	init_dcf77_time (&time_last);
	init_dcf77_time (&time_now);

	dcf77_data block_data;
	init_dcf77_data (&block_data);

	for (i = 0 ; i < 60 ; i++) data[i] = -1;
	for (i = 0 ; i < 60 ; i++) sig_stat[i] = 0;

	init_time_info (&sig_now);
	init_time_info (&sig_last);
	init_time_info (&sec_last);
	init_time_info (&min_last);

	pinMode(gpio[0], INPUT);
	pullUpDnControl(gpio[0], PUD_UP);

	if (gpio[1] >= 0) {
		pinMode(gpio[1], INPUT);
		pullUpDnControl(gpio[1], PUD_UP);
		wiringPiISR(gpio[0], INT_EDGE_RISING, &edge_sig);
		wiringPiISR(gpio[1], INT_EDGE_RISING, &edge_sig);
	}
	else {
		wiringPiISR(gpio[0], INT_EDGE_BOTH, &edge_sig);
	}

	while (flag_run) {

		if (sig_now.time.tv_nsec != sig_last.time.tv_nsec || sig_now.time.tv_sec != sig_last.time.tv_sec) {

			if (edge_dir != 0) {

				get_diff (&diff, &sec_last, &sig_now, tolerance);





// check for second marker
				if (diff.tv_sec && check_tolerance (&diff, diff.tv_sec, 0L, tolerance)) {

// store data
					if (sig_short && sig_long == 0) data[sec_cnt] = 0;
					if (sig_short == 0 && sig_long) data[sec_cnt] = 1;
					if (sig_short && sig_long && sig_short < sig_long) data[sec_cnt] = 0;
					if (sig_short && sig_long && sig_short > sig_long) data[sec_cnt] = 1;
					sig_short = 0;
					sig_long = 0;

// calculate starting second
					if (min_last.time.tv_sec)
						sec_cnt = get_second (&diff, &min_last, &sig_now, tolerance);
					else
						sec_cnt += diff.tv_sec;

// check more then a minute
					if (sec_cnt > 59 && diff.tv_sec != 2) {
						min_cnt++;
						for (i = 0 ; i < 60 ; i++) data[i] = -1;

						if (min_cnt > 2) {
							printf ("search for new minute start...\n");
							init_time_info (&min_last);
							init_dcf77_time (&time_last);
							min_cnt = 0;
						}
						else {
							add_minute (&time_last, &min_last, sec_cnt / 60);
						}
						sec_cnt -= (sec_cnt / 60) * 60;
					}

//					memcpy (&sec_last, &sig_now, sizeof(sig_now));
					sec_last.time.tv_sec++;
					sec_last.clock.tv_sec++;

					if (flag_debug) {
						long signal = diff.tv_nsec - tolerance;
						if (signal < 0) signal = -signal;
						signal = (tolerance - signal) / (tolerance / 100);
						printf ("= -> Dev: %+12.6lf msec / Signal: %ld%%\n", 0.000001 * (diff.tv_nsec - tolerance), signal);
						if (min_last.time.tv_sec)
							printf ("Sec: %02d\n", sec_cnt);
						else
							printf ("Sec: --\n");
					}

// gather data
					if (sec_cnt > 14 && fifo_name[0] != '\0' && time_last.stamp && block_data.string[(time_last.min % 3) * 14] == '\0')
						gather_data (&block_data, data, &time_last, fifo_name);

// check for minute marker
					if (min_last.time.tv_sec == 0 && diff.tv_sec == 2) {
						memcpy (&min_last, &sig_now, sizeof(sig_now));
						min_last.time.tv_sec -= 60;
						min_last.clock.tv_sec -= 60;
						if (sec_cnt < 59) {
							for (i = 58 ; i >= 0 && data[i] == -1 ; i--);
							for (j = 58 ; i >= 0 ; j--, i--) data[j] = data[i];
							for (; j >= 0 ; j--) data[j] = -1;
						}
					}

// check minute
					if (min_last.time.tv_sec && diff.tv_sec == 2) {
						get_diff (&diff, &min_last, &sig_now, tolerance);
						if (diff.tv_sec == 60) {

							if (flag_debug) {
								printf("Minute-Data:\n");
								for (i = 0 ; i < 60 ; i++) {
									if ((i % 10) == 0) printf("%02d: ", i);
									printf("%2d ", data[i]);
									if ((i % 10) == 9) printf("\n");
									else printf(" ");
								}
								printf ("--- Last ---\n");
								output_time (&time_last);
							}

							min_dev = ((min_dev * 15) + (diff.tv_nsec - tolerance)) / 16;
							check_data (data, &time_now, &time_last);
							for (i = 0 ; i < 60 ; i++) data[i] = -1;

							if (flag_debug) {
								printf ("--- Now ---\n");
								output_time (&time_now);
								printf ("Average Minute Deviation: %+12.6lf msec\n", 0.000001 * min_dev);
								printf ("Average Signal Deviation: %+12.6lf msec\n", 0.000001 * sig_avr);
								printf("Minute Start Stamp: %10ld.%09ld\n", sig_now.time.tv_sec, sig_now.time.tv_nsec);
								printf ("Sec: 00\n");
							}

							if (time_last.stamp == 0 && time_now.stamp) init_dcf77_data (&block_data);

							memcpy (&time_last, &time_now, sizeof(time_now));
							memcpy (&min_last, &sig_now, sizeof(sig_now));
							memcpy (&sec_last, &sig_now, sizeof(sig_now));
							min_cnt = 0;
							sec_cnt = 0;

							if (time_now.stamp) {
								if ((sig_now.clock.tv_sec + 1200) < (time_now.stamp - time_now.tz * 3600)) {
									if (flag_debug) printf ("Systemclock is more then 20 minutes off time. Set it hard!\n");
									sig_now.clock.tv_sec = time_now.stamp - (time_now.tz * 3600);
									if (sig_avr < 0) {
										sig_now.clock.tv_sec--;
										sig_now.clock.tv_nsec = 1000000000L + sig_avr;
									}
									else {
										sig_now.clock.tv_nsec = sig_avr;
									}
//									clock_settime (CLOCK_REALTIME, &sig_now.clock);
								}
								else if (ntp_shm) {
									set_ntp_shm (ntp_shm, &time_now, min_dev, sig_avr);
								}
							}

							init_dcf77_time (&time_now);
						}
					}

					noise--;
				}


// short signal == binary 0
				else if (check_tolerance (&diff, 0, 100000000L + sig_avr, tolerance)) {
					sig_short++;
					sig_stat[sig_cnt] = diff.tv_nsec - tolerance - 100000000L;
					sig_avr = 0;
					for (i = 0 ; i < 60 ; i++) sig_avr += sig_stat[i];
					sig_avr /= 60;

					if (flag_debug) {
						long signal = sig_stat[sig_cnt] - sig_avr;
						if (signal < 0) signal = -signal;
						signal = (tolerance - signal) / (tolerance / 100);
						printf ("0 -> Dev: %+12.6lf msec / Signal: %ld%%\n", 0.000001 * ((diff.tv_nsec - tolerance - 100000000L) - sig_avr), signal);
					}

					sig_cnt++;
					if (sig_cnt >= 60) sig_cnt = 0;
					noise--;
				}

// long signal == binary 1
				else if (check_tolerance (&diff, 0, 200000000L + sig_avr, tolerance)) {
					sig_long++;
					sig_stat[sig_cnt] = diff.tv_nsec - tolerance - 200000000L;
					sig_avr = 0;
					for (i = 0 ; i < 60 ; i++) sig_avr += sig_stat[i];
					sig_avr /= 60;

					if (flag_debug) {
						long signal = sig_stat[sig_cnt] - sig_avr;
						if (signal < 0) signal = -signal;
						signal = (tolerance - signal) / (tolerance / 100);
						printf ("1 -> Dev: %+12.6lf msec / Signal: %ld%%\n", 0.000001 * ((diff.tv_nsec - tolerance - 200000000L) - sig_avr), signal);
					}

					sig_cnt++;
					if (sig_cnt >= 60) sig_cnt = 0;
					noise--;
				}

// noise
				else {
					if (diff.tv_sec) {
// store data
						if (sig_short && sig_long == 0) data[sec_cnt] = 0;
						if (sig_short == 0 && sig_long) data[sec_cnt] = 1;
						if (sig_short && sig_long && sig_short < sig_long) data[sec_cnt] = 0;
						if (sig_short && sig_long && sig_short > sig_long) data[sec_cnt] = 1;
						sig_short = 0;
						sig_long = 0;

						sec_last.time.tv_sec += diff.tv_sec;
						sec_last.clock.tv_sec += diff.tv_sec;
						sec_cnt += diff.tv_sec;

						if (sec_cnt > 59) {
							min_cnt++;
							for (i = 0 ; i < 60 ; i++) data[i] = -1;

							if (min_cnt > 2) {
								printf ("search for new minute start...\n");
								init_time_info (&min_last);
								init_dcf77_time (&time_last);
								min_cnt = 0;
							}
							else {
								add_minute (&time_last, &min_last, sec_cnt / 60);
							}
							sec_cnt -= (sec_cnt / 60) * 60;
						}
						if (flag_debug) {
							if (min_last.time.tv_sec) printf ("Sec: %02d ?\n", sec_cnt);
							else printf ("Sec: -- ?\n");
						}
					}
					if (flag_debug) printf ("---- Dev: %+12.6lf msec\n", 0.000001 * (diff.tv_nsec - tolerance));
					noise++;
				}

				if (noise < 0) noise = 0;
				if (noise > 9) edge_dir = 0;
			}

// syncing
			else {
				init_dcf77_time (&time_last);
				init_dcf77_time (&time_now);
				init_dcf77_data (&block_data);
				for (i = 0 ; i < 60 ; i++) data[i] = -1;
				for (i = 0 ; i < 60 ; i++) sig_stat[i] = 0;
				init_time_info (&sec_last);
				init_time_info (&min_last);
				sig_short = 0;
				sig_long = 0;
				sig_cnt = 0;
				sig_avr = 0;
				min_cnt = 0;
				sec_cnt = 0;
				noise = 0;

				get_diff (&diff, &sig_last, &sig_now, tolerance);

				if (check_tolerance (&diff, 0, 100000000L, tolerance)) {
					edge_dir = -1;
					sig_short++;
					memcpy (&sec_last, &sig_last, sizeof(sig_last));
					if (flag_debug) printf("found falling edge\n");					
				}
				if (check_tolerance (&diff, 0, 200000000L, tolerance)) {
					edge_dir = -1;
					sig_long++;
					memcpy (&sec_last, &sig_last, sizeof(sig_last));
					if (flag_debug) printf("found falling edge\n");					
				}
				if (check_tolerance (&diff, 0, 800000000L, tolerance) || check_tolerance (&diff, 0, 900000000L, tolerance)) {
					edge_dir =  1;
					memcpy (&sec_last, &sig_now, sizeof(sig_now));
					if (flag_debug) printf("found rising edge\n");
				}
				if (check_tolerance (&diff, 1, 800000000L, tolerance) || check_tolerance (&diff, 1, 900000000L, tolerance)) {
					edge_dir =  1;
					memcpy (&sec_last, &sig_now, sizeof(sig_now));
					memcpy (&min_last, &sig_now, sizeof(sig_now));
					if (flag_debug) printf("found rising edge\n");
				}
				if (edge_dir == 0 && flag_debug) printf("syncing...\n");
			}
			memcpy (&sig_last, &sig_now, sizeof(sig_now));
			fflush (stdout);
		}

		delay(10);
	}

	if (ntp_shm) shmdt ((void *) ntp_shm);

	return 0;
}

