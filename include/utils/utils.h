#ifndef UTILS_H_
#define UTILS_H_

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <semaphore.h>

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#endif

#define UINT32_STR "4294967295"

#ifndef LOG_TAG
#define LOG_TAG "log"
#endif

#define ii(...) printf("\033[1;32m(ii)\033[0m [" LOG_TAG "] " __VA_ARGS__)
#define ww(...) printf("\033[1;33m(ww)\033[0m [" LOG_TAG "] " __VA_ARGS__)
#undef ee
#define ee(...) {\
	int errno__ = errno;\
	fprintf(stderr, "\033[1;31m(ee)\033[0m [" LOG_TAG "] " __VA_ARGS__);\
	fprintf(stderr, "\33[1;31m(ee)\033[0m [" LOG_TAG "] ^^ %s:%d | %s\n",\
	 __func__, __LINE__, __FILE__);\
	if (errno__) {\
		fprintf(stderr, "\033[1;31m(ee)\033[0m [" LOG_TAG "] %s (%d)\n",\
		 strerror(errno__), errno__);\
	}\
	errno = errno__;\
}

#define nop(...) ;

#define dd(...) {\
	printf("\033[0;33m(dd)\033[0m [" LOG_TAG "] " __VA_ARGS__);\
	printf("\033[0;33m(dd)\033[0m ^^ %s:%d | %s\n",\
	 __func__, __LINE__, __FILE__);\
}

#define unused_arg(a) __attribute__((unused)) a

static inline void sem_post_checked(sem_t *sem)
{
	int val = 1; /* to skip error checking */
	sem_getvalue(sem, &val);
	if (!val)
		sem_post(sem);
}

#endif // UTILS_H_
