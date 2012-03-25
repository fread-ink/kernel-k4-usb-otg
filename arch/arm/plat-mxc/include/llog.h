/* TODO: Finish this or obtain source file from Amazon */

#define _LLOG_LEVEL_DEFAULT 0
#define LLOG_LEVEL_DEBUG0 (1 << 0)
#define LLOG_LEVEL_DEBUG1 (1 << 1)
#define LLOG_LEVEL_DEBUG2 (1 << 2)
#define LLOG_LEVEL_DEBUG3 (1 << 3)
#define LLOG_LEVEL_DEBUG4 (1 << 4)
#define LLOG_LEVEL_DEBUG5 (1 << 5)
#define LLOG_LEVEL_DEBUG6 (1 << 6)
#define LLOG_LEVEL_DEBUG7 (1 << 7)
#define LLOG_LEVEL_DEBUG8 (1 << 8)
#define LLOG_LEVEL_DEBUG9 (1 << 9)

#define LLOG_LEVEL_PERF 0
#define LLOG_LEVEL_INFO 0
#define LLOG_LEVEL_WARN 0
#define LLOG_LEVEL_ERROR 0
#define LLOG_LEVEL_CRIT 0

#define LLOG_LEVEL_MASK_ALL 0
#define LLOG_LEVEL_MASK_INFO 0
#define LLOG_LEVEL_MASK_EXCEPTIONS 0

#define LLOG_INIT() {}

#define LLOG_DEBUG0(msg, ...) { }
#define LLOG_DEBUG1(msg, ...) { }
#define LLOG_DEBUG2(msg, ...) { }
#define LLOG_DEBUG3(msg, ...) { }
#define LLOG_DEBUG4(msg, ...) { }
#define LLOG_DEBUG5(msg, ...) { }

#define LLOG_INFO(location, foo, msg, ...) { }
#define LLOG_ERROR(location, foo, msg, ...) { }
#define LLOG_WARN(location, foo, msg, ...) { }
