#ifndef UTIL_H
#define UTIL_H

#define LOGD(fmt, args...) fprintf(stdout, "%s D: %s: " fmt"\n", Util::getLogTime(), __PRETTY_FUNCTION__, ##args)
#define LOGE(fmt, args...) fprintf(stderr, "%s E: %s: " fmt"\n", Util::getLogTime(), __PRETTY_FUNCTION__, ##args)

class Util
{
public:
    static char *getLogTime();
};

#endif
