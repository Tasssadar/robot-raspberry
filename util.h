#ifndef UTIL_H
#define UTIL_H


#define LOGD(fmt, args...) printf("D: %s: " fmt"\n", __PRETTY_FUNCTION__, ##args)
#define LOGE(fmt, args...) fprintf(stderr, "E: %s: " fmt"\n", __PRETTY_FUNCTION__, ##args)

#endif
