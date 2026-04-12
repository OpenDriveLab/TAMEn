#ifndef _COMMON_H_
#define _COMMON_H_

#define ASSERT_TRUE_OR_EXIT(expr, msg) \
    do { \
        if (!(expr)) \
        { \
            std::cout << (msg) << " fail:" << __LINE__ << std::endl; \
            std::flush(std::cout); \
            exit(EXIT_FAILURE); \
        } \
    } while (0)

#define ASSERT_TRUE_OR_LOG(expr, msg) \
    do { \
        if (!(expr)) \
        { \
            std::cout << (msg) << " fail:" << __LINE__ << std::endl; \
            std::flush(std::cout); \
        } \
    } while (0)

#endif // _COMMON_H_