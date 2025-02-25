#ifndef TEST_FRAMEWORK_H
#define TEST_FRAMEWORK_H
#include <stdio.h>
#include <stdlib.h>
int fnmatch(const char *pattern, const char *string, int flags);
typedef void (*TestFunc)();
typedef struct Test {
    char *name;
    TestFunc func;
    struct Test *next;
} Test;
extern Test *test_list;
extern const char *current_test;
void register_test(char *name, TestFunc func);
#define TEST(testname) \
    static void test_##testname(void); \
    static void register_##testname(void) __attribute__((constructor)); \
    static void register_##testname(void){ register_test(#testname, test_##testname); } \
    static void test_##testname(void)
void record_failure(const char *test_name, const char *expr, const char *file, int line);
#define ASSERT(expr) do { if(!(expr)) { record_failure(current_test, #expr, __FILE__, __LINE__); } } while(0)
#define ASSERT_EQ(a,b) do { if((a)!=(b)) { record_failure(current_test, #a " == " #b, __FILE__, __LINE__); } } while(0)
#include <math.h>
#define EXPECT_APPROX_EQ(got, expected, tol) \
	do { \
		if (fabsf((got) - (expected)) > (tol)) { \
			record_failure(current_test, #got " approximately equals " #expected, __FILE__, __LINE__); \
		} \
	} while(0)
#endif