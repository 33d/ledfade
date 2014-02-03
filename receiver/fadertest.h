#if !defined(FADERTEST_H)
#define FADERTEST_H

#include <CUnit/CUnit.h>

void testFaderSetColor();

CU_TestInfo faderTests[] = {
        { (char*) "faderSetColor", testFaderSetColor },
        CU_TEST_INFO_NULL
};

CU_SuiteInfo fadertestSuiteInfo = {
    (char*) "fadertest", NULL, NULL, faderTests,
};

#endif
