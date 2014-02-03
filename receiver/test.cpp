#include <CUnit/CUnit.h>
#include <CUnit/Basic.h>
#include <CUnit/Console.h>
#include <stdio.h>

#include "fadertest.h"

int main() {
  CU_pSuite suite = NULL;

  CU_initialize_registry();

/*
  CU_register_nsuites(1,
    fadertestSuiteInfo
  );
*/
  suite = CU_add_suite("fadertest", NULL, NULL);
  CU_add_test(suite, "faderSetColor", testFaderSetColor);

  CU_basic_set_mode(CU_BRM_VERBOSE);
  CU_basic_run_tests();

  CU_cleanup_registry();
  return CU_get_error();
}

