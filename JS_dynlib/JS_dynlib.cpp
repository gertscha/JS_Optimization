#include "JS_dynlib.h"

#include "loguru.hpp"
#include "JS_Lib.h"

#include <iostream>
#include <vector>


int test(int a) {
  std::vector<int> vec = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17};
  int pull = a > 16 ? 16 : a < 0 ? 0 : a;

  int res = JSOptimizer::Utility::remove_at(vec,pull);

  LOG_F(INFO, "Removed at: %i", pull);
  DLOG_F(INFO, "Loguru debug only print!");

  return res;
}
