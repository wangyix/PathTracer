/**
 * Presents the implementation of the semaphore class.
 * The implementation of semaphore::signal(on_thread_exit_t ote) is
 * based on information read from:
 *
 *   http://stackoverflow.com/questions/19176538/how-to-trigger-code-when-a-thread-exits-without-using-functions-at-thread-exi
 * Note that a C++11-specified function called notify_all_at_thread_exit could have been used, but
 * g++ doesn't actually implement it just yet.
 */

#include "semaphore.h"
#include <mutex>
#include <condition_variable>
#include <memory>
using namespace std;

semaphore::semaphore(int value) : value(value) {}

void semaphore::wait() {
  lock_guard<mutex> lg(m);
  cv.wait(m, [this]{ return value > 0; });
  value--;
}

void semaphore::signal() {
  lock_guard<mutex> lg(m);
  value++;
  if (value == 1) cv.notify_all();
}
