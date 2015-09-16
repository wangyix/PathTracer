/**
* File: thread-pool.h
* -------------------
* This class defines the ThreadPool class, which accepts a collection
* of thunks (which are zero-argument functions that don't return a value)
* and schedules them in a FIFO manner to be executed by a constant number
* of child threads that exist solely to invoke previously scheduled thunks.
*/

#ifndef _thread_pool_
#define _thread_pool_

#include <cstddef>     // for size_t
#include <functional>  // for the function template used in the schedule signature
#include <thread>      // for thread
#include <vector>      // for vector
#include <queue>       // for queue

#include "semaphore.h"

class ThreadPool {
public:

  /**
  * Constructs a ThreadPool configured to spawn up to the specified
  * number of threads.
  */
  ThreadPool(size_t numThreads = 1);

  /**
  * Schedules the provided thunk (which is something that can
  * be invoked as a zero-argument function without a return value)
  * to be executed by one of the ThreadPool's threads as soon as
  * all previously scheduled thunks have been handled.
  */
  void schedule(const std::function<void(void)>& thunk);

  /**
  * Blocks and waits until all previously scheduled thunks
  * have been executed in full.
  */
  void wait();

  /**
  * Blocks and waits until the number of unfinished thunks is equal or
  * below n.
  */
  void waitUntilNumUnfinishedTasks(int n);

  /**
  * Increases the number of threads in the pool to specified
  * number.  Number of threads cannot be decreased at the moment.
  */
  void increaseNumThreads(size_t n);

  /**
  * Waits for all previously scheduled thunks to execute, then waits
  * for all threads to be be destroyed, and then otherwise brings
  * down all resources associated with the ThreadPool.
  */
  ~ThreadPool();

  int getNumThreads() const { return numThreads; }

private:
  size_t numThreads;

  std::thread dt;                // dispatcher thread handle
  std::vector<std::thread> wts;  // worker thread handles

  // shared between main thread and dispatcher thread
  std::queue<std::function<void(void)>> tasks;
  std::mutex tasksM;
  semaphore dispTasksAvailable;
  bool noMoreTasks;

  // shared between dispatcher thread and worker threads
  std::mutex dispM;
  semaphore workerTaskAvail;    // dispatcher signals when task is ready
  semaphore workerTaskStart;    // worker signals when it starts task
  std::function<void(void)> workerTask;

  // shared between main thread and worker threads
  size_t numTasksNotDone;
  std::mutex numTasksNotDoneM;
  std::condition_variable_any allTasksDoneCv;


  void dispatcher();
  void worker(size_t workerID);

  /**
  * ThreadPools are the type of thing that shouldn't be cloneable, since it's
  * not clear what it means to clone a ThreadPool (should copies of all outstanding
  * functions to be executed be copied?).
  *
  * In order to prevent cloning, we remove the copy constructor and the
  * assignment operator.  By doing so, the compiler will ensure we never clone
  * a ThreadPool.
  */
  ThreadPool(const ThreadPool& original) = delete;
  ThreadPool& operator=(const ThreadPool& rhs) = delete;
};

#endif
