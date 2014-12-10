/**
 * File: thread-pool.cc
 * --------------------
 * Presents the implementation of the ThreadPool class.
 */

#include "thread-pool.h"

ThreadPool::ThreadPool(size_t numThreads) :
  numThreads(numThreads),
  noMoreTasks(false),
  numTasksNotDone(0)
{
  // launch dispatcher thread
  dt = std::thread(&ThreadPool::dispatcher, this);
}

ThreadPool::~ThreadPool() {
  wait();

  // tell dispatcher that there are no more tasks
  noMoreTasks = true;
  dispTasksAvailable.signal();
  
  // wait for dispatcher and worker threads to end
  dt.join();
  for (std::thread& wt : wts) {
    wt.join();
  }
}

void ThreadPool::increaseNumThreads(size_t n) {
    if (n > numThreads) {
        numThreads = n;
    }
}

void ThreadPool::schedule(const std::function<void(void)>& thunk) {
  // increment the number of unfinished tasks
  numTasksNotDoneM.lock();
  numTasksNotDone++;
  numTasksNotDoneM.unlock();
  
  // add this thunk to the list of tasks
  tasksM.lock();
  tasks.push(thunk);
  tasksM.unlock();

  // signal dispatcher that a new task is available
  dispTasksAvailable.signal();
}

void ThreadPool::wait() {
  // wait until number of unfinished tasks goes to 0
  numTasksNotDoneM.lock();
  allTasksDoneCv.wait(numTasksNotDoneM, [this] {
    return (numTasksNotDone == 0);
  });
  numTasksNotDoneM.unlock();
}

void ThreadPool::dispatcher() {
  while (true) {
    // wait for a task to dispatch from tasks list
    dispTasksAvailable.wait();

    if (noMoreTasks) {
      break;
    }

    // if we haven't spawned the max number of threads yet and there is no
    // worker in the dispatcher's "office" to accept the next task, spawn an
    // additional worker thread.
    if (wts.size() < numThreads && dispM.try_lock()) {
      dispM.unlock();
      size_t workerID = wts.size();
      wts.push_back(std::thread(&ThreadPool::worker, this, workerID));
    }
    
    // pop task from list, put it somewhere accessible by worker
    tasksM.lock();
    workerTask = tasks.front();
    tasks.pop();
    tasksM.unlock();

    // let that worker know there's a task ready for it
    workerTaskAvail.signal();

    // wait for worker to start task.  This is to make sure this worker has
    // copied workerTask and has left the dispatcher's "office" before we
    // try to see if we should spawn a new worker next iteration (by checking
    // if there's a worker in the "office" or not).
    workerTaskStart.wait();
  }

  // give all workers an empty task, signifying they should terminate
  workerTask = std::function<void(void)>();
  for (size_t i = 0; i < wts.size(); i++) {
    workerTaskAvail.signal();
    workerTaskStart.wait();
  }
}

void ThreadPool::worker(size_t workerID) {
  while (true) {
    // wait to gain exclusive access to dispatcher's "office", wait for a task
    // to be provided, then leave the dispatcher's "office"
    dispM.lock();
    workerTaskAvail.wait();
    std::function<void(void)> task = workerTask;
    dispM.unlock();

    // let dispatcher know this worker has received the task and has left the
    // dispatcher's "office".
    workerTaskStart.signal();
    
    // if dispatcher gave an empty task, this worker should stop
    if (!task) {
        break;
    }
    
    task();

    // decrement the number of unfinished tasks; if there are 0 unifinished
    // tasks, signal the main thread in case it's inside wait().
    numTasksNotDoneM.lock();
    numTasksNotDone--;
    if (numTasksNotDone == 0) {
      allTasksDoneCv.notify_all();
    }
    numTasksNotDoneM.unlock();
  }
}
