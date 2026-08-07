#pragma once

#include <mutex>

#if defined(__clang__)
#define LIVO_SLAM_TSA(x) __attribute__((x))
#else
#define LIVO_SLAM_TSA(x)
#endif

#define CAPABILITY(x) LIVO_SLAM_TSA(capability(x))
#define SCOPED_CAPABILITY LIVO_SLAM_TSA(scoped_lockable)
#define GUARDED_BY(x) LIVO_SLAM_TSA(guarded_by(x))
#define REQUIRES(...) LIVO_SLAM_TSA(requires_capability(__VA_ARGS__))
#define ACQUIRE(...) LIVO_SLAM_TSA(acquire_capability(__VA_ARGS__))
#define RELEASE(...) LIVO_SLAM_TSA(release_capability(__VA_ARGS__))
#define NO_THREAD_SAFETY_ANALYSIS LIVO_SLAM_TSA(no_thread_safety_analysis)

namespace livo_slam {

class CAPABILITY("mutex") Mutex {
 public:
  Mutex() = default;

  Mutex(const Mutex&) = delete;
  Mutex& operator=(const Mutex&) = delete;
  Mutex(Mutex&&) = delete;
  Mutex& operator=(Mutex&&) = delete;

  void lock() ACQUIRE() { mutex_.lock(); }
  void unlock() RELEASE() { mutex_.unlock(); }

  std::mutex& native() { return mutex_; }

 private:
  std::mutex mutex_;
};

class SCOPED_CAPABILITY MutexLock {
 public:
  explicit MutexLock(Mutex& mutex) ACQUIRE(mutex) : lock_(mutex.native()) {}
  ~MutexLock() RELEASE() {}

  MutexLock(const MutexLock&) = delete;
  MutexLock& operator=(const MutexLock&) = delete;
  MutexLock(MutexLock&&) = delete;
  MutexLock& operator=(MutexLock&&) = delete;

  std::unique_lock<std::mutex>& native() { return lock_; }

 private:
  std::unique_lock<std::mutex> lock_;
};

}  // namespace livo_slam
