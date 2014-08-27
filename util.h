#ifndef UTIL_H
#define UTIL_H

#include <queue>
#include <pthread.h>

#define LOGD(fmt, args...) fprintf(stdout, "%s D: %s: " fmt"\n", Util::getLogTime(), __PRETTY_FUNCTION__, ##args)
#define LOGE(fmt, args...) fprintf(stderr, "%s E: %s: " fmt"\n", Util::getLogTime(), __PRETTY_FUNCTION__, ##args)

class Util
{
public:
    static char *getLogTime();
    static ssize_t readFile(const char *path, char *buff, size_t max_len);
};

class MutexLocker
{
public:
    MutexLocker(pthread_mutex_t *mutex);
    virtual ~MutexLocker();

    pthread_mutex_t *mutex() const { return m_mutex; }
    inline void relock();
    inline void unlock();

protected:
    pthread_mutex_t *m_mutex;
    bool m_locked;
};

template <class T>
class SafeQueue
{
public:
    SafeQueue();
    virtual ~SafeQueue();

    bool empty();
    size_t size();

    bool waitForNotEmpty();
    void cancelNotEmptyWait();

    void push(const T& element);
    T pop();

protected:
    std::queue<T> m_queue;
    pthread_mutex_t m_mutex;
    pthread_cond_t m_push_cond;
};

template <class T>
SafeQueue<T>::SafeQueue()
{
    pthread_mutex_init(&m_mutex, NULL);
    pthread_cond_init(&m_push_cond, NULL);
}

template <class T>
SafeQueue<T>::~SafeQueue()
{
    pthread_mutex_destroy(&m_mutex);
    pthread_cond_destroy(&m_push_cond);
}

template <class T>
bool SafeQueue<T>::empty()
{
    MutexLocker l(&m_mutex);
    return m_queue.empty();
}

template <class T>
size_t SafeQueue<T>::size()
{
    MutexLocker l(&m_mutex);
    return m_queue.size();
}

template <class T>
void SafeQueue<T>::push(const T& element)
{
    MutexLocker l(&m_mutex);
    m_queue.push(element);
    pthread_cond_signal(&m_push_cond);
}

template <class T>
T SafeQueue<T>::pop()
{
    MutexLocker l(&m_mutex);
    T res = m_queue.front();
    m_queue.pop();
    return res;
}

template <class T>
bool SafeQueue<T>::waitForNotEmpty()
{
    MutexLocker l(&m_mutex);

    if(!m_queue.empty())
        return true;

    pthread_cond_wait(&m_push_cond, &m_mutex);
    return !m_queue.empty();
}

template <class T>
void SafeQueue<T>::cancelNotEmptyWait()
{
    MutexLocker l(&m_mutex);
    pthread_cond_signal(&m_push_cond);
}

#endif
