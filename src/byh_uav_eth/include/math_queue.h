#include <iostream>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <stdexcept>

template <typename T>
class SafeQueue 
{
public:
    // 添加数据到队列
    void Push(const T& item) 
	{
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.push(item);
        // 通知一个等待的线程
        condition_.notify_one();
    }

    // 从队列中取出数据
    bool Pop( T* data) 
	{
        std::unique_lock<std::mutex> lock(mutex_);
        // 等待直到队列不为空
        condition_.wait(lock, [this] { return !queue_.empty(); });

        *data = queue_.front();
        queue_.pop();
        return true;
    }

    // 检查队列是否为空
    bool Empty() const 
	{
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.empty();
    }

    // 获取队列大小
    size_t Size() const 
	{
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }

private:
    mutable std::mutex mutex_;
    std::queue<T> queue_;
    std::condition_variable condition_;
};
