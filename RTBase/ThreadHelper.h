#pragma once

#include <thread>
#include <condition_variable>
#include <queue>
#include <functional>

class TaskThread
{
	std::jthread myThread;

    std::mutex mtx;
    std::condition_variable waitOnQueue;
    std::condition_variable waitOnTask;

    std::queue<std::function<void(std::stop_token)>> taskQueue;
    std::atomic<unsigned int> tasksCompleted;
    std::atomic<unsigned int> tasksQueued;

    void loop(std::stop_token stop)
    {
        while (!stop.stop_requested())
        {
            std::unique_lock<std::mutex> guard(mtx);
            waitOnQueue.wait(guard, [&]() {return !taskQueue.empty(); });

            std::function<void(std::stop_token)> task = taskQueue.front();
            taskQueue.pop();

            guard.unlock();
            
            task(stop);
            tasksCompleted++;

            guard.lock();
            waitOnTask.notify_all();
        }
    }

public:

    TaskThread() : myThread([this](std::stop_token stop) {loop(stop); })
    {

    }

    ~TaskThread()
    {
        Stop();
    }

    unsigned int QueueTask(std::function<void(std::stop_token)> task)
    {
        std::unique_lock<std::mutex> guard(mtx);
        taskQueue.push(task);
        unsigned int toReturn = tasksQueued++;

        guard.unlock();
        waitOnQueue.notify_all();
        return toReturn;
    }

    void Join(unsigned int targetTask)
    {
        std::unique_lock<std::mutex> guard(mtx);
        if (tasksCompleted > targetTask)
            return;
        waitOnTask.wait(guard, [&]() {return tasksCompleted > targetTask; });
    }

    void Stop()
    {
        myThread.request_stop();
    }
};