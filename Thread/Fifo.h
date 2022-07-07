// Copyright (c) 2022, HELIOS, Southwest Jiaotong University, All Rights Reserved

//
// Created by liyif on 2022/4/23.
//

#pragma once

#include <queue>
#include <mutex>
#include <condition_variable>
//线程安全队列
template<typename T>
class FIFO {
private:
    std::mutex mut;
    std::queue<T> data_queue;
    std::condition_variable data_cond;
    size_t fifo_size;
public:
    explicit FIFO(size_t size = 1) : fifo_size(size) {

    }
    void push(T new_value) {
        std::lock_guard<std::mutex> lk(mut);
        if (fifo_size > 0 && data_queue.size() >= fifo_size) {
            data_queue.pop();
        }
        data_queue.push(new_value);
        data_cond.notify_one();
    }

    T wait_and_pop() {
        std::unique_lock<std::mutex> lk(mut);
        data_cond.wait(lk, [this] { return !data_queue.empty(); });
        T tmp = std::move(data_queue.front());
        data_queue.pop();
        return tmp;
    }
    T wait_and_get() {
        std::unique_lock<std::mutex> lk(mut);
        data_cond.wait(lk, [this] { return !data_queue.empty(); });
        T value = data_queue.front();
        return value;
    }
};
