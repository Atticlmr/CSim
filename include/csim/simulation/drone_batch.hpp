#pragma once

#include <csim/simulation/drone.hpp>

#include <algorithm>
#include <condition_variable>
#include <exception>
#include <mutex>
#include <thread>
#include <vector>

namespace csim::simulation {

struct DroneBatchSnapshot {
    double time;
    dynamics::DroneState state;
    dynamics::DroneControl control;
};

class DroneBatch {
public:
    DroneBatch(std::shared_ptr<DroneModel> model, std::size_t count, std::size_t threads = 0)
        : model_(std::move(model)), count_(count) {
        if (!model_ || count == 0) {
            throw std::invalid_argument("Drone batch requires a model and positive environment count");
        }
        thread_count_ = std::min(count, threads == 0
            ? std::max(1u, std::thread::hardware_concurrency()) : threads);
        dynamics::DroneState initial;
        if (model_->asset()) {
            initial.position_W = model_->asset()->initial_pose_WB.position;
            initial.q_WB = model_->asset()->initial_pose_WB.orientation;
        }
        data_.assign(count, makeData(model_, initial));
        candidates_ = data_;
        controls_.resize(count);
        errors_.resize(count);
        try {
            for (std::size_t worker = 1; worker < thread_count_; ++worker) {
                workers_.emplace_back([this, worker] { workerLoop(worker); });
            }
        } catch (...) {
            shutdown();
            throw;
        }
    }

    ~DroneBatch() { shutdown(); }
    DroneBatch(const DroneBatch&) = delete;
    DroneBatch& operator=(const DroneBatch&) = delete;

    std::shared_ptr<DroneModel> model() const { return model_; }
    std::size_t size() const noexcept { return count_; }
    std::size_t threads() const noexcept { return thread_count_; }

    void step(const std::vector<dynamics::DroneControl>& controls, std::size_t substeps = 1) {
        std::lock_guard<std::mutex> operation(operation_mutex_);
        if (controls.size() != size() || substeps == 0) {
            throw std::invalid_argument("Require one control per environment and positive substeps");
        }
        for (const auto& control : controls) dynamics::Drone::validateControl(control);
        controls_ = controls;
        std::fill(errors_.begin(), errors_.end(), std::exception_ptr{});
        {
            std::lock_guard<std::mutex> lock(worker_mutex_);
            substeps_ = substeps;
            pending_ = workers_.size();
            ++generation_;
        }
        ready_.notify_all();
        advanceRange(0);
        {
            std::unique_lock<std::mutex> lock(worker_mutex_);
            done_.wait(lock, [this] { return pending_ == 0; });
        }
        for (const auto& error : errors_) {
            if (error) std::rethrow_exception(error);
        }
        data_.swap(candidates_);
    }

    void reset(const std::vector<std::size_t>& indices,
               const std::vector<dynamics::DroneState>& states) {
        std::lock_guard<std::mutex> operation(operation_mutex_);
        if (indices.size() != states.size()) {
            throw std::invalid_argument("Reset requires one state per environment index");
        }
        std::vector<bool> selected(size(), false);
        std::vector<DroneData> replacements;
        replacements.reserve(states.size());
        for (std::size_t entry = 0; entry < indices.size(); ++entry) {
            const auto index = indices[entry];
            if (index >= size()) throw std::out_of_range("Batch environment index out of range");
            if (selected[index]) throw std::invalid_argument("Duplicate batch reset index");
            selected[index] = true;
            replacements.push_back(makeData(model_, states[entry]));
        }
        for (std::size_t entry = 0; entry < indices.size(); ++entry) {
            data_[indices[entry]] = std::move(replacements[entry]);
        }
    }

    std::vector<DroneBatchSnapshot> getState() const {
        std::lock_guard<std::mutex> operation(operation_mutex_);
        std::vector<DroneBatchSnapshot> snapshots;
        snapshots.reserve(size());
        for (const auto& data : data_) {
            snapshots.push_back({data.time_, data.state_, data.control_});
        }
        return snapshots;
    }

private:
    void advanceRange(std::size_t worker) noexcept {
        const auto block = size() / thread_count_;
        const auto remainder = size() % thread_count_;
        const auto begin = worker * block + std::min(worker, remainder);
        const auto end = begin + block + (worker < remainder ? 1 : 0);
        for (std::size_t index = begin; index < end; ++index) {
            try {
                candidates_[index] = data_[index];
                setControl(*model_, candidates_[index], controls_[index]);
                for (std::size_t substep = 0; substep < substeps_; ++substep) {
                    simulation::step(*model_, candidates_[index]);
                }
            } catch (...) {
                errors_[index] = std::current_exception();
            }
        }
    }

    void workerLoop(std::size_t worker) {
        std::size_t previous_generation = 0;
        std::unique_lock<std::mutex> lock(worker_mutex_);
        for (;;) {
            ready_.wait(lock, [&] { return stopping_ || generation_ != previous_generation; });
            if (stopping_) return;
            previous_generation = generation_;
            lock.unlock();
            advanceRange(worker);
            lock.lock();
            if (--pending_ == 0) done_.notify_one();
        }
    }

    void shutdown() noexcept {
        {
            std::lock_guard<std::mutex> lock(worker_mutex_);
            stopping_ = true;
        }
        ready_.notify_all();
        for (auto& worker : workers_) worker.join();
    }

    const std::shared_ptr<DroneModel> model_;
    const std::size_t count_;
    std::size_t thread_count_ = 1;
    std::vector<DroneData> data_;
    std::vector<DroneData> candidates_;
    std::vector<dynamics::DroneControl> controls_;
    std::vector<std::exception_ptr> errors_;
    mutable std::mutex operation_mutex_;
    std::mutex worker_mutex_;
    std::condition_variable ready_;
    std::condition_variable done_;
    std::vector<std::thread> workers_;
    std::size_t generation_ = 0;
    std::size_t pending_ = 0;
    std::size_t substeps_ = 1;
    bool stopping_ = false;
};

} // namespace csim::simulation
