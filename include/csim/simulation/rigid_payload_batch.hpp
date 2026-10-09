#pragma once

#include <csim/simulation/rigid_payload.hpp>

#include <condition_variable>
#include <algorithm>
#include <exception>
#include <mutex>
#include <thread>
#include <vector>

namespace csim::simulation {

struct RigidPayloadBatchSnapshot {
    double time;
    dynamics::RigidPayloadState state;
    dynamics::DroneControl control;
    std::vector<CableEvent> events;
};

class RigidPayloadBatch {
public:
    RigidPayloadBatch(std::shared_ptr<RigidPayloadModel> model, std::size_t count,
                      std::size_t threads = 0)
        : model_(std::move(model)), count_(count) {
        if (!model_ || count_ == 0)
            throw std::invalid_argument("Rigid payload batch requires a model and positive environment count");
        thread_count_ = std::min(count_, threads == 0
            ? std::max(1u, std::thread::hardware_concurrency()) : threads);
        data_.resize(count_);
        candidates_.resize(count_);
        events_.resize(count_);
        candidate_events_.resize(count_);
        controls_.resize(count_);
        errors_.resize(count_);
        auto initial = model_->initialState();
        initial.slack = model_->hybridCable();
        initial = resetState(initial);
        for (std::size_t index = 0; index < count_; ++index) {
            data_[index].state = initial;
            data_[index].control = {};
            data_[index].time = 0;
        }
        try {
            for (std::size_t worker = 1; worker < thread_count_; ++worker)
                workers_.emplace_back([this, worker] { workerLoop(worker); });
        } catch (...) {
            shutdown();
            throw;
        }
    }

    ~RigidPayloadBatch() { shutdown(); }
    RigidPayloadBatch(const RigidPayloadBatch&) = delete;
    RigidPayloadBatch& operator=(const RigidPayloadBatch&) = delete;

    std::shared_ptr<RigidPayloadModel> model() const { return model_; }
    std::size_t size() const noexcept { return count_; }
    std::size_t threads() const noexcept { return thread_count_; }

    void step(const std::vector<dynamics::DroneControl>& controls, std::size_t substeps = 1) {
        std::lock_guard<std::mutex> operation(operation_mutex_);
        if (controls.size() != count_ || substeps == 0)
            throw std::invalid_argument("Require one control per environment and positive substeps");
        for (const auto& control : controls) dynamics::Drone::validateControl(control);
        controls_ = controls;
        substeps_ = substeps;
        std::fill(errors_.begin(), errors_.end(), std::exception_ptr{});
        {
            std::lock_guard<std::mutex> lock(worker_mutex_);
            pending_ = workers_.size();
            ++generation_;
        }
        ready_.notify_all();
        advanceRange(0);
        {
            std::unique_lock<std::mutex> lock(worker_mutex_);
            done_.wait(lock, [this] { return pending_ == 0; });
        }
        for (const auto& error : errors_)
            if (error) std::rethrow_exception(error);
        data_.swap(candidates_);
        events_.swap(candidate_events_);
    }

    void reset(const std::vector<std::size_t>& indices,
               const std::vector<dynamics::RigidPayloadState>& states) {
        std::lock_guard<std::mutex> operation(operation_mutex_);
        if (indices.size() != states.size())
            throw std::invalid_argument("Reset requires one state per environment index");
        std::vector<bool> selected(count_, false);
        std::vector<BatchData> replacements;
        replacements.reserve(states.size());
        for (std::size_t entry = 0; entry < states.size(); ++entry) {
            const auto index = indices[entry];
            if (index >= count_) throw std::out_of_range("Rigid payload environment index out of range");
            if (selected[index]) throw std::invalid_argument("Duplicate rigid payload reset index");
            selected[index] = true;
            replacements.push_back({resetState(states[entry]), {}, 0});
        }
        for (std::size_t entry = 0; entry < indices.size(); ++entry) {
            data_[indices[entry]] = std::move(replacements[entry]);
            events_[indices[entry]].clear();
        }
    }

    std::vector<RigidPayloadBatchSnapshot> getState() const {
        std::lock_guard<std::mutex> operation(operation_mutex_);
        std::vector<RigidPayloadBatchSnapshot> snapshots;
        snapshots.reserve(count_);
        for (std::size_t index = 0; index < count_; ++index)
            snapshots.push_back({data_[index].time, data_[index].state,
                data_[index].control, events_[index]});
        return snapshots;
    }

private:
    dynamics::RigidPayloadState resetState(dynamics::RigidPayloadState state) const {
        state = model_->physics().validated(state);
        state = rigid_cable_detail::admissible(*model_,state,{},0);
        (void)model_->physics().observe(state,{},0,false);
        return state;
    }

    struct BatchData {
        dynamics::RigidPayloadState state;
        dynamics::DroneControl control;
        double time;
    };

    void advanceRange(std::size_t worker) noexcept {
        const auto block = count_ / thread_count_;
        const auto remainder = count_ % thread_count_;
        const auto begin = worker * block + std::min(worker, remainder);
        const auto end = begin + block + (worker < remainder ? 1 : 0);
        for (std::size_t index = begin; index < end; ++index) {
            try {
                candidates_[index] = data_[index];
                candidate_events_[index].clear();
                auto& candidate = candidates_[index];
                candidate.control = controls_[index];
                for (std::size_t substep = 0; substep < substeps_; ++substep) {
                    const auto result = rigid_cable_detail::advance(
                        *model_, candidate.state, candidate.control, candidate.time);
                    candidate.state = result.state;
                    candidate.time += model_->timestep();
                    candidate_events_[index].insert(candidate_events_[index].end(),
                        result.events.begin(), result.events.end());
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

    const std::shared_ptr<RigidPayloadModel> model_;
    const std::size_t count_;
    std::size_t thread_count_ = 1;
    std::vector<BatchData> data_, candidates_;
    std::vector<std::vector<CableEvent>> events_, candidate_events_;
    std::vector<dynamics::DroneControl> controls_;
    std::vector<std::exception_ptr> errors_;
    mutable std::mutex operation_mutex_;
    std::mutex worker_mutex_;
    std::condition_variable ready_, done_;
    std::vector<std::thread> workers_;
    std::size_t generation_ = 0, pending_ = 0, substeps_ = 1;
    bool stopping_ = false;
};

} // namespace csim::simulation
