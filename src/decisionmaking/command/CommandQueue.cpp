/**
 * \file CommandQueue.cpp
 * \brief CommandQueue class implementation.
 * \author Olivier
 * \author christophebedard
 */

#include "CommandQueue.h"

bool CommandQueue::empty() {
    return q_.empty();
}

void CommandQueue::push(std::unique_ptr<Command> cmd) {
    q_.push_back(std::move(cmd));
}

void CommandQueue::pop() {
    q_.pop_front();
}

Command* CommandQueue::back() {
    return q_.back().get();
}

Command* CommandQueue::front() {
    return q_.front().get();
}

void CommandQueue::clear() {
    q_.clear();
}