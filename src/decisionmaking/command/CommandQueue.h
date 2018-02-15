#ifndef ELIKOS_DUNGEONMASTER_COMMANDQUEUE_H
#define ELIKOS_DUNGEONMASTER_COMMANDQUEUE_H

/**
 * \file CommandQueue.h
 * \brief command queue class declaration
 * \author Olivier
 * \author christophebedard
 */

#include <memory>
#include <deque>
#include "Command.h"

/**
 * \class CommandQueue
 * \brief class which wraps a Command deque.
 */
class CommandQueue
{
public:
    CommandQueue() = default;
    ~CommandQueue() = default;

    bool empty();
    void push(std::unique_ptr<Command> cmd);
    void pop();
    Command* back();
    Command* front();
    void clear();

private:
    std::deque<std::unique_ptr<Command>> q_;

};

#endif // ELIKOS_DUNGEONMASTER_COMMANDQUEUE_H
