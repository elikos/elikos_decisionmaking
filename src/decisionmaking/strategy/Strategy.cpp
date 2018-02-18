/**
 * \file Strategy.cpp
 * \brief asbtract strategy class implementation
 * \author christophebedard
 */

#include "Strategy.h"

Strategy::Strategy() {

}

Strategy::~Strategy() {
}

Behaviour* Strategy::getCurrentBehaviour() {
    return currentBehaviour_;
}