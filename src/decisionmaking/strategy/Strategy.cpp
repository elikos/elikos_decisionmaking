/**
 * \file Strategy.cpp
 * \brief asbtract strategy class implementation
 * \author christophebedard
 */

#include "Strategy.h"

Strategy::Strategy() {

}

Strategy::~Strategy() {
    //delete currentBehaviour_;
}

Behaviour* Strategy::getCurrentBehaviour() {
    return currentBehaviour_;
}

Strategy::takeoff() {

}

Strategy::land() {
    
}
