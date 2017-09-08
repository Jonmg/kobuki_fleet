#include "StateBase.h"
#include <stddef.h>


StateBase::StateBase()
{
  _agent = NULL;
}

StateBase::~StateBase()
{

}

void StateBase::setAgent(Agent* agent)
{
  _agent = agent;
}

Agent* StateBase::getAgent()
{
  return _agent;
}

