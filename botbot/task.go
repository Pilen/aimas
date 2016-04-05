package main

type Task interface {
	// precondition() int
	// getPlan(robot, box, destination, currentState, currentActions) {State, actions}
}


// BoxTask
// RobotTask
// RobotStorageTask
// BoxStorageTask
// NopTask


// HIGHLEVEL ACTIONS:
type highlevelAction interface {
  destination() Coordinate
  precondition() int
  state() *State
  get_agent() Robot
  previousAction() *highlevelAction
  //checkPreconditions(state) bool
}

type moveTo struct {
  //state State
  dest Coordinate
  agent Robot
  prevAction *highlevelAction
}

type pushTo struct {
  //state State
  dest Coordinate
  agent Robot
  box Box
  prevAction *highlevelAction
}

type pullTo struct {
  //state State
  dest Coordinate
  agent Robot
  box Box
  prevAction *highlevelAction
}

func (p *moveTo) destination() Coordinate {
  return p.dest
}

func (p *moveTo) precondition() int{
  agentCoord := Coordinate{p.agent.x, p.agent.y}
  return checked_distance(agentCoord, p.dest)
}

func (p *moveTo) state() *State{
    return nil
}

func (p *moveTo) get_agent() Robot{
    return p.agent
}

func (p *moveTo) previousAction() *highlevelAction{
    return p.prevAction
}

func (p *pushTo) destination() Coordinate{
  return p.dest
}

func (p *pushTo) precondition() int{
  if(p.agent.color != p.box.color){
    return -1
  }
  agentCoord := Coordinate{p.agent.x, p.agent.y}
  boxCoord := Coordinate{p.box.x, p.box.y}
  if(!isNeigbours(agentCoord, boxCoord)){
    return -1
  }
  return checked_distance(agentCoord, p.dest)
}

func (p *pushTo) state() *State{
    return nil
}

func (p *pushTo) get_agent() Robot{
    return p.agent
}

func (p *pushTo) previousAction() *highlevelAction{
    return p.prevAction
}

func (p *pullTo) destination() Coordinate{
  return p.dest
}

func (p *pullTo) precondition() int{
  if(p.agent.color != p.box.color){
    return -1
  }
  agentCoord := Coordinate{p.agent.x, p.agent.y}
  boxCoord := Coordinate{p.box.x, p.box.y}
  if(!isNeigbours(agentCoord, boxCoord)){
    return -1
  }
  return checked_distance(agentCoord, p.dest)
}

func (p *pullTo) state() *State{
    return nil
}

func (p *pullTo) get_agent() Robot{
    return p.agent
}

func (p *pullTo) previousAction() *highlevelAction{
    return p.prevAction
}
