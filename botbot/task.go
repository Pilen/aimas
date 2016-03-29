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
  destination() (int, int)
  //checkPreconditions(state) bool
}

type moveTo struct {
  x, y int
}

type pushTo struct {
  x, y int
}

type pullTo struct {
  x, y int
}

func (p *moveTo) destination() (int, int){
  return p.x, p.y
}

func (p *pushTo) destination() (int, int){
  return p.x, p.y
}

func (p *pullTo) destination() (int, int){
  return p.x, p.y
}
