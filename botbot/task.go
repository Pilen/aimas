package main

type Task interface {
	// precondition() int
	// getPlan(robot, box, destination, currentState, currentActions) State
}


// BoxTask
// RobotTask
// RobotStorageTask
// BoxStorageTask
// NopTask


// HIGHLEVEL ACTIONS:
type highlevelAction interface {
  destination() Coordinate
  //checkPreconditions(state) bool
}

type moveTo struct {
  dest Coordinate
}

type pushTo struct {
  dest Coordinate
}

type pullTo struct {
  dest Coordinate
}

func (p *moveTo) destination() Coordinate {
  return p.dest
}

func (p *pushTo) destination() Coordinate{
  return p.dest
}

func (p *pullTo) destination() Coordinate{
  return p.dest
}
