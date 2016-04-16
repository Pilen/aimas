package main

import (
  "strconv"
)

// func search(robots []Robot, boxes []Box, heuristic func([]Robot, []Box) int, goalReached([]Robot, []Box) bool) {
func search() [][]agentAction {

  print("SEARCHING")
  // INITIALIZE STATE
	var frontier Heap
  var initialActions []agentAction
  var initialAction agentAction
  //var states []*SimpleState
  previous := &SimpleState{initialAction, nil, robots, boxes}
  state    := State{0, make(map[Coordinate]bool), initialActions, robots, boxes, len(goals)}
  visitedStates := make(map[string]bool)
  visitedStates[getHash(previous)]=true
  frontier.Insert(previous, 0)


	// A* algorithm
  for !frontier.IsEmpty() {
    previous = frontier.Extract().(*SimpleState)
    print("checking: " + getHash(previous))
    if(previous != nil && previous.action != nil){
      print( "Type " + previous.action.toString())
    }
    if(isDone(previous.boxes) == 0){
      var result [][]agentAction
      for previous.previous != nil {
        print(previous.action.toString())
        newActionList := make([]agentAction, 1)
        newActionList[0] = previous.action
        result = append([][]agentAction{newActionList}, result...)
        previous = previous.previous
      }
      print("GOOD PATH")
      return result
    }
    generate_robot_actions(robots[0], previous, &state, &frontier, &visitedStates)
  }

  print("NO PATH")
  return make([][]agentAction, 0)
}

type SimpleState struct {
	action agentAction
  previous *SimpleState
	robots []*Robot
	boxes []*Box
}

/* Creates a string from the SimpleState that can be used in a hashmap.
 * only robots and boxes is considered, and the runtime is O(#robots+#boxes)
*/ 
func getHash(state *SimpleState) string {
  str := ""
  for _, r := range state.robots {
    str += "(" + strconv.Itoa(r.pos.x) + ";" + string(r.color) + strconv.Itoa(r.pos.y) + ")"
  }
  for _, b := range state.boxes{
    str += "(" + strconv.Itoa(b.pos.x) + ";" + string(b.color) + strconv.Itoa(b.pos.y) + ")"
  }
  return str
}

func all_child_states(state *State) []State {
	var current []*SimpleState
	// var previous = []*SimpleState{&SimpleState{[], []}}
	for _, robot := range state.robots {
		current = make([]*SimpleState, 0)

		_ = current
		_ = robot


		// previous = current
	}
	return nil
}

func heuristic(state *SimpleState) int {
  printf("h: %d\n", isDone(state.boxes))
  return isDone(state.boxes)
}

func generate_robot_actions(robot *Robot, previous *SimpleState, state *State, frontier *Heap, visitedStates *map[string]bool) {
	// Dont reserve current, it is already marked as occupied

  // TODO  REMOvE THIS
  robot = previous.robots[0]

  printf("GENERATING: %d; %d\n", robot.pos.x, robot.pos.y)
	// NOP
	if !reserved(robot.pos, previous) {
		// Create simple state - no extra reservations

		// *newStates = append(*newStates, state)
	}

	// MOVE
	for _, neighbour := range neighbours(robot.pos) {
    printf("NEIGHBOUR: %d; %d\n", neighbour.x, neighbour.y)
    if ( isFree(neighbour, previous) ) {
      print("neighbour is free");
			// Create simple state - Reserve neighbour
      newRobots := newRobotsState(previous, robot, neighbour)
      newAction := move{direction(robot.pos, neighbour)}
      newState := SimpleState{&newAction, previous, newRobots, previous.boxes}
      newHash := getHash(&newState)
      print(newHash)
      if(!(*visitedStates)[newHash]){
        print("INSERTING")
        frontier.Insert(&newState, heuristic(&newState))
        (*visitedStates)[newHash] = true
      } else {
         // TODO: this should check if the new way is better
      }
		}
	}

	// PUSH / PULL
	for _, box := range previous.boxes {
    print("BOX");
		if !isNeigbours(robot.pos, box.pos) { /* || reserved(box.pos, previous) */
      printf("NN: %d; %d -> %d; %d\n", robot.pos.x, robot.pos.y, box.pos.x, box.pos.y)
			continue
		}
		for _, box_dest := range neighbours(box.pos) {
      printf("CONSIDERING: %d; %d -> %d; %d\n", robot.pos.x, robot.pos.y, box_dest.x, box_dest.y)
			if box_dest == robot.pos {
				// PULL
				for _, robot_dest := range neighbours(robot.pos) {
					if robot_dest != box_dest {
				    // Create simple state - Reserve box + dest
            if(isFree(robot_dest, previous)){
              newRobots := newRobotsState(previous, robot, robot_dest)
              newAction := pull{direction(robot.pos, robot_dest), direction(box_dest, box.pos)}
              newBoxes  := newBoxState(previous, box, box_dest)
              newState  := SimpleState{&newAction, previous, newRobots, newBoxes}
              newHash   := getHash(&newState)
              if(!(*visitedStates)[newHash]){
                print("INSERTING PULL")
                frontier.Insert(&newState, heuristic(&newState))
                (*visitedStates)[newHash] = true
              } else {
                 // TODO: this should check if the new way is better
              }
            }
					}
				}
			} else {
				// PUSH
        if(isFree(box_dest, previous)){
          // Create simple state - Reserve box + dest
          newRobots := newRobotsState(previous, robot, box.pos)
          newAction := push{direction(robot.pos, box.pos), direction(box.pos, box_dest)}
          newBoxes  := newBoxState(previous, box, box_dest)
          newState  := SimpleState{&newAction, previous, newRobots, newBoxes}
          newHash   := getHash(&newState)
          if(!(*visitedStates)[newHash]){
            print("INSERTING PUSH")
            frontier.Insert(&newState, heuristic(&newState))
            (*visitedStates)[newHash] = true
          } else {
             // TODO: this should check if the new way is better
          }
        }
			}
		}
	}
}

/*
 * Returns a new []*Robot where each *Robot is the same as in state.robots
 * Except for the one that is being moved, which is a new *Robot with updated
 * coordinate
*/
func newRobotsState(state *SimpleState, robot *Robot, newPos Coordinate) []*Robot {
  newRobots := make([]*Robot, len(state.robots))
  // Copy robots for new state:
  for i, r := range state.robots {
    // If the robot is the one that is being moved we create a new robot and change the coordinate,
    // otherwise we can keep the old robot to save memory.
    if(r.pos == robot.pos){
      newRobots[i] = &Robot{newPos, r.color, nil}
    } else {
      newRobots[i] = r
    }
  }
  return newRobots
}

func newBoxState(state *SimpleState, box *Box, newPos Coordinate) []*Box{
  newBoxes := make([]*Box, len(state.boxes))
  // Copy boxes for new state:
  for i, b := range state.boxes {
    // If the box is the one that is being moved we create a new box and change the coordinate,
    // otherwise we can keep the old box to save memory.
    if(b.pos == box.pos){
      newBoxes[i] = &Box{newPos, b.color}
    } else {
      newBoxes[i] = b
    }
  }
  return newBoxes
}

func isFree(coordinate Coordinate, state *SimpleState) bool {
  //printf("isFree(%d, %d) ?: " + getHash(state), coordinate.x, coordinate.y)
  if(wallMap[coordinate.x][coordinate.y]){
    return false
  } 

  for _, r := range state.robots {
    if(r.pos == coordinate){
      return false
    }
  }

  for _, b := range state.boxes {
    if(b.pos == coordinate){
      return false
    }
  }
  return true
}

func reserved(coordinate Coordinate, previous *SimpleState) bool {
	return true
}
func occupied(x int, y int, state *State) bool {
	return state.reservations[Coordinate{x, y}]
}

func isDone(boxes []*Box) int {
  //TODO: make faster
  res := 0
  for _, goal := range goals {
    found := false
    for _, box := range boxes {
      if(goal.pos == box.pos){
        found = true
        break
      }
    }
    if !found {
       res += 1
    }
  }
  return res
}

// NoOp
// Move(N)
// Move(S)
// move(E)
// move(W)

// Push(N, N)
// Push(N, E)
// Push(N, W)
// Push(S, S)
// Push(S, E)
// Push(S, W)
// Push(E, N)
// Push(E, S)
// Push(E, E)
// Push(W, N)
// Push(W, S)
// Push(W, W)

// Pull(N, )
// Pull(S)
// Pull(E)
// Pull(W)
