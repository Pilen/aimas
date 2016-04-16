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
    if(isDone(previous.boxes) || /* TODO */ (previous.robots[0].pos.y == 3 && previous.robots[0].pos.x == 4)){
      //result := make([][]agentAction, 1)
      var result [][]agentAction
      //i := 0
      for previous.previous != nil {
        print(previous.action.toString())
        newActionList := make([]agentAction, 1)
        newActionList[0] = previous.action
        result = append(result, newActionList)
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
  return 0
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
 // 	if (!reserved(neighbour, previous) &&
 // 		!state.reservations[neighbour] &&
 // 		!wallMap[neighbour.x][neighbour.y]) {
    if ( isFree(neighbour, previous) ) {
			// Create simple state - Reserve neighbour
      newRobots := make([]*Robot, len(previous.robots))
      // Copy robots for new state:
      for i, r := range previous.robots {
        // If the robot is the one that is being moved we create a new robot and change the coordinate,
        // otherwise we can keep the old robot to save memory.
        if(r.pos == robot.pos){
          newRobots[i] = &Robot{neighbour, r.color, nil};
        } else {
          newRobots[i] = r;
        }
      }
      newAction := move{direction(robot.pos, neighbour)}
      newState := SimpleState{&newAction, previous, newRobots, boxes}
      newHash := getHash(&newState)
      if(!(*visitedStates)[newHash]){
        print("INSERTING")
        frontier.Insert(&newState, heuristic(&newState))
        //appended := append(*newStates, &newState)
        //newStates = &appended
        (*visitedStates)[newHash] = true
      }
		}
	}

	// PUSH / PULL
	for _, box := range state.boxes {
		if !isNeigbours(robot.pos, box.pos) || reserved(box.pos, previous) {
			continue
		}
		for _, box_dest := range neighbours(box.pos) {
			if box_dest == robot.pos {
				// PULL
				for _, robot_dest := range neighbours(robot.pos) {
					if robot_dest != box_dest {
						if (!reserved(robot_dest, previous) &&
							!state.reservations[robot_dest] &&
							!wallMap[robot_dest.x][robot_dest.y]) {
							// Create simple state - Reserve box + dest
						}
					} // else already handled
				}
			} else {
				// PUSH
				if (!reserved(box_dest, previous) &&
					!state.reservations[box_dest] &&
					!wallMap[box_dest.x][box_dest.y]) {
					// Create simple state - Reserve box + dest
				}
			}
		}
	}
}

func isFree(coordinate Coordinate, state *SimpleState) bool {
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

func isDone(boxes []*Box) bool {
  //TODO: make faster
  for _, goal := range goals {
    found := false
    for _, box := range boxes {
      if(goal.pos == box.pos){
        found = true
        break
      }
    }
    if !found {
      return false
    }
  }
  return true
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
