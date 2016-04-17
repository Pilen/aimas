package main

import (
  "strconv"
)

// func search(robots []Robot, boxes []Box, heuristic func([]Robot, []Box) int, goalReached([]Robot, []Box) bool) {
func search() [][]agentAction {

  print("SEARCHING")
  // INITIALIZE STATE
	var frontier Heap
  var initialA []agentAction
  //var initialAction agentAction
  //var states []*SimpleState
  previous := &SimpleState{initialActions(), nil, robots, boxes, initialCaluclated()}
  state    := State{0, make(map[Coordinate]bool), initialA, robots, boxes, len(goals)}
  visitedStates := make(map[string]bool)
  visitedStates[getHash(previous)]=true
  frontier.Insert(previous, 0)


	// A* algorithm
  for !frontier.IsEmpty() {
    previous = frontier.Extract().(*SimpleState)
    print("checking: " + getHash(previous))
    if(isDone(previous.boxes) == 0){
      var result [][]agentAction
      for previous.previous != nil {
        result = append([][]agentAction{*previous.action}, result...)
        previous = previous.previous
      }
      print("GOOD PATH")
      return result
    }
    // Generate actions for the first robot that has not been calculated using
    // the previous actions
    for i, b := range previous.calculated {
      if(!b){
        // If the i'th robot has not had its actions calculated then do it:
        printf("index = %d (true)\n", i)
        generate_robot_actions(i, previous.robots[i], previous, &state, &frontier, &visitedStates)
        break
      }else if (i == len(previous.calculated)-1) {
        // If all robots had its actions generated from this state, create a new one.
        printf("index = 0 (false)\n")
        generate_robot_actions(0, previous.robots[0], previous, &state, &frontier, &visitedStates)
        break
      }
    }
  }

  print("NO PATH")
  return make([][]agentAction, 0)
}

type SimpleState struct {
	action *[]agentAction
  previous *SimpleState
	robots []*Robot
	boxes []*Box
  calculated []bool
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
  return isDone(state.boxes)
}

func initialActions() *[]agentAction{
  res := make([]agentAction, len(robots))
  for i, _ := range res {
    res[i] = &noop{}
  }
  return &res
}

func newActionsState(state *SimpleState, i int) *[]agentAction {
  if(i == 0){
    return initialActions()
  }
  res := make([]agentAction, len(robots))
  for j, a := range *state.action {
    res[j] = a
    print((*a).toString())
  }
  return &res
}


func initialCaluclated() []bool {
  res := make([]bool, len(robots))
  for j, _ := range res {
    res[j] = false
  }
  return res
}

func newCalculatedState(state *SimpleState, i int) []bool {
  var res []bool
  if(i==0){
    res = initialCaluclated()
  } else {
    res = make([]bool, len(robots))
    for j, b := range state.calculated {
        res[j] = b
    }
  }
  res[i] = true
  return res
}

func generate_robot_actions(i int, robot *Robot, previous *SimpleState, state *State, frontier *Heap, visitedStates *map[string]bool) {
	// Dont reserve current, it is already marked as occupied

  // TODO  REMOvE THIS
  //robot = previous.robots[0]

  printf("GENERATING: %d; %d\n", robot.pos.x, robot.pos.y)
	// NOP
	if !reserved(robot.pos, previous) {
		// Create simple state - no extra reservations

		// *newStates = append(*newStates, state)
	}
  newCalculatedState := newCalculatedState(previous, i)
  var newPrevious *SimpleState
  if(i==0){
    newPrevious = previous
  }else{
    newPrevious = previous.previous
  }
  //TODO: NOOP 
	// MOVE
	for _, neighbour := range neighbours(robot.pos) {
    printf("NEIGHBOUR: %d; %d\n", neighbour.x, neighbour.y)
    if ( isFree(neighbour, previous) ) {
      print("neighbour is free");
			// Create simple state - Reserve neighbour
      newRobots := newRobotsState(previous, robot, neighbour)
      newActions := newActionsState(previous, i)
      (*newActions)[i] = &move{direction(robot.pos, neighbour)}
      newState := SimpleState{newActions, newPrevious, newRobots, previous.boxes, newCalculatedState}
      newHash := getHash(&newState)
      print(newHash)
      if(!(*visitedStates)[newHash]){
        print("INSERTING")
        frontier.Insert(&newState, heuristic(&newState))
        (*visitedStates)[newHash] = true
        for _, a := range *previous.action {
          print((*a).toString())
        }
      } else {
         // TODO: this should check if the new way is better
      }
		}
	}

	// PUSH / PULL
	for _, box := range previous.boxes {
    print("BOX");
		if (!isNeigbours(robot.pos, box.pos) || robot.color != box.color) { /* || reserved(box.pos, previous) */
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
              newAction := newActionsState(previous, i)
              (*newAction)[i] = &pull{direction(robot.pos, robot_dest), direction(box_dest, box.pos)}
              newBoxes  := newBoxState(previous, box, box_dest)
              newState  := SimpleState{newAction, newPrevious, newRobots, newBoxes, newCalculatedState}
              newHash   := getHash(&newState)
              if(!(*visitedStates)[newHash]){
                print("INSERTING PULL")
                frontier.Insert(&newState, heuristic(&newState))
                (*visitedStates)[newHash] = true
                for _, a := range *previous.action {
                  print((*a).toString())
                }
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
          newAction := newActionsState(previous, i)
          (*newAction)[i] = &push{direction(robot.pos, box.pos), direction(box.pos, box_dest)}
          newBoxes  := newBoxState(previous, box, box_dest)
          newState  := SimpleState{newAction, newPrevious, newRobots, newBoxes, newCalculatedState}
          newHash   := getHash(&newState)
          if(!(*visitedStates)[newHash]){
            print("INSERTING PUSH")
            frontier.Insert(&newState, heuristic(&newState))
            (*visitedStates)[newHash] = true
            for _, a := range *previous.action {
              print((*a).toString())
            }
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
      newBoxes[i] = &Box{newPos, b.color, b.letter}
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
      //print("isDone: " + string(goal.letter) + " =? " + string(box.letter))
      if(goal.pos == box.pos && goal.letter == box.letter){
        //print(" match")
        found = true
        break
      }
    }
    if !found {
       res += 1
    }
  }
  printf("isDone: %d\n", res)
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
