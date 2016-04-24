package main

import (
  "strconv"
)

// func search(robots []Robot, boxes []Box, heuristic func([]Robot, []Box) int, goalReached([]Robot, []Box) bool) {
func search() [][]agentAction {
  dprint("SEARCHING")
  // INITIALIZE STATE
	var frontier Heap
  previous := &SimpleState{initialActions(),make([][]agentAction, 0), 0, make([]Heap, len(robots)), nil, robots, boxes, 0, getInitialGoals(boxes), make([]*agentGoal, len(robots))}
  visitedStates := make(map[string]bool)
  visitedStates[getHash(previous)]=true
  frontier.Insert(previous, 0)


	// A* algorithm
  for !frontier.IsEmpty() {
    previous = frontier.Extract().(*SimpleState)
    dprintf("checking: " + getHash(previous) + "out of %d type "+ (*previous.actualActions)[0].toString(), frontier.Size())
    if(isDone(previous.boxes) == 0){
      var result [][]agentAction
      for previous.previous != nil {
        result = append([][]agentAction{*previous.actualActions}, result...)
        previous = previous.previous
      }
      dprint("GOOD PATH")
      return result
    }
    // Generate all possible actions, if they have not allready been found:
    if(previous.combinationLevel == 0){
      for i, _ := range previous.robots {
        previous.actionHeap[i] = generate_robot_actions(i, previous, &visitedStates)
      }
    } else {
      dprint("HAD ACTIONS")
    }

    previous.actionCombinations = generateCombinations(previous)

    //TODO:
    for _, actions := range previous.actionCombinations {
      dprint("has action")
      newState := joinActions(actions, previous)     
      newHash := getHash(newState)
      dprintf(newHash)
      if(!visitedStates[newHash]){
        dprint("inserting")
        frontier.Insert(newState, newState.cost + heuristic(newState))
        (*visitedStates)[newHash] = true
      }
    }

    // TODO: insert only if there are more actions left
    // TODO: Dont recalculate heuristic
    if(previous.combinationLevel >= 0){
      frontier.Insert(previous, previous.cost + heuristic(previous))
    }
  }

  dprint("NO PATH")
  return make([][]agentAction, 0)
}

// Must have one action for each agent
func joinActions(actions []agentAction, state *SimpleState) *SimpleState {
  newState := nextState(state)
  for i, action := range actions {
      dprint(action.toString())
      var ac interface{} = action
      switch a := (ac).(type) {
        case *move:
          dprint("MOVE")
          newRPos := applyDirection(state.robots[i].pos, a.direction)
          if(isFree(newRPos, newState)){
            newState.robots[i] = &Robot{newRPos, state.robots[i].color, state.robots[i].next}
          } else {
            dprint("Invalid move")
            actions[i] = &noop{}
          }
        case *push:
          dprint("PUSH")
          newRPos := applyDirection(state.robots[i].pos, a.agentDirection)
          newBPos := applyDirection(state.boxes[a.boxIdx].pos, a.boxDirection)
          if(isFree(newBPos, newState)){
            newState.robots[i] = &Robot{newRPos, state.robots[i].color, state.robots[i].next}
            newState.boxes[a.boxIdx] = &Box{newBPos, state.boxes[a.boxIdx].color, state.boxes[a.boxIdx].letter}
          } else {
            dprint("Invalid push")
            actions[i] = &noop{}
          }
        case *pull:
          dprint("PULL")
          newRPos := applyDirection(state.robots[i].pos, a.agentDirection)
          newBPos := applyDirection(state.boxes[a.boxIdx].pos, a.boxDirection)
          if(isFree(newRPos, newState)){
            newState.robots[i] = &Robot{newRPos, state.robots[i].color, state.robots[i].next}
            newState.boxes[a.boxIdx] = &Box{newBPos, state.boxes[a.boxIdx].color, state.boxes[a.boxIdx].letter}
          } else {
            dprint("Invalid pull")
            actions[i] = &noop{}
          }
      }
  }
  newState.actualActions = &actions
  return newState
}

func generateCombinations(state *SimpleState) [][]agentAction {
  // TODO: test with multiagents
  dprintf("combi lvl: %d", state.combinationLevel)
  // If we have no more possible actions
  if state.combinationLevel < 0 {
    return make([][]agentAction, 0)
  }
  //TODO: finish! this is only mockup
  state.combinationLevel++ // Increment before, since it starts at 0
  // [robotIdx][iAction] contains robot actions which must be combined
  // iAction is min(state.combinations, heapsize for robotIdx)
  combis := make([][]agentAction, len(state.robots))
  empty := true
  for i:=0; i<len(state.robots); i++ {
    size := min(state.combinationLevel, state.actionHeap[i].size)
    dprintf("Inserting %d, %d, %d", size, state.combinationLevel, state.actionHeap[i].size)
    combis[i] = make([]agentAction, size)
    if state.actionHeap[i].size > state.combinationLevel {
      empty = false
    }
    for j:=0; j<size; j++ {
      // fucking retarded crap typesystem in GO
      // How about GO to hell
      var ac interface{} = state.actionHeap[i].Extract()
      switch a := (ac).(type) {
        case move:
          dprint(a.toString())
          combis[i][j] = &a // how can I put a pointer in a nonpointer array?
        case push:
          dprint(a.toString())
          combis[i][j] = &a
        case pull:
          dprint(a.toString())
          combis[i][j] = &a
        default:
          dprint("invalid!!!")
      }
    }
  }

  if empty {
    state.combinationLevel = -1
  }

  for i:=0; i<len(state.robots); i++ {
    size := min(state.combinationLevel, state.actionHeap[i].size)
    for j:=0; j<size; j++ {
      dprint("combi " + (*combis[i][j]).toString()) // Why am I dereferencing a non-pointer
    }
  }

  res := make([][]agentAction, 0)
  res2 := make([][]agentAction, 0)
  for r, actions := range combis {
    for _, action := range actions {
      dprint("new:" + action.toString())
      // For each of the previous results, copy the results and insert all new actions
      if r == 0 {
        dprint("action " + action.toString())
        robotRes := make([]agentAction, len(actions))
        robotRes[0] = action
        dprint("res action " + robotRes[0].toString())
        res = append(res, robotRes)
        for _, ra := range res {
          dprint(" got " + ra[0].toString())
        }
      } else {
        dprint("IN ELSE:")
        for _, actionsRes := range res {
          robotRes := make([]agentAction, len(actions))
          for i, actionRes := range actionsRes {
            robotRes[i] = actionRes
          }
          robotRes[r] = action
          res2 = append(res2, robotRes)
        }
        res = res2
      }
    }
  }

  dprint("actions------------")
  for _, actions := range res2 {
    for _, action := range actions {
      if(action == nil){
        dprint("null agent")
        return make([][]agentAction, 0)
      }
      dprint(action.toString())
    }
    dprint("---")
  }
  return res
}

type SimpleState struct {
	actualActions *[]agentAction
  actionCombinations [][]agentAction // TODO: this should be done directly and stored in frontier. Dont save here.
  combinationLevel int
  actionHeap []Heap
  previous *SimpleState
	robots []*Robot
	boxes []*Box
  cost int
  goals []agentGoal
  activeGoals []*agentGoal
}

/*
 * creates a copy of state containing only the information needed to
 * measure the heurisic of the state.
*/
func nextState(state *SimpleState) *SimpleState {
  //TODO: does this work?
  newState := SimpleState{nil, make([][]agentAction, 0), 0, make([]Heap, len(state.robots)), state, make([]*Robot, len(state.robots)), make([]*Box, len(state.boxes)), state.cost+1, state.goals, state.activeGoals}

  for i, robot := range state.robots {
    newState.robots[i] = robot
  }

  for i, box := range state.boxes {
    newState.boxes[i] = box
  }
  //TODO: should the goals be copied?
//  for i, goal := range state.goals {
//    newState.goals[i] = goal
//  }
//
//  for i, goal := range state.activeGoals {
//    newState.activeGoals[i] = goal
//  }

  return &newState
}

func generate_robot_actions(i int, previous *SimpleState, visitedStates *map[string]bool) Heap {

	var actions Heap
  robot := previous.robots[i];

  dprintf("GENERATING: %d; %d for %d", robot.pos.x, robot.pos.y, i)
  if(previous.activeGoals[i] != nil){
    dprintf("GOAL: %v (%d,%d) -> (%d,%d)",previous.boxes[previous.activeGoals[i].boxIdx].letter, previous.boxes[previous.activeGoals[i].boxIdx].pos.x, previous.boxes[previous.activeGoals[i].boxIdx].pos.y, goals[previous.activeGoals[i].goalIdx].pos.x, goals[previous.activeGoals[i].goalIdx].pos.y)
  }

  //TODO: NOOP 
	// MOVE
  newState := nextState(previous)
	for _, neighbour := range neighbours(robot.pos) {
    if ( isFree(neighbour, previous) ) {
      newState.robots[i] = &Robot{neighbour, robot.color, robot.next}
      newHash := getHash(newState)
      if(!(*visitedStates)[newHash]){
        dprint((&move{direction(robot.pos, neighbour)}).toString())
        actions.Insert(move{direction(robot.pos, neighbour)}, heuristic(newState))
      }
		}
	}

	// PUSH / PULL
	for bIdx, box := range previous.boxes {
		if (!isNeigbours(robot.pos, box.pos) || robot.color != box.color) {
			continue
		}
		for _, box_dest := range neighbours(box.pos) {
			if box_dest == robot.pos {
				// PULL
				for _, robot_dest := range neighbours(robot.pos) {
					if robot_dest != box_dest && isFree(robot_dest, previous) {
            newState.robots[i] = &Robot{robot_dest, robot.color, robot.next}
            newState.boxes[bIdx] = &Box{box_dest, box.color, box.letter}
            newHash   := getHash(newState)
            if(!(*visitedStates)[newHash]){
              dprint((&pull{direction(robot.pos, robot_dest), direction(box_dest, box.pos), bIdx}).toString())
              actions.Insert(pull{direction(robot.pos, robot_dest), direction(box_dest, box.pos), bIdx}, heuristic(newState))
            }
					}
				}
			} else if isFree(box_dest, previous) {
				// PUSH
        newState.robots[i] = &Robot{box.pos, robot.color, robot.next}
        newState.boxes[bIdx] = &Box{box_dest, box.color, box.letter}
        newHash   := getHash(newState)
        if(!(*visitedStates)[newHash]){
          dprint((&push{direction(robot.pos, box.pos), direction(box.pos, box_dest), bIdx}).toString())
          actions.Insert(push{direction(robot.pos, box.pos), direction(box.pos, box_dest), bIdx}, heuristic(newState))
        }
			}
		}
	}
  return actions
}

/*
 * Returns a new []*Robot where each *Robot is the same as in state.robots
 * Except for the one that is being moved, which is a new *Robot with updated
 * coordinate
*/
//func newRobotsState(state *SimpleState, robot *Robot, newPos Coordinate) []*Robot {
//  newRobots := make([]*Robot, len(state.robots))
//  // Copy robots for new state:
//  for i, r := range state.robots {
//    // If the robot is the one that is being moved we create a new robot and change the coordinate,
//    // otherwise we can keep the old robot to save memory.
//    if(r.pos == robot.pos){
//      newRobots[i] = &Robot{newPos, r.color, nil}
//    } else {
//      newRobots[i] = r
//    }
//  }
//  return newRobots
//}
//
//func newBoxState(state *SimpleState, box *Box, newPos Coordinate) []*Box{
//  newBoxes := make([]*Box, len(state.boxes))
//  // Copy boxes for new state:
//  for i, b := range state.boxes {
//    // If the box is the one that is being moved we create a new box and change the coordinate,
//    // otherwise we can keep the old box to save memory.
//    if(b.pos == box.pos){
//      newBoxes[i] = &Box{newPos, b.color, b.letter}
//    } else {
//      newBoxes[i] = b
//    }
//  }
//  return newBoxes
//}

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

func isDone(boxes []*Box) int {
  //TODO: make faster
  res := 0
  for _, goal := range goals {
    found := false
    for _, box := range boxes {
      if(goal.pos == box.pos && goal.letter == box.letter){
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

/* Creates a string from the SimpleState that can be used in a hashmap.
 * only robots and boxes is considered, and the runtime is O(#robots+#boxes)
*/ 
func getHash(state *SimpleState) string {
  if state == nil || state.robots == nil || state.boxes == nil {
    dprint("INVALID STATE TO HASH")
    return ""
  }
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
	for _, robot := range state.robots {
		current = make([]*SimpleState, 0)

		_ = current
		_ = robot

	}
	return nil
}

func copyGoals(state *SimpleState) {
  newGoals := make([]agentGoal, len(state.goals))
  newActiveGoals := make([]*agentGoal, len(state.activeGoals))

  for i:=0; i<len(newGoals); i++ {
    newGoals[i] = state.goals[i]
  }

  for i:=0; i<len(newActiveGoals); i++ {
    newActiveGoals[i] = state.activeGoals[i]
  }

  state.goals = newGoals
  state.activeGoals = newActiveGoals
}

func initialActions() *[]agentAction{
  res := make([]agentAction, len(robots))
  for i, _ := range res {
    res[i] = &noop{}
  }
  return &res
}

//func newActionsState(state *SimpleState, i int) *[]agentAction {
//  if(i == 0){
//    return initialActions()
//  }
//  res := make([]agentAction, len(robots))
//  for j, a := range *state.action {
//    res[j] = a
//    dprint((*a).toString())
//  }
//  return &res
//}

//func initialCaluclated() []bool {
//  res := make([]bool, len(robots))
//  for j, _ := range res {
//    res[j] = false
//  }
//  return res
//}

//func newCalculatedState(state *SimpleState, i int) []bool {
//  var res []bool
//  if(i==0){
//    res = initialCaluclated()
//  } else {
//    res = make([]bool, len(robots))
//    for j, b := range state.calculated {
//        res[j] = b
//    }
//  }
//  res[i] = true
//  return res
//}
