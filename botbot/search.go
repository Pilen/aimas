package main

import (
  "strconv"
)

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
  heuristicModifier int // used to punish/award generations of states
}

// func search(robots []Robot, boxes []Box, heuristic func([]Robot, []Box) int, goalReached([]Robot, []Box) bool) {
func search() [][]agentAction {
  costModifier := 0
  dprint("SEARCHING")
  // INITIALIZE STATE
	var frontier Heap
  previous := &SimpleState{initialActions(),make([][]agentAction, 0), 0, make([]Heap, len(robots)), nil, robots, boxes, 0, getInitialGoals(boxes), make([]*agentGoal, len(robots)), 0}
  visitedStates := make(map[string]bool)
  visitedStates[getHash(previous)]=true
  frontier.Insert(previous, 0)

  // Create storage orders on all boxes that lies on a critical path
  //for i, box := range boxes {
  //  if(storage_map[box.pos.x][box.pos.y] == 0){
  //    addStorageOrder(i, previous)
  //  }
  //}

	// A* algorithm
  for !frontier.IsEmpty() {
    dprintf("min key: %d", frontier.minKey())
    previous = frontier.Extract().(*SimpleState)
    if(!frontier.IsEmpty()){
      dprintf("new min key: %d", frontier.minKey())
    }
    aStrings := ""
    for _, a := range (*previous.actualActions) {
      aStrings += a.toString()
    }
    dprintf("checking: " + getHash(previous) + "out of %d type "+ aStrings + " cost: %d ", frontier.Size(), previous.cost)
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

    dprint("in :Combinatins")
    actionCombinations := generateCombinations(previous)
    dprint("out :Combinatins")

    for _, actions := range actionCombinations {
      newState := joinActions(actions, previous)
      newHash := getHash(newState)
      dprintf(newHash)
      if(!visitedStates[newHash]){
        dprintf("inserting cost %d", costModifier * newState.cost + heuristic(newState, 0))
        frontier.Insert(newState, costModifier * newState.cost + heuristic(newState, 0))
        visitedStates[newHash] = true
      }
    }

    // TODO: Dont recalculate heuristic
    if(previous.combinationLevel >= 0){
      frontier.Insert(previous, costModifier * previous.cost + medianKey(previous.actionHeap) + previous.combinationLevel)
    } else {
      dprint("no reinsertion of " + getHash(previous))
    }
  }

  dprint("NO PATH")
  return make([][]agentAction, 0)
}

func medianKey(heaps []Heap) int {
  total := 0

  for _, h := range heaps {
    if(!h.IsEmpty()){
      total += h.minKey()
    }
  }

  return total / len(heaps)
}

// Must have one action for each agent
func joinActions(actions []agentAction, state *SimpleState) *SimpleState {
  newState := nextState(state)
  for i, action := range actions {
      if(action == nil) { // TODO: remove
        dprintf("ERROR: nil action")
        actions[i] = &noop{}
        continue
      }
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
          newRPos := state.boxes[a.boxIdx].pos
          newBPos := applyDirection(state.boxes[a.boxIdx].pos, a.boxDirection)
          // check that the move is still valid after the first agents has moved
          if(isFree(newBPos, newState) && state.boxes[a.boxIdx].pos == applyDirection(state.robots[i].pos, a.agentDirection)){
            checkAddGoal(newState, a.boxIdx)
            newState.robots[i] = &Robot{newRPos, state.robots[i].color, state.robots[i].next}
            newState.boxes[a.boxIdx] = &Box{newBPos, state.boxes[a.boxIdx].color, state.boxes[a.boxIdx].letter}
            checkRemoveGoal(newState, a.boxIdx)
          } else {
            dprint("Invalid push")
            actions[i] = &noop{}
          }
        case *pull:
          dprint("PULL")
          newRPos := applyDirection(state.robots[i].pos, a.agentDirection)
          newBPos := state.robots[i].pos;
          // check that the move is still valid after the first agents has moved
          if(isFree(newRPos, newState) && newState.boxes[a.boxIdx].pos == applyDirection(newState.robots[i].pos, a.boxDirection)){
            checkAddGoal(newState, a.boxIdx)
            newState.robots[i] = &Robot{newRPos, state.robots[i].color, state.robots[i].next}
            newState.boxes[a.boxIdx] = &Box{newBPos, state.boxes[a.boxIdx].color, state.boxes[a.boxIdx].letter}
            checkRemoveGoal(newState, a.boxIdx)
          } else {
            dprint("Invalid pull")
            actions[i] = &noop{}
          }
      }
  }
  newState.actualActions = &actions
  return newState
}

/*
 * Remove a task if it is completed
*/
func checkRemoveGoal(state *SimpleState, boxIdx int){
  for goalIdx, g := range goals {
    if g.pos == state.boxes[boxIdx].pos && g.letter == state.boxes[boxIdx].letter {
      // Find task in active goals and remove if it fits
      for i, t := range state.activeGoals {
        if(t == nil){
          continue
        }
        if(t.exactGoal){
          if(t.boxIdx == boxIdx && t.goalIdx == goalIdx) {
            state.activeGoals[i] = nil
            dprintf("REMOVED active GOAL!! %d=%s", boxIdx, string(g.letter) )
          }
        } else {
          // Check if a non-exact goal has been completed:
          // TODO: make better, what if there are no lvl 2 goals:
        }
      }
      // Find task in global goals
      idx := -1
      for i, t := range state.goals {
        if(t.boxIdx == boxIdx && t.goalIdx == goalIdx) {
          idx = i
          dprintf("REMOVED active GOAL!! %d=%s", boxIdx, string(g.letter) )
          break
        }
      }
      if(idx >= 0){
        state.goals = append(state.goals[:idx], state.goals[idx+1:]...)
      }
      return
    }
  }
}

/*
 * Add a new goal to the state if boxIdx is moved away from a goal
*/
func checkAddGoal(state *SimpleState, boxIdx int){
  for goalIdx, g := range goals {
    if g.pos == state.boxes[boxIdx].pos && g.letter == state.boxes[boxIdx].letter {
      // add goal to put the box back IF no other box is intended for that goal:
      // IS any other box intended for that goal?
      intended := false
      for _, t := range state.activeGoals {
        if(t != nil && t.goalIdx == goalIdx) {
          intended = true
        }
      }
      // Find task in global goals
      for _, t := range state.goals {
        if(t.goalIdx == goalIdx) {
          intended = true
        }
      }
      // Otherwise add that goal
      if(!intended){
        state.goals = append(state.goals, agentGoal{true, boxIdx, goalIdx, Coordinate{-1, -1}})
        dprintf("ADDED GOAL!! %d=%s", boxIdx, string(g.letter) )
      }
      return
    }
  }
}

func generateCombinations(state *SimpleState) [][]agentAction {
  dprintf("combi lvl: %d", state.combinationLevel)
  // If we have no more possible actions
  if state.combinationLevel < 0 {
    return make([][]agentAction, 0)
  }

  state.combinationLevel++ // Increment before, since it starts at 0
  // [robotIdx][iAction] contains robot actions which must be combined
  // iAction is min(state.combinations, heapsize for robotIdx)
  combis := make([][]agentAction, len(state.robots))
  empty := true
  for i:=0; i<len(state.robots); i++ {
    size := min(state.combinationLevel, state.actionHeap[i].size)
    dprintf("Inserting %d, %d, %d", size, state.combinationLevel, state.actionHeap[i].size)
    combisLine := make([]agentAction, 0)
    heuristics := make([]int, size) // TODO: make better
    putback    := make([]interface{}, size)

    if state.actionHeap[i].size > state.combinationLevel {
      empty = false
    }
    for j:=0; j<size; j++ {
      dprintf("adding something %d, %d", i, j)
      heuristics[j] = state.actionHeap[i].minKey(); // TODO: make sure this is correct
      //heuristics = append(heuristics, state.actionHeap[i].minKey())
      // fucking retarded crap typesystem in GO
      // How about GO to hell
      var ac interface{} = state.actionHeap[i].Extract()
      putback[j] = ac
      switch a := (ac).(type) {
        case move:
          dprint(a.toString())
          combisLine = append(combisLine, &a) // how can I put a pointer in a nonpointer array?
        case push:
          dprint(a.toString())
          combisLine= append(combisLine, &a)
        case pull:
          dprint(a.toString())
          combisLine = append(combisLine, &a)
        default:
          dprint("invalid!!!")
      }
    }
    for j, _ := range combisLine {
      state.actionHeap[i].Insert(putback[j], heuristics[j])
    }
    combis[i] = combisLine
  }

  if empty {
    state.combinationLevel = -1
  }

  res := make([][]agentAction, 0)
  for r, actions := range combis { // one pr robot
    for _, action := range actions {
      // For each of the previous results, copy the results and insert all new actions
      if r == 0 {
        robotRes := make([]agentAction, len(combis))
        robotRes[0] = action
        res = append(res, robotRes)
      } else {
        res2 := make([][]agentAction, 0)
        for _, actionsRes := range res {
          robotRes := make([]agentAction, len(combis))
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
  return res
}

/*
 * creates a copy of state containing only the information needed to
 * measure the heurisic of the state.
*/
func nextState(state *SimpleState) *SimpleState {
  newState := SimpleState{nil, make([][]agentAction, 0), 0, make([]Heap, len(state.robots)), state, make([]*Robot, len(state.robots)), make([]*Box, len(state.boxes)), state.cost+1, state.goals, make([]*agentGoal, len(state.activeGoals)), state.heuristicModifier}

  for i, robot := range state.robots {
    newState.robots[i] = robot
  }

  for i, box := range state.boxes {
    newState.boxes[i] = box
  }

  for i, g := range state.activeGoals {
    newState.activeGoals[i] = g
  }

  return &newState
}

func generate_robot_actions(i int, previous *SimpleState, visitedStates *map[string]bool) Heap {

	var actions Heap
  robot := previous.robots[i];

  dprintf("GENERATING: %d; %d for %d", robot.pos.x, robot.pos.y, i)
  if(previous.activeGoals[i] != nil){
    if(previous.activeGoals[i].exactGoal){
      dprintf("GOAL: %s (%d,%d) -> (%d,%d)",string(previous.boxes[previous.activeGoals[i].boxIdx].letter),previous.boxes[previous.activeGoals[i].boxIdx].pos.x, previous.boxes[previous.activeGoals[i].boxIdx].pos.y, goals[previous.activeGoals[i].goalIdx].pos.x, goals[previous.activeGoals[i].goalIdx].pos.y)
    } else {
      dprintf("Inexact GOAL: %s (%d, %d) -> (%d, %d)", string(previous.boxes[previous.activeGoals[i].boxIdx].letter), previous.boxes[previous.activeGoals[i].boxIdx].pos.x, previous.boxes[previous.activeGoals[i].boxIdx].pos.y, previous.activeGoals[i].pos.x,previous.activeGoals[i].pos.y)
    }
  }

	// MOVE
  newState := nextState(previous)
	for _, neighbour := range neighbours(robot.pos) {
    if ( isFree(neighbour, previous) ) {
      newState.robots[i] = &Robot{neighbour, robot.color, robot.next}
      newHash := getHash(newState)
      if(!(*visitedStates)[newHash]){
        dprint((&move{direction(robot.pos, neighbour)}).toString())
        actions.Insert(move{direction(robot.pos, neighbour)}, heuristic(newState, 1))
      }
    // TODO is this good:
    // If a box is is on a critical path, it may be in the way, and we reassign goals,
    // so that if there is a 
		}/* else if storage_map[neighbour.x][neighbour.y] == 0 {
      reassignGoals(previous)
    }*/
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
            // reset what may change during the heuristic calculation
            newState.heuristicModifier = previous.heuristicModifier
            newState.activeGoals[i] = previous.activeGoals[i]
            // apply new state
            newState.robots[i] = &Robot{robot_dest, robot.color, robot.next}
            newState.boxes[bIdx] = &Box{box_dest, box.color, box.letter}
            newHash   := getHash(newState)
            if(!(*visitedStates)[newHash]){
              dprint((&pull{direction(robot.pos, robot_dest), direction(box_dest, box.pos), bIdx}).toString())
              actions.Insert(pull{direction(robot.pos, robot_dest), direction(box_dest, box.pos), bIdx}, heuristic(newState, 1))
            } else {
              dprint("has visited " + newHash)
            }
					}
				}
			} else if isFree(box_dest, previous) {
				// PUSH
        // reset what may change during the heuristic calculation
        newState.heuristicModifier = previous.heuristicModifier
        newState.activeGoals[i] = previous.activeGoals[i]
        // apply new state
        newState.robots[i] = &Robot{box.pos, robot.color, robot.next}
        newState.boxes[bIdx] = &Box{box_dest, box.color, box.letter}
        newHash   := getHash(newState)
        if(!(*visitedStates)[newHash]){
          dprint((&push{direction(robot.pos, box.pos), direction(box.pos, box_dest), bIdx}).toString())
          actions.Insert(push{direction(robot.pos, box.pos), direction(box.pos, box_dest), bIdx}, heuristic(newState, 1))
        }
			}
		}
	}
  return actions
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
