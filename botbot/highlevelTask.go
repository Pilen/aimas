package main

import (
    //"strconv"
)

var enable_storage bool

var storage_map [][]int
// -1 = wall
// 0 = on a critical path
// 1 = not on a crititical path, but is road
// 2 = not on a critical path, is room


type agentGoal struct {
  exactGoal bool //If true the box must be moved to the goal, otherwise it should just end up somewhere around (for storage)
  boxIdx int
  goalIdx int
  pos Coordinate // the coordinate of the approximate storage location, used when exactGoal == false
}

/*
 * Put all active goals into unassigned goals, and set the active goals to nil,
 * so that they will be reassigned in the next iteration.
 *
 * This may create more fitting goals
*/
func reassignGoals(state *SimpleState){
  dprint("Reassigning goals")
  for i, g := range state.activeGoals {
    if(g != nil){
      state.goals = append(state.goals, *g)
      state.activeGoals[i] = nil
    }
  }

}

func heuristic(state *SimpleState, heuristic int) int {
  // TODO: When a goal is reached the next goal is picked the heuristic 
  //       grows a lot, and the solutions that only almost solves the task is picked.

  // totalDistance is the sum of distances from each agent to its goal
  //////////////////////////////////////////////////////////////////////////////
  totalDistance := 0
  for i, r := range state.robots {
    totalDistance += heuristicForAgent(i, r, state, true)
  }

  // goalDistance is the distance of all EXACT goals remaining:
  //////////////////////////////////////////////////////////////////////////////
  goalDistance := 0
  for _, g := range state.goals {
    box := state.boxes[g.boxIdx]
    if(g.exactGoal){
      goal := goals[g.goalIdx]
      goalDistance += checked_distance(box.pos, goal.pos)
    } else if (g.pos != Coordinate{-1, -1}) {
      goalDistance += checked_distance(box.pos, g.pos)
    }
  }

  // goalCount is the number of goals that does not have a box and does not have
  // priority less than an incomplete goal of higher priority
  // This makes it more expensive to move boxes that are already on a box
  //////////////////////////////////////////////////////////////////////////////
  //goalCount := isDone(state)
  minIncPrioriy := 0
  for _, goal := range goals {
    found := false
    for _, box := range state.boxes {
      if(goal.pos == box.pos && goal.letter == box.letter){
        found = true
        break
      }
    }
    if !found && goal.priority > minIncPrioriy {
      minIncPrioriy = goal.priority
    }
  }
  goalCount := 0
  for _, goal := range goals {
    found := false
    for _, box := range state.boxes {
      if(goal.pos == box.pos && goal.letter == box.letter && goal.priority >= minIncPrioriy){
        found = true
        break
      }
    }
    if !found {
       goalCount += 1
    }
  }


  // Storage task count:
  //////////////////////////////////////////////////////////////////////////////
  storageTaskCount := 0
  // add in-exact goals:
  for _, a := range state.goals {
    if(!a.exactGoal){
      dprint("found inexact goal")
      if(!storageGoalIsFinised(&a, state)){
        dprint("incomplete in-exact goal")
        storageTaskCount += 1
      }
    }
  }

  for _, a := range state.activeGoals {
    if(a != nil && !a.exactGoal){
      dprint("found inexact goal")
      if(!storageGoalIsFinised(a, state)){
        dprint("incomplete in-exact goal")
        storageTaskCount += 1
      }
    }
  }
  //goalCount := len(state.goals)// TODO: len of goals does not work as a heuristic, why?
  //for _, ag := range state.activeGoals {
  //  if(ag != nil){
  //    goalCount += 1
  //  }
  //}

  //dprint("inactive goals")
  //for _, g := range state.goals {
  //  if(g.exactGoal){
  //    dprint("exact")
  //    //goal := goals[g.box
  //    //print("%v, (%d, %d) -> (%d, %d)", g.
  //  } else {
  //    dprint("non-exact")
  //  }
  //}

  //dprint("active goals")
  //for _, g := range state.activeGoals {
  //  if(g == nil){
  //    dprint("nil")
  //  } else if (g.exactGoal) {
  //    dprint("exact")
  //  } else {
  //    dprint("non-exact")
  //  }
  //}

  // storage punishmet adds up punishment for each box that is on a non-storage
  // area
  //////////////////////////////////////////////////////////////////////////////
  storagePun := 0
  for i, box := range state.boxes {
      // If the box is an active goal, we do not punish the box for being at a
      // critical path
      boxIsActive := false
      for _, goal := range state.activeGoals {
          if(goal != nil && goal.boxIdx == i){
            boxIsActive = true
            break
          }
      }
      if(boxIsActive){
        break
      }

      // Deside punishment based on the storage level of the box
      storage := storage_map[box.pos.x][box.pos.y]
      if(storage == 0){  // 0 = on a critical path
        storagePun += 2
      }
      if(storage == 1){  // 1 = not on a crititical path, but is road
        storagePun += 1
      }
      if(storage == 2){  // 2 = not on a critical path, is room
        storagePun += 0
      }
  }

  // sameRoad is the number of agents that are on the same road:
  //////////////////////////////////////////////////////////////////////////////
  sameRoad := 0
  visitedRoads := make(map[int]bool)
  for _, agent := range state.robots {
      roomIdx := room_map[agent.pos.x][agent.pos.y]
      if(!rooms[roomIdx].isRoom){
        if(visitedRoads[roomIdx]){
          sameRoad += 1
        } else {
          visitedRoads[roomIdx] = true
        }
      }
  }

  // boxInCorridorPun is the number of agents that are pushing/pulling a box
  // inside a corridor when the box is not part of its goal
  //////////////////////////////////////////////////////////////////////////////
  boxInCorridorPun := 0
  //if(state.actualActions != nil){
  //  for _, action := range *state.actualActions {
  //    var ac interface{} = action
  //    switch a := (ac).(type) {
  //      case *move:
  //        // not pushing a box
  //      case *push:
  //        // is the box being pushed the box from its goal
  //        if(
  //        //boxAction := state.boxes[a.boxIdx]
  //      
  //      case *pull:
  //        box := state.boxes[a.boxIdx]
  //    }
  //  }
  //}

  // heuristicModifier is used to remember how many storage goals has been completed.
  //////////////////////////////////////////////////////////////////////////////
  modifier := state.heuristicModifier

  if(heuristic == 0 || true){ // TODO: maybe use different heuristics for states and actions
    goalCount        = goalCount        * 100
    totalDistance    = totalDistance    * 1
    goalDistance     = goalDistance     * 1
    storageTaskCount = storageTaskCount * -20
    storagePun       = storagePun       * 0
    sameRoad         = sameRoad         * 30
    modifier         = modifier         * -45
    boxInCorridorPun = boxInCorridorPun * 0
  }
  result := goalCount + totalDistance + goalDistance + storageTaskCount + storagePun + sameRoad + modifier + boxInCorridorPun

  dprintf("H = %d, tD: %d, gD: %d, gC: %d, sp: %d sr: %d storageC: %d heuMod: %d, boxInCor: %d", result, totalDistance, goalDistance, goalCount, storagePun, sameRoad, storageTaskCount, modifier, boxInCorridorPun)
  return result
}

func addStorageOrder(boxIdx int, state *SimpleState) {
  if(!enable_storage){
    return
  }
  // Find storage area
  box := state.boxes[boxIdx]
  dprint("Adding storage order")
  state.goals = append(state.goals, agentGoal{false, boxIdx, room_map[box.pos.x][box.pos.y], Coordinate{-1, -1}})
}

func newGoal(robotIdx int, state *SimpleState) {

  copyGoals(state)

  dprintf("New goal for %d", robotIdx)
  var nextGoal agentGoal
  idx := -1
  distance := 9999999
  priority := 0
  for i, g := range state.goals {
    box := state.boxes[g.boxIdx]
    robot := state.robots[robotIdx]
    // Check if robot and box are all compatible
    if(box.color != robot.color){
      continue
    }

    if(g.exactGoal) {
      goal := goals[g.goalIdx] // TODO: this will not work when using storage tasks
      newDistA := checked_distance(robot.pos, box.pos)
      newDistB := checked_distance(box.pos, goal.pos)
      if(newDistA < 0 || newDistB < 0){
        continue
      }

      if(priority < goal.priority || priority == goal.priority && distance > newDistA + newDistB) {
        idx = i
        nextGoal = g
        distance = newDistA + newDistB
        priority = goal.priority
      }
    } else {
      newDistA := checked_distance(robot.pos, box.pos)
      //newDistB := checked_distance(box.pos, g.pos)
      //if(newDistB < 0){
      //  newDistB = 0
      //}
      dprint("checking Storage GOAL! %v dist: %d", string(boxes[g.boxIdx].letter), newDistA)
      if(priority < 999999 || distance > newDistA/* + newDistB*/){
        idx = i
        nextGoal = g
        priority = 999999 // always higher priority than ordinary goals
        distance = newDistA/* + newDistB*/
        // distance = TODO
        dprintf("FANDT Storage GOAL! %v", string(boxes[g.boxIdx].letter))
      }
    }
  }

  if(idx < 0){
    // TODO: handle
    dprint("no goal found");
    return
  }

  state.goals = append(state.goals[:idx], state.goals[idx+1:]...)
  state.activeGoals[robotIdx] = &nextGoal
}

// return true if the goal is a storage goal, and it has been completed
func storageGoalIsFinised(goal *agentGoal, state *SimpleState) bool {

  if(goal == nil || goal.exactGoal){
    dprintf("bad false")
    return false
  }
  dprint("checking")
  box := state.boxes[goal.boxIdx]
  roomIdx := room_map[box.pos.x][box.pos.y]
  // We must be at a proper storage location, AND not in the same corridor
  // that we are trying to move away from (or the cell just outside the corridor
  //printf("storage val: %d comp: %d <> %d box(%d, %d) =? (%d,%d) =? (%d,%d)", storage_map[box.pos.x][box.pos.y], goal.goalIdx,roomIdx, box.pos.x, box.pos.y, rooms[roomIdx].in_pos.x, rooms[roomIdx].in_pos.y, rooms[roomIdx].out_pos.x, rooms[roomIdx].out_pos.y)
  res := storage_map[box.pos.x][box.pos.y] >= 0 && goal.goalIdx != roomIdx && box.pos != rooms[goal.goalIdx].in_pos && box.pos != rooms[goal.goalIdx].out_pos

  if(res){
    dprint("storage goal check: TRUE")
  } else {
    dprint("storage goal check: FLASE")
  }

  return res
}

func heuristicForAgent(i int, r *Robot, state *SimpleState, again bool) int {
  if(state.activeGoals[i] == nil) {
    dprint("Goal is nil!!");
    newGoal(i, state)
  }

  if(state.activeGoals[i] == nil){
    // if there is no goal, we punish depending on the storage value of this position
    return 2 - storage_map[r.pos.x][r.pos.y]
  }
  aGoal := state.activeGoals[i]
  robot := state.robots[i]
  box   := state.boxes[aGoal.boxIdx]
  distA := checked_distance(robot.pos, box.pos)

  // if we are in a corridor we will punish the robot if the next box in the
  // corridor is not part of its goal:
  roomIdx := room_map[robot.pos.x][robot.pos.y]
  if(!rooms[roomIdx].isRoom && enable_storage){
    nextBoxIdx := -1
    for _, pos := range neighbours(robot.pos) {
      // is the neigbour in the same room (corridor) as the agent
      if(room_map[pos.x][pos.y] == roomIdx) {
        nextBoxIdx = nextBoxInWay(pos, robot.pos, roomIdx, state)
        if(nextBoxIdx >= 0){ // if we found a box, break, otherwise look in the other direction
          break 
        }
      }
    }

    // if we have found a box in the way, and it is NOT the box from the goal
    if(nextBoxIdx >= 0 && (aGoal == nil || nextBoxIdx != aGoal.boxIdx)){
      // If we cannot move the box, a storage task is added:
      if(robot.color != state.boxes[nextBoxIdx].color) {
        state.goals = append(state.goals, agentGoal{false, nextBoxIdx, room_map[state.boxes[nextBoxIdx].pos.x][state.boxes[nextBoxIdx].pos.y], Coordinate{-1, -1}})
      } else {
        // save the current agent goal:
        if(aGoal != nil){
          state.goals = append(state.goals, *aGoal)
        }
        // If a store box goal already exists, then remove it:
        idx := -1
        for j, g := range state.goals {
          if(g.exactGoal && g.boxIdx == nextBoxIdx){
            idx = j
          }
        }
        if(idx >= 0){
          state.goals = append(state.goals[:idx], state.goals[idx+1:]...)
        }

        for j, g := range state.activeGoals {
          if(g != nil && g.exactGoal && g.boxIdx == nextBoxIdx){
            state.activeGoals[j] = nil
          }
        }

        // Now we can assign a move goal to this agent
        dprint("adding inexact goal to agent")
        state.activeGoals[i] = &agentGoal{false, nextBoxIdx, room_map[state.boxes[nextBoxIdx].pos.x][state.boxes[nextBoxIdx].pos.y], Coordinate{-1, -1}}
        //state.activeGoals[i] =  &agentGoal{false, 1, 1, Coordinate{-1, -1}}
      }
    }
  }

  // Are we moving a box to its goal
  if(aGoal.exactGoal){
    goal := goals[aGoal.goalIdx]

    distB := checked_distance(box.pos, goal.pos)


    //if state.boxes[state.activeGoals[i].boxIdx].pos == goals[state.activeGoals[i].goalIdx].pos {
    //  state.activeGoals[i] = nil
    //}
    return distA + distB
  }

  if(!enable_storage){
    return 0
  }

  //Otherwise move to a storage area
  // If the box has a goal, we should prioritize storage areas that are close to the goal
  // Find the position of an exact goal with this box

  // are we done now?
  if(storageGoalIsFinised(aGoal, state)){
    state.activeGoals[i] = nil
    dprint("removed storage goal")
    state.heuristicModifier += 1
    return 0
  }

  dprint("Calculating heuristic for storage goal")
  goalPos := Coordinate{-1,-1}
  for _, agentG := range state.activeGoals {
    if(agentG != nil && agentG.exactGoal && agentG.boxIdx == aGoal.boxIdx){
      goalPos = goals[agentG.goalIdx].pos
      break
    }
  }

  for _, agentG := range state.goals {
    if(agentG.exactGoal && agentG.boxIdx == aGoal.boxIdx){
      goalPos = goals[agentG.goalIdx].pos
      break
    }
  }

  // find best storage position:
  storageValue := -1
  storagePos := Coordinate{-1, -1}
  storageDistance := 999999
  for x:=0; x<width; x++ {
    for y:=0; y<height; y++ {
      if(!isFree(Coordinate{x, y}, state)){
        continue
      }
      newSVal := storage_map[x][y]
      distance := checked_distance(box.pos, Coordinate{x,y})
      if(distance < 0) {
        continue
      }
      if(goalPos != Coordinate{-1, -1}){
        d2 := checked_distance(goalPos, Coordinate{x,y})
        if(d2 < 0) {
          continue
        }
        distance += d2
      }

      if(storageValue < newSVal || storageValue == newSVal && distance < storageDistance){
        storageValue = newSVal
        storagePos = Coordinate{x, y}
        storageDistance = distance
        state.activeGoals[i].pos = storagePos
      }
    }
  }

  if storageValue >= 0 {
    return storageDistance + distA
  }

  return distA
  //// Storage: // if we are in a room that is not the room that we are moving the box from
  //if(rooms[room_map[box.pos.x][box.pos.y]].isRoom && agentGoal.goalIdx != room_map[box.pos.x][box.pos.y]){
  //  // are we done?
  //  if(distA <= 1){
  //    state.activeGoals[i] = nil
  //  }
  //  return distA // TODO: plus more
  //}

  //// If we are in a corridor, we need to get out
  //if(!rooms[room_map[box.pos.x][box.pos.y]].isRoom){

  //}
  //return 0
}

func getInitialGoals(boxes []*Box) []agentGoal{

  reserved := make([]bool, len(boxes))
  agentGoals := make([]agentGoal, 0)

  for i, g := range goals {
    var box int
    distance := 999999999

    for j, b := range boxes {
      newDist := checked_distance(b.pos, g.pos)
      if(newDist < 0 || b.letter != g.letter || reserved[j]){
        continue
      }
      if(distance > newDist){
        box = j
        distance = newDist
      }
    }
    agentGoals = append(agentGoals, agentGoal{true, box, i, Coordinate{-1, -1}})
    reserved[box] = true
  }

  dprint("GOALS:")
  for _, g := range agentGoals {
    dprintf("%v (%d,%d) -> %v (%d,%d)",boxes[g.boxIdx].letter, boxes[g.boxIdx].pos.x, boxes[g.boxIdx].pos.y, goals[g.goalIdx].letter, goals[g.goalIdx].pos.x, goals[g.goalIdx].pos.y)
  }

  calculate_storage(agentGoals)
  return agentGoals
}

/*
 * initialize storage map to the values:
 * -1 = wall
 * 0 = on a critical path
 * 1 = not on a crititical path, but is road
 * 2 = not on a critical path, is room
 * 3 = TODO none of the neigbours are blocked - in th middle of the room => good storage
 *
 * This is calculated using the initial positions of boxes
*/
func calculate_storage(aGoals []agentGoal) {
  // initialize storage map:
  storage_map = make([][]int, width);
  for x := 0; x < width; x++ {
      storage_map[x] = make([]int, height);
      for y := 0; y<height; y++ {
        roomIdx := room_map[x][y]
        if(roomIdx == -1){
          storage_map[x][y] = -1
          continue
        }

        if(rooms[roomIdx].isRoom && rooms[roomIdx].size == 1){ // connections are bad storage areas
          storage_map[x][y] = 0
        } else if rooms[roomIdx].isRoom {
          storage_map[x][y] = 2
          // if either of the room cells neigbours are a corridor, this is not a good storage spot:
          for _, pos := range neighbours(Coordinate{x,y}) {
            if(room_map[pos.x][pos.y] >= 0 && !rooms[room_map[pos.x][pos.y]].isRoom){
              storage_map[x][y] = 1
              break
            }
          }
        } else {
          storage_map[x][y] = 1
        }
      }
  }

  // mark locations in storage map that are critical paths
  for _, g := range aGoals {
    markPath(boxes[g.boxIdx].pos, goals[g.goalIdx].pos)
    // if the goal or box is inside a road, we mark the whole road
    roomIdxB := room_map[boxes[g.boxIdx].pos.x][boxes[g.boxIdx].pos.y]
    startB   := rooms[roomIdxB].in_pos
    endB     := rooms[roomIdxB].out_pos
    markRoad(Coordinate{-1, -1}, startB, endB, roomIdxB, 0)

    roomIdxG := room_map[goals[g.goalIdx].pos.x][goals[g.goalIdx].pos.y]
    startG   := rooms[roomIdxG].in_pos
    endG     := rooms[roomIdxG].out_pos
    markRoad(Coordinate{-1, -1}, startG, endG, roomIdxG, 0)
  }

  for _, g := range goals {
    storage_map[g.pos.x][g.pos.y] = 0
  }


  // Debug print
//  for j:=0; j<height; j++ {
//    str := ""
//    for i:=0; i<width; i++ {
//      if storage_map[i][j] >= 0 {
//        str = str + " "
//      }
//      str = str + strconv.Itoa(storage_map[i][j])
//    }
//    print(str)
//  }

}

/*
 * Iterates through all cells on a road between start and end coordinate and
 * marks the cells with storage value storageVal  
*/
func markRoad(previous, current, endCoord Coordinate, roadIdx, storageVal int) {
  if(!isInsideP(current)){
    return
  }
  storage_map[current.x][current.y] = storageVal

  if(current == endCoord){
    return
  }

  for _, pos := range neighbours(current) {
    if(pos != previous && room_map[current.x][current.y] == roadIdx){
      markRoad(current, pos, endCoord, roadIdx, storageVal)
    }
  }
}

/*
 * Goes through a path, and finds the next box
*/
func nextBoxInWay(current, previous Coordinate, room int, state *SimpleState) int {
  if(room_map[current.x][current.y] != room){
    return -1
  }
  for i, b := range state.boxes {
    if(b.pos == current){
      return i
    }
  }

  for _, pos := range neighbours(current) {
    if(room_map[pos.x][pos.y] == room && pos != previous){
      return nextBoxInWay(pos, current, room, state)
    }
  }

  return -1
}
