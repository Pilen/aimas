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


// For improving agent heuristics, calculated once per state
var occupied_map[70][70]bool

/*
 * Put all active goals into unassigned goals, and set the active goals to nil,
 * so that they will be reassigned in the next iteration.
 *
 * This may create more fitting goals
*/
func reassignGoals(state *State){
  dprint("Reassigning goals")
  for i, t := range state.activeTasks{
    if(t != nil){
      state.unassignedTasks = append(state.unassignedTasks, *t)
      state.activeTasks[i] = nil
    }
  }

}

func heuristic(state *State, heuristic int) int {
  // TODO: When a task is completed and the next is picked the heuristic
    //       grows a lot, and the solutions that only almost solves the task is picked.

    for x := 0; x < width; x++ {
        for y := 0; y < height; y++ {
            occupied_map[x][y] = wallMap[x][y]
        }
    }
    for _, b := range state.boxes {
        occupied_map[b.pos.x][b.pos.y] = true
    }
    // Todo should other robots be considered in occupancy?
    for _, r := range state.robots {
        occupied_map[r.pos.x][r.pos.y] = true
    }

  // totalDistance is the sum of distances from each agent to its goal
  //////////////////////////////////////////////////////////////////////////////
  totalDistance := 0
  for i, r := range state.robots {
    totalDistance += heuristicForAgent(i, r, state, true)
  }

  // taskDistance is the distance of all EXACT goals remaining:
  //////////////////////////////////////////////////////////////////////////////
  taskDistance := 0
  for _, t := range state.unassignedTasks {
    box := state.boxes[t.boxIdx]
    if(t.exactGoal){
      goal := goals[t.goalIdx]
      taskDistance += checked_distance(box.pos, goal.pos)
    } else if (t.pos != Coordinate{-1, -1}) {
      taskDistance += checked_distance(box.pos, t.pos)
    }
  }

  // goalCount is the number of goals that does not have a box and does not have
  // priority less than an incomplete goal of higher priority
  // This makes it more expensive to move boxes that are already on a box
  //////////////////////////////////////////////////////////////////////////////
  //goalCount := isDone(state.boxes)
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
  for _, a := range state.unassignedTasks {
    if(!a.exactGoal){
      dprint("found inexact goal")
      if(!storageGoalIsFinised(&a, state)){
        dprint("incomplete in-exact goal")
        storageTaskCount += 1
      }
    }
  }

  for _, a := range state.activeTasks{
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
      // If the box is an active task, we do not punish the box for being at a
      // critical path
      boxIsTask := false
      for _, task := range state.activeTasks {
          if(task != nil && task.boxIdx == i){
            boxIsTask = true
            break
          }
      }
      if(boxIsTask){
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

  // Level misconfiguration punishment
  //////////////////////////////////////////////////////////////////////////////
  misconfiguration := levelMisconfiguration(state)

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
    totalDistance = totalDistance * 1
    taskDistance  = taskDistance  * 1
    storageTaskCount = storageTaskCount * -20
    goalCount     = goalCount     * 100
    storagePun    = storagePun    * 2
    sameRoad      = sameRoad      * 30
    modifier         = modifier         * -45
    misconfiguration = misconfiguration * 0
    boxInCorridorPun = boxInCorridorPun * 0
  }
  result := totalDistance + taskDistance + storageTaskCount + goalCount + storagePun + sameRoad + modifier + misconfiguration + boxInCorridorPun

  //dprintf("H = %d, tD: %d, gD: %d, gC: %d, sp: %d, sr: %d, m: %d", result, totalDistance, taskDistance, goalCount, storagePun, sameRoad, misconfiguration)

  return result
}

func addStorageOrder(boxIdx int, state *State) {
  if(!enable_storage){
    return
  }
  // Find storage area
  box := state.boxes[boxIdx]
  dprint("Adding storage order")
  state.unassignedTasks = append(state.unassignedTasks, Task{false, boxIdx, room_map[box.pos.x][box.pos.y], Coordinate{-1, -1}})
}

func newTask(robotIdx int, state *State) {

  copyTasks(state)

  dprintf("New task for %d", robotIdx)
  var nextTask Task
  idx := -1
  distance := 9999999
  priority := 0
  for i, t := range state.unassignedTasks {
    box := state.boxes[t.boxIdx]
    robot := state.robots[robotIdx]
    // Check if robot and box are all compatible
    if(box.color != robot.color){
      continue
    }

    if(t.exactGoal) {
      goal := goals[t.goalIdx]
      newDistA := checked_distance(robot.pos, box.pos)
      newDistB := checked_distance(box.pos, goal.pos)
      if(newDistA < 0 || newDistB < 0){
        continue
      }

      if(priority < goal.priority || priority == goal.priority && distance > newDistA + newDistB) {
        idx = i
        nextTask = t
        distance = newDistA + newDistB
        priority = goal.priority
      }
    } else {
      newDistA := checked_distance(robot.pos, box.pos)
      //newDistB := checked_distance(box.pos, g.pos)
      //if(newDistB < 0){
      //  newDistB = 0
      //}
      dprint("checking Storage GOAL! %v dist: %d", string(boxes[t.boxIdx].letter), newDistA)
      if(priority < 999999 || distance > newDistA/* + newDistB*/){
        idx = i
        nextTask = t
        priority = 999999 // always higher priority than ordinary goals
        distance = newDistA/* + newDistB*/
        // distance = TODO
        dprintf("FANDT Storage GOAL! %v", string(boxes[t.boxIdx].letter))
      }
    }
  }

  if(idx < 0){
    // TODO: handle
    dprint("no goal found");
    return
  }

  state.unassignedTasks = append(state.unassignedTasks[:idx], state.unassignedTasks[idx+1:]...)
  state.activeTasks[robotIdx] = &nextTask
}

// return true if the goal is a storage goal, and it has been completed
func storageGoalIsFinised(task *Task, state *State) bool {

  if(task == nil || task.exactGoal){
    dprintf("bad false")
    return false
  }
  dprint("checking")
  box := state.boxes[task.boxIdx]
  roomIdx := room_map[box.pos.x][box.pos.y]
  // We must be at a proper storage location, AND not in the same corridor
  // that we are trying to move away from (or the cell just outside the corridor
  //printf("storage val: %d comp: %d <> %d box(%d, %d) =? (%d,%d) =? (%d,%d)", storage_map[box.pos.x][box.pos.y], goal.goalIdx,roomIdx, box.pos.x, box.pos.y, rooms[roomIdx].in_pos.x, rooms[roomIdx].in_pos.y, rooms[roomIdx].out_pos.x, rooms[roomIdx].out_pos.y)
  res := storage_map[box.pos.x][box.pos.y] >= 0 && task.goalIdx != roomIdx && box.pos != rooms[task.goalIdx].in_pos && box.pos != rooms[task.goalIdx].out_pos

  if(res){
    dprint("storage task check: TRUE")
  } else {
    dprint("storage task check: FLASE")
  }

  return res
}

func heuristicForAgent(i int, r *Robot, state *State, again bool) int {
  if(state.activeTasks[i] == nil) {
    dprint("Task is nil!!");
    newTask(i, state)
  }

  if(state.activeTasks[i] == nil){
    // if there is no goal, we punish depending on the storage value of this position
    return 2 - storage_map[r.pos.x][r.pos.y]
  }

  task  := state.activeTasks[i]
  robot := state.robots[i]
  box   := state.boxes[task.boxIdx]

  distA := checked_distance(robot.pos, box.pos)
  badness := badnessOfShortestPath(i, r, state)

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
    if(nextBoxIdx >= 0 && (task == nil || nextBoxIdx != task.boxIdx)){
      // If we cannot move the box, a storage task is added:
      if(robot.color != state.boxes[nextBoxIdx].color) {
        state.unassignedTasks = append(state.unassignedTasks, Task{false, nextBoxIdx, room_map[state.boxes[nextBoxIdx].pos.x][state.boxes[nextBoxIdx].pos.y], Coordinate{-1, -1}})
      } else {
        // save the current agent goal:
        if(task != nil){
          state.unassignedTasks = append(state.unassignedTasks, *task)
        }
        // If a store box goal already exists, then remove it:
        idx := -1
        for j, g := range state.unassignedTasks {
          if(!g.exactGoal && g.boxIdx == nextBoxIdx){
            idx = j
          }
        }
        if(idx >= 0){
          state.unassignedTasks = append(state.unassignedTasks[:idx], state.unassignedTasks[idx+1:]...)
        }

        for j, t := range state.activeTasks {
          if(t != nil && !t.exactGoal && t.boxIdx == nextBoxIdx){
            state.activeTasks[j] = nil
          }
        }

        // Now we can assign a move goal to this agent
        dprint("adding inexact goal to agent")
        state.activeTasks[i] = &Task{false, nextBoxIdx, room_map[state.boxes[nextBoxIdx].pos.x][state.boxes[nextBoxIdx].pos.y], Coordinate{-1, -1}}
      }
    }
  }

  // Are we moving a box to its goal
  if(task.exactGoal){
    goal := goals[task.goalIdx]
    distB := checked_distance(box.pos, goal.pos)


    //if state.boxes[state.activeGoals[i].boxIdx].pos == goals[state.activeGoals[i].goalIdx].pos {
    //  state.activeGoals[i] = nil
    //}
    // GIT removed in merge
    return distA + distB + badness
  }
  // GIT removed in merge
  //// Storage: // if we are in a room that is not the room that we are moving the box from
  //if(rooms[room_map[box.pos.x][box.pos.y]].isRoom && task.goalIdx != room_map[box.pos.x][box.pos.y]){
  //  // are we done?
  //  if(distA <= 1){
  //    state.activeTasks[i] = nil
  //  }
  //  return distA + badness // TODO: plus more

  if(!enable_storage){
    return badness
  }

  //Otherwise move to a storage area
  // If the box has a goal, we should prioritize storage areas that are close to the goal
  // Find the position of an exact goal with this box

  // are we done now?
  if(storageGoalIsFinised(task, state)){
    state.activeTasks[i] = nil
    dprint("removed storage goal")
    state.heuristicModifier += 1
    return badness
  }

  dprint("Calculating heuristic for storage goal")

  // Find out if there exists a task needing this box, if so, we want to store
  // it closer to the goal
  goalPos := Coordinate{-1,-1}
  for _, t := range state.activeTasks {
    if(t != nil && t.exactGoal && t.boxIdx == t.boxIdx){
      goalPos = goals[t.goalIdx].pos
      break
    }
  }

  for _, t := range state.unassignedTasks{
    if(t.exactGoal && t.boxIdx == t.boxIdx){
      goalPos = goals[t.goalIdx].pos
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
        state.activeTasks[i].pos = storagePos
      }
    }
  }

  if storageValue >= 0 {
    return storageDistance + distA + badness
  }
  return distA + badness
}

func badnessOfShortestPath(i int, robot *Robot, state *State) int {
    // Should we simply ignore robots or not?
    task := state.activeTasks[i]
    goal := goals[task.goalIdx]
    box := state.boxes[task.boxIdx]
    _ = box
    var destination Coordinate


    var distance_to [70][70]int
    for x := 0; x < width; x++ {
        for y := 0; y < height; y++ {
            distance_to[x][y] = 9000 // Infinity... ish
        }
    }

    var came_from [70][70]Coordinate
    var frontier Heap

    if isNeigbours(robot.pos, goal.pos) {
        // determine path from box to goal
        if !task.exactGoal {
            // Move box around
            return 0
        } else {
            // ignore / add current box position
            distance_to[robot.pos.x][robot.pos.y] = 0
            distance_to[goal.pos.x][goal.pos.y] = 1
            came_from[goal.pos.x][goal.pos.y] = robot.pos
            frontier.Insert(robot.pos, 0)
            frontier.Insert(goal.pos, 1)
            destination = goal.pos
        }
    } else {
        // determine path from robot to box
        // distance_to[robot.pos.x][robot.pos.y] = 0
        // frontier.Insert(robot.pos, 0)
        // destination = box.pos

        // TODO fix this so we can estimate distance to box
        return 0
    }


    destination_room := room_map[destination.x][destination.y]
    var current Coordinate
    var distance int
    for !frontier.IsEmpty() {
        current = frontier.Extract().(Coordinate)
        distance = distance_to[current.x][current.y]
        current_room := room_map[current.x][current.y]

        // if done?
        if (current == destination) { // We want to end when we completed the path
            goto done
        }
        if isNeigbours(current, destination) {
            goto done
        }
        if (current_room != destination_room) { // We want to ensure we can get through the final room
            if (rooms[current_room].isRoom) {
                if distance > 20 {
                    goto done
                }
            } else {
                if distance > 40 {
                    goto done

                }
            }
        }
        new_distance := distance + 1
        for _, neighbour := range neighbours(current) {
            if (occupied_map[neighbour.x][neighbour.y] ||
                new_distance < distance_to[neighbour.x][neighbour.y]) {

                distance_to[neighbour.x][neighbour.y] = new_distance
                came_from[neighbour.x][neighbour.y] = current;
                priority := new_distance + checked_distance(neighbour, destination)
                frontier.Insert(neighbour, priority)
            }
        }
    }
    // There is no way!
    return 100
done:
    // Badness is defined as how far away from the destination we are by going "distance" steps by the acual level compared to the empty
    ideal := robot.pos
    ideal_remaining := 9000 //infinity
    for i := 0; i <= distance; i++ {
        if ideal == destination {
            ideal_remaining = 0;
            break
        }
        if isNeigbours(ideal, destination) {
            break
        }
        for _, neighbour := range neighbours(ideal) {
            new_remaining := checked_distance(neighbour, destination)
            if new_remaining < ideal_remaining {
                ideal = neighbour
                ideal_remaining = new_remaining
            }
        }
    }
    return checked_distance(current, destination) - ideal_remaining
}

func levelMisconfiguration(state *State) int {
    total_badness := 0

    room_fill := make([]int, len(rooms))
    completed := make([]bool, len(state.boxes))

    for _, box := range state.boxes {
        room_id := room_map[box.pos.x][box.pos.y]
        room_fill[room_id]++
        goal_id := goalMap[box.pos.x][box.pos.y]
        if goal_id >= 0 {
            goal := goals[goal_id]
            if box.letter == goal.letter {
                completed[goal_id] = true
            }
        }
    }

    for goal_id, goal := range goals {
        if completed[goal_id] {
            continue
        }
        room := rooms[room_map[goal.pos.x][goal.pos.y]]
        if room.isRoom {
            for _, corridor_id := range room.connections {
                fill := room_fill[corridor_id]
                total_badness += goal.priority * fill
            }
        }
    }

    dprint("total_badness: ", total_badness)
    return total_badness
}

func getInitialTasks(boxes []*Box) []Task{

  reserved := make([]bool, len(boxes))
  tasks := make([]Task, 0)

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
    tasks = append(tasks, Task{true, box, i, Coordinate{-1, -1}})
    reserved[box] = true
  }

  dprint("TASKS:")
  for _, t := range tasks {
    dprintf("%v (%d,%d) -> %v (%d,%d)",boxes[t.boxIdx].letter, boxes[t.boxIdx].pos.x, boxes[t.boxIdx].pos.y, goals[t.goalIdx].letter, goals[t.goalIdx].pos.x, goals[t.goalIdx].pos.y)
  }

  calculate_storage(tasks)
  return tasks
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
func calculate_storage(tasks []Task) {
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
  for _, t := range tasks {
    markPath(boxes[t.boxIdx].pos, goals[t.goalIdx].pos)
    // if the goal or box is inside a road, we mark the whole road
    roomIdxB := room_map[boxes[t.boxIdx].pos.x][boxes[t.boxIdx].pos.y]
    startB   := rooms[roomIdxB].in_pos
    endB     := rooms[roomIdxB].out_pos
    markRoad(Coordinate{-1, -1}, startB, endB, roomIdxB, 0)

    roomIdxG := room_map[goals[t.goalIdx].pos.x][goals[t.goalIdx].pos.y]
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
func nextBoxInWay(current, previous Coordinate, room int, state *State) int {
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
