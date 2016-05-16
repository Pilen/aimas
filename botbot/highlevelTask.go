package main

import (
    "strconv"
)

var storage_map [][]int
// -1 = wall
// 0 = on a critical path
// 1 = not on a crititical path, but is road
// 2 = not on a critical path, is room


// For improving agent heuristics, calculated once per state
var occupied_map[70][70]bool


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

  // taskDistance is the distance of all tasks remaining:
  // TODO: This is actually already in the cost of the total distance?
  // TODO: Also, how does this affect storage tasks?
  //////////////////////////////////////////////////////////////////////////////
  taskDistance := 0
  for _, t := range state.unassignedTasks {
    box := state.boxes[t.boxIdx]
    goal := goals[t.goalIdx]
    taskDistance += checked_distance(box.pos, goal.pos)
  }

  // goalCount is the number of goal that does not have a box
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

  // Level misconfiguration punishment
  //////////////////////////////////////////////////////////////////////////////
  misconfiguration := levelMisconfiguration(state)

  //taskCount := len(state.tasks) TODO: len of tasks does not work as a heuristic, why?
  if(heuristic == 0 || true){ // TODO: maybe use different heuristics for states and actions
    totalDistance = totalDistance * 1
    taskDistance  = taskDistance  * 1
    goalCount     = goalCount     * 100
    storagePun    = storagePun    * 2
    misconfiguration = misconfiguration * 1
  }
  result := totalDistance + taskDistance + goalCount + storagePun + misconfiguration

  dprintf("H = %d, tD: %d, gD: %d, gC: %d, sp: %d, m:%d", result, totalDistance, taskDistance, goalCount, storagePun, misconfiguration)
  return result
}

func addStorageOrder(boxIdx int, state *State) {
  // Find storage area
  box := state.boxes[boxIdx]
  // TODO: Why does this use the room_map?
  state.unassignedTasks = append(state.unassignedTasks, Task{false, boxIdx, room_map[box.pos.x][box.pos.y]})
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
    goal := goals[t.goalIdx] // TODO: this will not work when using storage tasks
    robot := state.robots[robotIdx]
    // Check if robot and box are all compatible
    if(box.color != robot.color){
      continue
    }
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
  }

  if(idx < 0){
    // TODO: handle
    dprint("no goal found");
    return
  }

  state.unassignedTasks = append(state.unassignedTasks[:idx], state.unassignedTasks[idx+1:]...)
  state.activeTasks[robotIdx] = &nextTask
}

func heuristicForAgent(i int, r *Robot, state *State, again bool) int {
  if(state.activeTasks[i] == nil) {
    dprint("Task is nil!!");
    newTask(i, state)
  }

  if(state.activeTasks[i] == nil){
    return 0
  }
  task := state.activeTasks[i]
  robot := state.robots[i]
  box := state.boxes[task.boxIdx]
  goal := goals[task.goalIdx]

  distA := checked_distance(robot.pos, box.pos)
  badness := badnessOfShortestPath(i, r, state)
  // Are we moving a box to its goal or to storage?
  if(task.exactGoal){
    distB := checked_distance(box.pos, goal.pos)

    //if state.boxes[state.activeTasks[i].boxIdx].pos == goals[state.activeTasks[i].goalIdx].pos {
    //  state.activeTasks[i] = nil
    //}
    return distA + distB + badness
  }
  // Storage: // if we are in a room that is not the room that we are moving the box from
  if(rooms[room_map[box.pos.x][box.pos.y]].isRoom && task.goalIdx != room_map[box.pos.x][box.pos.y]){
    // are we done?
    if(distA <= 1){
      state.activeTasks[i] = nil
    }
    return distA + badness // TODO: plus more
  }

  // If we are in a corridor, we need to get out
  if(!rooms[room_map[box.pos.x][box.pos.y]].isRoom){

  }
  return 0 + badness
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
    return 0
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
    tasks = append(tasks, Task{true, box, i})
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

  // Debug print
  for j:=0; j<height; j++ {
    str := ""
    for i:=0; i<width; i++ {
      if storage_map[i][j] >= 0 {
        str = str + " "
      }
      str = str + strconv.Itoa(storage_map[i][j])
    }
    print(str)
  }

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
