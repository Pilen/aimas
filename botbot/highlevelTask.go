package main

import (
    "strconv"
)

var storage_map [][]int
// -1 = wall
// 0 = on a critical path
// 1 = not on a crititical path, but is road
// 2 = not on a critical path, is room


type agentGoal struct {
  exactGoal bool //If true the box must be moved to the goal, otherwise it should just end up somewhere around (for storage)
  boxIdx int
  goalIdx int
}

func heuristic(state *State, heuristic int) int {
  // TODO: When a goal is reached the next goal is picked the heuristic
  //       grows a lot, and the solutions that only almost solves the task is picked.

  // totalDistance is the sum of distances from each agent to its goal
  //////////////////////////////////////////////////////////////////////////////
  totalDistance := 0
  for i, r := range state.robots {
    totalDistance += heuristicForAgent(i, r, state, true)
  }

  // goalDistance is the distance of all goals remaining:
  //////////////////////////////////////////////////////////////////////////////
  goalDistance := 0
  for _, g := range state.goals {
    box := state.boxes[g.boxIdx]
    goal := goals[g.goalIdx]
    goalDistance += checked_distance(box.pos, goal.pos)
  }

  // goalCount is the number of goal that does not have a box
  // This makes it more expensive to move boxes that are already on a box
  //////////////////////////////////////////////////////////////////////////////
  goalCount := isDone(state.boxes)

  // storage punishmet adds up punishment for each box that is on a non-storage
  // area
  //////////////////////////////////////////////////////////////////////////////
  storagePun := 0
  for i, box := range state.boxes {
      // If the box is an active goal, we do not punish the box for being at a
      // critical path
      boxIsGoal := false
      for _, goal := range state.activeGoals {
          if(goal != nil && goal.boxIdx == i){
            boxIsGoal = true
            break
          }
      }
      if(boxIsGoal){
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

  //goalCount := len(state.goals) TODO: len of goals does not work as a heuristic, why?
  if(heuristic == 0 || true){ // TODO: maybe use different heuristics for states and actions
    goalCount     = goalCount     * 100
    totalDistance = totalDistance * 1
    goalDistance  = goalDistance  * 1
    storagePun    = storagePun    * 2
  }
  result := totalDistance + goalDistance + goalCount + storagePun

  dprintf("H = %d, tD: %d, gD: %d, gC: %d, sp: %d", result, totalDistance, goalDistance, goalCount, storagePun)
  return result
}

func addStorageOrder(boxIdx int, state *State) {
  // Find storage area
  box := state.boxes[boxIdx]
  state.goals = append(state.goals, agentGoal{false, boxIdx, room_map[box.pos.x][box.pos.y]})
}

func newGoal(robotIdx int, state *State) {

  copyGoals(state)

  dprintf("New goal for %d", robotIdx)
  var nextGoal agentGoal
  idx := -1
  distance := 9999999
  priority := 0
  for i, g := range state.goals {
    box := state.boxes[g.boxIdx]
    goal := goals[g.goalIdx] // TODO: this will not work when using storage tasks
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
      nextGoal = g
      distance = newDistA + newDistB
      priority = goal.priority
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

func heuristicForAgent(i int, r *Robot, state *State, again bool) int {
  if(state.activeGoals[i] == nil) {
    dprint("Goal is nil!!");
    newGoal(i, state)
  }

  if(state.activeGoals[i] == nil){
    return 0
  }
  agentGoal := state.activeGoals[i]
  robot := state.robots[i]
  box := state.boxes[agentGoal.boxIdx]
  goal := goals[agentGoal.goalIdx]

  distA := checked_distance(robot.pos, box.pos)

  // Are we moving a box to its goal or to storage?
  if(agentGoal.exactGoal){
    distB := checked_distance(box.pos, goal.pos)

    //if state.boxes[state.activeGoals[i].boxIdx].pos == goals[state.activeGoals[i].goalIdx].pos {
    //  state.activeGoals[i] = nil
    //}
    return distA + distB
  }
  // Storage: // if we are in a room that is not the room that we are moving the box from
  if(rooms[room_map[box.pos.x][box.pos.y]].isRoom && agentGoal.goalIdx != room_map[box.pos.x][box.pos.y]){
    // are we done?
    if(distA <= 1){
      state.activeGoals[i] = nil
    }
    return distA // TODO: plus more
  }

  // If we are in a corridor, we need to get out
  if(!rooms[room_map[box.pos.x][box.pos.y]].isRoom){

  }
  return 0
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
    agentGoals = append(agentGoals, agentGoal{true, box, i})
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
