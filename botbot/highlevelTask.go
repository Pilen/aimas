package main

func heuristic(state *SimpleState) int {
  // TODO: add goals when they are violated
  // TODO: When a goal is reached the next goal is picked the heuristic 
  //       grows a lot, and the solutions that only almost solves the task is picked.


  // totalDistance is the sum of distances from each agent to its goal
  //////////////////////////////////////////////////////////////////////////////
  totalDistance := 0
  for i, r := range state.robots {
    totalDistance += heuristicForAgent(i, r, state, true)
    dprintf("H = %d", totalDistance)
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
  return totalDistance + goalDistance + goalCount*100
}

func newGoal(robotIdx int, state *SimpleState) {

  copyGoals(state)

  dprintf("New goal for %d", robotIdx)
  var nextGoal agentGoal
  idx := -1
  distance := 9999999
  for i, g := range state.goals {
    box := state.boxes[g.boxIdx] 
    goal := goals[g.goalIdx] 
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

    if(distance > newDistA + newDistB){
      idx = i
      nextGoal = g
      distance = newDistA + newDistB
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

func heuristicForAgent(i int, r *Robot, state *SimpleState, again bool) int {
  if(state.activeGoals[i] == nil || state.boxes[state.activeGoals[i].boxIdx].pos == goals[state.activeGoals[i].goalIdx].pos) {
    if(state.activeGoals[i] == nil){
      dprint("Goal is nil!!");
    }
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
  distB := checked_distance(box.pos, goal.pos)
  return distA + distB
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
    agentGoals = append(agentGoals, agentGoal{box, i})
    reserved[box] = true
  }

  dprint("GOALS:")
  for _, g := range agentGoals {
    dprintf("%v (%d,%d) -> %v (%d,%d)",boxes[g.boxIdx].letter, boxes[g.boxIdx].pos.x, boxes[g.boxIdx].pos.y, goals[g.goalIdx].letter, goals[g.goalIdx].pos.x, goals[g.goalIdx].pos.y)
  }

  return agentGoals
}

type agentGoal struct {
  boxIdx int
  goalIdx int
}
