package main

func heuristic(state *SimpleState) int {
  // TODO: make sure that each agent does not have the same goal


  // totalDistance is the sum of distances from each agent to its goal
  //////////////////////////////////////////////////////////////////////////////
  totalDistance := 0
  for i, r := range state.robots {
    totalDistance += heuristicForAgent(i, r, state, true)
    dprintf("H = %d", totalDistance)
  }

  // goalCount is the number of goal that does not have a box
  // This makes it more expensive to move boxes that are already on a box
  //////////////////////////////////////////////////////////////////////////////
  goalCount := isDone(state.boxes)
  return totalDistance + goalCount*50
}

func heuristicForAgent(i int, r *Robot, state *SimpleState, again bool) int {
  if(state.agentToGoal[i] != nil){
    dprintf("AGENT HEURISTIC: %d mission: %d (%d, %d)",i, state.agentToBox[i], state.agentToGoal[i].pos.x, state.agentToGoal[i].pos.y)
    dprint("Action: " + (*state.action)[0].toString())
  }
  if(state.agentToBox[i] >= 0 ){
    var dist int
    dist = checked_distance(r.pos, state.boxes[state.agentToBox[i]].pos)
    //dprintf("dist 0 %d\n", dist)
    if(dist > 1) {
      // If we need to move to the box:
      dprint("to box")
      return dist
    }
    if (state.agentToGoal[i] != nil) {
      // if we are already able to push the box:
      dist = checked_distance(state.boxes[state.agentToBox[i]].pos, state.agentToGoal[i].pos)
      //dprintf("dist 1 %d\n", dist)

      // if we are at the target, make sure to get a new goal next iteration
      if(dist == 0){
        state.agentToBox[i] = -1
      }
      dprint("to goal")
      return dist
    }
  } else if again {
    //dprint("Next goal")
    //Calculate new goal
    newGoal, newBox := nextGoal(state, r)
    state.agentToGoal[i] = newGoal
    state.agentToBox[i]  = newBox
    //Check the heuristic for the new goals
    return heuristicForAgent(i, r, state, false)
  }
  // if we cannot find a new goal just return 0
  return 0
}

func nextGoal(state *SimpleState, robot *Robot) (*Goal, int) {
  dprintf("NEXT GOAL")
  var nextGoal *Goal
  nextBox := -1
  distance := 9999999
  for _, g := range goals {
    //check if goal is already completed TODO: make fast
    for _, b := range state.boxes {
      if(b.pos == g.pos && b.letter == g.letter){
        continue
      }
    }
    for i, b := range state.boxes {
      // Check if goal, robot and box are all compatible
      if(b.color != robot.color || b.letter != g.letter){
        //dprintf("INCOMPATIBLE GOAL")
        continue
      }

      newDistA := checked_distance(robot.pos, b.pos)
      newDistB := checked_distance(b.pos, g.pos)
      // Check if either is unreachable
      if(newDistA < 0 || newDistB < 0){
        //dprintf("UNREACHABLE GOAL")
        continue
      }

      if(distance > newDistA + newDistB){
        nextGoal = g
        nextBox = i
        distance = newDistA + newDistB
        //dprintf("NEW BEST GOAL")
      } else {
        //dprintf("WORSE GOAL")
      }
    }
  }

  return nextGoal, nextBox
}
