package main

func findTask(State, []agentAction) [][]Task {
	result := make([][]Task, 0)
	return result
}

func solve(state State) [][]agentAction {
  hlActions := aStar(state)
  var agentActions []agentAction
  for _, hlAction := range hlActions{
    for _, agAction := range highlevelToSimpleAction(hlAction) {
      agentActions = append(agentActions, agAction)
    }
  }
  return singleAgentMerge(agentActions)
}

/*
 * Gennerate all possible actions from a state
*/
func addActions(state State, heap Heap, cost int) Heap {
  for _, agent := range state.robots {
    for _, box := range state.boxes {
      for _, goal := range goals {
        action := pushTo{Coordinate{goal.x, goal.y}, *agent, *box, nil}
        if(action.precondition() >= 0){
          heap.Insert(action, cost + action.precondition())
        }
        action2 := pullTo{Coordinate{goal.x, goal.y}, *agent, *box, nil}
        if(action2.precondition() >= 0){
          heap.Insert(action2, cost + action2.precondition())
        }
      }
      action3 := moveTo{Coordinate{box.x+1, box.y}, *agent, nil}
      if(action3.precondition() >= 0){
        heap.Insert(action3, cost + action3.precondition())
      }
      action3 = moveTo{Coordinate{box.x-1, box.y}, *agent, nil}
      if(action3.precondition() >= 0){
        heap.Insert(action3, cost + action3.precondition())
      }
      action3 = moveTo{Coordinate{box.x, box.y+1}, *agent, nil}
      if(action3.precondition() >= 0){
        heap.Insert(action3, cost + action3.precondition())
      }
      action3 = moveTo{Coordinate{box.x, box.y-1}, *agent, nil}
      if(action3.precondition() >= 0){
        heap.Insert(action3, cost + action3.precondition())
      }
    }
  }
  return heap
}

func numGoalsSatisfied(state State) int{
  res := 0
  for _, goal := range goals {
    for _, box := range state.boxes {
      if(box.x == goal.x && box.y == goal.y /* && goal.letter == box.color TODO*/){
        res++;
        continue;
      }
    }
  }
  return res;
}

func aStar(state State) []highlevelAction{

    var boundary Heap
    addActions(state, boundary, 0)

    for !boundary.IsEmpty() {
        currentAction := boundary.Extract().(highlevelAction)
        if(numGoalsSatisfied(*currentAction.state()) == len(goals)){
            var actions []highlevelAction
            for currentAction.previousAction() != nil{
                actions = append([]highlevelAction{currentAction}, actions...);
                currentAction = *currentAction.previousAction();
            }

            return actions
        }
        addActions(*currentAction.state(), boundary, state.relevance)
    }

    print("NO SOLUTION WAS FOUND!!")
    return nil
}

func highlevelToSimpleAction(hlAction highlevelAction) []agentAction{
  var hla highlevelAction
  hla = hlAction
  switch hla := hla.(type){
    case *moveTo:
      return moveToPlan(Coordinate{hla.get_agent().x, hla.get_agent().y}, hla.destination()) 
  }
  return make([]agentAction, 0)
}
