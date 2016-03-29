package main

func merge(actions []agentAction) [][]agentAction {
    var res [][]agentAction
    res = make([][]agentAction, len(actions))

    print("Hej")

    for i, action := range actions {
      res[i] = make([]agentAction, 1) // TODO: should be the number of agents
      res[i][0] = action
      
    }

    return res 
}
