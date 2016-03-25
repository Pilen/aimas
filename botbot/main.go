package main

import (
    // "net/http"
    // _ "net/http/pprof"
    // "sync"
    // "time"
)

func main() {
    // go func() {
    //  print(http.ListenAndServe("localhost:6060", nil))
    // }()
    // time.Sleep(10 * time.Second)

    section("Start")
    setupState()
    Parse()
    section("Goal priorities")
    calculateGoalPriorities()
    section("APSP")
    all_pairs_shortest_path(&wallMap, width, height)

    section("Work")

    printf("APSP work: %v", apspWork)
    section("Finished")
    // var wg sync.WaitGroup
    // wg.Add(1)
    // wg.Wait()
    beginIOLoop(hardcodedSolution2())
}

// Hardcoded solution for SAsimple1.lvl
func hardcodedSolution1() [][]agentAction {
    var actions [][]agentAction
    actions = make([][]agentAction, 7)
    actions[0] = make([]agentAction, 1)
    actions[1] = make([]agentAction, 1)
    actions[2] = make([]agentAction, 1)
    actions[3] = make([]agentAction, 1)
    actions[4] = make([]agentAction, 1)
    actions[5] = make([]agentAction, 1)
    actions[6] = make([]agentAction, 1)
    actions[0][0] = &move{'E'}
    actions[1][0] = &move{'E'}
    actions[2][0] = &move{'E'}
    actions[3][0] = &move{'E'}
    actions[4][0] = &move{'E'}
    actions[5][0] = &move{'E'}
    actions[5][0] = &push{'E', 'E'}

    return actions
}

// Hardcoded solution for MAsimple1.lvl
func hardcodedSolution2() [][]agentAction {
    var actions [][]agentAction
    actions = make([][]agentAction, 17)

    for i := 0; i < 8; i++ {
        actions[i] = make([]agentAction, 2)
        actions[i][0] = &noop{}
        actions[i][1] = &move{'E'}
    }
    for i := 8; i < 16; i++ {
        actions[i] = make([]agentAction, 2)
        actions[i][0] = &move{'E'}
        actions[i][1] = &move{'E'}
    }
    actions[16] = make([]agentAction, 2)
    actions[16][0] = &push{'E', 'E'}
    actions[16][1] = &push{'E', 'E'}

    return actions
}
