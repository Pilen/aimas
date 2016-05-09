package main

import (
    // "net/http"
    // _ "net/http/pprof"
    "sync"
    "strconv"
    // "time"
)

var wg sync.WaitGroup
func main() {
    debugPrint = true
    section("Start")
    setupState()
    Parse()
    printf("w:%d, h:%d\n", width, height)
    section("Goal priorities")
    calculateGoalPriorities()
    section("APSP")
    all_pairs_shortest_path(&wallMap)
    printf("APSP work: %v", apspWork)

    section("Room")
    generate_rooms(&wallMap)
    for j:=0; j<height; j++ {
      str := ""
      for i:=0; i<width; i++ {
        if room_map[i][j] >= 0 {
          str = str + " "
        }
        str = str + strconv.Itoa(room_map[i][j])
      }
      print(str)
    }

    for i, r := range rooms {
      print( i )
      if r.isRoom {
        str := "connections: "
        for _, i := range r.connections {
          str = str + strconv.Itoa(i) + " "
        }
        print(str)
      } else {
        str := "exits: "
        str = str + strconv.Itoa(r.in_idx)
        str = str + " " +  strconv.Itoa(r.out_idx)
        print(str)
      }
    }

    printf("sp: %d", checked_distance(Coordinate{1,1}, Coordinate{2,1}))
    section("Work")
    //printAPSP(apsp);
    // path := moveToPlan(apsp, 8, 1, &moveTo{10, 3})
    // path2 := pushToPlan(apsp, 10, 3, 10, 4, &pushTo{3, 5})
    // path3 := pullToPlan(apsp, 4, 5, 3, 5, &pullTo{7, 1})
    // mPath := merge(append(path, append(path2, path3...)...))
    plan := search()

    section("Send plan")
    beginIOLoop(plan)

    section("Finished")
    close(logCh)
    wg.Wait()
    // for {print(".\n")}
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
    actions[5][0] = &push{'E', 'E', 0}

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
    actions[16][0] = &push{'E', 'E', 0}
    actions[16][1] = &push{'E', 'E', 0}

    return actions
}






func createSolution() {
    // current := Startstate
    // for !isSolved(current) {
    //     findTask -> heap
    //     tasklist <- heap
    //     get naiveplans from tasklist
    //     merge naiveplans

    // }
}
