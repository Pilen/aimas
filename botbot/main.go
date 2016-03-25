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
}
