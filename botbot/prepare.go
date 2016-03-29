package main

import (
)

func calculateGoalPriorities() {
    var priorityMap [70][70]int
    for x := 0; x < 70; x++ {
        for y := 0; y < 70; y++ {
            priorityMap[x][y] = 70 * 70 + 1
        }
    }

    var toBeVisited Heap

    for _, robot := range robots {
        node := priorityNode{robot.x, robot.y, 0}
        toBeVisited.Insert(node, 0)
    }

    for !toBeVisited.IsEmpty() {
        current := toBeVisited.Extract().(priorityNode)
        // if wallMap[current.x][current.y] {panic(fmt.Sprintf("wall at %v, %v\n", current.x, current.y))}
        priority := current.priority
        if goalMap[current.x][current.y] {
            priority++
        }
        // printf("%v, %v = %v", current.x, current.y, priority)
        if priority < priorityMap[current.x][current.y] {
            priorityMap[current.x][current.y] = priority
            for _, neighbour := range neighbours(current.x, current.y) {
                toBeVisited.Insert(priorityNode{neighbour.x, neighbour.y, priority}, priority)
            }
        }
    }

    for _, goal := range goals {
        goal.priority = priorityMap[goal.x][goal.y]
        printf("goal %c: (%v,%v) = %v", goal.letter, goal.x, goal.y, goal.priority)
    }
}

