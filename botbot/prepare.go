package main

import (
      "fmt"
//    "container/heap"
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
        // printf("%v, %v = %v\n", current.x, current.y, priority)
        if priority < priorityMap[current.x][current.y] {
            priorityMap[current.x][current.y] = priority
            for _, neighbour := range neighbours(current.x, current.y) {
                toBeVisited.Insert(priorityNode{neighbour.x, neighbour.y, priority}, priority)
            }
        }
    }

    print("==== Goal priorities ====")
    for _, goal := range goals {
        goal.priority = priorityMap[goal.x][goal.y]
        printf("goal %c: (%v,%v) = %v\n", goal.letter, goal.x, goal.y, goal.priority)
    }
}

var apspWork int

type PathArray [][]*[][]int;

func all_pairs_shortest_path(walls *[70][70]bool, width int, height int) PathArray{
    // TODO Fix...
    width -= 2;
    height -= 1;

    var pairs PathArray;
    pairs = make([][]*[][]int, width);

    for x := 0; x < width; x++ {
        pairs[x] = make([]*[][]int, height);
        for y := 0; y < height; y++ {
            pairs[x][y] = shortest_path(x, y, walls, width, height);
        }
    }

    printAPSP(width, height, pairs);

    return pairs;
}

func distance(apsp PathArray, xStart int, yStart int, xEnd int, yEnd int) int{
    if(apsp[xStart][yStart] == nil || apsp[xEnd][yEnd] == nil){
        return -1;
    }

    return (*apsp[xStart][yStart])[xEnd][yEnd];
}

func checked_distance(apsp PathArray, xStart int, yStart int, xEnd int, yEnd int, width int, height int) int {
    if(isInside(xStart, yStart, width, height) && isInside(xEnd, yEnd, width, height)){
        return distance(apsp, xStart, yStart, xEnd, yEnd);
    }

    return -1;
}

func getShortestPath(apsp PathArray, xStart int, yStart int, xEnd int, yEnd int, width int, height int) {
    if(apsp[xStart][yStart] == nil || apsp[xEnd][yEnd] == nil){
        //TODO: Return no path
        print("NO PATH1");
        return;
    }

    if(xStart == xEnd && yStart == yEnd){
        //TODO: Return path
        print("FOUND PATH");
        return;
    }

    distX := checked_distance(apsp, xStart+1, yStart, xEnd, yEnd, width, height);
    distX_ := checked_distance(apsp, xStart-1, yStart, xEnd, yEnd, width, height);
    distY := checked_distance(apsp, xStart, yStart+1, xEnd, yEnd, width, height);
    distY_ := checked_distance(apsp, xStart, yStart-1, xEnd, yEnd, width, height);

    if(distX < 0 && distX_ < 0 && distY_ < 0 && distY < 0){
        // TODO: return no path, should not happen
        print("NO PATH2");
        return;
    }
    x := xStart+1;
    y := yStart;
    minDist := distX;

    if(distX < 0 || distX_ < minDist && distX_ >= 0){
        x = xStart-1;
        y = yStart;
        minDist = distX_;
    }
    if(distX < 0 && distX_ < 0 || distY < minDist && distY >= 0){
        x = xStart;
        y = yStart+1;
        minDist = distY;
    }
    if(distX < 0 && distX_ < 0 && distY < 0 || distY_ < minDist && distY_ >= 0){
        x = xStart;
        y = yStart-1;
        minDist = distY_;
    }

    //TODO: Add to path and return the final path:
    printf("%02d; %02d\n", x, y);
    getShortestPath(apsp, x, y, xEnd, yEnd, width, height);
}

// HELPER FUNCTIONS:
type priorityNode struct {
    x, y int
    priority int
}

func shortest_path(x int, y int, walls *[70][70]bool, width int, height int) *[][]int {

    var arr [][]int;
    arr = make([][]int, width);
    var added [70][70]bool;

    if(walls[x][y]){
        return nil;
    }

    for i := 0; i < width; i++ {
        arr[i] = make([]int, height);
        for j := 0; j < height; j++ {
            apspWork++
            arr[i][j] = -1;
            added[i][j] = false;
        }
    }

    first :=  priorityNode{x, y, 0}

    var cells Heap
    cells.Insert(first, 0)

    added[x][y] = true
    for !cells.IsEmpty() {
        currentCell := cells.Extract().(priorityNode)
        arr[currentCell.x][currentCell.y] = currentCell.priority
        add(currentCell.x+1, currentCell.y, currentCell.priority, &added, walls, width, height, &cells)
        add(currentCell.x-1, currentCell.y, currentCell.priority, &added, walls, width, height, &cells)
        add(currentCell.x, currentCell.y+1, currentCell.priority, &added, walls, width, height, &cells)
        add(currentCell.x, currentCell.y-1, currentCell.priority, &added, walls, width, height, &cells)
    }
    return &arr;
}

func add(x int, y int, length int, added *[70][70]bool, walls *[70][70]bool, width int, height int, cells* Heap){
    if( isInside(x, y, width, height) && !walls[x][y] && !added[x][y]){
        newCell := priorityNode{x, y, length+1}
        added[x][y] = true
        (*cells).Insert(newCell, length+1)
    }
}

func isInside(x int, y int, width int, height int) bool {
    return x >= 0 && x <= width-1 && y >= 0 && y <= height-1 ;
}

// PRINT All Pairs shortest path
func printAPSP(width int, height int, pairs PathArray){
    for y := 0; y < height; y++ {
      for x := 0; x < width; x++ {
          fmt.Print(x);
          fmt.Print(";");
          print(y);
          if(pairs[x][y] != nil){
            for i := 0; i < height; i++ {
              for j := 0; j < width; j++ {
                  printf("%02d ", (*pairs[x][y])[j][i]);
              }
              print("");
            }
          } else {
            fmt.Print("N ");
          }
          print("");
      }
      print("");
    }
}
