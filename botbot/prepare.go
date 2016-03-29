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

var apspWork int

type PathArray [][]*[][]int;

func all_pairs_shortest_path(walls *[70][70]bool) PathArray{
    // TODO Fix...
    //width -= 2;
    //height -= 1;

    var pairs PathArray;
    pairs = make([][]*[][]int, width);

    for x := 0; x < width; x++ {
        pairs[x] = make([]*[][]int, height);
        for y := 0; y < height; y++ {
            pairs[x][y] = shortest_path(x, y, walls);
        }
    }

    //printAPSP(width, height, pairs);

    return pairs;
}

func distance(apsp PathArray, xStart int, yStart int, xEnd int, yEnd int) int{
    if(apsp[xStart][yStart] == nil || apsp[xEnd][yEnd] == nil){
        return -1;
    }

    return (*apsp[xStart][yStart])[xEnd][yEnd];
}

func checked_distance(apsp PathArray, xStart int, yStart int, xEnd int, yEnd int) int {
    if(isInside(xStart, yStart) && isInside(xEnd, yEnd)){
        return distance(apsp, xStart, yStart, xEnd, yEnd);
    }

    return -1;
}

/*
*/
func moveToPlan(apsp PathArray,
            xStart, yStart int,
            hlAction highlevelAction) []agentAction{

    xEnd, yEnd := hlAction.destination()
    if(apsp[xStart][yStart] == nil || apsp[xEnd][yEnd] == nil){
        return nil;
    }
    if(xStart == xEnd && yStart == yEnd){
        return nil;
    }
    x, y, dir := nextNode(apsp, xStart, yStart, xEnd, yEnd)
    action := move{dir}
    // TODO: running time considerations for prepend
    return append([]agentAction{&action}, moveToPlan(apsp, x, y, hlAction)...);
}

func pushToPlan(apsp PathArray,
                xStart, yStart,
                xBox, yBox int, 
                hlAction highlevelAction) []agentAction{

    xEnd, yEnd := hlAction.destination()
    if(apsp[xStart][yStart] == nil || apsp[xEnd][yEnd] == nil || apsp[xBox][yBox] == nil){
        return nil;
    }
    if(xBox == xEnd && yBox== yEnd){
        return nil;
    }

    _, _, dir := nextNode(apsp, xStart, yStart, xEnd, yEnd)
    xB, yB, dirB := nextNode(apsp, xBox, yBox, xEnd, yEnd)
    action := push{dir, dirB}
    // TODO: running time considerations for prepend
    return append([]agentAction{&action}, pushToPlan(apsp, xBox, yBox, xB, yB, hlAction)...);
}

func pullToPlan(apsp PathArray,
                xStart, yStart,
                xBox, yBox int, 
                hlAction highlevelAction) []agentAction{

    xEnd, yEnd := hlAction.destination()
    if(apsp[xStart][yStart] == nil || apsp[xEnd][yEnd] == nil || apsp[xBox][yBox] == nil){
        return nil;
    }
    if(xBox == xEnd && yBox== yEnd){
        return nil;
    }

    x, y, dir := nextNode(apsp, xStart, yStart, xEnd, yEnd)
    _, _, dirB := nextNode(apsp, xStart, yStart, xBox, yBox) // Direction of the box
    action := pull{dir, dirB}
    // TODO: running time considerations for prepend
    return append([]agentAction{&action}, pullToPlan(apsp, x, y, xStart, yStart, hlAction)...);
}

// HELPER FUNCTIONS:

/*
 * Find the node adjecent to (xStart, yStart) that is not a wall or outside the map,
 * with the shortest path to (xEnd, yEnd). Return the x, y, dir where (x,y) is the
 * coordinate of the found node, and dir is the direction from the original node.
*/
func nextNode(apsp PathArray, xStart, yStart, xEnd, yEnd int) (x, y int, dir rune) {

    distX := checked_distance(apsp, xStart+1, yStart, xEnd, yEnd);
    distX_ := checked_distance(apsp, xStart-1, yStart, xEnd, yEnd);
    distY := checked_distance(apsp, xStart, yStart+1, xEnd, yEnd);
    distY_ := checked_distance(apsp, xStart, yStart-1, xEnd, yEnd);

    x = xStart+1
    y = yStart
    minDist := distX
    dir = 'E'

    if(minDist < 0 || distX_ < minDist && distX_ >= 0){
        x = xStart-1
        y = yStart
        minDist = distX_
        dir = 'W'
    }
    if(minDist < 0 || distY < minDist && distY >= 0){
        x = xStart
        y = yStart+1
        minDist = distY
        dir = 'S'
    }
    if(minDist < 0 || distY_ < minDist && distY_ >= 0){
        x = xStart
        y = yStart-1
        minDist = distY_
        dir = 'N'
    }

    return x, y, dir
}
type priorityNode struct {
    x, y int
    priority int
}

func shortest_path(x int, y int, walls *[70][70]bool) *[][]int {

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
        add(currentCell.x+1, currentCell.y, currentCell.priority, &added, walls, &cells)
        add(currentCell.x-1, currentCell.y, currentCell.priority, &added, walls, &cells)
        add(currentCell.x, currentCell.y+1, currentCell.priority, &added, walls, &cells)
        add(currentCell.x, currentCell.y-1, currentCell.priority, &added, walls, &cells)
    }
    return &arr;
}

func add(x int, y int, length int, added *[70][70]bool, walls *[70][70]bool, cells* Heap){
    if( isInside(x, y) && !walls[x][y] && !added[x][y]){
        newCell := priorityNode{x, y, length+1}
        added[x][y] = true
        (*cells).Insert(newCell, length+1)
    }
}

func isInside(x int, y int,) bool {
    return x >= 0 && x <= width-1 && y >= 0 && y <= height-1 ;
}

// PRINT All Pairs shortest path
func printAPSP(pairs PathArray){
    for y := 0; y < height; y++ {
      for x := 0; x < width; x++ {
          printf("%d:%d\n",x,y);
          if(pairs[x][y] != nil){
            for i := 0; i < height; i++ {
              for j := 0; j < width; j++ {
                  printf("%02d ", (*pairs[x][y])[j][i]);
              }
              print("");
            }
          } else {
            print("N ");
          }
          print("");
      }
      print("");
    }
}
