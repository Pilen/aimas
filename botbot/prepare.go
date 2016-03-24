package main

import (
    "container/heap"
)

func calculateGoalPriorities() {
	// Using a floodfill
	type priorityNode struct {
		x, y int
		priority int
	}
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

type PathArray [][]*[][]int16;

func all_pairs_shortest_path(walls *[70][70]bool, width int, height int) PathArray{
  // TODO Fix...
  width -= 2;
  height -= 1;

  var pairs PathArray;
  pairs = make([][]*[][]int16, width);

  for x := 0; x < width; x++ {
    pairs[x] = make([]*[][]int16, height);
    for y := 0; y < height; y++ {
      pairs[x][y] = shortest_path(x, y, walls, width, height);
    }
  }

  // PRINT All Pairs shortest path
  //for y := 0; y < height; y++ {
  //  for x := 0; x < width; x++ {
  //      fmt.Print(x);
  //      fmt.Print(";");
  //      print(y);
  //      if(pairs[x][y] != nil){
  //        for i := 0; i < height; i++ {
  //          for j := 0; j < width; j++ {
  //              printf("%02d ", (*pairs[x][y])[j][i]);
  //          }
  //          print("");
  //        }
  //      } else {
  //        fmt.Print("N ");
  //      }
  //      print("");
  //  }
  //  print("");
  //}


  //TEST TODO: Remove
  print(distance(pairs, 1, 1, 2, 1));
  print(distance(pairs, 1, 1, 0, 1));
  print(distance(pairs, 3, 1, 3, 3));
  getShortestPath(pairs, 3, 1, 3, 3, width, height);

  return pairs;
}

func distance(apsp PathArray, xStart int, yStart int, xEnd int, yEnd int) int16{
  if(apsp[xStart][yStart] == nil || apsp[xEnd][yEnd] == nil){
    return -1;
  }

  return (*apsp[xStart][yStart])[xEnd][yEnd];
}

func checked_distance(apsp PathArray, xStart int, yStart int, xEnd int, yEnd int, width int, height int) int16 {
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
func shortest_path(x int, y int, walls *[70][70]bool, width int, height int) *[][]int16 {

  var arr [][]int16;
  arr = make([][]int16, width);
  var added [70][70]bool;

  if(walls[x][y]){
    return nil;
  }

  for i := 0; i < width; i++ {
    arr[i] = make([]int16, height);
    for j := 0; j < height; j++ {
        apspWork++
      arr[i][j] = -1;
      added[i][j] = false;
    }
  }

  first :=  Cell{
                  x: x,
                  y: y,
                  length: 0,
                  index: 0,
                }
  cells := &PriorityQueue{&first};
  heap.Init(cells);
  added[x][y] = true;
  for cells.Len() > 0 {
    currentCell := *heap.Pop(cells).(*Cell);
    arr[currentCell.x][currentCell.y] = currentCell.length;
    add(currentCell.x+1, currentCell.y, currentCell.length, &added, walls, width, height, cells);
    add(currentCell.x-1, currentCell.y, currentCell.length, &added, walls, width, height, cells);
    add(currentCell.x, currentCell.y+1, currentCell.length, &added, walls, width, height, cells);
    add(currentCell.x, currentCell.y-1, currentCell.length, &added, walls, width, height, cells);
  }
  return &arr;
}

func add(x int, y int, length int16, added *[70][70]bool, walls *[70][70]bool, width int, height int, cells *PriorityQueue){
    if( isInside(x, y, width, height) && !walls[x][y] && !added[x][y]){
      newCell := Cell{
                      x: x,
                      y: y,
                      length: length+1,
                      index: 0,
                    }
      added[x][y] = true;
      heap.Push(cells, &newCell);
    }
}

func isInside(x int, y int, width int, height int) bool {
  return x >= 0 && x <= width-1 && y >= 0 && y <= height-1 ;
}


// Used for the heap in shortest_path
// Inspired by golang.org example
type Cell struct {
  x int;
  y int;
  length int16;
  index int;
}

type PriorityQueue []*Cell

func (pq PriorityQueue) Len() int { return len(pq) }

func (pq PriorityQueue) Less(i, j int) bool {
  return (*pq[i]).length < (*pq[j]).length
}

func (pq PriorityQueue) Swap(i, j int) {
  pq[i], pq[j] = pq[j], pq[i]
  (*pq[i]).index = i
  (*pq[j]).index = j
}

func (pq *PriorityQueue) Push(x interface{}) {
  n := len(*pq)
  item := x.(*Cell)
  item.index = n
  *pq = append(*pq, item)
}

func (pq *PriorityQueue) Pop() interface{} {
  old := *pq
  n := len(old)
  item := old[n-1]
  (*item).index = -1;
  *pq = old[0 : n-1]
  return item
}

func (pq *PriorityQueue) update(item *Cell, length int16) {
  (*item).length = length
  heap.Fix(pq, item.index)
}
