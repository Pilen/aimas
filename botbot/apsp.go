package main
var apspWork int
type PathArray [][]*[][]int;

func all_pairs_shortest_path(walls *[70][70]bool) {
    // TODO Fix...
    //width -= 2;
    //height -= 1;

    apsp = make([][]*[][]int, width);

    for x := 0; x < width; x++ {
        apsp[x] = make([]*[][]int, height);
        for y := 0; y < height; y++ {
            apsp[x][y] = shortest_path(x, y, walls);
        }
    }
}

/*
 * Get the shortest distance between between two points
*/
func checked_distance(start, end Coordinate) int {
    if(isInside(start.x, start.y) && isInside(end.x, end.y)){
        if(apsp[start.x][start.y] == nil || apsp[end.x][end.y] == nil){
            return -1;
        }

        return (*apsp[start.x][start.y])[end.x][end.y];
    }

    return -1;
}

// HELPER FUNCTIONS:

/*
 * Find the next coordinate and direction in the shortest path form start to end
*/
func nextNode(start, end Coordinate) (next Coordinate, dir rune) {

    distX := checked_distance(Coordinate{start.x+1, start.y}, end);
    distX_ := checked_distance(Coordinate{start.x-1, start.y}, end);
    distY := checked_distance(Coordinate{start.x, start.y+1}, end);
    distY_ := checked_distance(Coordinate{start.x, start.y-1}, end);

    next = Coordinate{start.x+1, start.y}
    minDist := distX
    dir = 'E'

    if(minDist < 0 || distX_ < minDist && distX_ >= 0){
        next.x = start.x-1
        next.y = start.y
        minDist = distX_
        dir = 'W'
    }
    if(minDist < 0 || distY < minDist && distY >= 0){
        next.x = start.x
        next.y = start.y+1
        minDist = distY
        dir = 'S'
    }
    if(minDist < 0 || distY_ < minDist && distY_ >= 0){
        next.x = start.x
        next.y = start.y-1
        minDist = distY_
        dir = 'N'
    }

    return next, dir
}

/*
 * Return an array where each cell contains the length of the shortest path to (coord)
 * Used to get the all pairs shortest path
*/
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

/*
 * Add the coordinate (x,y) to the border if it is inside the map, is not a
 * wall and have not already been added
*/
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
func printAPSP(){
    for y := 0; y < height; y++ {
      for x := 0; x < width; x++ {
          printf("%d:%d\n",x,y);
          if(apsp[x][y] != nil){
            for i := 0; i < height; i++ {
              for j := 0; j < width; j++ {
                  printf("%02d ", (*apsp[x][y])[j][i]);
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
