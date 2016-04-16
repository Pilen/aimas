package main

type Coordinate struct {
	x, y int
}

func neighbours(coordinate Coordinate) ([]Coordinate) {
	x := coordinate.x
	y := coordinate.y
	// North, South, West, East
	coordinates := make([]Coordinate, 0, 4)
	if y != 0 && !wallMap[x][y-1] {
		coordinates = append(coordinates, Coordinate{x, y-1})
	}
	if y < height-1 && !wallMap[x][y+1] {
		coordinates = append(coordinates, Coordinate{x, y+1})
	}
	if x != 0 && !wallMap[x-1][y]{
		coordinates = append(coordinates, Coordinate{x-1, y})
	}
	if x < width-1 && !wallMap[x+1][y] {
		coordinates = append(coordinates, Coordinate{x+1, y})
	}

	return coordinates
}

func isNeigbours(a, b Coordinate) bool{
  if(a.x == b.x && a.y == b.y+1){
    return true
  }
  if(a.x == b.x && a.y == b.y-1){
    return true
  }
  if(a.x == b.x+1 && a.y == b.y){
    return true
  }
  if(a.x == b.x-1 && a.y == b.y){
    return true
  }

  return false
}

func direction(from Coordinate, to Coordinate) rune {
	if !isNeigbours(from, to) {panic("ASSERTION FAILED: from and to must be neighbours")}
	if to.y < from.y {
		return 'N'
	}
	if to.y > from.y {
		return 'S'
	}
	if to.x > from.x {
		return 'E'
	}
	if to.x < from.x {
		return 'W'
	}
	panic("ASSERTION FAILED: from == to")
}
