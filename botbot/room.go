package main

var room_map [][]int
var rooms []room

var next_room = -1;

type room struct {
  isRoom bool
  in_idx, out_idx int // index to connecting rooms (-1 if this is not a 
  in_pos, out_pos Coordinate
  connections []int
}

func generate_rooms(walls *[70][70]bool) {

    room_map = make([][]int, width);

    for x := 0; x < width; x++ {
        room_map[x] = make([]int, height);
        for y := 0; y<height; y++ {
          room_map[x][y] = -1
        }
    }

    //roomify(walls, Coordinate{1, 1}, 0)
    for x := 0; x < width; x++ {
        for y := 0; y < height; y++ {
            roomify(walls, Coordinate{x, y}, -1)
        }
    }
}

func roomify(walls *[70][70]bool, coord Coordinate, id int) {

  x := coord.x
  y := coord.y
  if(!isInside(x, y) || room_map[x][y] >= 0 ||  walls[x][y]){
    return
  }

  E := x+1 < width  && !walls[x+1][y]
  W := x-1 >= 0      && !walls[x-1][y]
  N := y+1 < height && !walls[x][y+1]
  S := y-1 >= 0      && !walls[x][y-1]

  // If there are only one adjecent tile we have a road
  if( E && !W && !N && !S ||
     !E &&  W && !N && !S ||
     !E && !W &&  N && !S ||
     !E && !W && !N &&  S ){
    recurse(false, id, coord, walls)
    return
  }

  // If the two on the oposite site is blocked, we must have a road
  if(!E && !W &&  N &&  S ||
      E &&  W && !N && !S ){
    recurse(false, id, coord, walls)
    return
  }

  // If all neigbours are free we must have a room
  if( E &&  W &&  N &&  S ){
    recurse(true, id, coord, walls)
    return
  }

  // if 3 neighbours are free it must be a room
  if( E &&  W &&  N && !S ||
      E &&  W && !N &&  S ||
      E && !W &&  N &&  S ||
     !E &&  W &&  N &&  S ){
    recurse(true, id, coord, walls)
    return
  }

  // If two neigbours are free it may be a room or a floor depending on the corners
  NE := y+1 < height && x+1 < width && !walls[x+1][y+1]
  NW := y+1 < height && x-1 >= 0    && !walls[x-1][y+1]
  SE := y-1 >= 0     && x+1 < width && !walls[x+1][y-1]
  SW := y-1 >= 0     && x-1 >= 0    && !walls[x-1][y-1]

  if(N && E && NE ||
     N && W && NW ||
     S && E && SE ||
     S && W && SW ){
    recurse(true, id, coord, walls)
    return
  }

  if(N && E && !NE ||
     N && W && !NW ||
     S && E && !SE ||
     S && W && !SW ){
    recurse(false, id, coord, walls)
    return
  }
}

func recurse(isRoom bool, id int, coord Coordinate, walls *[70][70]bool) {

  if( id < 0 ){
    id = get_next_room()
    rooms = append(rooms, room{isRoom, -1, -1, Coordinate{-1,-1}, Coordinate{-1,-1}, make([]int, 0)})
  } else if isRoom != rooms[id].isRoom { // If the type (room/wall) is different than the type of the id
    return
  }

  room_map[coord.x][coord.y] = id
  nbrs := neighbours(coord)
  for _, pos := range nbrs {
    if(room_map[pos.x][pos.y] >= 0 && room_map[pos.x][pos.y] != id){
      connect(isRoom, pos, id)
    } else {
      roomify(walls, pos, id)
    }
  }
}

func connect(isRoom bool, coord Coordinate, id int) {
  id1 := -1
  id2 := -1
  if(isRoom){
    // connect room with road
    id1 = room_map[coord.x][coord.y]
    id2 = id
  } else {
    id1 = id
    id2 = room_map[coord.x][coord.y]
  }
  // connect other to this,
  if(rooms[id1].in_idx < 0 ){
    rooms[id1].in_idx = id2
    rooms[id1].in_pos = coord
  } else {
    rooms[id1].out_idx = id2
    rooms[id1].out_pos = coord
  }
  rooms[id2].connections = append(rooms[id2].connections, id1)
}

func get_next_room() int {
  next_room++
  return next_room
}
