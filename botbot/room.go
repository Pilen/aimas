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
    rooms = append(rooms, room{isRoom, -1, -1, coord, Coordinate{-1,-1}, make([]int, 0)})
  } else if isRoom != rooms[id].isRoom { // If the type (room/wall) is different than the type of the id
    newId := -1
    if(!isRoom){
      // connect room with road
      var r room // road
      // Does it already exist?
      if(room_map[coord.x][coord.y] > 0){
        newId = room_map[coord.x][coord.y]
        r = rooms[newId]
      } else {
        // create new and connect
        r = room{false, id, -1, coord, Coordinate{-1,-1}, make([]int, 0)}
        r.in_idx = id
        r.in_pos = coord
        rooms = append(rooms, r)
        newId = len(rooms)-1
      }
      // connect other to this,
      r.out_idx = newId
      r.out_pos = coord

      // connect this to existing
      rooms[id].connections = append(rooms[id].connections, newId)
    } else {
      // connect road with room
      newId = room_map[coord.x][coord.y]
      // Does it already exist?
      if(newId > 0){
        rooms[newId].connections = append(rooms[newId].connections, room_map[coord.x][coord.y])
      } else {
        r := room{true, -1, -1, Coordinate{-1,-1}, Coordinate{-1,-1}, make([]int, 0)}
        rooms = append(rooms, r)
        newId = len(rooms) - 1
        rooms[newId].connections = append(rooms[newId].connections, id)
      }

      rooms[id].out_idx = newId
      rooms[id].out_pos = coord
    }
    id = newId
  }

  room_map[coord.x][coord.y] = id
  nbrs := neighbours(coord)
  for _, pos := range nbrs {
    roomify(walls, pos, id)
  }
}

func get_next_room() int {
  next_room++
  return next_room
}
