package main

/*
*/
func moveToPlan(start, end Coordinate) []agentAction{

    if(apsp[start.x][start.y] == nil || apsp[end.x][end.y] == nil){
        return nil;
    }
    if(start.x == end.y && start.y == end.y){
        return nil;
    }
    coord, dir := nextNode(start, end)
    action := move{dir}
    // TODO: running time considerations for prepend
    return append([]agentAction{&action}, moveToPlan(coord, end)...);
}

func pushToPlan(start, end, box Coordinate) []agentAction{

    if(apsp[start.x][start.y] == nil || apsp[end.x][end.y] == nil || apsp[box.x][box.y] == nil){
        return nil;
    }

    if(box.x== end.x && box.y == end.y){
        return nil;
    }

    _, dir := nextNode(start, end)
    coord, dirB := nextNode(box, end)
    action := push{dir, dirB}
    // TODO: running time considerations for prepend
    return append([]agentAction{&action}, pushToPlan(box, end, coord)...);
}

func pullToPlan(start, end, box Coordinate) []agentAction{

    if(apsp[start.x][start.y] == nil || apsp[end.x][end.y] == nil || apsp[box.x][box.y] == nil){
        return nil;
    }

    if(box.x== end.x && box.y == end.y){
        return nil;
    }

    coord, dir := nextNode(start, end)
    _, dirB := nextNode(start, end) // Direction of the box
    action := pull{dir, dirB}
    // TODO: running time considerations for prepend
    return append([]agentAction{&action}, pullToPlan(coord, end, start)...);
}

