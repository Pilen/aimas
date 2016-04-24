package main

import (
)

// func search(robots []Robot, boxes []Box, heuristic func([]Robot, []Box) int, goalReached([]Robot, []Box) bool) {
func search() {
	// A* algorithm
	// var frontier Heap


}

type SimpleState struct {
	actions []agentAction
	robots []*Robot
	boxes []*Box
}


// assumption can be lots of boxes, medium amount of agents


func all_child_states(state *State) []State {
	var current []*SimpleState
	// var previous = []*SimpleState{&SimpleState{[], []}}
	for _, robot := range state.robots {
		current = make([]*SimpleState, 0)

		_ = current
		_ = robot


		// previous = current
	}

	// Convert SimpleStates to States
	return nil
}


func generate_robot_actions(robot *Robot, previous *SimpleState, state *State, newStates *[]*SimpleState) {
	// Dont reserve current, it is already marked as occupied

	// NOP
	if !reserved(robot.pos, previous) {
		// Create simple state - no extra reservations

		// *newStates = append(*newStates, state)
	}

	// MOVE
	for _, neighbour := range neighbours(robot.pos) {
		if (!reserved(neighbour, previous) &&
			!state.reservations[neighbour] &&
			!wallMap[neighbour.x][neighbour.y]) {
			// Create simple state - Reserve neighbour
		}
	}

	// PUSH / PULL
	for _, box := range state.boxes {
		if !isNeigbours(robot.pos, box.pos) || reserved(box.pos, previous) {
			continue
		}
		for _, box_dest := range neighbours(box.pos) {
			if box_dest == robot.pos {
				// PULL
				for _, robot_dest := range neighbours(robot.pos) {
					if robot_dest != box_dest {
						if (!reserved(robot_dest, previous) &&
							!state.reservations[robot_dest] &&
							!wallMap[robot_dest.x][robot_dest.y]) {
							// Create simple state - Reserve box + dest
						}
					} // else already handled
				}
			} else {
				// PUSH
				if (!reserved(box_dest, previous) &&
					!state.reservations[box_dest] &&
					!wallMap[box_dest.x][box_dest.y]) {
					// Create simple state - Reserve box + dest
				}
			}
		}
	}
}

func reserved(coordinate Coordinate, previous *SimpleState) bool {
	return true
}
func occupied(x int, y int, state *State) bool {
	return state.reservations[Coordinate{x, y}]
}

// NoOp
// Move(N)
// Move(S)
// move(E)
// move(W)

// Push(N, N)
// Push(N, E)
// Push(N, W)
// Push(S, S)
// Push(S, E)
// Push(S, W)
// Push(E, N)
// Push(E, S)
// Push(E, E)
// Push(W, N)
// Push(W, S)
// Push(W, W)

// Pull(N, )
// Pull(S)
// Pull(E)
// Pull(W)
