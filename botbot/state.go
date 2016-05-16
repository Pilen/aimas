package main

var object_colors map[rune]Color
var width, height int

var wallMap [70][70]bool
var boxes []*Box
var robots []*Robot
var goals []*Goal
var goalMap [70][70]int


var apsp PathArray
func setupState() {
	object_colors = make(map[rune]Color)
	for x := 0; x < 70; x++ {
		for y := 0; y < 70; y++ {
			goalMap[x][y] = -1
		}
	}
}

type Color string

type Box struct {
	// x, y int
	pos Coordinate
	color Color
  letter rune
}

type Robot struct {
	// x, y int
	pos Coordinate
	color Color
	next *Robot // Internaly used by the state generator
}

type Goal struct {
	// x, y int
	pos Coordinate
	letter rune
	priority int
}

type Task struct {
  exactGoal bool //If true the box must be moved to the goal, otherwise it should just end up somewhere around (for storage)
  boxIdx int
  goalIdx int
}

// type State struct {
// 	depth int
// 	parent *State
// 	relevance int
// 	reservations map[Coordinate]bool
// 	actions []agentAction
// 	robots []*Robot
// 	boxes []*Box
// 	goalsLeft int
// 	// task *Task
// }

type State struct {
  actualActions *[]agentAction
  actionCombinations [][]agentAction // TODO: this should be done directly and stored in frontier. Dont save here.
  combinationLevel int
  actionHeap []Heap
  previous *State
  robots []*Robot
  boxes []*Box
  cost int
  unassignedTasks []Task
  activeTasks []*Task
}
