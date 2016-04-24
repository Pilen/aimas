package main

var object_colors map[rune]Color
var width, height int

var wallMap [70][70]bool
var boxes []*Box
var robots []*Robot
var goals []*Goal
var goalMap [70][70]bool


var apsp PathArray
func setupState() {
	object_colors = make(map[rune]Color)
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

type State struct {
	depth int
	parent *State
	relevance int
	reservations map[Coordinate]bool
	actions []agentAction
	robots []*Robot
	boxes []*Box
	goalsLeft int
	// task *Task
}
