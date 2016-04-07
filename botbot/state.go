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
	x, y int
	// coordinate Coordinate
	color Color
}

type Robot struct {
	x, y int
	// coordinate Coordinate
	color Color
	next *Robot // Internaly used by the state generator
}

type Goal struct {
	x, y int
	// coordinate Coordinate
	letter rune
	priority int
}

type State struct {
	relevance int
	reservations map[Coordinate]bool
	actions []agentAction
	robots []*Robot
	boxes []*Box
	goalsLeft int
	// task *Task
}
