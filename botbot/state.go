package main

var object_colors map[rune]Color
var width, height int

var wallMap [70][70]bool
var robots []*Robot
var goals []*Goal
var goalMap [70][70]bool


var apsp PathArray
func setupState() {
	object_colors = make(map[rune]Color)
}

type Color string
type Robot struct {
	x, y int
	color Color
}

type Goal struct {
	x, y int
	letter rune
	priority int
}

type State struct {
	relevance int
	reservations [][]Coordinate
	actions []agentAction
	task *Task
}
