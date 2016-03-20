package main

var object_colors map[rune]Color
var width, height int

var robots []*Robot
var goals []Goal

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
}
