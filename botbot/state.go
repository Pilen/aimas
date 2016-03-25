package main

var object_colors map[rune]Color
var width, height int

var wallMap [70][70]bool
var robots []*Robot
var goals []*Goal
var goalMap [70][70]bool

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

// ACTIONS:

type agentAction interface {
  toString() string
}

type move struct {
  direction rune
}

func (p *move) toString() string{
  return "Move(" + string(p.direction) + ")";
}

type push struct {
  agentDirection rune
  boxDirection rune
}

func (p *push) toString() string{
  return "Push(" + string(p.agentDirection)+", " + string(p.boxDirection) + ")";
}

type pull struct {
  agentDirection rune
  boxDirection rune
}

func (p *pull) toString() string{
  return "Pull(" + string(p.agentDirection)+", " + string(p.boxDirection) + ")";
}

type noop struct {
}

func (p *noop) toString() string{
  return "NoOp"
}
