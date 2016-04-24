package main


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
  boxIdx int
}

func (p *push) toString() string{
  return "Push(" + string(p.agentDirection)+", " + string(p.boxDirection) + ")";
}

type pull struct {
  agentDirection rune
  boxDirection rune
  boxIdx int
}

func (p *pull) toString() string{
  return "Pull(" + string(p.agentDirection)+", " + string(p.boxDirection) + ")";
}

type noop struct {
}

func (p *noop) toString() string{
  return "NoOp"
}
