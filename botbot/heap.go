package main

import (
	"errors"
)
// An implementation of Pairing heaps
// https://en.wikipedia.org/wiki/Pairing_heap
//
// As defined by
// Fredman, Michael L.; Sedgewick, Robert; Sleator, Daniel D.; Tarjan, Robert E. (1986).
// "The pairing heap: a new form of self-adjusting heap"
// http://www.cs.cmu.edu/~sleator/papers/pairing-heaps.pdf
//
// Insertion O(1)
// Extraction O(log n) amortized

type Heap struct {
	root *HeapNode
}

type HeapNode struct {
	value interface{}
	key int
	children *HeapNode
	siblings *HeapNode
}


func (h *Heap) Insert(value interface{}, key int) {
	node := &HeapNode{value, key, nil, nil}
	if h.root == nil {
		h.root = node
	} else {
		h.root = heapLink(h.root, node)
	}

}
func (h *Heap) IsEmpty() bool {
	return h.root == nil
}
func (h *Heap) Extract() interface{} {
	if h.root == nil {
		panic("Heap empty")
	} else {
		value := h.root.value
		h.root = heapReduce(h.root)
		return value
	}
}


func (h *Heap) Peek() (interface{}, error){
	if h.root == nil {
		return nil, errors.New("Heap empty")
	} else {
		return h.root.value, nil
	}
}

func heapLink(h1, h2 *HeapNode) (*HeapNode) {
	if h1.key < h2.key {
		if h2.siblings != nil {panic("Assertion failed: h2.siblings != nil")}
		h2.siblings = h1.children
		h1.children = h2
		return h1
	} else {
		if h1.siblings != nil {panic("Assertion failed: h1.siblings != nil")}
		h1.siblings = h2.children
		h2.children = h1
		return h2
	}
}

func heapReduce(h *HeapNode) (*HeapNode) {
	if (h.children == nil) {
		return nil
	}
	var h1, h2, current *HeapNode
	current = h.children
	stack := make([]*HeapNode, 0, 10)
	for {
		if current == nil {break}
		h1 = current
		current = current.siblings
		h1.siblings = nil
		if current == nil {
			stack = append(stack, h1)
			break
		}
		h2 = current
		current = current.siblings
		h2.siblings = nil

		paired := heapLink(h1, h2)
		stack = append(stack, paired)
	}

	current = stack[0]
	for i := 1; i < len(stack); i++ {
		current = heapLink(current, stack[i])
	}
	return current
}
