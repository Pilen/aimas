package main

import (
	"fmt"
)

func d(v interface{}) {
	fmt.Printf("%#v %T\n", v, v)
}
func p(v interface{}) {
	fmt.Printf("%v\n", v)
}

func main() {
	setupState()
	Parse()
}
