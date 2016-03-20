package main

import (
	"fmt"
)

func d(v interface{}) {
	fmt.Printf("%#v %T\n", v, v)
}

func main() {
	setupState()
	Parse()
}
