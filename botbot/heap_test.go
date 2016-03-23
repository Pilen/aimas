package main

import (
	"testing"
	"math/rand"
	"sort"
)

func TestHeap1(t *testing.T) {
	for tests := 0; tests < 10000; tests++ {
		var heap Heap
		var data sort.IntSlice

		var length = rand.Intn(50)
		var key int
		for i := 0; i < length; i++ {
			key = rand.Intn(100)
			heap.Insert(key, key)
			data = append(data, key)
		}

		data.Sort()
		for _, v := range data {
			if heap.IsEmpty() {t.Errorf("Heap unexpectedly empty")}
			key := heap.Extract().(int)
			if key != v {
				t.Errorf("Keys are not equal %v %v", key, v)
			}
		}
		if !heap.IsEmpty() {t.Errorf("Heap not empty")}
	}
}

func TestHeap2(t *testing.T) {
	for tests := 0; tests < 10000; tests++ {
		var heap Heap
		var data sort.IntSlice

		var length = rand.Intn(50)
		var key int

		var inserted = 0
		for i := 0; i < length; i++ {
			var insert = rand.Intn(10) >= 5
			if insert {
				key = rand.Intn(100)
				heap.Insert(key, key)
				data = append(data, key)
				inserted++
			} else {
				if (inserted == 0) {continue}
				key = heap.Extract().(int)
				data.Sort()
				if key != data[0] {
					t.Errorf("Keys are not equal %v %v", key, data[0])
				}
				data = data[1:]
				inserted--
			}
		}

		data.Sort()
		for _, v := range data {
			if heap.IsEmpty() {t.Errorf("Heap is unexpectedly empty, removing")}
			key = heap.Extract().(int)
			if key != v {
				t.Errorf("Keys are not equal %v %v", key, v)
			}
			inserted--
		}
		if !heap.IsEmpty() {t.Errorf("Heap not empty")}
		if inserted != 0 {t.Errorf("inserted not 0")}
	}
}
