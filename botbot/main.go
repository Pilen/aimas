package main

import (
    "fmt"
    "runtime"
    "math"
    "os"
    // "net/http"
    // _ "net/http/pprof"
    // "sync"
    // "time"
)

func d(v interface{}) {
    printf("%#v %T\n", v, v)
}

func print(values ...interface{}) {
    fmt.Fprintln(os.Stderr, values...)
}

func printf(format string, values ...interface{}) {
    fmt.Fprintf(os.Stderr, format, values...)
}

func humanBytes(bytes uint64) string {
    // Using SI units (1000 not 1024)
    if bytes < 1000 {
        return fmt.Sprintf("%v B", bytes)
    }
    size := float64(bytes)
    exp := int(math.Log10(size) / math.Log10(1000))
    prefix := "kMGTPE"[exp-1]
    result := size/math.Pow(1000, float64(exp))
    return fmt.Sprintf("%.2f %cB", result, prefix)
}

func memoryUsage() {
    // Alloc      uint64 // bytes allocated and not yet freed
    // TotalAlloc uint64 // bytes allocated (even if freed)
    // HeapAlloc    uint64 // bytes allocated and not yet freed (same as Alloc above)
    // HeapSys      uint64 // bytes obtained from system
    var mem runtime.MemStats
    runtime.ReadMemStats(&mem)
    printf("==== Memory usage ====\n")
    printf("Alloc: %v\nTotalAlloc: %v\nHeapAlloc: %v\nHeapSys: %v\nGC: %v\n",
        humanBytes(mem.Alloc),
        humanBytes(mem.TotalAlloc),
        humanBytes(mem.HeapAlloc),
        humanBytes(mem.HeapSys),
        mem.NumGC)

}

func allocALot() *[]byte {
    s := make([]byte, 1000000)
    return &s
}

func main() {
    // go func() {
    //  print(http.ListenAndServe("localhost:6060", nil))
    // }()
    // time.Sleep(10 * time.Second)

    setupState()
    Parse()
    memoryUsage()

    var data []*[]byte
    for i := 0; i < 1000; i++ {
        d := allocALot()
        data = append(data, d)
        if len(data) >= 10 {
            data = data[0:]
        }
    }
    printf("APSP work: %v\n", apspWork)
    memoryUsage()
    // var wg sync.WaitGroup
    // wg.Add(1)
    // wg.Wait()
}
