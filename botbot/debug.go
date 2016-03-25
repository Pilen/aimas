package main

import (
    "fmt"
    "math"
    "os"
    "runtime"
    "time"
)

func d(v interface{}) {
    printf("%#v %T\n", v, v)
}

func print(values ...interface{}) {
    fmt.Fprintln(os.Stderr, values...)
}

func printf(format string, values ...interface{}) {
    fmt.Fprintf(os.Stderr, format + "\n", values...)
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

func section(title string) {
    print("")
    memoryUsage()
    timeElapsed()
    printf("\n\n====== %s ======", title)
}
func memoryUsage() {
    // Alloc      uint64 // bytes allocated and not yet freed
    // TotalAlloc uint64 // bytes allocated (even if freed)
    // HeapAlloc    uint64 // bytes allocated and not yet freed (same as Alloc above)
    // HeapSys      uint64 // bytes obtained from system
    var mem runtime.MemStats
    runtime.ReadMemStats(&mem)
    printf("# Memory usage #")
    printf("Alloc: %v\nTotalAlloc: %v\nHeapAlloc: %v\nHeapSys: %v\nGC: %v",
        humanBytes(mem.Alloc),
        humanBytes(mem.TotalAlloc),
        humanBytes(mem.HeapAlloc),
        humanBytes(mem.HeapSys),
        mem.NumGC)

}

func allocALot() []*[]byte {
    var data []*[]byte
    for i := 0; i < 1000; i++ {
        s := make([]byte, 1000000)
        data = append(data, &s)
        if len(data) >= 10 {
            data = data[0:]
        }
    }
    return data
}

var startTime time.Time
var lastTime time.Time
func timeElapsed() {
    now := time.Now()
    sinceLast := now.Sub(lastTime)
    sinceStart := now.Sub(startTime)
    lastTime = now
    printf("Time: %s / %s", sinceLast, sinceStart)
}

func init() {
    startTime = time.Now()
    lastTime = startTime
}
