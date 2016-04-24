package main

import (
    "fmt"
    "math"
    "os"
    "runtime"
    "time"
    "golang.org/x/net/websocket"
)

var debugPrint bool

func d(v interface{}) {
    logCh <- fmt.Sprintf("%#v %T\n", v, v)
}

func dprint(values ...interface{}) {
  if (debugPrint) {
    logCh <- fmt.Sprintln(values...)
  }
}

func dprintf(format string, values ...interface{}) {
  if (debugPrint) {
    logCh <- fmt.Sprintf(format + "\n", values...)
  }
}

func print(values ...interface{}) {
    logCh <- fmt.Sprintln(values...)
}
func printf(format string, values ...interface{}) {
    logCh <- fmt.Sprintf(format + "\n", values...)
}
func printr(values ...interface{}) {
    logCh <- fmt.Sprint(values...)
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
    printf("\n\n\n\n<section>====== %s ======</section>", title)
}
func memoryUsage() {
    // Alloc      uint64 // bytes allocated and not yet freed
    // TotalAlloc uint64 // bytes allocated (even if freed)
    // HeapAlloc    uint64 // bytes allocated and not yet freed (same as Alloc above)
    // HeapSys      uint64 // bytes obtained from system
    var mem runtime.MemStats
    runtime.ReadMemStats(&mem)
	tag := "<memory>\n"
	header := "# Memory usage #\n"
	content := "Alloc: %v\nTotalAlloc: %v\nHeapAlloc: %v\nHeapSys: %v\nGC: %v\n"
	endtag := "</memory>"
    printf(tag + header + content + endtag,
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
    printf("<time>Time: %s / %s</time>", sinceLast, sinceStart)
}

var logCh chan string
func logSender(ch chan string, ws *websocket.Conn) {
    defer wg.Done()
    if ws == nil {
        for msg := range ch {
            fmt.Fprint(os.Stderr, msg)
        }
    } else {
        for msg := range ch {
            _, err := ws.Write([]byte(msg))
            if err != nil {
                fmt.Fprint(os.Stderr, err)
            }
        }
    }
}
func init() {
    startTime = time.Now()
    lastTime = startTime


    logCh = make(chan string, 100)
    logCh <- "####START####"
    origin := "http://localhost/"
    url := "ws://localhost:8000/websocket"
    ws, err := websocket.Dial(url, "", origin)
    if err != nil {
        ws = nil
    }
    wg.Add(1)
    go logSender(logCh, ws)
}
