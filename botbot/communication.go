package main

import (
    "os"
    "bufio"
    "fmt"
)

func beginIOLoop(actions [][]agentAction) {
  
    reader := bufio.NewReader(os.Stdin)
    var line string
    var err error

    section("IO Loop")

    for _, jointAction := range actions {
        fmt.Printf("[")
        print("[")
        for i, action := range jointAction{
            fmt.Printf(action.toString())
            print(action.toString())
            if( i < len(jointAction)-1) {
              fmt.Printf(", ")
              print(", ")
            }
        }
        fmt.Printf("]\n")
        printf("]\n")
        line, err = reader.ReadString('\n')
        print(line)
    }

}
