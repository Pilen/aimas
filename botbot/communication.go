package main

import (
    "os"
    "bufio"
    "fmt"
    "regexp"
    "strings"
)

func beginIOLoop(actions [][]agentAction) {

    response_regex, _ := regexp.Compile("^\\s*\\[\\s*true(\\s*,\\s*true)*\\s*\\]\\s*$")

    reader := bufio.NewReader(os.Stdin)
    var line string
    var err error

    for _, jointAction := range actions {
        fmt.Print("[")
        printr("[")
        for i, action := range jointAction{
            fmt.Print(action.toString())
            printr(action.toString())
            if( i < len(jointAction)-1) {
              fmt.Print(", ")
              printr(", ")
            }
        }
        fmt.Printf("]\n")
        print("]")

        line, err = reader.ReadString('\n')
        if err != nil {
            printr(err)
            return
        }

        line = strings.TrimSpace(line)
        print("line:", line)

        if line == "success" {
            return
        }
        if line == "timeout" {
            print("TIMEOUT")
            return
        }
        if !response_regex.MatchString(line) {
            print("ILLEGAL MOVE ENCOUNTERED")
            return
        }
    }

}
