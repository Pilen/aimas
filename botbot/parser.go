package main

import (
    "bufio"
    "fmt"
    "os"
    "regexp"
    "strings"
    "unicode"
)

func Parse() {
    line_regex, _ := regexp.Compile("^([a-z]+)\\s*:\\s*([0-9A-Z](\\s*,\\s*[0-9A-Z])*)\\s*$")
    object_regex, _ := regexp.Compile("[0-9A-Z]")

    reader := bufio.NewReader(os.Stdin)
    var line string
    var err error

    for err == nil {
        line, err = reader.ReadString('\n')

        line = strings.TrimSpace(line)
        //So apparently files with CRLF lineendings are interpreted as LFLF in go?!
        if (line == "") {
            continue
        }

        if !line_regex.MatchString(line) {
            break
        }
        fmt.Println(line)
        matches := line_regex.FindStringSubmatch(line)
        color := Color(matches[1])
        objects := object_regex.FindAllString(matches[2], -1)

        for _, object := range objects {
            object_colors[rune(object[0])] = color
        }
    }

    var level [70][70]rune // x, y
    var walls [70][70]bool // x, y
    y := 0

    // var width, height int
    for err == nil {
        if (line == "") {
            continue
        }
        for x, c := range line {
            level[x][y] = c
            if c == '+' {
                walls[x][y] = true
            } else if unicode.IsDigit(c) {
                NewRobot(c, x, y)
            } else if unicode.IsLetter(c) && unicode.IsLower(c) {
                NewGoal(c, x, y)
            } else if unicode.IsLetter(c) && unicode.IsUpper(c) {
                NewBox(c, x, y)
            }
        }

        y++
        height++
        if len(line) > width {
            width = len(line)
        }
        line, err = reader.ReadString('\n')
    }


    flood_fill(&walls, width, height)
    all_pairs_shortest_path(&walls, width, height)
}

func NewRobot(c rune, x int, y int) {
    color := object_colors[c]
    robot := Robot{x, y, color}
    robots = append(robots, &robot)
}

func NewGoal(c rune, x int, y int) {
    goal := Goal{x, y, c}
    goals = append(goals, goal)
}

func NewBox(c rune, x int, y int) {

}
