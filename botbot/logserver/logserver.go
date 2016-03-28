package main

import (
	"fmt"
	"net/http"
	"golang.org/x/net/websocket"
)

func main() {
	fmt.Println("Starting logserver")
	fmt.Println("Listening on localhost:8000")
	ws := NewWebsocketServer("/websocket")
	go ws.Listen()
	go httpServer()
	http.ListenAndServe(":8000", nil)

}

func httpServer() {
	http.Handle("/", http.FileServer(http.Dir("files/")))
}

var maxId int = 0

type WebsocketServer struct {
	url string
	addCh chan *Connection
	delCh chan *Connection
	connections map[int] *Connection
	incomming chan string
	messages []string
}

func NewWebsocketServer(url string) *WebsocketServer {
	addCh := make(chan *Connection, 10)
	delCh := make(chan *Connection, 10)
	connections := make(map[int]*Connection)
	incomming := make(chan string, 10)
	messages := make([]string, 0)
	return &WebsocketServer{url, addCh, delCh, connections, incomming, messages}
}

func (s *WebsocketServer) Listen() {
	onConnect := func(ws *websocket.Conn) {
		c := NewConnection(ws, s)
		c.Start()
	}

	h := websocket.Handler(onConnect)
	http.Handle(s.url, h)

	for {
		select {
		case c := <- s.addCh:
			for _, msg := range s.messages {
				c.msgCh <- msg
			}
			s.connections[c.id] = c
		case c := <- s.delCh:
			delete(s.connections, c.id)
		case msg := <- s.incomming:
			fmt.Print(msg)
			s.messages = append(s.messages, msg)
			for _, c := range s.connections {
				c.msgCh <- msg
			}
		}
	}
}

type Connection struct {
	id int
	ws *websocket.Conn
	msgCh chan string
	server *WebsocketServer
}
func NewConnection(ws *websocket.Conn, server *WebsocketServer) *Connection {
	maxId++
	id := maxId
	msgCh := make(chan string, 100)
	return &Connection{id, ws, msgCh, server}
}

func (c *Connection) Start() {
	defer func() {
		c.server.delCh <- c
		c.ws.Close()
	}()
	go c.receiver()
	c.server.addCh <- c
	for msg := range c.msgCh {
		websocket.Message.Send(c.ws, msg)
	}
}

func (c *Connection) receiver() {
	for {
		var msg string
		err := websocket.Message.Receive(c.ws, &msg)
		if err != nil {
			break
		}
		c.server.incomming <- msg
	}
}
