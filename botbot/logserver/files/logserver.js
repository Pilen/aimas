
var startMessage = "####START####";
var sessions = [];
var current;
var ws;

function init() {
    ws = new WebSocket("ws://localhost:8000/websocket");
    ws.onmessage = skipMessage;
}
window.onload = init;

function skipMessage(e) {
    console.log("sM: " + e.data);
    if (e.data.startsWith(startMessage)) {
        ws.onmessage = readMessage;
        readMessage(e);
    }
}

function readMessage(e) {
    console.log("rM: " + e.data);
    if (e.data.startsWith(startMessage)) {
        $("#messages").empty();
        current = $("<pree></pree>");
        $("#messages").append(current);
        sessions.push(current);
    }
    // current.append($("<div>" + e.data + "</div>"));
    current[0].innerHTML += e.data;
}
