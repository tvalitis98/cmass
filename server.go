package main

import (
	"encoding/json"
	"flag"
	"fmt"
	"io/ioutil"
	"log"
	"net/http"
	"net/url"
	"strconv"
	"time"
)

//imports

//Robot must be exported so that json package can access it to encode
// all strings because they're going to be serialized to JSON anyways
type Robot struct {
	Name      string // the name of the robot
	IP        string // the ip address of the robot
	X         string // x coordinate
	Y         string // y coordinate
	Alive     string // if the robot is currently active
	LastAlive string // epoch time since robot last pinged server
}

//declare variables
var robots []Robot

var portNumber int
var file string
var debug bool

var saveTicker = time.NewTicker(20 * time.Second) // controls time between saves

//define functions for server endpoints
////    /update           called by robots with their info
////    /robot/<name>     called to access a specific robot's info
////    /json             called to access all robots' info
////    /hostsjson        legacy support, serves json of robotname:IP
////    /hostsalivejson   legacy support, serves json of robotname:IP of active robots

func update(w http.ResponseWriter, r *http.Request) {
	query := r.URL.Query()
	pdebug("IP of request: " + r.RemoteAddr)
	fmt.Fprintf(w, updateRobot(query, r.RemoteAddr))
	save()
}

func serveBasicHTML(f func() string) func(http.ResponseWriter, *http.Request) {
	return func(w http.ResponseWriter, r *http.Request) {
		fmt.Fprintf(w, f())
	}
}

//utility functions

func pdebug(message string) {
	if debug {
		fmt.Println(message)
	}
}

//error helper, prints error message if there's an error
func checkErr(err error, message string) {
	if err != nil {
		log.Printf("Error: %s\n", message)
		log.Println(err)
	}
}

func save() {
	pdebug("saving to local file")
	bytes, err := json.Marshal(robots)
	checkErr(err, "couldn't marshal the DNS")
	err = ioutil.WriteFile(file, bytes, 0644)
	checkErr(err, "couldn't write to "+file)
}

func load() {
	pdebug("reading from " + file)
	bytes, err := ioutil.ReadFile(file)
	checkErr(err, "couldn't read from "+file)
	err = json.Unmarshal(bytes, &robots)
	checkErr(err, "couldn't unmarshal data read from "+file)
}

func addRobot(query url.Values, addr string) {
	bot := Robot{
		Name: query.Get("name"),
		IP:   addr,
		X:    query.Get("x"),
		Y:    query.Get("y")}
	pdebug("Adding new robot: " + string(bot.Name))
	robots = append(robots, bot)
}

func updateRobot(query url.Values, addr string) string {
	for i, bot := range robots {
		if bot.Name == query.Get("name") {
			robots[i].IP = addr
			robots[i].X = query.Get("x")
			robots[i].Y = query.Get("y")
			pdebug("Updated " + query.Get("name"))
			return "updated " + query.Get("name")
		}
	}
	addRobot(query, addr) // if it's not in robots, add it
	return "Added " + query.Get("name")
}

func jsonOutput() string {
	bytes, err := json.Marshal(robots)
	checkErr(err, "couldn't jsonify")
	return string(bytes)
}

func textOutput() string {
	ret := ""
	for _, bot := range robots {
		ret += bot.String()
	}
	return ret
}

func (bot Robot) String() string {
	ret := ""
	ret += bot.Name + "\n"
	ret += "\t" + "IP: " + bot.IP + "\n"
	ret += "\t" + "Coordinates: (" + bot.X + ", " + bot.Y + ")\n"
	ret += "\t" + "Time Last Alive: " + bot.LastAlive + "\n"
	return ret
}

func main() {
	flag.BoolVar(&debug, "debug", false, "print debug info")
	flag.IntVar(&portNumber, "port", 7978, "port number")
	flag.StringVar(&file, "file", ".robot_statuses", "file to save robot statuses")
	flag.Parse()

	//load from local files/database
	load()

	//bind/start server
	http.HandleFunc("/update", update)
	http.HandleFunc("/json", serveBasicHTML(jsonOutput))
	http.HandleFunc("/text", serveBasicHTML(textOutput))

	fmt.Println("starting server on port " + strconv.Itoa(portNumber))
	log.Fatal(http.ListenAndServe(":"+strconv.Itoa(portNumber), nil))
}
