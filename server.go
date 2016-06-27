package main

//imports

type robot struct {
	name      string // the name of the robot
	ip        string // the ip address of the robot
	position  []int  // array of postition coordinates
	alive     bool   // if the robot is currently active
	lastAlive int    // epoch time since robot last pinged server

}

//declare variables
var robots []robot

var portNumber string
var debug bools

//define functions for server endpoints
////    /update           called by robots with their info
////    /robot/<name>     called to access a specific robot's info
////    /json             called to access all robots' info
////    /hostsjson        legacy support, serves json of robotname:IP
////    /hostsalivejson   legacy support, serves json of robotname:IP of active robots

//utility functions
//// save
//// load
//// jsonify
//// prettify

//
