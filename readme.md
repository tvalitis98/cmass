# CMASS
####Centralized Multi-Agent Status Server (pronounced "sea mass")

CMASS allows ROS robots to communicate diagnostics to a server which can then serve them on the web. 

![Error finding image](https://i.imgur.com/XFXwHSN.png "An overview of the structure of the system")

---
### Client

Launch with `roslaunch cmass cmass_client`

The client subscribes to ROS topics and periodically publishes them via HTTP to the server.
It also sends the name of the computer (enabling it to serve as a DNS) and the name of the account that's logged in.

### Server

Start with `go run server/server.go`

Command line args: 
* `-debug`
  print debug info
    	
* `-file <string filename>`
  file to save robot statuses (default ".robot_statuses")
    	
* `-insecure`
  don't require token to authenticate robot updates
    	
* `-port <int portnumber>`
  port number to run on (default 7978)

### Security

TODO: finish readme
