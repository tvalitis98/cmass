all: server.go
	go build server.go

linux32:
	env GOOS=linux GOARCH=386 go build -o bin/server-linux-32 -v server.go

linux64:
	env GOOS=linux GOARCH=amd64 go build -o bin/server-linux-64 -v server.go

clean:
	$(RM) server bin/server-linux-32 bin/server-linux-64
