package main

import (
	"mavlink-sim-bridge/pkg/server"
)

func main() {
	srv := server.
		MakeServer(":4560").
		Connect()

	defer srv.Close()

	srv.Listen()
}
