package main

import (
	"mavlink-sim-bridge/pkg/logging"

	"github.com/bluenviron/gomavlib/v2"
	dialect "github.com/bluenviron/gomavlib/v2/pkg/dialects/ardupilotmega"
)

var log = logging.Logger()

func main() {
	// create a node which communicates with a UDP endpoint in server mode
	node, err := gomavlib.NewNode(gomavlib.NodeConf{
		Endpoints: []gomavlib.EndpointConf{
			gomavlib.EndpointUDPServer{Address: ":14550"},
		},
		Dialect:     dialect.Dialect,
		OutVersion:  gomavlib.V2, // change to V1 if you're unable to communicate with the target
		OutSystemID: 10,
	})
	if err != nil {
		panic(err)
	}
	defer node.Close()

	for evt := range node.Events() {
		if frm, ok := evt.(*gomavlib.EventFrame); ok {
			// Receive only Simulator-related messages
			switch msg := frm.Message().(type) {
			case *dialect.MessageHilSensor:
				log.Infow("HIL Sensor", "Date", msg)
			case *dialect.MessageHilStateQuaternion:
				log.Infow("Hil State quaternion", "Date", msg)
			case *dialect.MessageGpsRawInt:
				log.Infow("Raw GPS", "Date", msg)
			case *dialect.MessageAttitudeQuaternion:
				log.Infow("Attitude quaternion", "Date", msg)
			case *dialect.MessageHighresImu:
				log.Infow("High Resolution IMU", "Date", msg)
			case *dialect.MessageRawImu:
				log.Infow("Raw IMU", "Data", msg)
			case *dialect.MessageScaledImu:
				log.Infow("Scaled IMU", "Data", msg)
			case *dialect.MessageScaledImu2:
				log.Infow("Scaled IMU", "Data", msg)
			case *dialect.MessageScaledImu3:
				log.Infow("Scaled IMU", "Data", msg)
			case *dialect.MessageSysStatus:
				log.Infow("System status", "Data", msg)
			case *dialect.MessageRawPressure:
				log.Infow("Raw pressure", "Data", msg)
			case *dialect.MessageScaledPressure:
				log.Infow("Scaled pressure", "Data", msg)
			}
		}
	}
}
