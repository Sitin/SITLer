package server

import (
	"github.com/bluenviron/gomavlib/v2"
	dialect "github.com/bluenviron/gomavlib/v2/pkg/dialects/development"
	"github.com/bluenviron/gomavlib/v2/pkg/message"
	"mavlink-sim-bridge/pkg/interfaces"
	"mavlink-sim-bridge/pkg/logging"
	"mavlink-sim-bridge/pkg/sitls"
	"time"
)

var log = logging.Logger()

type Server interface {
	Connect() Server
	Close()
	Listen()
}

func MakeServer(address string) Server {
	return &server{
		endpoint:       gomavlib.EndpointTCPServer{Address: address},
		abort:          make(chan struct{}),
		node:           nil,
		connectedSITLs: make(map[*gomavlib.Channel]*sitls.PX4Sitl),
	}
}

type server struct {
	endpoint       gomavlib.EndpointTCPServer
	abort          chan struct{}
	node           *gomavlib.Node
	connectedSITLs map[*gomavlib.Channel]*sitls.PX4Sitl
}

func (srv *server) Connect() Server {
	// create a node which communicates with a TCP endpoint in server mode
	node, err := gomavlib.NewNode(gomavlib.NodeConf{
		Endpoints:   []gomavlib.EndpointConf{srv.endpoint},
		Dialect:     dialect.Dialect,
		OutVersion:  gomavlib.V2,
		OutSystemID: 255,
	})
	if err != nil {
		panic(err)
	}

	srv.node = node
	srv.connectedSITLs = make(map[*gomavlib.Channel]*sitls.PX4Sitl)
	srv.abort = make(chan struct{})

	log.Infof("server started at %s.", srv.endpoint.Address)

	return srv
}

func (srv *server) Close() {
	srv.node.Close()
	srv.abort <- struct{}{}
	defer close(srv.abort)
}

func (srv *server) Listen() {
	for evt := range srv.node.Events() {
		if frm, ok := evt.(*gomavlib.EventFrame); ok {
			// Register new connection if necessary
			if _, ok := srv.connectedSITLs[frm.Channel]; !ok {
				srv.registerSITL(frm)
			}
			// Set current sitl
			sitl := srv.connectedSITLs[frm.Channel]

			switch msg := frm.Message().(type) {
			case *dialect.MessageHeartbeat:
				srv.exchangeHeartBeat(sitl, msg, frm)
			case *dialect.MessageHilActuatorControls:
				//Switch to lock-step mode
				if !sitl.IsLockStep {
					log.Infow("Actuator controls received. Switching to lock-step mode",
						"SystemId", frm.SystemID(),
						"ComponentId", frm.ComponentID())
					sitl.IsLockStep = true
				}
				// Spawn step event

				sitl.RequestStep(interfaces.SimulationStepEvent{Controls: msg})
			default:
				log.Infow("Message received",
					"SystemId", frm.SystemID(),
					"ComponentId", frm.ComponentID(),
					"MessageId", frm.Message().GetID(),
					"Data", frm.Message())
			}
		}
	}
}

func (srv *server) registerSITL(frm *gomavlib.EventFrame) {
	log.Infow("Registered new connection",
		"Channel", frm.Channel,
		"SystemId", frm.SystemID(),
		"ComponentId", frm.ComponentID())

	sitl := &sitls.PX4Sitl{
		Autopilot:   dialect.MAV_AUTOPILOT_INVALID,
		Steps:       make(chan interfaces.SimulationStepEvent),
		IsLockStep:  false,
		StartTimeUs: time.Now().UnixMicro(),
		SendMessage: func(msg message.Message) {
			srv.node.WriteMessageTo(frm.Channel, msg)
		},
	}
	srv.connectedSITLs[frm.Channel] = sitl

	go sitl.Run()
}

func (srv *server) exchangeHeartBeat(
	sitl *sitls.PX4Sitl,
	heartbeat *dialect.MessageHeartbeat,
	frm *gomavlib.EventFrame,
) {
	// Update channel with autopilot type if required
	if sitl.Autopilot == dialect.MAV_AUTOPILOT_INVALID {
		sitl.Autopilot = heartbeat.Autopilot
		log.Infow("Update sitl settings",
			"Channel", frm.Channel,
			"SystemId", frm.SystemID(),
			"ComponentId", frm.ComponentID(),
			"Autopilot", sitl.Autopilot)
	}

	// Respond with heartbeat
	sitl.SendMessage(&dialect.MessageHeartbeat{
		Type:      dialect.MAV_TYPE_QUADROTOR,
		Autopilot: dialect.MAV_AUTOPILOT_INVALID,
	})

	log.Infow("Heartbeats exchanged",
		"Channel", frm.Channel,
		"SystemId", frm.SystemID(),
		"CompanyId", frm.ComponentID(),
		"Data", frm.Message())
}
