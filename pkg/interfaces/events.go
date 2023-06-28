package interfaces

import dialect "github.com/bluenviron/gomavlib/v2/pkg/dialects/development"

type SimulationStepEvent struct {
	Controls *dialect.MessageHilActuatorControls
}
