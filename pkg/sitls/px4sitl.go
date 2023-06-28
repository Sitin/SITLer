package sitls

import (
	dialect "github.com/bluenviron/gomavlib/v2/pkg/dialects/development"
	"github.com/bluenviron/gomavlib/v2/pkg/message"
	"github.com/westphae/quaternion"
	"math"
	"math/rand"
	"mavlink-sim-bridge/pkg/interfaces"
	"mavlink-sim-bridge/pkg/logging"
	"mavlink-sim-bridge/pkg/utils"
	"time"
)

const MaxIdleTimeOnLockStep = 500 * time.Millisecond
const NonLockingStepInterval = 50 * time.Millisecond

var log = logging.Logger()

type PX4Sitl struct {
	Autopilot   dialect.MAV_AUTOPILOT
	Steps       chan interfaces.SimulationStepEvent
	IsLockStep  bool
	SendMessage func(msg message.Message)
	StartTimeUs int64

	stepCounter uint64
	abort       chan struct{}
	lastUpdate  int64
}

func (sitl *PX4Sitl) RequestStep(step interfaces.SimulationStepEvent) {
	sitl.Steps <- step
}

func (sitl *PX4Sitl) NextStepIteration() uint64 {
	sitl.stepCounter++
	return sitl.stepCounter
}

func (sitl *PX4Sitl) Run() {
	if sitl.abort != nil {
		log.Warn("Attempt to run PX4Sitl which is already running")
		return
	}

	sitl.abort = make(chan struct{})
	sitl.lastUpdate = time.Now().UnixMicro()

	for {
		select {
		case <-sitl.abort:
			return
		case step := <-sitl.Steps:
			sitl.performStep(step)
		default:
			sitl.scheduleStep()
		}
	}
}

func (sitl *PX4Sitl) Close() {
	sitl.abort <- struct{}{}
	sitl.abort = nil
	defer close(sitl.abort)
}

func (sitl *PX4Sitl) scheduleStep() {
	time.Sleep(NonLockingStepInterval)

	nowUs := time.Now().UnixMicro()
	timeSinceLastUpdateUs := nowUs - sitl.lastUpdate

	// Skip if the last step was too recent
	if timeSinceLastUpdateUs < NonLockingStepInterval.Microseconds() {
		return
	}

	// Skip if in lock-step mode
	if sitl.IsLockStep {
		// Skip only last update was long time ago
		if timeSinceLastUpdateUs < MaxIdleTimeOnLockStep.Microseconds() {
			return
		}
		// Otherwise proceed to step dispatch
		log.Debugw("Force update",
			"Time since last update (µs)", timeSinceLastUpdateUs,
			"Max (µs)", MaxIdleTimeOnLockStep.Microseconds())
	}

	// Perform step
	sitl.performStep(interfaces.SimulationStepEvent{})
}

func (sitl *PX4Sitl) performStep(step interfaces.SimulationStepEvent) {
	currentTimeUs := time.Now().UnixMicro()
	simulationTimeUs := currentTimeUs - sitl.StartTimeUs
	updateTimeUsec := uint64(currentTimeUs)

	// Check time synchronisation
	if step.Controls != nil && utils.LocalizeTimeUs(int64(step.Controls.TimeUsec), sitl.StartTimeUs) > simulationTimeUs {
		log.Debugw("PX4Sitl time is ahead of simulation",
			"PX4Sitl time (µs)", utils.LocalizeTimeUs(int64(step.Controls.TimeUsec), sitl.StartTimeUs),
			"Simulation time (µs)", simulationTimeUs)
	}

	iteration := sitl.NextStepIteration()

	sitl.lastUpdate = time.Now().UnixMicro()

	log.Debugw("Entering step",
		"Simulation time (µs)", simulationTimeUs,
		"IsLockStep", sitl.IsLockStep,
		"OnControls", step.Controls != nil,
		"Iteration", iteration)

	noiseFactor := 0.001
	attitudeQ := quaternion.Quaternion{
		W: 1,
		X: 0,
		Y: 0,
		Z: 0,
	}
	attitudeQ = quaternion.Prod(attitudeQ, quaternion.FromEuler(
		rand.Float64()*noiseFactor,
		rand.Float64()*noiseFactor,
		rand.Float64()*noiseFactor,
	))

	roll, pitch, yaw := attitudeQ.Euler()

	sitl.SendMessage(&dialect.MessageHilGps{
		TimeUsec:          updateTimeUsec,
		FixType:           0,
		Lat:               504890914, // 50.4549775
		Lon:               305195581, // 30.5195581
		Alt:               150 * 1000,
		Eph:               uint16(70 * (1 + noiseFactor*rand.Float64())),
		Epv:               uint16(110 * (1 + noiseFactor*rand.Float64())),
		Vel:               3,
		Vn:                0,
		Ve:                0,
		Vd:                0,
		Cog:               math.MaxUint16,
		SatellitesVisible: 30,
		Id:                0,
		Yaw:               0, // Not available
	})
	sitl.SendMessage(&dialect.MessageHilStateQuaternion{
		TimeUsec: updateTimeUsec,
		AttitudeQuaternion: [4]float32{
			float32(attitudeQ.W), float32(attitudeQ.X), float32(attitudeQ.Y), float32(attitudeQ.Z),
		},
		Rollspeed:    0.0,
		Pitchspeed:   0.0,
		Yawspeed:     0.0,
		Lat:          504549775,
		Lon:          305195581,
		Alt:          150.0 * 1000,
		Vx:           0.0,
		Vy:           0.0,
		Vz:           0.0,
		IndAirspeed:  0,
		TrueAirspeed: 0,
		Xacc:         0,
		Yacc:         0,
		Zacc:         0,
	})
	sitl.SendMessage(&dialect.MessageHilRcInputsRaw{
		TimeUsec:  updateTimeUsec,
		Chan1Raw:  0,
		Chan2Raw:  0,
		Chan3Raw:  0,
		Chan4Raw:  0,
		Chan5Raw:  0,
		Chan6Raw:  0,
		Chan7Raw:  0,
		Chan8Raw:  0,
		Chan9Raw:  0,
		Chan10Raw: 0,
		Chan11Raw: 0,
		Chan12Raw: 0,
		Rssi:      math.MaxUint8,
	})
	sitl.SendMessage(&dialect.MessageSimState{
		Q1:         float32(attitudeQ.W),
		Q2:         float32(attitudeQ.X),
		Q3:         float32(attitudeQ.Y),
		Q4:         float32(attitudeQ.Z),
		Roll:       float32(roll),
		Pitch:      float32(pitch),
		Yaw:        float32(yaw),
		Xacc:       0,
		Yacc:       0,
		Zacc:       0,
		Xgyro:      0,
		Ygyro:      0,
		Zgyro:      0,
		Lat:        50.4549775,
		Lon:        30.5195581,
		Alt:        150.0,
		StdDevHorz: 0,
		StdDevVert: 0,
		Vn:         0,
		Ve:         0,
		Vd:         0,
		LatInt:     504549775,
		LonInt:     305195581,
	})
	sitl.SendMessage(&dialect.MessageHilSensor{
		TimeUsec:      updateTimeUsec,
		Xacc:          0.0,
		Yacc:          0.0,
		Zacc:          0.0,
		Xgyro:         0.0,
		Ygyro:         0.0,
		Zgyro:         0.0,
		Xmag:          float32(noiseFactor) * rand.Float32(),
		Ymag:          float32(noiseFactor) * rand.Float32(),
		Zmag:          float32(noiseFactor) * rand.Float32(),
		AbsPressure:   1013.25 * (1 + float32(noiseFactor)*rand.Float32()), // Ground-level pressure
		DiffPressure:  0.0 * (1 + float32(noiseFactor)*rand.Float32()),
		PressureAlt:   150.0,
		Temperature:   22.0,
		FieldsUpdated: 1 | 2 | 4 | 8 | 16 | 32 | 64 | 128 | 256 | 512 | 1024 | 2048 | 4096, // All sensors
		Id:            0,
	})
}
