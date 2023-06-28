SITLer
======

Unified interface to integrate between simulators and autopilots.

Install
-------

Prerequisites:
- Go 1.20+
- Python 3.11
- [Pipenv](https://pipenv.pypa.io/en/latest/index.html)

Install dependencies:

```shell
go mod download
pipenv install
pipenv shell
```

The last step enters Python virtual environment in your current shell.

Run
---

Start server:

```shell
go run ./cmd/server.go
```

Start PX4 SITL in external simulator mode:

```shell
make px4_sitl none_iris
```

If you are running on different machines, set `PX4_SIM_HOSTNAME` environment variable to the machine running SITLer.

Roadmap
-------

Or, say, it's more a wish list ツ

- [ ] [PX4](https://px4.io/) support via MAVLink [protocol](https://docs.px4.io/main/en/simulation/#simulator-mavlink-api).
- [ ] [ArduPilot](https://ardupilot.org/) support via [JSON interface](https://ardupilot.org/dev/docs/sitl-with-JSON.html).
- [ ] Bullet3 integration.
- [ ] JABSim integration.
- [ ] Gazebo integration.
