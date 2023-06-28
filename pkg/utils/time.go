package utils

import "time"

const YearInUs = time.Hour * 24 * 365 / 1000

func LocalizeTimeUs[T int64 | uint64](timeUs T, startTimeUs int64) T {
	if timeUs > T(5*YearInUs) {
		return timeUs - T(startTimeUs)
	}
	return timeUs
}
