//go:build rp2350

package main

import (
	"machine"
	"time"

	"github.com/soypat/patopico"
)

const sleepDuration = 10_000

func main() {
	// Initialize POWMAN with current time (0ms at boot)
	patopico.Init(0)

	// Configure LED for work indication
	led := machine.LED
	led.Configure(machine.PinConfig{Mode: machine.PinOutput})

	// Main loop: do work, sleep, repeat
	for {
		doWork(led)

		// Enter low power sleep for 2 minutes
		// Note: This will restart from main() on wakeup
		patopico.SleepForMs(sleepDuration)
	}
}

// doWork simulates some work by blinking the LED
func doWork(led machine.Pin) {
	// Blink LED 5 times to indicate we're doing work
	for i := 0; i < 5; i++ {
		led.High()
		time.Sleep(100 * time.Millisecond)
		led.Low()
		time.Sleep(100 * time.Millisecond)
	}
}
