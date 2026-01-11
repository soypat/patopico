# patopico

TinyGo hardware drivers for RP2350 (Pico 2).

## Features

- **POWMAN**: Power management with low-power sleep modes, GPIO/timer wakeup

## Usage

```go
import "github.com/soypat/patopico"

func main() {
    patopico.Init(0)
    // Do work...
    patopico.SleepForMs(10_000) // Sleep 10 seconds, restarts from main() on wakeup
}
```

## References
Functions are documented with reference source code:
```go
// Init initializes the POWMAN subsystem with the specified time.
// Call this early in main() to set up the always-on timer.
//
// csdk: pico-examples/powman/powman_example.c:14 void powman_example_init(uint64_t abs_time_ms)
func Init(absTimeMs uint64) {
```

- [pico-sdk/src/rp2_common/hardware_powman](https://github.com/raspberrypi/pico-sdk/tree/master/src/rp2_common/hardware_powman) - C SDK powman implementation
- [pico-examples/powman](https://github.com/raspberrypi/pico-examples/tree/master/powman) - C example code
- [RP2350 Datasheet](https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf) - Chapter 6: POWMAN
