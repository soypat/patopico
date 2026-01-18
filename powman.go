//go:build rp2350

package patopico

import (
	"device"
	"runtime/volatile"
	"time"
	"unsafe"
)

const tracelog = true

// POWMAN base address for RP2350
const powmanBase = 0x40100000

// Password required for all POWMAN register writes (upper 16 bits)
const powmanPassword = 0x5afe0000

// powmanHW provides access to POWMAN hardware registers
var powmanHW = (*powmanRegs)(unsafe.Pointer(uintptr(powmanBase)))

// powmanRegs represents the memory-mapped POWMAN registers
type powmanRegs struct {
	BADPASSWD           volatile.Register32 // 0x00 - Bad password indicator
	VREG_CTRL           volatile.Register32 // 0x04 - Voltage regulator control
	VREG_STS            volatile.Register32 // 0x08 - Voltage regulator status
	VREG                volatile.Register32 // 0x0C - Voltage regulator settings
	VREG_LP_ENTRY       volatile.Register32 // 0x10 - VREG low power entry settings
	VREG_LP_EXIT        volatile.Register32 // 0x14 - VREG low power exit settings
	BOD_CTRL            volatile.Register32 // 0x18 - Brown-out detection control
	BOD                 volatile.Register32 // 0x1C - Brown-out detection settings
	BOD_LP_ENTRY        volatile.Register32 // 0x20 - BOD low power entry settings
	BOD_LP_EXIT         volatile.Register32 // 0x24 - BOD low power exit settings
	LPOSC               volatile.Register32 // 0x28 - Low power oscillator control
	CHIP_RESET          volatile.Register32 // 0x2C - Chip reset control and status
	WDSEL               volatile.Register32 // 0x30 - Watchdog select
	SEQ_CFG             volatile.Register32 // 0x34 - Power sequencer configuration
	STATE               volatile.Register32 // 0x38 - Power state control
	POW_FASTDIV         volatile.Register32 // 0x3C - POWMAN clock divider
	POW_DELAY           volatile.Register32 // 0x40 - Power state machine delays
	EXT_CTRL0           volatile.Register32 // 0x44 - External control 0
	EXT_CTRL1           volatile.Register32 // 0x48 - External control 1
	EXT_TIME_REF        volatile.Register32 // 0x4C - External time reference
	LPOSC_FREQ_KHZ_INT  volatile.Register32 // 0x50 - LPOSC freq integer part
	LPOSC_FREQ_KHZ_FRAC volatile.Register32 // 0x54 - LPOSC freq fractional part
	XOSC_FREQ_KHZ_INT   volatile.Register32 // 0x58 - XOSC freq integer part
	XOSC_FREQ_KHZ_FRAC  volatile.Register32 // 0x5C - XOSC freq fractional part
	SET_TIME_63TO48     volatile.Register32 // 0x60 - Set time bits 63:48
	SET_TIME_47TO32     volatile.Register32 // 0x64 - Set time bits 47:32
	SET_TIME_31TO16     volatile.Register32 // 0x68 - Set time bits 31:16
	SET_TIME_15TO0      volatile.Register32 // 0x6C - Set time bits 15:0
	READ_TIME_UPPER     volatile.Register32 // 0x70 - Read time bits 63:32
	READ_TIME_LOWER     volatile.Register32 // 0x74 - Read time bits 31:0
	ALARM_TIME_63TO48   volatile.Register32 // 0x78 - Alarm time bits 63:48
	ALARM_TIME_47TO32   volatile.Register32 // 0x7C - Alarm time bits 47:32
	ALARM_TIME_31TO16   volatile.Register32 // 0x80 - Alarm time bits 31:16
	ALARM_TIME_15TO0    volatile.Register32 // 0x84 - Alarm time bits 15:0
	TIMER               volatile.Register32 // 0x88 - Timer control
	PWRUP0              volatile.Register32 // 0x8C - GPIO powerup 0
	PWRUP1              volatile.Register32 // 0x90 - GPIO powerup 1
	PWRUP2              volatile.Register32 // 0x94 - GPIO powerup 2
	PWRUP3              volatile.Register32 // 0x98 - GPIO powerup 3
	CURRENT_PWRUP_REQ   volatile.Register32 // 0x9C - Current powerup request
	LAST_SWCORE_PWRUP   volatile.Register32 // 0xA0 - Last switched core powerup
	DBG_PWRCFG          volatile.Register32 // 0xA4 - Debug power config
	BOOTDIS             volatile.Register32 // 0xA8 - Boot disable
	DBGCONFIG           volatile.Register32 // 0xAC - Debug config
	SCRATCH0            volatile.Register32 // 0xB0 - Scratch register 0
	SCRATCH1            volatile.Register32 // 0xB4 - Scratch register 1
	SCRATCH2            volatile.Register32 // 0xB8 - Scratch register 2
	SCRATCH3            volatile.Register32 // 0xBC - Scratch register 3
	SCRATCH4            volatile.Register32 // 0xC0 - Scratch register 4
	SCRATCH5            volatile.Register32 // 0xC4 - Scratch register 5
	SCRATCH6            volatile.Register32 // 0xC8 - Scratch register 6
	SCRATCH7            volatile.Register32 // 0xCC - Scratch register 7
	BOOT0               volatile.Register32 // 0xD0 - Boot register 0
	BOOT1               volatile.Register32 // 0xD4 - Boot register 1
	BOOT2               volatile.Register32 // 0xD8 - Boot register 2
	BOOT3               volatile.Register32 // 0xDC - Boot register 3
	INTR                volatile.Register32 // 0xE0 - Raw interrupts
	INTE                volatile.Register32 // 0xE4 - Interrupt enable
	INTF                volatile.Register32 // 0xE8 - Interrupt force
	INTS                volatile.Register32 // 0xEC - Interrupt status
}

// Timer register bits
const (
	timerUsingGPIO1Hz  = 1 << 19
	timerUsingGPIO1kHz = 1 << 18
	timerUsingLPOSC    = 1 << 17
	timerUsingXOSC     = 1 << 16
	timerUseGPIO1Hz    = 1 << 13
	timerUseGPIO1kHz   = 1 << 10
	timerUseXOSC       = 1 << 9
	timerUseLPOSC      = 1 << 8
	timerAlarm         = 1 << 6
	timerPwrupOnAlarm  = 1 << 5
	timerAlarmEnab     = 1 << 4
	timerRun           = 1 << 1
)

// State register bits
const (
	stateChanging    = 1 << 13
	stateWaiting     = 1 << 12
	stateBadSWReq    = 1 << 10
	stateReqIgnored  = 1 << 8
	stateReqMask     = 0xF0
	stateReqLSB      = 4
	stateCurrentMask = 0x0F
)

// Seq config register bits
const (
	seqCfgHWPwrupSRAM0 = 1 << 1
	seqCfgHWPwrupSRAM1 = 1 << 0
)

// PWRUP register bits
const (
	pwrupStatus    = 1 << 9
	pwrupModeLSB   = 8
	pwrupModeLevel = 0
	pwrupModeEdge  = 1
	pwrupDirection = 1 << 7
	pwrupEnable    = 1 << 6
	pwrupSourceLSB = 0
)

// Interrupt enable bits
const (
	inteTimer = 1 << 1
)

// VREG control bits
const (
	vregCtrlUnlock = 1 << 13
)

// PowerDomain represents RP2350 power domains.
//
// csdk: hardware_powman/include/hardware/powman.h:163 enum powman_power_domains
type PowerDomain uint8

const (
	PowerDomainSRAMBank1    PowerDomain = 0 // Top 256K SRAM + scratch X/Y
	PowerDomainSRAMBank0    PowerDomain = 1 // Bottom 256K SRAM
	PowerDomainXIPCache     PowerDomain = 2 // 2x8K instruction cache
	PowerDomainSwitchedCore PowerDomain = 3 // Processors, bus fabric, peripherals
)

// PowerState represents a power configuration for all domains.
//
// csdk: hardware_powman/include/hardware/powman.h:171 powman_power_state
type PowerState uint32

// PowerStateNone represents all domains powered off.
//
// csdk: hardware_powman/include/hardware/powman.h:191 POWMAN_POWER_STATE_NONE
const PowerStateNone PowerState = 0

// WithDomainOn returns a new PowerState with the specified domain enabled.
//
// csdk: hardware_powman/include/hardware/powman.h:198 powman_power_state powman_power_state_with_domain_on(powman_power_state orig, enum powman_power_domains domain)
func (ps PowerState) WithDomainOn(domain PowerDomain) PowerState {
	return ps | (1 << domain)
}

// WithDomainOff returns a new PowerState with the specified domain disabled.
//
// csdk: hardware_powman/include/hardware/powman.h:208 powman_power_state powman_power_state_with_domain_off(powman_power_state orig, enum powman_power_domains domain)
func (ps PowerState) WithDomainOff(domain PowerDomain) PowerState {
	return ps &^ (1 << domain)
}

// IsDomainOn returns true if the specified domain is enabled in this state.
//
// csdk: hardware_powman/include/hardware/powman.h:218 bool powman_power_state_is_domain_on(powman_power_state state, enum powman_power_domains domain)
func (ps PowerState) IsDomainOn(domain PowerDomain) bool {
	return ps&(1<<domain) != 0
}

// powmanWrite writes a value with the required password.
//
// csdk: hardware_powman/powman.c:30 void powman_write(volatile uint32_t *reg, uint32_t value)
func powmanWrite(reg *volatile.Register32, value uint32) {
	if tracelog {
		println("hardware_powman/powman.c:30 powman_write", reg, value)
		serialflush()
	}
	reg.Set(powmanPassword | (value & 0xFFFF))
}

// powmanSetBits sets bits in a register with the required password.
//
// csdk: hardware_powman/include/hardware/powman.h:110 void powman_set_bits(volatile uint32_t *reg, uint32_t bits)
func powmanSetBits(reg *volatile.Register32, bits uint32) {
	if tracelog {
		println("hardware_powman/include/hardware/powman.h:110 powman_set_bits", reg, bits)
		serialflush()
	}
	// Use atomic set register (base + 0x2000)
	regAddr := uintptr(unsafe.Pointer(reg))
	atomicSet := (*volatile.Register32)(unsafe.Pointer(regAddr + 0x2000))
	atomicSet.Set(powmanPassword | bits)
}

// powmanClearBits clears bits in a register with the required password.
//
// csdk: hardware_powman/include/hardware/powman.h:124 void powman_clear_bits(volatile uint32_t *reg, uint32_t bits)
func powmanClearBits(reg *volatile.Register32, bits uint32) {
	if tracelog {
		println("hardware_powman/include/hardware/powman.h:124 powman_clear_bits", reg, bits)
		serialflush()
	}
	// Use atomic clear register (base + 0x3000)
	regAddr := uintptr(unsafe.Pointer(reg))
	atomicClear := (*volatile.Register32)(unsafe.Pointer(regAddr + 0x3000))
	atomicClear.Set(powmanPassword | bits)
}

// TimerStart starts the POWMAN timer.
//
// csdk: hardware_powman/include/hardware/powman.h:146 void powman_timer_start(void)
func TimerStart() {
	if tracelog {
		println("hardware_powman/include/hardware/powman.h:146 powman_timer_start")
		serialflush()
	}
	powmanSetBits(&powmanHW.TIMER, timerRun)
}

// TimerStop stops the POWMAN timer.
//
// csdk: hardware_powman/include/hardware/powman.h:139 void powman_timer_stop(void)
func TimerStop() {
	if tracelog {
		println("hardware_powman/include/hardware/powman.h:139 powman_timer_stop")
		serialflush()
	}
	powmanClearBits(&powmanHW.TIMER, timerRun)
}

// TimerIsRunning returns true if the POWMAN timer is running.
//
// csdk: hardware_powman/include/hardware/powman.h:132 bool powman_timer_is_running(void)
func TimerIsRunning() bool {
	if tracelog {
		println("hardware_powman/include/hardware/powman.h:132 powman_timer_is_running")
		serialflush()
	}
	return powmanHW.TIMER.Get()&timerRun != 0
}

// TimerGetMs returns the current timer value in milliseconds.
//
// csdk: hardware_powman/powman.c:46 uint64_t powman_timer_get_ms(void)
func TimerGetMs() uint64 {
	if tracelog {
		println("hardware_powman/powman.c:46 powman_timer_get_ms")
		serialflush()
	}
	// Need to ensure upper 32 bits don't change during read
	hi := powmanHW.READ_TIME_UPPER.Get()
	for {
		lo := powmanHW.READ_TIME_LOWER.Get()
		nextHi := powmanHW.READ_TIME_UPPER.Get()
		if hi == nextHi {
			return (uint64(hi) << 32) | uint64(lo)
		}
		hi = nextHi
	}
}

// TimerSetMs sets the timer value in milliseconds.
//
// csdk: hardware_powman/powman.c:36 void powman_timer_set_ms(uint64_t time_ms)
func TimerSetMs(timeMs uint64) {
	if tracelog {
		println("hardware_powman/powman.c:36 powman_timer_set_ms", timeMs)
		serialflush()
	}
	wasRunning := TimerIsRunning()
	if wasRunning {
		TimerStop()
	}
	powmanWrite(&powmanHW.SET_TIME_15TO0, uint32(timeMs&0xFFFF))
	powmanWrite(&powmanHW.SET_TIME_31TO16, uint32((timeMs>>16)&0xFFFF))
	powmanWrite(&powmanHW.SET_TIME_47TO32, uint32((timeMs>>32)&0xFFFF))
	powmanWrite(&powmanHW.SET_TIME_63TO48, uint32((timeMs>>48)&0xFFFF))
	if wasRunning {
		TimerStart()
	}
}

// TimerSetTickSourceLPOSC sets the timer to use the ~32kHz low power oscillator.
//
// csdk: hardware_powman/powman.c:64 void powman_timer_set_1khz_tick_source_lposc(void)
func TimerSetTickSourceLPOSC() {
	if tracelog {
		println("hardware_powman/powman.c:64 powman_timer_set_1khz_tick_source_lposc")
		serialflush()
	}
	TimerSetTickSourceLPOSCWithHz(32768)
}

// TimerSetTickSourceLPOSCWithHz sets the timer to use the low power oscillator
// with a specific frequency for accurate timekeeping.
//
// csdk: hardware_powman/powman.c:68 void powman_timer_set_1khz_tick_source_lposc_with_hz(uint32_t lposc_freq_hz)
func TimerSetTickSourceLPOSCWithHz(lposcFreqHz uint32) {
	if tracelog {
		println("hardware_powman/powman.c:68 powman_timer_set_1khz_tick_source_lposc_with_hz", lposcFreqHz)
		serialflush()
	}
	wasRunning := TimerIsRunning()
	if wasRunning {
		TimerStop()
	}
	lposcFreqKhz := lposcFreqHz / 1000
	lposcFreqKhzFrac16 := (lposcFreqHz % 1000) * 65536 / 1000
	powmanWrite(&powmanHW.LPOSC_FREQ_KHZ_INT, lposcFreqKhz)
	powmanWrite(&powmanHW.LPOSC_FREQ_KHZ_FRAC, lposcFreqKhzFrac16)
	powmanSetBits(&powmanHW.TIMER, timerUseLPOSC)
	if wasRunning {
		TimerStart()
		for powmanHW.TIMER.Get()&timerUsingLPOSC == 0 {
		}
	}
}

// TimerSetTickSourceXOSC sets the timer to use the crystal oscillator (12MHz default).
//
// csdk: hardware_powman/powman.c:82 void powman_timer_set_1khz_tick_source_xosc(void)
func TimerSetTickSourceXOSC() {
	if tracelog {
		println("hardware_powman/powman.c:82 powman_timer_set_1khz_tick_source_xosc")
		serialflush()
	}
	TimerSetTickSourceXOSCWithHz(12_000_000)
}

// TimerSetTickSourceXOSCWithHz sets the timer to use the crystal oscillator
// with a specific frequency.
//
// csdk: hardware_powman/powman.c:86 void powman_timer_set_1khz_tick_source_xosc_with_hz(uint32_t xosc_freq_hz)
func TimerSetTickSourceXOSCWithHz(xoscFreqHz uint32) {
	if tracelog {
		println("hardware_powman/powman.c:86 powman_timer_set_1khz_tick_source_xosc_with_hz", xoscFreqHz)
		serialflush()
	}
	wasRunning := TimerIsRunning()
	if wasRunning {
		TimerStop()
	}
	xoscFreqKhz := xoscFreqHz / 1000
	xoscFreqKhzFrac16 := (xoscFreqHz % 1000) * 65536 / 1000
	powmanWrite(&powmanHW.XOSC_FREQ_KHZ_INT, xoscFreqKhz)
	powmanWrite(&powmanHW.XOSC_FREQ_KHZ_FRAC, xoscFreqKhzFrac16)
	powmanSetBits(&powmanHW.TIMER, timerUseXOSC)
	if wasRunning {
		TimerStart()
		for powmanHW.TIMER.Get()&timerUsingXOSC == 0 {
		}
	}
}

// gpioToExtTimeRefSource maps GPIO pins to external time reference sources.
// Valid GPIOs are: 12->0, 20->1, 14->2, 22->3
//
// csdk: hardware_powman/powman.c:109 uint32_t gpio_to_powman_ext_time_ref_source(uint gpio, uint32_t default_ext_time_ref_source)
func gpioToExtTimeRefSource(gpio uint8) uint32 {
	switch gpio {
	case 12:
		return 0
	case 20:
		return 1
	case 14:
		return 2
	case 22:
		return 3
	default:
		return 0 // Invalid GPIO, caller should validate
	}
}

// TimerSetTickSourceGPIO sets the timer to use an external 1kHz GPIO source.
// Valid GPIOs are: 12, 14, 20, 22
//
// csdk: hardware_powman/powman.c:126 void powman_timer_set_1khz_tick_source_gpio(uint32_t gpio)
func TimerSetTickSourceGPIO(gpio uint8) {
	if tracelog {
		println("hardware_powman/powman.c:126 powman_timer_set_1khz_tick_source_gpio", gpio)
		serialflush()
	}
	wasRunning := TimerIsRunning()
	if wasRunning {
		TimerStop()
	}
	source := gpioToExtTimeRefSource(gpio)
	// TODO: Enable GPIO input (requires GPIO package integration)
	powmanWrite(&powmanHW.EXT_TIME_REF, source)
	powmanSetBits(&powmanHW.TIMER, timerUseGPIO1kHz)
	if wasRunning {
		TimerStart()
		for powmanHW.TIMER.Get()&timerUsingGPIO1kHz == 0 {
		}
	}
}

// TimerEnableGPIO1HzSync enables synchronization to a 1Hz GPIO signal
// for accurate seconds timekeeping (e.g., from GPS).
// Valid GPIOs are: 12, 14, 20, 22
//
// csdk: hardware_powman/powman.c:131 void powman_timer_enable_gpio_1hz_sync(uint32_t gpio)
func TimerEnableGPIO1HzSync(gpio uint8) {
	if tracelog {
		println("hardware_powman/powman.c:131 powman_timer_enable_gpio_1hz_sync", gpio)
		serialflush()
	}
	wasRunning := TimerIsRunning()
	if wasRunning {
		TimerStop()
	}
	source := gpioToExtTimeRefSource(gpio)
	// TODO: Enable GPIO input (requires GPIO package integration)
	powmanWrite(&powmanHW.EXT_TIME_REF, source)
	powmanSetBits(&powmanHW.TIMER, timerUseGPIO1Hz)
	if wasRunning {
		TimerStart()
		for powmanHW.TIMER.Get()&timerUsingGPIO1Hz == 0 {
		}
	}
}

// TimerDisableGPIO1HzSync disables the 1Hz GPIO synchronization.
//
// csdk: hardware_powman/powman.c:136 void powman_timer_disable_gpio_1hz_sync(void)
func TimerDisableGPIO1HzSync() {
	if tracelog {
		println("hardware_powman/powman.c:136 powman_timer_disable_gpio_1hz_sync")
		serialflush()
	}
	powmanClearBits(&powmanHW.TIMER, timerUseGPIO1Hz)
}

// ClearAlarm clears a fired alarm.
// Note: The alarm must be disabled first.
//
// csdk: hardware_powman/include/hardware/powman.h:156 void powman_clear_alarm(void)
func ClearAlarm() {
	if tracelog {
		println("hardware_powman/include/hardware/powman.h:156 powman_clear_alarm")
		serialflush()
	}
	powmanClearBits(&powmanHW.TIMER, timerAlarm)
}

// TimerEnableAlarmAtMs sets an alarm to fire at the specified absolute time.
//
// csdk: hardware_powman/powman.c:226 void powman_timer_enable_alarm_at_ms(uint64_t alarm_time_ms)
func TimerEnableAlarmAtMs(alarmTimeMs uint64) {
	if tracelog {
		println("hardware_powman/powman.c:226 powman_timer_enable_alarm_at_ms", alarmTimeMs)
		serialflush()
	}
	powmanSetBits(&powmanHW.INTE, inteTimer)
	powmanClearBits(&powmanHW.TIMER, timerAlarmEnab)
	// Alarm must be disabled to set the alarm time
	powmanWrite(&powmanHW.ALARM_TIME_15TO0, uint32(alarmTimeMs&0xFFFF))
	powmanWrite(&powmanHW.ALARM_TIME_31TO16, uint32((alarmTimeMs>>16)&0xFFFF))
	powmanWrite(&powmanHW.ALARM_TIME_47TO32, uint32((alarmTimeMs>>32)&0xFFFF))
	powmanWrite(&powmanHW.ALARM_TIME_63TO48, uint32((alarmTimeMs>>48)&0xFFFF))
	ClearAlarm()
	powmanSetBits(&powmanHW.TIMER, timerAlarmEnab)
}

// TimerDisableAlarm disables the timer alarm.
//
// csdk: hardware_powman/powman.c:239 void powman_timer_disable_alarm(void)
func TimerDisableAlarm() {
	if tracelog {
		println("hardware_powman/powman.c:239 powman_timer_disable_alarm")
		serialflush()
	}
	powmanClearBits(&powmanHW.INTE, inteTimer)
	powmanClearBits(&powmanHW.TIMER, timerAlarmEnab)
}

// GetPowerState returns the current power state.
//
// csdk: hardware_powman/powman.c:140 powman_power_state powman_get_power_state(void)
func GetPowerState() PowerState {
	if tracelog {
		println("hardware_powman/powman.c:140 powman_get_power_state")
		serialflush()
	}
	stateReg := ^powmanHW.STATE.Get() & stateCurrentMask
	return PowerState(stateReg)
}

// SetPowerState requests a new power state.
// Returns nil on success, or an error if the state change failed.
// Note: If turning off switched core, this function will not return.
//
// csdk: hardware_powman/powman.c:152 int powman_set_power_state(powman_power_state state)
func SetPowerState(state PowerState) error {
	if tracelog {
		println("hardware_powman/powman.c:152 powman_set_power_state", state)
		serialflush()
	}
	// Clear req ignored in case it has been set
	powmanClearBits(&powmanHW.STATE, stateReqIgnored)

	// Request the new state (inverted encoding)
	powmanWrite(&powmanHW.STATE, (^uint32(state)<<stateReqLSB)&stateReqMask)

	// Check if request was ignored
	if powmanHW.STATE.Get()&stateReqIgnored != 0 {
		return errPreconditionNotMet
	}

	// Check if state is valid
	if powmanHW.STATE.Get()&stateBadSWReq != 0 {
		return errInvalidArg
	}

	// If turning off switched core, wait for WAITING flag
	if !state.IsDomainOn(PowerDomainSwitchedCore) {
		for i := 0; i < 100; i++ {
			if powmanHW.STATE.Get()&stateWaiting != 0 {
				return nil
			}
		}
		return errTimeout
	}

	// Wait for state change to complete
	for powmanHW.STATE.Get()&stateChanging != 0 {
	}
	return nil
}

// ConfigureWakeupState configures the power states for sleep and wakeup.
// sleepState must have switched core OFF.
// wakeupState must have switched core ON.
// Returns true if the configuration is valid.
//
// csdk: hardware_powman/powman.c:196 bool powman_configure_wakeup_state(powman_power_state sleep_state, powman_power_state wakeup_state)
func ConfigureWakeupState(sleepState, wakeupState PowerState) bool {
	if tracelog {
		println("hardware_powman/powman.c:196 powman_configure_wakeup_state", sleepState, wakeupState)
		serialflush()
	}
	// Must be entering P1 state (SWCORE OFF)
	valid := !sleepState.IsDomainOn(PowerDomainSwitchedCore)
	// Must be waking to P0 state (SWCORE ON)
	valid = valid && wakeupState.IsDomainOn(PowerDomainSwitchedCore)

	currentState := GetPowerState()
	currentSRAM0 := currentState.IsDomainOn(PowerDomainSRAMBank0)
	currentSRAM1 := currentState.IsDomainOn(PowerDomainSRAMBank1)
	sleepSRAM0 := sleepState.IsDomainOn(PowerDomainSRAMBank0)
	sleepSRAM1 := sleepState.IsDomainOn(PowerDomainSRAMBank1)
	wakeupSRAM0 := wakeupState.IsDomainOn(PowerDomainSRAMBank0)
	wakeupSRAM1 := wakeupState.IsDomainOn(PowerDomainSRAMBank1)

	// Sleep state cannot turn ON SRAM if currently OFF
	if !currentSRAM0 {
		valid = valid && !sleepSRAM0
	}
	if !currentSRAM1 {
		valid = valid && !sleepSRAM1
	}
	// Wakeup cannot turn OFF SRAM if sleep has it ON
	if sleepSRAM0 {
		valid = valid && wakeupSRAM0
	}
	if sleepSRAM1 {
		valid = valid && wakeupSRAM1
	}

	if valid {
		// Configure power sequencer for SRAM wakeup
		powmanClearBits(&powmanHW.SEQ_CFG, seqCfgHWPwrupSRAM0|seqCfgHWPwrupSRAM1)
		var seqCfgSet uint32
		if sleepSRAM0 == wakeupSRAM0 {
			seqCfgSet |= seqCfgHWPwrupSRAM0
		}
		if sleepSRAM1 == wakeupSRAM1 {
			seqCfgSet |= seqCfgHWPwrupSRAM1
		}
		powmanSetBits(&powmanHW.SEQ_CFG, seqCfgSet)
	}
	return valid
}

// EnableAlarmWakeup enables waking from sleep at the specified time.
//
// csdk: hardware_powman/powman.c:244 void powman_enable_alarm_wakeup_at_ms(uint64_t alarm_time_ms)
func EnableAlarmWakeup(alarmTimeMs uint64) {
	if tracelog {
		println("hardware_powman/powman.c:244 powman_enable_alarm_wakeup_at_ms", alarmTimeMs)
		serialflush()
	}
	TimerEnableAlarmAtMs(alarmTimeMs)
	powmanSetBits(&powmanHW.TIMER, timerPwrupOnAlarm)
}

// DisableAlarmWakeup disables timer-based wakeup.
//
// csdk: hardware_powman/powman.c:249 void powman_disable_alarm_wakeup(void)
func DisableAlarmWakeup() {
	if tracelog {
		println("hardware_powman/powman.c:249 powman_disable_alarm_wakeup")
		serialflush()
	}
	TimerDisableAlarm()
	powmanClearBits(&powmanHW.TIMER, timerPwrupOnAlarm)
}

// EnableGPIOWakeup configures a GPIO to wake the chip from sleep.
// wakeupNum: hardware wakeup channel (0-3)
// gpio: GPIO pin number (0-47)
// edge: true for edge-triggered, false for level-triggered
// high: true for active-high/rising-edge, false for active-low/falling-edge
//
// csdk: hardware_powman/powman.c:254 void powman_enable_gpio_wakeup(uint gpio_wakeup_num, uint32_t gpio, bool edge, bool high)
func EnableGPIOWakeup(wakeupNum uint8, gpio uint8, edge bool, high bool) {
	if tracelog {
		println("hardware_powman/powman.c:254 powman_enable_gpio_wakeup", wakeupNum, gpio, edge, high)
		serialflush()
	}
	if wakeupNum > 3 {
		return
	}

	// TODO: Enable GPIO input (requires GPIO package integration)

	// Configure wakeup source
	var pwrup uint32
	if edge {
		pwrup = pwrupModeEdge << pwrupModeLSB
	} else {
		pwrup = pwrupModeLevel << pwrupModeLSB
	}
	if high {
		pwrup |= pwrupDirection
	}
	pwrup |= uint32(gpio) << pwrupSourceLSB

	pwrupReg := getPwrupReg(wakeupNum)
	powmanWrite(pwrupReg, pwrup)

	// Clear status bit in case edge is already latched
	powmanClearBits(pwrupReg, pwrupStatus)

	// Enable the wakeup source
	powmanSetBits(pwrupReg, pwrupEnable)
}

// DisableGPIOWakeup disables a GPIO wakeup source.
//
// csdk: hardware_powman/powman.c:273 void powman_disable_gpio_wakeup(uint gpio_wakeup_num)
func DisableGPIOWakeup(wakeupNum uint8) {
	if tracelog {
		println("hardware_powman/powman.c:273 powman_disable_gpio_wakeup", wakeupNum)
		serialflush()
	}
	if wakeupNum > 3 {
		return
	}
	pwrupReg := getPwrupReg(wakeupNum)
	powmanClearBits(pwrupReg, pwrupEnable)
}

// DisableAllWakeups disables all wakeup sources.
//
// csdk: hardware_powman/powman.c:278 void powman_disable_all_wakeups(void)
func DisableAllWakeups() {
	if tracelog {
		println("hardware_powman/powman.c:278 powman_disable_all_wakeups")
		serialflush()
	}
	for i := uint8(0); i < 4; i++ {
		DisableGPIOWakeup(i)
	}
	DisableAlarmWakeup()
}

// SetDebugPowerRequestIgnored controls whether debugger power requests are ignored.
// Set to true to allow sleep even with debugger attached.
//
// csdk: hardware_powman/include/hardware/powman.h:272 void powman_set_debug_power_request_ignored(bool ignored)
func SetDebugPowerRequestIgnored(ignored bool) {
	if tracelog {
		println("hardware_powman/include/hardware/powman.h:272 powman_set_debug_power_request_ignored", ignored)
		serialflush()
	}
	if ignored {
		powmanSetBits(&powmanHW.DBG_PWRCFG, 1)
	} else {
		powmanClearBits(&powmanHW.DBG_PWRCFG, 1)
	}
}

// UnlockVREG unlocks the voltage regulator control interface.
// Used in powman_timer example for low-power voltage transitions.
//
// csdk: (direct register access) powman_hw->vreg_ctrl |= POWMAN_VREG_CTRL_UNLOCK_BITS
func UnlockVREG() {
	if tracelog {
		println("(direct register access) powman_hw->vreg_ctrl |= POWMAN_VREG_CTRL_UNLOCK_BITS")
		serialflush()
	}
	powmanSetBits(&powmanHW.VREG_CTRL, vregCtrlUnlock)
}

// Scratch returns a pointer to a scratch register (0-7).
// Scratch registers persist across sleep/wakeup cycles.
//
// csdk: (direct register access) powman_hw->scratch[n]
func Scratch(num uint8) *volatile.Register32 {
	if num > 7 {
		return nil
	}
	base := uintptr(unsafe.Pointer(&powmanHW.SCRATCH0))
	return (*volatile.Register32)(unsafe.Pointer(base + uintptr(num)*4))
}

// Boot returns a pointer to a boot register (0-3).
//
// csdk: (direct register access) powman_hw->boot[n]
func Boot(num uint8) *volatile.Register32 {
	if num > 3 {
		return nil
	}
	base := uintptr(unsafe.Pointer(&powmanHW.BOOT0))
	return (*volatile.Register32)(unsafe.Pointer(base + uintptr(num)*4))
}

// getPwrupReg returns a pointer to the specified PWRUP register.
//
// csdk: (direct register access) powman_hw->pwrup[n]
func getPwrupReg(num uint8) *volatile.Register32 {
	base := uintptr(unsafe.Pointer(&powmanHW.PWRUP0))
	return (*volatile.Register32)(unsafe.Pointer(base + uintptr(num)*4))
}

// Error types for power management operations
type powmanError uint8

const (
	errInvalidArg         powmanError = 1
	errPreconditionNotMet powmanError = 2
	errTimeout            powmanError = 3
	errInvalidState       powmanError = 4
)

func (e powmanError) Error() string {
	switch e {
	case errInvalidArg:
		return "powman: invalid argument"
	case errPreconditionNotMet:
		return "powman: precondition not met (pending pwrup request)"
	case errTimeout:
		return "powman: timeout waiting for state change"
	case errInvalidState:
		return "powman: invalid state configuration"
	default:
		return "powman: unknown error"
	}
}

// SleepConfig holds configuration for entering low-power sleep mode.
// Based on the power states used in pico-examples/powman/powman_example.c
type SleepConfig struct {
	// SleepState defines which power domains remain on during sleep
	// Switched core must be OFF for sleep
	SleepState PowerState
	// WakeupState defines which power domains to enable on wakeup
	// Switched core must be ON for wakeup
	WakeupState PowerState
}

// DefaultSleepConfig returns a default sleep configuration.
// Sleep: All domains off (P1.7)
// Wakeup: Switched core + XIP cache on (P0.3)
//
// csdk: pico-examples/powman/powman_example.c:14 void powman_example_init(uint64_t abs_time_ms)
func DefaultSleepConfig() SleepConfig {
	wakeup := PowerStateNone.
		WithDomainOn(PowerDomainSwitchedCore).
		WithDomainOn(PowerDomainXIPCache)
	return SleepConfig{
		SleepState:  PowerStateNone,
		WakeupState: wakeup,
	}
}

// Sleep enters low-power mode and waits for a wakeup event.
// The wakeup source must be configured before calling this function.
// This function will not return if switched core is powered off.
// On wakeup, the device will restart from the beginning of main.
//
// csdk: pico-examples/powman/powman_example.c:34 static int powman_example_off(void)
func (cfg *SleepConfig) Sleep() error {
	if tracelog {
		println("pico-examples/powman/powman_example.c:34 powman_example_off")
		serialflush()
	}
	valid := ConfigureWakeupState(cfg.SleepState, cfg.WakeupState)
	if !valid {
		return errInvalidState
	}

	// Clear boot registers to reboot to main on wakeup
	Boot(0).Set(0)
	Boot(1).Set(0)
	Boot(2).Set(0)
	Boot(3).Set(0)

	// Request the sleep power state
	err := SetPowerState(cfg.SleepState)
	if err != nil {
		return err
	}

	// Wait for interrupt (processor halts here, woken by wakeup event)
	// WFI instruction - processor enters low-power state until interrupt
	// In TinyGo, this is typically provided by device/arm or runtime
	// For now, use a busy wait as fallback - replace with proper WFI when available
	// The actual power savings come from POWMAN state transition, not WFI
	device.Asm("wfi")
	return nil
}

// SleepForMs puts the device into low-power sleep for the specified duration.
// Uses the POWMAN timer for wakeup.
// This function will not return - on wakeup, the device restarts from main.
//
// csdk: pico-examples/powman/powman_example.c:99 int powman_example_off_for_ms(uint64_t duration_ms)
func SleepForMs(durationMs uint64) error {
	if tracelog {
		println("pico-examples/powman/powman_example.c:99 powman_example_off_for_ms", durationMs)
		serialflush()
	}
	cfg := DefaultSleepConfig()
	return cfg.SleepForMs(durationMs)
}

// SleepForMs puts the device into low-power sleep for the specified duration.
//
// csdk: pico-examples/powman/powman_example.c:99 int powman_example_off_for_ms(uint64_t duration_ms)
func (cfg *SleepConfig) SleepForMs(durationMs uint64) error {
	if tracelog {
		println("pico-examples/powman/powman_example.c:99 powman_example_off_for_ms", durationMs)
		serialflush()
	}
	alarmTime := TimerGetMs() + durationMs
	EnableAlarmWakeup(alarmTime)
	return cfg.Sleep()
}

// SleepUntilMs puts the device into low-power sleep until the specified absolute time.
// This function will not return - on wakeup, the device restarts from main.
//
// csdk: pico-examples/powman/powman_example.c:91 int powman_example_off_until_time(uint64_t abs_time_ms)
func SleepUntilMs(absTimeMs uint64) error {
	if tracelog {
		println("pico-examples/powman/powman_example.c:91 powman_example_off_until_time", absTimeMs)
		serialflush()
	}
	cfg := DefaultSleepConfig()
	return cfg.SleepUntilMs(absTimeMs)
}

// SleepUntilMs puts the device into low-power sleep until the specified absolute time.
//
// csdk: pico-examples/powman/powman_example.c:91 int powman_example_off_until_time(uint64_t abs_time_ms)
func (cfg *SleepConfig) SleepUntilMs(absTimeMs uint64) error {
	if tracelog {
		println("pico-examples/powman/powman_example.c:91 powman_example_off_until_time", absTimeMs)
		serialflush()
	}
	EnableAlarmWakeup(absTimeMs)
	return cfg.Sleep()
}

// SleepUntilGPIOHigh puts the device into low-power sleep until the specified GPIO goes high.
// This function will not return - on wakeup, the device restarts from main.
//
// csdk: pico-examples/powman/powman_example.c:61 int powman_example_off_until_gpio_high(int gpio)
func SleepUntilGPIOHigh(gpio uint8) error {
	if tracelog {
		println("pico-examples/powman/powman_example.c:61 powman_example_off_until_gpio_high", gpio)
		serialflush()
	}
	cfg := DefaultSleepConfig()
	return cfg.SleepUntilGPIOHigh(gpio)
}

// SleepUntilGPIOHigh puts the device into low-power sleep until the specified GPIO goes high.
//
// csdk: pico-examples/powman/powman_example.c:61 int powman_example_off_until_gpio_high(int gpio)
func (cfg *SleepConfig) SleepUntilGPIOHigh(gpio uint8) error {
	if tracelog {
		println("pico-examples/powman/powman_example.c:61 powman_example_off_until_gpio_high", gpio)
		serialflush()
	}
	EnableGPIOWakeup(0, gpio, false, true) // Level-triggered, active high
	return cfg.Sleep()
}

// SleepUntilGPIOLow puts the device into low-power sleep until the specified GPIO goes low.
// This function will not return - on wakeup, the device restarts from main.
//
// csdk: pico-examples/powman/powman_example.c:76 int powman_example_off_until_gpio_low(int gpio)
func SleepUntilGPIOLow(gpio uint8) error {
	if tracelog {
		println("pico-examples/powman/powman_example.c:76 powman_example_off_until_gpio_low", gpio)
		serialflush()
	}
	cfg := DefaultSleepConfig()
	return cfg.SleepUntilGPIOLow(gpio)
}

// SleepUntilGPIOLow puts the device into low-power sleep until the specified GPIO goes low.
//
// csdk: pico-examples/powman/powman_example.c:76 int powman_example_off_until_gpio_low(int gpio)
func (cfg *SleepConfig) SleepUntilGPIOLow(gpio uint8) error {
	if tracelog {
		println("pico-examples/powman/powman_example.c:76 powman_example_off_until_gpio_low", gpio)
		serialflush()
	}
	EnableGPIOWakeup(0, gpio, false, false) // Level-triggered, active low
	return cfg.Sleep()
}

// Init initializes the POWMAN subsystem with the specified time.
// Call this early in main() to set up the always-on timer.
//
// csdk: pico-examples/powman/powman_example.c:14 void powman_example_init(uint64_t abs_time_ms)
func Init(absTimeMs uint64) {
	if tracelog {
		println("pico-examples/powman/powman_example.c:14 powman_example_init", absTimeMs)
		serialflush()
	}
	TimerStart()
	TimerSetMs(absTimeMs)
	// Allow power down even with debugger connected
	SetDebugPowerRequestIgnored(true)
}

func serialflush() {
	time.Sleep(time.Millisecond)
}
