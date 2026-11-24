//! Read-only telemetry for every connected V5 smart motor.
//! No SmartPort or Motor is constructed, and no device state is changed.
use ozton_diagnostics::serial::Frame;
use vex_sdk::{
    V5_DeviceType, V5_MAX_DEVICE_PORTS, V5MotorControlMode, V5MotorEncoderUnits,
    vexDeviceGetByIndex, vexDeviceGetStatus, vexDeviceMotorActualVelocityGet,
    vexDeviceMotorBrakeModeGet, vexDeviceMotorCurrentGet, vexDeviceMotorEncoderUnitsGet,
    vexDeviceMotorModeGet, vexDeviceMotorPositionGet, vexDeviceMotorTargetGet,
    vexDeviceMotorTemperatureGet, vexDeviceMotorTorqueGet, vexDeviceMotorVelocityGet,
    vexDeviceMotorVoltageGet,
};

/// Discover connected motors from one device table snapshot and report measured
/// values. Firmware target registers are reported separately from software commands.
pub fn sample(frame: Frame) {
    let mut devices = [V5_DeviceType::kDeviceTypeNoSensor; V5_MAX_DEVICE_PORTS];
    // SAFETY: the buffer is the SDK's documented maximum device table length.
    unsafe { vexDeviceGetStatus(devices.as_mut_ptr()); }
    for (index, kind) in devices.iter().take(21).enumerate() {
        if *kind != V5_DeviceType::kDeviceTypeMotorSensor {
            continue;
        }
        let port = index + 1;
        // SAFETY: the status snapshot identifies a motor on this valid smart port.
        // Only read SDK telemetry; never acquire ownership or issue a motor command.
        let sample = unsafe {
            let device = vexDeviceGetByIndex(index as u32);
            if device.is_null() {
                continue;
            }
            let mode = vexDeviceMotorModeGet(device);
            let units = vexDeviceMotorEncoderUnitsGet(device);
            let degrees = |value: f64| match units {
                V5MotorEncoderUnits::kMotorEncoderDegrees => value,
                V5MotorEncoderUnits::kMotorEncoderRotations => value * 360.0,
                _ => f64::NAN,
            };
            let target_rpm = match mode {
                V5MotorControlMode::kMotorControlModeVELOCITY
                | V5MotorControlMode::kMotorControlModePROFILE => f64::from(vexDeviceMotorVelocityGet(device)),
                _ => f64::NAN,
            };
            let target_deg = match mode {
                V5MotorControlMode::kMotorControlModeSERVO
                | V5MotorControlMode::kMotorControlModePROFILE => degrees(vexDeviceMotorTargetGet(device)),
                _ => f64::NAN,
            };
            (
                f64::from(vexDeviceMotorVoltageGet(device)) / 1000.0,
                f64::from(vexDeviceMotorCurrentGet(device)) / 1000.0,
                vexDeviceMotorTemperatureGet(device),
                vexDeviceMotorActualVelocityGet(device),
                vexDeviceMotorTorqueGet(device),
                degrees(vexDeviceMotorPositionGet(device)),
                mode.0,
                vexDeviceMotorBrakeModeGet(device).0,
                target_rpm,
                target_deg,
            )
        };
        let (volts, current, temperature, rpm, torque, position, mode, brake, target_rpm, target_deg) = sample;
        frame.row("MOTOR", format_args!(
            "name=port_{port},port={port},measured_v={volts:.3},current_a={current:.3},temp_c={temperature:.2},rpm={rpm:.2},torque_nm={torque:.4},request_v=NaN,position_deg={position:.3},command_mode=sdk_{mode},request_rpm={target_rpm},request_deg={target_deg},brake_mode=sdk_{brake}"
        ));
    }
}
