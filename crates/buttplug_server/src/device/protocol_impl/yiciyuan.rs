// Buttplug Rust Source Code File - See https://buttplug.io for more info.
//
// Copyright 2016-2026 Nonpolynomial Labs LLC. All rights reserved.
//
// Licensed under the BSD 3-Clause license. See LICENSE file in the project root
// for full license information.

use async_trait::async_trait;
use std::sync::Arc;
use std::sync::atomic::{AtomicU8, Ordering};
use uuid::{Uuid, uuid};

use futures_util::future::BoxFuture;
use futures_util::{FutureExt, future};

use buttplug_core::errors::ButtplugDeviceError;
use buttplug_core::message::{InputReadingV4, InputType, InputTypeReading, InputValue};
use buttplug_server_device_config::Endpoint;

use buttplug_server_device_config::{
  ProtocolCommunicationSpecifier,
  ServerDeviceDefinition,
  UserDeviceIdentifier,
};

use crate::device::{
  hardware::{
    Hardware,
    HardwareCommand,
    HardwareEvent,
    HardwareSubscribeCmd,
    HardwareUnsubscribeCmd,
    HardwareWriteCmd,
  },
  protocol::{
    ProtocolHandler,
    ProtocolIdentifier,
    ProtocolInitializer,
    generic_protocol_initializer_setup,
  },
};

const YICIYUAN_PROTOCOL_UUID: Uuid = uuid!("d5987116-2fba-4c30-a7aa-ef567a3bf35d");

// Stroke range cap. FJB-01 / FJB-02 / YS-TD-0x clamp to 0..=20; FJB-03 doubles
// it to 0..=40 (per app source: `A ? t>40&&(t=40) : t>20&&(t=20)`).
const STROKE_MAX_DEFAULT: u8 = 0x14;
const STROKE_MAX_FJB03: u8 = 0x28;
// Vibe and axis_c are always clamped to 0..=20 regardless of model.
const VIBE_AXIS_MAX: u8 = 0x14;

// Output feature indices, matching the YAML order under `defaults.features`.
const FEATURE_STROKE: u32 = 0;
const FEATURE_VIBE: u32 = 1;
const FEATURE_AXIS_C: u32 = 2;

generic_protocol_initializer_setup!(Yiciyuan, "yiciyuan");

#[derive(Default)]
pub struct YiciyuanInitializer {}

#[async_trait]
impl ProtocolInitializer for YiciyuanInitializer {
  async fn initialize(
    &mut self,
    hardware: Arc<Hardware>,
    _def: &ServerDeviceDefinition,
  ) -> Result<Arc<dyn ProtocolHandler>, ButtplugDeviceError> {
    // FJB-03 is the only model in the family with a model-specific code path
    // in the official app: 6-byte frame with a 1-byte modular checksum and a
    // doubled stroke range. Detect it once at init by the advertised name.
    let is_fjb03 = hardware.name() == "YCY-FJB-03";
    Ok(Arc::new(Yiciyuan {
      is_fjb03,
      stroke: AtomicU8::new(0),
      vibe: AtomicU8::new(0),
      axis_c: AtomicU8::new(0),
    }))
  }
}

/// Per-device state. The protocol sends all three axes in every packet, so
/// we keep the last commanded value for each axis here and rebuild the
/// packet on any axis change.
pub struct Yiciyuan {
  is_fjb03: bool,
  stroke: AtomicU8,
  vibe: AtomicU8,
  axis_c: AtomicU8,
}

impl Yiciyuan {
  fn store(&self, feature_index: u32, value: u32) -> Result<(), ButtplugDeviceError> {
    let v = value.min(100) as u16;
    let level = match feature_index {
      FEATURE_STROKE => {
        let cap = if self.is_fjb03 {
          STROKE_MAX_FJB03
        } else {
          STROKE_MAX_DEFAULT
        };
        ((v * cap as u16 + 50) / 100) as u8
      }
      FEATURE_VIBE | FEATURE_AXIS_C => ((v * VIBE_AXIS_MAX as u16 + 50) / 100) as u8,
      _ => {
        return Err(ButtplugDeviceError::ProtocolSpecificError(
          "Yiciyuan".to_owned(),
          format!("Unknown feature index {}", feature_index),
        ));
      }
    };
    match feature_index {
      FEATURE_STROKE => self.stroke.store(level, Ordering::Relaxed),
      FEATURE_VIBE => self.vibe.store(level, Ordering::Relaxed),
      FEATURE_AXIS_C => self.axis_c.store(level, Ordering::Relaxed),
      _ => unreachable!(),
    }
    Ok(())
  }

  fn build_packet(&self) -> Vec<u8> {
    let stroke = self.stroke.load(Ordering::Relaxed);
    let vibe = self.vibe.load(Ordering::Relaxed);
    let axis_c = self.axis_c.load(Ordering::Relaxed);
    if self.is_fjb03 {
      // FJB-03: 6-byte frame
      //   [0]=0x35, [1]=0x12, [2]=stroke, [3]=vibe, [4]=axis_c, [5]=checksum.
      // Checksum is the low byte of the sum of bytes 0..5 (per the JS
      // `_checksum` helper in mixins/pump.js, which sums every 2-hex-char
      // byte mod 256).
      let body = [0x35u8, 0x12, stroke, vibe, axis_c];
      let checksum = body.iter().fold(0u16, |acc, b| acc + *b as u16) as u8;
      let mut packet = Vec::with_capacity(6);
      packet.extend_from_slice(&body);
      packet.push(checksum);
      packet
    } else {
      // FJB-01 / FJB-02 / YS-TD-0x: 16-byte zero-padded frame.
      //   [0]=0x35, [1]=0x12, [2]=stroke, [3]=vibe, [4]=axis_c, [5..16]=0.
      // Firmware accepts shorter writes but with quirky behaviour (axes left
      // unset hold their previous value); the official app pads to 16 bytes
      // via `(i+Array(32).join("0")).slice(0,32)` in mixins/os_base.js.
      let mut packet = vec![0u8; 16];
      packet[0] = 0x35;
      packet[1] = 0x12;
      packet[2] = stroke;
      packet[3] = vibe;
      packet[4] = axis_c;
      packet
    }
  }

  fn handle_axis_cmd(
    &self,
    feature_index: u32,
    value: u32,
  ) -> Result<Vec<HardwareCommand>, ButtplugDeviceError> {
    self.store(feature_index, value)?;
    Ok(vec![
      HardwareWriteCmd::new(
        &[YICIYUAN_PROTOCOL_UUID],
        Endpoint::Tx,
        self.build_packet(),
        false,
      )
      .into(),
    ])
  }
}

impl ProtocolHandler for Yiciyuan {
  fn handle_output_oscillate_cmd(
    &self,
    feature_index: u32,
    _feature_id: Uuid,
    speed: u32,
  ) -> Result<Vec<HardwareCommand>, ButtplugDeviceError> {
    self.handle_axis_cmd(feature_index, speed)
  }

  fn handle_output_vibrate_cmd(
    &self,
    feature_index: u32,
    _feature_id: Uuid,
    speed: u32,
  ) -> Result<Vec<HardwareCommand>, ButtplugDeviceError> {
    self.handle_axis_cmd(feature_index, speed)
  }

  fn handle_input_subscribe_cmd(
    &self,
    _device_index: u32,
    device: Arc<Hardware>,
    _feature_index: u32,
    feature_id: Uuid,
    sensor_type: InputType,
  ) -> BoxFuture<'_, Result<(), ButtplugDeviceError>> {
    match sensor_type {
      InputType::Battery => {
        async move {
          device
            .subscribe(&HardwareSubscribeCmd::new(
              feature_id,
              Endpoint::RxBLEBattery,
            ))
            .await?;
          Ok(())
        }
      }
      .boxed(),
      _ => future::ready(Err(ButtplugDeviceError::UnhandledCommand(
        "Command not implemented for this sensor".to_string(),
      )))
      .boxed(),
    }
  }

  fn handle_input_unsubscribe_cmd(
    &self,
    device: Arc<Hardware>,
    _feature_index: u32,
    feature_id: Uuid,
    sensor_type: InputType,
  ) -> BoxFuture<'_, Result<(), ButtplugDeviceError>> {
    match sensor_type {
      InputType::Battery => {
        async move {
          device
            .unsubscribe(&HardwareUnsubscribeCmd::new(
              feature_id,
              Endpoint::RxBLEBattery,
            ))
            .await?;
          Ok(())
        }
      }
      .boxed(),
      _ => future::ready(Err(ButtplugDeviceError::UnhandledCommand(
        "Command not implemented for this sensor".to_string(),
      )))
      .boxed(),
    }
  }

  fn handle_battery_level_cmd(
    &self,
    device_index: u32,
    device: Arc<Hardware>,
    feature_index: u32,
    feature_id: Uuid,
  ) -> BoxFuture<'_, Result<InputReadingV4, ButtplugDeviceError>> {
    // The cup pushes battery autonomously at ~1Hz as `35 13 01 P C` on the
    // notify characteristic. Subscribe and wait for the first frame whose
    // prefix matches `0x35 0x13`. Other notify frames (uptime ticks
    // `0x35 0x14 ..`, device-info responses `0x35 0x10 ..`) are skipped.
    let mut event_stream = device.event_stream();
    async move {
      device
        .subscribe(&HardwareSubscribeCmd::new(
          feature_id,
          Endpoint::RxBLEBattery,
        ))
        .await?;
      while let Ok(event) = event_stream.recv().await {
        match event {
          HardwareEvent::Notification(_, endpoint, data) => {
            if endpoint != Endpoint::RxBLEBattery {
              continue;
            }
            // Battery frame layout: [0]=0x35, [1]=0x13, [2]=0x01, [3]=pct.
            if data.len() >= 4 && data[0] == 0x35 && data[1] == 0x13 {
              return Ok(InputReadingV4::new(
                device_index,
                feature_index,
                InputTypeReading::Battery(InputValue::new(data[3])),
              ));
            }
            // Not a battery frame — keep waiting for the next notify.
            continue;
          }
          HardwareEvent::Disconnected(_) => {
            return Err(ButtplugDeviceError::ProtocolSpecificError(
              "Yiciyuan".to_owned(),
              "Yiciyuan device disconnected while waiting for battery push.".to_owned(),
            ));
          }
        }
      }
      Err(ButtplugDeviceError::ProtocolSpecificError(
        "Yiciyuan".to_owned(),
        "Yiciyuan device event stream closed before battery push arrived.".to_owned(),
      ))
    }
    .boxed()
  }
}
