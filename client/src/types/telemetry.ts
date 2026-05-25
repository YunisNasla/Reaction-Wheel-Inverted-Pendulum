export type RWIPState = "IDLE" | "SWING_UP" | "CATCHING" | "BALANCING" | "FAULT";

export interface TelemetryMessage {
  type: "telemetry";
  angle: number;
  velocity: number;
  pwm: number;
  state: RWIPState;
  balance_duration_ms: number;
  timestamp_ms: number;
}

export interface TelemetryPoint {
  time: number;
  angle: number;
  pwm: number;
}

export interface Session {
  id: number;
  started_at: string;
  ended_at: string;
  duration_ms: number;
  max_angle: number;
}

export type ControlCommand = "start" | "stop" | "estop";
