export interface DriveInput {
  steering: number;
  throttle: number;
}

interface PadLike {
  axes: readonly number[];
  buttons: readonly { value: number }[];
}

const STICK_DEADZONE = 0.08;
const TRIGGER_DEADZONE = 0.05;
const STEER_AXIS = 0;
const FORWARD_BUTTON = 7;
const REVERSE_BUTTON = 6;

export function stick(raw: number) {
  if (Math.abs(raw) < STICK_DEADZONE) return 0;
  return Math.sign(raw) * Math.min((Math.abs(raw) - STICK_DEADZONE) / (1 - STICK_DEADZONE), 1);
}

function trigger(raw: number) {
  return raw < TRIGGER_DEADZONE ? 0 : Math.min(raw, 1);
}

export function fromGamepad(pad: PadLike): DriveInput {
  const forward = trigger(pad.buttons[FORWARD_BUTTON]?.value ?? 0);
  const reverse = trigger(pad.buttons[REVERSE_BUTTON]?.value ?? 0);
  return {
    steering: stick(pad.axes[STEER_AXIS] ?? 0),
    throttle: Math.max(-1, Math.min(1, forward - reverse)),
  };
}
