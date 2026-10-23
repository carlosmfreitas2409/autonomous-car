import { type CSSProperties, memo, useId } from 'react';

import { ART_VIEWBOX } from './utils/camera';
import { ART, type Ellipse, GRADIENTS, GROUND, LIGHTS, type LightShape } from './utils/cart-art';

export type TurnSignal = 'left' | 'right' | 'hazard' | null;

interface CartProps {
  isBraking: boolean;
  turnSignal: TurnSignal;
  underglow: string | null;
  width: number;
  style?: CSSProperties;
}

interface RearLightProps {
  shape: LightShape;
  uid: string;
  isBraking: boolean;
  isSignalling: boolean;
  turnSignal: TurnSignal;
}

const SIGNAL_COLOUR = '#ffa31a';
const SIGNAL_RING = '#ffcc81';
const PERIOD_S = 0.9;
const SWEEP_S = 0.34;
const OFF_AT = 0.58;
const SEGMENTS = 8;

const percent = (share: number) => `${(share * 100).toFixed(2)}%`;
const dark = percent(OFF_AT + 0.02);

const SIGNAL_CSS = [
  `@keyframes cart-pool{0%{opacity:0}${percent(SWEEP_S / PERIOD_S)},${percent(OFF_AT)}{opacity:1}${dark},100%{opacity:0}}`,
  `@keyframes cart-gate{0%,${percent(OFF_AT)}{opacity:0}${dark},100%{opacity:1}}`,
  `.cart-pool{animation:cart-pool ${PERIOD_S}s linear infinite}`,
  `.cart-gate{animation:cart-gate ${PERIOD_S}s linear infinite}`,
  '.cart-fade{transition:opacity .12s ease-out}',
  ...Array.from({ length: SEGMENTS }, (_, i) => {
    const start = ((i / (SEGMENTS - 1)) * SWEEP_S) / PERIOD_S;
    return (
      `@keyframes cart-seq${i}{0%,${percent(start)}{opacity:0}${percent(start + 0.005)},${percent(OFF_AT)}{opacity:1}${dark},100%{opacity:0}}` +
      `.cart-seq${i}{animation:cart-seq${i} ${PERIOD_S}s linear infinite}`
    );
  }),
].join('');

function Oval({ shape, ...props }: { shape: Ellipse } & React.ComponentProps<'ellipse'>) {
  const [cx, cy, rx, ry] = shape;
  return <ellipse cx={cx} cy={cy} rx={rx} ry={ry} {...props} />;
}

const Body = memo(function Body({ uid }: { uid: string }) {
  return (
    <g>
      <defs>
        {GRADIENTS.map(([x1, y1, x2, y2, from, to], i) => (
          <linearGradient
            key={i}
            id={`${uid}-g${i}`}
            gradientUnits="userSpaceOnUse"
            x1={x1}
            y1={y1}
            x2={x2}
            y2={y2}
          >
            <stop offset="0" stopColor={from} />
            <stop offset="1" stopColor={to} />
          </linearGradient>
        ))}
      </defs>

      {ART.map(([kind, d, value], i) => {
        if (kind === 1) {
          return (
            <g key={i} fill="none" strokeLinecap="round" strokeLinejoin="round">
              <path d={d} stroke="#0b0c0f" strokeWidth={value} />
              <path
                d={d}
                stroke="#1d1f24"
                strokeWidth={value * 0.62}
                transform={`translate(${-0.06 * value} ${-0.08 * value})`}
              />
              <path
                d={d}
                stroke="#5d626c"
                strokeOpacity="0.85"
                strokeWidth={value * 0.2}
                transform={`translate(${-0.132 * value} ${-0.176 * value})`}
              />
            </g>
          );
        }
        if (kind === 2) {
          return (
            <path
              key={i}
              d={d}
              fill="none"
              stroke="#8b929f"
              strokeOpacity={value}
              strokeWidth="1.3"
              strokeLinecap="round"
            />
          );
        }
        const paint = typeof value === 'number' ? `url(#${uid}-g${value})` : value;
        return (
          <path
            key={i}
            d={d}
            fill={paint}
            stroke={paint}
            strokeWidth="0.9"
            strokeLinejoin="round"
          />
        );
      })}
    </g>
  );
});

function RearLight({ shape, uid, isBraking, isSignalling, turnSignal }: RearLightProps) {
  const isTail = !isSignalling;

  return (
    <g>
      <path d={shape.house} fill="#060607" />
      <path
        d={shape.lens}
        fill={isTail ? '#5a0a10' : '#1a0b0d'}
        stroke={isTail ? '#d8222c' : '#3b1619'}
        strokeWidth="2.2"
      />
      {isTail && (
        <path
          d={shape.lens}
          fill="none"
          stroke="#ff2a2a"
          strokeWidth="6"
          opacity="0.45"
          filter={`url(#${uid}-soft)`}
        />
      )}

      <g
        key={isSignalling ? `gate-${turnSignal}` : 'steady'}
        className={isSignalling ? 'cart-gate' : undefined}
      >
        <g className="cart-fade" style={{ opacity: isBraking ? 1 : 0 }}>
          <path
            d={shape.lens}
            fill="#ff2626"
            stroke="#ff2626"
            strokeWidth="12"
            opacity="0.85"
            filter={`url(#${uid}-glow)`}
          />
          <path d={shape.lens} fill="#ff3030" stroke="#ff8d80" strokeWidth="2.2" />
          <path d={shape.core} fill="#ffe0da" opacity="0.9" />
        </g>
      </g>

      {isSignalling && (
        <g key={turnSignal}>
          <g filter={`url(#${uid}-glow)`} opacity="0.9">
            {shape.segments.map((d, i) => (
              <path
                key={i}
                d={d}
                className={`cart-seq${i}`}
                fill={SIGNAL_COLOUR}
                stroke={SIGNAL_COLOUR}
                strokeWidth="9"
                strokeLinejoin="round"
              />
            ))}
          </g>
          {shape.segments.map((d, i) => (
            <path
              key={i}
              d={d}
              className={`cart-seq${i}`}
              fill={SIGNAL_COLOUR}
              stroke={SIGNAL_RING}
              strokeWidth="1.2"
              strokeLinejoin="round"
            />
          ))}
          <g opacity="0.9">
            {shape.segmentCores.map((d, i) => (
              <path key={i} d={d} className={`cart-seq${i}`} fill="#fff4d6" />
            ))}
          </g>
        </g>
      )}
    </g>
  );
}

export const Cart = memo(function Cart({
  isBraking,
  turnSignal,
  underglow,
  width,
  style,
}: CartProps) {
  const uid = `cart${useId().replace(/[^\w-]/g, '')}`;
  const [vx, vy, vw, vh] = ART_VIEWBOX;
  const isLeft = turnSignal === 'left' || turnSignal === 'hazard';
  const isRight = turnSignal === 'right' || turnSignal === 'hazard';
  const filterRegion = {
    filterUnits: 'userSpaceOnUse',
    x: vx - 200,
    y: vy - 200,
    width: vw + 400,
    height: vh + 400,
  } as const;
  const lightProps = { uid, isBraking, turnSignal };

  return (
    <svg
      viewBox={ART_VIEWBOX.join(' ')}
      width={width}
      height={(width * vh) / vw}
      overflow="visible"
      className="pointer-events-none absolute"
      style={style}
      aria-hidden
    >
      <style>{SIGNAL_CSS}</style>
      <defs>
        <radialGradient id={`${uid}-underglow`}>
          <stop offset="0" stopColor={underglow ?? undefined} stopOpacity="0.95" />
          <stop offset="0.55" stopColor={underglow ?? undefined} stopOpacity="0.42" />
          <stop offset="1" stopColor={underglow ?? undefined} stopOpacity="0" />
        </radialGradient>
        <radialGradient id={`${uid}-shadow`}>
          <stop offset="0" stopColor="#000" stopOpacity="0.7" />
          <stop offset="1" stopColor="#000" stopOpacity="0" />
        </radialGradient>
        <radialGradient id={`${uid}-brake`}>
          <stop offset="0" stopColor="#ff2d2d" stopOpacity="0.5" />
          <stop offset="1" stopColor="#ff2d2d" stopOpacity="0" />
        </radialGradient>
        <radialGradient id={`${uid}-signal`}>
          <stop offset="0" stopColor={SIGNAL_COLOUR} stopOpacity="0.6" />
          <stop offset="1" stopColor={SIGNAL_COLOUR} stopOpacity="0" />
        </radialGradient>
        <filter id={`${uid}-glow`} {...filterRegion}>
          <feGaussianBlur stdDeviation="7" />
        </filter>
        <filter id={`${uid}-soft`} {...filterRegion}>
          <feGaussianBlur stdDeviation="3" />
        </filter>
      </defs>

      {underglow ? (
        <Oval shape={GROUND.glow} fill={`url(#${uid}-underglow)`} />
      ) : (
        <Oval shape={GROUND.shadow} fill={`url(#${uid}-shadow)`} opacity="0.6" />
      )}
      {GROUND.tires.map((tire, i) => (
        <Oval key={i} shape={tire} fill={`url(#${uid}-shadow)`} />
      ))}
      <Oval
        shape={GROUND.brake}
        fill={`url(#${uid}-brake)`}
        className="cart-fade"
        style={{ opacity: isBraking ? 1 : 0 }}
      />
      {isLeft && (
        <Oval
          key={`left-${turnSignal}`}
          shape={GROUND.signalLeft}
          fill={`url(#${uid}-signal)`}
          className="cart-pool"
        />
      )}
      {isRight && (
        <Oval
          key={`right-${turnSignal}`}
          shape={GROUND.signalRight}
          fill={`url(#${uid}-signal)`}
          className="cart-pool"
        />
      )}

      <Body uid={uid} />

      <RearLight shape={LIGHTS.left} isSignalling={isLeft} {...lightProps} />
      <RearLight shape={LIGHTS.right} isSignalling={isRight} {...lightProps} />
    </svg>
  );
});
