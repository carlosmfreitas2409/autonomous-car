import type { CartState, Letter, TrackLayout } from '@/types';
import { cn } from '@/utils/cn';

import { positionText } from './utils/delivery-status';
import { ROAD, VIEW, exitPoint, letterPositions, roads, routeLegs } from './utils/track-geometry';

interface TrackMapProps {
  state: CartState | null;
  layout: TrackLayout;
}

const ROUTE_WIDTH = 34;
const NO_ROUTE = { current: null, next: null };

export function TrackMap({ state, layout }: TrackMapProps) {
  const mission = state?.mission;
  const route = mission ? routeLegs(layout, mission) : NO_ROUTE;
  const next = mission?.next ?? null;
  const letters = letterPositions(layout);

  return (
    <svg
      viewBox={`${VIEW.x} ${VIEW.y} ${VIEW.w} ${VIEW.h}`}
      preserveAspectRatio="xMidYMid meet"
      className="block w-full"
      role="img"
      aria-label={positionText(mission)}
    >
      {Object.entries(roads(layout)).map(([key, d]) => (
        <path key={key} d={d} className="fill-none stroke-line" strokeWidth={ROAD} />
      ))}
      {route.next && (
        <path
          d={route.next}
          className="fill-none stroke-accent opacity-35"
          strokeWidth={ROUTE_WIDTH}
          strokeLinecap="round"
          strokeLinejoin="round"
        />
      )}
      {route.current && (
        <path
          d={route.current}
          className="fill-none stroke-accent"
          strokeWidth={ROUTE_WIDTH}
          strokeLinecap="round"
          strokeLinejoin="round"
        />
      )}

      <path
        d="M 390 100 l 40 35 l -40 35"
        className="fill-none stroke-ink-3"
        strokeWidth={14}
        strokeLinecap="round"
        strokeLinejoin="round"
      />

      {layout.exits.map((exit) => {
        const point = exitPoint(exit);
        const isUpcoming = next?.exit === exit;
        return (
          <g key={exit}>
            <circle
              cx={point.x}
              cy={point.y}
              r={isUpcoming ? 34 : 22}
              className={isUpcoming ? 'fill-ink' : 'fill-line-strong'}
            />
            {isUpcoming && (
              <text
                x={point.x}
                y={point.y - 78}
                textAnchor="middle"
                fontSize={84}
                className="fill-ink font-semibold"
              >
                Saída {exit} → {next.take === 'detour' ? 'desvio' : 'reto'}
              </text>
            )}
          </g>
        );
      })}

      {(Object.keys(letters) as Letter[]).map((letter) => {
        const point = letters[letter];
        const isPickup = mission?.pickup === letter;
        const isDelivery = !isPickup && mission?.delivery === letter;
        return (
          <g key={letter}>
            <circle
              cx={point.x}
              cy={point.y}
              r={74}
              strokeWidth={9}
              className={cn(
                'fill-white stroke-line-strong',
                isPickup && 'fill-ink stroke-none',
                isDelivery && 'fill-accent stroke-none',
              )}
            />
            {mission?.target === letter && (
              <circle
                cx={point.x}
                cy={point.y}
                r={100}
                className="fill-none stroke-accent"
                strokeWidth={12}
              />
            )}
            <text
              x={point.x}
              y={point.y + 29}
              textAnchor="middle"
              fontSize={82}
              className={cn('font-semibold', isPickup || isDelivery ? 'fill-white' : 'fill-ink')}
            >
              {letter}
            </text>
          </g>
        );
      })}
    </svg>
  );
}
