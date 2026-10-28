import type { CartState, Mission } from '@/types';
import { missionLine } from '@/utils/loop-text';

export type DeliveryAction = 'cancel' | 'stop' | 'start' | null;

export interface DeliveryStatus {
  title: string;
  route: string;
  isPicking: boolean;
  isRunning: boolean;
  isSet: boolean;
  progress: number;
  progressLabel: string;
  note: string;
  action: DeliveryAction;
  actionLabel: string;
}

function title(mission: Mission, route: string, isPicking: boolean, isFresh: boolean) {
  if (mission.stage === 'done') return `Entregue em ${mission.parked_at ?? mission.delivery}`;
  if (isPicking) return isFresh ? `Pronto: ${route}` : 'Nova entrega';
  if (mission.stage === 'pickup') return `Indo coletar em ${mission.pickup}`;
  if (mission.stage === 'delivery') return `Entregando em ${mission.delivery}`;
  return 'Sem entrega';
}

function note(mission: Mission, isSet: boolean, isStopped: boolean) {
  if (!mission.applied && isSet && !isStopped) return 'Não aplicado no manual';
  if (mission.state === 'slowing' || mission.state === 'leaving') return missionLine(mission);
  return '';
}

export function deliveryStatus(state: CartState): DeliveryStatus {
  const mission = state.mission;
  const isStopped = state.drive.estopped;
  const route = mission.pickup && mission.delivery ? `${mission.pickup} → ${mission.delivery}` : '';
  const isSet = mission.stage !== 'idle';
  const isDone = mission.stage === 'done';
  // Docked, even at the start of an order, the delivery is under way.
  const isFresh = isSet && !isDone && mission.step === 0 && !mission.dock;
  // Parked after a delivery, the next order is given where it stands.
  const isParked = mission.parked_at !== null && !mission.dock;
  const isPicking = (isStopped && (!isSet || isFresh)) || isParked;
  const isRunning = isSet && !isFresh;

  let action: DeliveryAction = null;
  let actionLabel = 'Escolha a coleta e a entrega';
  if (isRunning || (isSet && !isStopped)) {
    action = 'cancel';
    actionLabel = isDone ? 'Limpar entrega' : 'Cancelar entrega';
  } else if (!isStopped) {
    action = 'stop';
    actionLabel = 'Pare para definir uma entrega';
  } else if (isFresh) {
    action = 'start';
    actionLabel = `Iniciar entrega ${route}`;
  }

  return {
    title: title(mission, route, isPicking, isFresh),
    route,
    isPicking,
    isRunning,
    isSet,
    progress: isDone ? 1 : mission.route.length ? mission.step / mission.route.length : 0,
    progressLabel: isDone ? 'Concluída' : `Etapa ${mission.step + 1} de ${mission.route.length}`,
    note: note(mission, isSet, isStopped),
    action,
    actionLabel,
  };
}

export function positionText(mission: Mission | undefined) {
  if (!mission?.segment) return 'Posição desconhecida até uma entrega começar';
  return mission.segment.kind === 'detour'
    ? `Dentro do desvio ${mission.segment.exit}`
    : `Pista principal, rumo à saída ${mission.segment.towards}`;
}
