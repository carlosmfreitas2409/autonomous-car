import type { CartState, DockCheck, Mission, Policy, Reply } from '@/types';

import { formatNumber } from './format';

const CONTROLLER_STOPS: Record<string, string> = {
  obstacle: 'Obstáculo à frente',
  'lane lost': 'Faixa perdida',
};

const POLICY_REASONS: Record<string, string> = {
  'stopped at sign': 'Parado na placa de pare',
  'waiting for green': 'Aguardando o sinal verde',
  'stop sign ahead': 'Placa de pare à frente',
  'red light ahead': 'Sinal vermelho à frente',
  stale: 'Câmera de placas atrasada',
};

const POLICY_STATES: Record<string, string> = {
  slowing: 'Reduzindo',
  holding: 'Parado na placa de pare',
  dwelling: 'Parado na placa de pare',
  waiting_green: 'Aguardando o sinal verde',
};

export const CHECK_TEXT: Record<DockCheck, string> = {
  announcing: 'Avisando que vai sair',
  lid: 'Aguardando fechar a tampa',
  weighing: 'Pesando a carga',
  clearing: 'Conferindo se a frente está livre',
  blocked: 'Frente bloqueada · afaste-se',
};

export const CHECK_FAILED_TEXT = {
  empty: 'Nenhuma carga detectada',
  still_loaded: 'Ainda há carga no compartimento',
} as const;

const COMMANDS: Record<string, string> = {
  estop: 'Parar',
  mode: 'Modo',
  challenge: 'Prova',
  path: 'Trajeto',
  mission: 'Entrega',
  lights: 'Luzes',
  speed: 'Velocidade',
  pid: 'PID',
  dock: 'Doca',
  open_cargo: 'Compartimento',
};

const REFUSALS: Record<string, string> = {
  'e-stopped, resume first': 'Caminhão parado, retome primeiro',
  'e-stop first': 'Pare o caminhão primeiro',
  'no docks in the speed test': 'Sem docas na prova de velocidade',
  'the stage can only be changed while e-stopped': 'Pare o caminhão para trocar a prova',
  'the speed test drives Straight only': 'A prova de velocidade só segue reto',
  'not leaving a dock': 'O caminhão não está saindo da doca',
  'pickup/delivery can only be changed while e-stopped or parked at a delivery':
    'Pare o caminhão para mudar a entrega',
  'invalid command': 'Comando inválido',
  'bad json': 'Mensagem inválida',
  busy: 'Caminhão ocupado, tente de novo',
};

export function missionLine(mission: Mission): string {
  const { dock, target } = mission;
  if (mission.state === 'docked' && dock) {
    if (dock.check_failed && !dock.override) return CHECK_FAILED_TEXT[dock.check_failed];
    if (dock.role === 'pickup') return `Carregando em ${dock.letter}`;
    if (dock.role === 'delivery') return `Descarregando em ${dock.letter}`;
    return `Pronto para sair de ${dock.letter}`;
  }
  if (mission.state === 'checking') {
    return mission.check ? CHECK_TEXT[mission.check] : `Saindo de ${dock?.letter ?? ''}`.trim();
  }
  if (mission.state === 'slowing') {
    return mission.stage === 'pickup'
      ? `Chegando para coletar em ${target}`
      : `Chegando para entregar em ${target}`;
  }
  if (mission.state === 'leaving') return `Seguindo para ${target}`;
  if (mission.state === 'done') return `Entregue em ${mission.parked_at ?? mission.delivery}`;
  if (mission.stage === 'pickup') return `Indo coletar em ${target}`;
  if (mission.stage === 'delivery') return `Indo entregar em ${target}`;
  return mission.reason;
}

export function policyText(policy: Policy): string {
  return POLICY_REASONS[policy.reason] ?? POLICY_STATES[policy.state] ?? policy.reason;
}

export function stopTitle(state: CartState): string | null {
  const { drive } = state;
  if (!drive.stop_text) return null;
  switch (drive.stop_source) {
    case 'estop':
      return 'Parado';
    case 'controller':
      return CONTROLLER_STOPS[drive.stop_text] ?? drive.stop_text;
    case 'sign':
      return policyText(state.policy);
    case 'mission':
      return missionLine(state.mission);
    case 'holding':
      return `Segurando ${formatNumber(state.path.holding_s, 1)} s`;
    default:
      return drive.stop_text;
  }
}

export function refusalText(reply: Pick<Reply, 'cmd' | 'reason'>) {
  const command = (reply.cmd && COMMANDS[reply.cmd]) ?? reply.cmd ?? 'Comando';
  const reason = reply.reason ?? '';
  return `${command} · ${REFUSALS[reason] ?? (reason || 'recusado')}`;
}
