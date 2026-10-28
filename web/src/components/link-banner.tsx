import { CONTRACT_VERSION } from '@/constants';
import type { CartLink } from '@/hooks/use-cart-link';
import { cn } from '@/utils/cn';
import { formatNumber } from '@/utils/format';

import { Mark } from './ui/mark';

interface LinkBannerProps {
  link: CartLink;
  isCompact: boolean;
}

function bannerText({ isConnected, ageS, mismatchVersion }: CartLink) {
  if (mismatchVersion !== null) {
    return `Painel e caminhão em versões diferentes (v${CONTRACT_VERSION} e v${mismatchVersion}): recompile web/`;
  }
  if (!isConnected) {
    return 'Sem conexão com o caminhão. No autônomo ele continua andando: pare pelo PS5.';
  }
  if (Number.isFinite(ageS)) return `Dados do caminhão com ${formatNumber(ageS, 1)} s de atraso.`;
  return 'Conectado. Aguardando o primeiro estado do caminhão.';
}

export function LinkBanner({ link, isCompact }: LinkBannerProps) {
  return (
    <div
      role="alert"
      className={cn(
        'absolute z-10 flex items-center gap-2.5 rounded-2xl bg-stop-soft px-4 py-2.5 text-base font-medium text-stop-ink shadow-float',
        isCompact
          ? 'inset-x-4 top-[calc(max(16px,env(safe-area-inset-top))+56px)]'
          : 'top-6 left-1/2 max-w-[min(560px,40vw)] -translate-x-1/2',
      )}
    >
      <Mark tone="stop" />
      {bannerText(link)}
    </div>
  );
}
