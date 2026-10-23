import { type VariantProps, cva } from 'class-variance-authority';

import { cn } from '@/utils/cn';

const panelVariants = cva('', {
  variants: {
    elevation: {
      base: 'float',
      raised: 'float-raised',
    },
  },
  defaultVariants: { elevation: 'base' },
});

const buttonGroupVariants = cva('pointer-events-auto flex items-center gap-1 rounded-2xl p-1', {
  variants: {
    surface: {
      floating: 'float',
      inset: 'bg-fill',
    },
  },
  defaultVariants: { surface: 'floating' },
});

type PanelProps = React.ComponentProps<'div'> & VariantProps<typeof panelVariants>;

type ButtonGroupProps = React.ComponentProps<'div'> & VariantProps<typeof buttonGroupVariants>;

function Panel({ className, elevation, ...props }: PanelProps) {
  return <div className={cn(panelVariants({ elevation }), className)} {...props} />;
}

function ButtonGroup({ className, surface, ...props }: ButtonGroupProps) {
  return (
    <div role="group" className={cn(buttonGroupVariants({ surface }), className)} {...props} />
  );
}

export { ButtonGroup, Panel };
