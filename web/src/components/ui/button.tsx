import { type VariantProps, cva } from 'class-variance-authority';

import { cn } from '@/utils/cn';

const buttonVariants = cva(
  'inline-flex items-center justify-center gap-2 font-semibold whitespace-nowrap transition-[background-color,color,scale,filter] duration-100 active:enabled:scale-97 disabled:cursor-not-allowed',
  {
    variants: {
      variant: {
        ghost: 'text-ink/85 hover:enabled:bg-black/5 disabled:text-ink-3',
        selected: 'bg-ink text-white',
        soft: 'bg-fill text-ink hover:enabled:bg-line',
        stop: 'bg-stop-soft text-stop-ink hover:enabled:brightness-95',
        accent: 'bg-accent text-white hover:enabled:brightness-95',
        overlay: 'bg-black/60 text-white hover:enabled:bg-black/75',
      },
      size: {
        md: 'h-10 rounded-xl px-5 text-md',
        lg: 'h-12 rounded-xl px-4 text-md',
        xl: 'h-16 rounded-2xl px-6 text-xl',
        icon: 'size-8 rounded-lg',
      },
    },
    defaultVariants: { variant: 'ghost', size: 'md' },
  },
);

type ButtonProps = React.ComponentProps<'button'> & VariantProps<typeof buttonVariants>;

function Button({ className, variant, size, type = 'button', ...props }: ButtonProps) {
  return (
    <button
      type={type}
      data-slot="button"
      className={cn(buttonVariants({ variant, size }), className)}
      {...props}
    />
  );
}

export { Button };
