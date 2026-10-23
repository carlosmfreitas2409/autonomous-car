import { cn } from '@/utils/cn';

interface SwitchProps extends Omit<React.ComponentProps<'button'>, 'onClick'> {
  isChecked: boolean;
  onCheckedChange: (isChecked: boolean) => void;
}

function Switch({ isChecked, onCheckedChange, className, children, ...props }: SwitchProps) {
  return (
    <button
      type="button"
      role="switch"
      aria-checked={isChecked}
      data-slot="switch"
      className={cn(
        'flex h-10 w-full items-center justify-between gap-4 text-left text-md disabled:cursor-not-allowed disabled:opacity-50',
        className,
      )}
      onClick={() => onCheckedChange(!isChecked)}
      {...props}
    >
      {children}
      <span
        aria-hidden
        className={cn(
          'flex h-6 w-10 flex-none items-center rounded-full p-0.5 transition-colors duration-150',
          isChecked ? 'bg-ink' : 'bg-line-strong',
        )}
      >
        <span
          className={cn(
            'size-5 rounded-full bg-white shadow-float transition-transform duration-150',
            isChecked && 'translate-x-4',
          )}
        />
      </span>
    </button>
  );
}

export { Switch };
