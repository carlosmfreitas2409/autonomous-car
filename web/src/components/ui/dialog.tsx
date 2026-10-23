import { useEffect, useRef } from 'react';

import { cn } from '@/utils/cn';

interface DialogProps extends Omit<React.ComponentProps<'dialog'>, 'open'> {
  isOpen: boolean;
  onClose: () => void;
}

function Dialog({ isOpen, onClose, className, children, ...props }: DialogProps) {
  const ref = useRef<HTMLDialogElement>(null);

  useEffect(() => {
    const dialog = ref.current;
    if (!dialog) return;
    if (isOpen && !dialog.open) dialog.showModal();
    if (!isOpen && dialog.open) dialog.close();
  }, [isOpen]);

  useEffect(() => {
    const dialog = ref.current;
    if (!dialog) return;
    function handleBackdropClick(event: MouseEvent) {
      if (event.target === dialog) onClose();
    }
    dialog.addEventListener('click', handleBackdropClick);
    return () => dialog.removeEventListener('click', handleBackdropClick);
  }, [onClose]);

  return (
    <dialog
      ref={ref}
      data-slot="dialog"
      className={cn(
        'pointer-events-auto m-auto max-h-[calc(100dvh-32px)] overflow-visible bg-transparent p-0 text-ink backdrop:bg-black/25',
        className,
      )}
      onClose={onClose}
      {...props}
    >
      {isOpen && children}
    </dialog>
  );
}

export { Dialog };
