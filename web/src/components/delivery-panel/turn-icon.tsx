interface TurnIconProps {
  take: 'detour' | 'straight';
}

export function TurnIcon({ take }: TurnIconProps) {
  return (
    <svg
      viewBox="0 0 20 20"
      aria-hidden="true"
      className="size-4 shrink-0"
      fill="none"
      stroke="currentColor"
      strokeWidth={2}
      strokeLinecap="round"
      strokeLinejoin="round"
    >
      {take === 'detour' ? (
        <path d="M6 17V10a4 4 0 0 1 4-4h6M13 3l3 3-3 3" />
      ) : (
        <path d="M10 17V4M6 8l4-4 4 4" />
      )}
    </svg>
  );
}
