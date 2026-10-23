#!/usr/bin/env python3
"""Hear the dock prompts exactly as the car says them (src/voice.py).

    python tools/voice_preview.py                   # every prompt, the car's voice
    python tools/voice_preview.py released blocked  # just these
    python tools/voice_preview.py --voice Luciana   # try a `say` voice instead
    python tools/voice_preview.py --list            # the Brazilian voices installed
    python tools/voice_preview.py --save .context/voice   # .aiff files to play later

The car speaks in the Mac's system voice: System Settings > Accessibility >
Spoken Content > System voice, Portuguese (Brazil), pick a Siri voice there.
`say -v` cannot name Siri voices, so --list does not show them.
"""

import argparse
import shutil
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from voice import TEXT, brazilian_voices  # noqa: E402

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("keys", nargs="*", help=f"prompts to say (default all): {', '.join(TEXT)}")
parser.add_argument("--voice", help="a `say` voice name instead of the Mac's system voice")
parser.add_argument("--letter", default="A", help="the dock letter in the prompts (default A)")
parser.add_argument("--list", action="store_true", help="list the Brazilian Portuguese voices and exit")
parser.add_argument("--save", help="write one .aiff per prompt into this folder instead of speaking")
args = parser.parse_args()

say = shutil.which("say")
if say is None:
    sys.exit("No `say` here: the voice only exists on macOS.")
if args.list:
    print("\n".join(brazilian_voices(say)) or "No pt_BR voice installed.")
    sys.exit()

voice = None if args.voice == "system" else args.voice
keys = args.keys or list(TEXT)
unknown = [k for k in keys if k not in TEXT]
if unknown:
    sys.exit(f"Unknown prompt(s) {', '.join(unknown)}; known: {', '.join(TEXT)}")
if args.save:
    Path(args.save).mkdir(parents=True, exist_ok=True)

print(f"voice: {voice or 'system voice'}")
for key in keys:
    text = TEXT[key].format(letter=args.letter)
    print(f"{key:16} {text}")
    command = [say] + (["-v", voice] if voice else [])
    if args.save:
        command += ["-o", str(Path(args.save) / f"{key}.aiff")]
    subprocess.run(command + [text], check=True)
