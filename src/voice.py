"""Spoken prompts for the people at a dock, in Portuguese.

The laptop rides on the car, so its speaker is the truck's voice: it tells a
stranger at the dock what the truck is waiting for, and to step back before it
moves. The mission decides when something is said (`Announcement`); this only
turns it into speech. macOS `say` runs in its own process so the loop never
waits on it. A prompt that arrives while one is playing waits for it to end,
and a newer one replaces it: only the latest still matters. Without `say` (not
macOS) or with `--no-voice` it does nothing.

It speaks in the Mac's system voice (System Settings > Accessibility > Spoken
Content > System voice), which is where a Siri voice is chosen: `say -v`
cannot name Siri voices, but a bare `say` uses them.
"""

import logging
import shutil
import subprocess
from typing import Optional

from input.mission import Announcement

logger = logging.getLogger(__name__)

TEXT = {
    "docked_pickup": "Caminhão na doca {letter}. Pode carregar.",
    "docked_delivery": "Entrega na doca {letter}. Pode descarregar.",
    "remind_pickup": "Aguardando o carregamento na doca {letter}.",
    "remind_delivery": "Aguardando a descarga na doca {letter}.",
    "released": "Liberado. Afaste-se, o caminhão vai partir.",
    "blocked": "Por favor, afaste-se da frente do caminhão.",
    "delivered": "Entrega concluída na doca {letter}.",
    "docked_pickup_lid": "Caminhão na doca {letter}. O compartimento está destravado: carregue e feche a tampa.",
    "docked_delivery_lid": "Entrega na doca {letter}. O compartimento está destravado: retire a carga e feche a tampa.",
    "remind_pickup_lid": "Aguardando o carregamento na doca {letter}. Feche a tampa quando terminar.",
    "remind_delivery_lid": "Aguardando a descarga na doca {letter}. Feche a tampa quando terminar.",
    "released_lid": "Compartimento fechado. Afaste-se, o caminhão vai partir.",
    "close_lid": "Feche a tampa do compartimento.",
    "empty": "Nenhuma carga detectada. Coloque a carga e feche a tampa. "
             "Se fechar de novo sem carga, o caminhão parte assim mesmo.",
    "still_loaded": "Ainda há carga no compartimento. Retire a carga e feche a tampa. "
                    "Se fechar de novo com carga, o caminhão parte assim mesmo.",
}

def brazilian_voices(say: str) -> list[str]:
    """The pt_BR voices installed, as `say -v` names them."""
    try:
        listing = subprocess.run([say, "-v", "?"], capture_output=True, text=True, timeout=3).stdout
    except (OSError, subprocess.SubprocessError):
        return []
    return [line.split(" pt_BR ")[0].strip() for line in listing.splitlines() if " pt_BR " in line]


class Voice:
    def __init__(self, enabled: bool = True, voice: Optional[str] = None):
        """`voice`: a `say` voice name (e.g. Luciana); None or "system" is
        the Mac's system voice."""
        self._say = shutil.which("say") if enabled else None
        self._voice = None if self._say is None or voice == "system" else voice
        if self._say is not None:
            logger.info("voice: %s", self._voice or "system voice")
        self._process: Optional[subprocess.Popen] = None
        self._pending: Optional[str] = None
        if enabled and self._say is None:
            logger.info("voice: no `say` on this system, dock prompts are silent")

    def announce(self, announcement: Optional[Announcement]) -> None:
        if announcement is None or self._say is None:
            return
        self._pending = TEXT[announcement.key].format(letter=announcement.letter)
        self.update()

    @property
    def speaking(self) -> bool:
        """A prompt is playing, or waiting for the one before it."""
        return self._pending is not None or (self._process is not None and self._process.poll() is None)

    def update(self) -> None:
        """Start the waiting prompt once the last one has finished; call every frame."""
        if self._pending is None or (self._process is not None and self._process.poll() is None):
            return
        command = [self._say] + (["-v", self._voice] if self._voice else []) + [self._pending]
        self._pending = None
        try:
            self._process = subprocess.Popen(command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except OSError as error:
            logger.warning("voice: %s", error)
            self._say = None

    def close(self) -> None:
        self._pending = None
        if self._process is not None and self._process.poll() is None:
            self._process.terminate()
