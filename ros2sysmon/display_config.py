"""Display configuration data structure."""
from dataclasses import dataclass


@dataclass
class DisplayConfig:
    """Display preferences configuration."""
    show_colors: bool
    show_progress_bars: bool
    time_format: str