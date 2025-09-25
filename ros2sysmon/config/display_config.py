"""Display configuration data structure."""
from dataclasses import dataclass
from .panel_layout_config import PanelLayoutConfig


@dataclass
class DisplayConfig:
    """Display preferences configuration."""
    show_colors: bool
    show_progress_bars: bool
    time_format: str
    panel_layout: PanelLayoutConfig