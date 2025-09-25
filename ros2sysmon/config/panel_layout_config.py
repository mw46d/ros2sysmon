"""Panel layout configuration data structure."""
from dataclasses import dataclass, field
from typing import List


@dataclass
class PanelLayoutConfig:
    """Configuration for panel layout across screens."""
    screen_1: List[str] = field(default_factory=lambda: ["topics", "tfs"])
    screen_2: List[str] = field(default_factory=lambda: ["nodes", "processes"])
    hidden: List[str] = field(default_factory=list)

    def __post_init__(self):
        """Validate panel layout configuration."""
        valid_panels = {"topics", "tfs", "nodes", "processes"}
        all_panels = set(self.screen_1 + self.screen_2 + self.hidden)

        # Check for missing panels
        missing = valid_panels - all_panels
        if missing:
            raise ValueError(f"Missing panels in layout: {missing}")

        # Check for extra panels
        extra = all_panels - valid_panels
        if extra:
            raise ValueError(f"Unknown panels in layout: {extra}")

        # Check for duplicates
        all_panels_list = self.screen_1 + self.screen_2 + self.hidden
        if len(all_panels_list) != len(set(all_panels_list)):
            raise ValueError("Duplicate panels found in layout")

        # Check screen limits (max 2 panels per screen for readability)
        if len(self.screen_1) > 2:
            raise ValueError("Screen 1 cannot have more than 2 panels")
        if len(self.screen_2) > 2:
            raise ValueError("Screen 2 cannot have more than 2 panels")

    def get_panels_for_screen(self, screen_num: int) -> List[str]:
        """Get ordered list of panels for a screen."""
        if screen_num == 1:
            return self.screen_1.copy()
        elif screen_num == 2:
            return self.screen_2.copy()
        return []