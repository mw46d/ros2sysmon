"""Simple Rich layout test program with four rows."""

import time
from rich.console import Console
from rich.live import Live
from rich.layout import Layout
from rich.text import Text


def create_layout() -> Layout:
    """Create a four-row layout structure."""
    layout = Layout()
    
    layout.split_column(
        Layout(name="row1", size=5),
        Layout(name="row2", size=8),
        Layout(name="row3", size=5),
        Layout(name="row4", size=3),
    )
    
    # Split second row into three columns
    layout["row2"].split_row(
        Layout(name="left"),
        Layout(name="middle"),
        Layout(name="right")
    )
    
    return layout


def initialize_panels(layout: Layout):
    """Initialize all panels with grey backgrounds and black text."""
    layout["row1"].update(
        Text("First row - single pane\n\n\n", style="red on bright_white", justify="center")
    )
    
    layout["left"].update(
        Text("Left pane\nof second row", style="green on bright_white", justify="center")
    )
    layout["middle"].update(
        Text("Middle pane\nof second row", style="blue on bright_white", justify="center")
    )
    layout["right"].update(
        Text("Right pane\nof second row", style="purple on bright_white", justify="center")
    )
    
    layout["row3"].update(
        Text("Third row - single pane\nLine 2\nLine 3", style="orange on bright_white", justify="center")
    )
    layout["row4"].update(
        Text("Fourth row - one line", style="bright_red on bright_white", justify="center")
    )


def main():
    """Main function to run the Rich layout test."""
    console = Console()
    layout = create_layout()
    initialize_panels(layout)
    
    try:
        with Live(layout, console=console, screen=True) as live:
            # Keep display alive for 10 seconds
            time.sleep(10)
    except KeyboardInterrupt:
        console.print("Exiting...")


if __name__ == "__main__":
    main()