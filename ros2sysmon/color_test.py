"""Test program to display Rich color options."""

from rich.console import Console
from rich.table import Table
from rich.text import Text

def show_colors():
    """Display available Rich colors."""
    console = Console()
    
    # Standard colors
    standard_colors = [
        "black", "red", "green", "yellow", "blue", "magenta", "cyan", "white",
        "bright_black", "bright_red", "bright_green", "bright_yellow", 
        "bright_blue", "bright_magenta", "bright_cyan", "bright_white"
    ]
    
    # Grey shades
    grey_colors = [
        f"grey{i}" for i in range(0, 101, 5)
    ]
    
    # Create table for standard colors
    table1 = Table(title="Standard Colors")
    table1.add_column("Color Name")
    table1.add_column("Sample")
    
    for color in standard_colors:
        table1.add_row(
            color,
            Text(f"Sample text", style=f"black on {color}")
        )
    
    console.print(table1)
    console.print()
    
    # Create table for grey shades
    table2 = Table(title="Grey Shades")
    table2.add_column("Color Name")
    table2.add_column("Sample")
    
    for color in grey_colors:
        table2.add_row(
            color,
            Text(f"Sample text", style=f"black on {color}")
        )
    
    console.print(table2)

if __name__ == "__main__":
    show_colors()