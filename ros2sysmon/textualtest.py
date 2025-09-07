"""Simple Textual layout test program with four rows."""

from textual.app import App
from textual.containers import Vertical, Horizontal
from textual.widgets import Static


class LayoutTestApp(App):
    """Textual app with four-row layout."""
    
    CSS = """
    #row1 {
        height: 5;
        background: red;
        color: white;
        text-align: center;
        border-bottom: solid white;
    }
    
    #left {
        height: 8;
        width: 1fr;
        background: green;
        color: white;
        text-align: center;
        border-right: solid white;
        border-bottom: solid white;
    }
    
    #middle {
        height: 8;
        width: 1fr;
        background: blue;
        color: white;
        text-align: center;
        border-right: solid white;
        border-bottom: solid white;
    }
    
    #right {
        height: 8;
        width: 1fr;
        background: purple;
        color: white;
        text-align: center;
        border-bottom: solid white;
    }
    
    #row3 {
        height: 5;
        background: orange;
        color: white;
        text-align: center;
        border-bottom: solid white;
    }
    
    #row4 {
        height: 3;
        background: cyan;
        color: black;
        text-align: center;
    }
    
    .row2 {
        height: 8;
        border: solid white;
    }
    """

    def compose(self):
        """Create the layout structure."""
        with Vertical():
            yield Static("First row - single pane", id="row1")
            
            with Horizontal(classes="row2"):
                yield Static("Left pane\nof second row", id="left")
                yield Static("Middle pane\nof second row", id="middle")
                yield Static("Right pane\nof second row", id="right")
            
            yield Static("Third row - single pane\nLine 2\nLine 3", id="row3")
            yield Static("Fourth row - one line", id="row4")


def main():
    """Main function to run the Textual layout test."""
    app = LayoutTestApp()
    app.run()


if __name__ == "__main__":
    main()