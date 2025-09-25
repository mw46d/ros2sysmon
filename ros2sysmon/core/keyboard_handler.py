"""Keyboard input handler for simple key commands."""
import sys
import termios
import tty
import threading
import select
from typing import Callable, Optional


class KeyboardHandler:
    """Simple keyboard handler for single key commands."""
    
    def __init__(self):
        """Initialize the keyboard handler."""
        self.running = False
        self.thread: Optional[threading.Thread] = None
        self.exit_callback: Optional[Callable] = None
        self.old_settings = None
    
    def set_exit_callback(self, callback: Callable):
        """Set callback function to call when 'x' is pressed."""
        self.exit_callback = callback
    
    def start_listening(self):
        """Start listening for keyboard input in background thread."""
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self._listen_loop, daemon=True)
            self.thread.start()
    
    def stop_listening(self):
        """Stop listening for keyboard input."""
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=0.1)
        self._restore_terminal()
    
    def _setup_terminal(self):
        """Setup terminal for raw input."""
        try:
            if sys.stdin.isatty():
                self.old_settings = termios.tcgetattr(sys.stdin.fileno())
                tty.setraw(sys.stdin.fileno())
        except (termios.error, AttributeError):
            # Not a TTY or termios not available
            pass
    
    def _restore_terminal(self):
        """Restore terminal to original settings."""
        try:
            if self.old_settings and sys.stdin.isatty():
                termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, self.old_settings)
        except (termios.error, AttributeError):
            pass
    
    def _listen_loop(self):
        """Main keyboard listening loop."""
        self._setup_terminal()
        
        try:
            while self.running:
                # Check if input is available without blocking
                if sys.stdin.isatty() and select.select([sys.stdin], [], [], 0.1)[0]:
                    try:
                        key = sys.stdin.read(1).lower()
                        if key == 'x' and self.exit_callback:
                            self.exit_callback()
                            break
                    except (OSError, IOError):
                        # Handle cases where stdin is not available
                        break
        finally:
            self._restore_terminal()